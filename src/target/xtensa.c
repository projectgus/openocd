/***************************************************************************
 *   Copyright (C) 2015 by Angus Gratton                                   *
 *   gus@projectgus.com                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

/*
 * Xtensa arch target implementation. Very preliminary & restricted in scope:
 *
 * - Assumes OCD debug feature.
 * - Other features/config modelled closely on lx106 (esp8266 SoC) at this time.
 * - Assumes little endian target.
 * - Assumes none of the following options configured:
 *   * "DIR Array option"
 *   * "Instruction cache option" (XCHAL_ICACHE_SIZE >0 in xtensa-config.h)
 *   * "MMU Option" (XCHAL_HAVE_MMU in xtensa-config.h)
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"
#include "register.h"

#include "xtensa.h"

/*******************************************************************
 *                        Xtensa Config Options                    *
 *******************************************************************/

/* This is stuff which is currently hard-coded but ideally would
 * eventually be per-target configured.
 */
#define XT_INS_NUM_BITS 24
#define XT_DEBUGLEVEL    2 /* XCHAL_DEBUGLEVEL in xtensa-config.h */


/*******************************************************************
 *                        Xtensa OCD TAP instructions              *
 *******************************************************************/

/* Xtensa OCD TAP instructions */
enum xtensa_tap_ins_idx {
	TAP_INS_ENABLE_OCD  = 0,
	TAP_INS_DEBUG_INT = 1,
	TAP_INS_EXECUTE_DI = 2,
	TAP_INS_LOAD_DI = 3,
	TAP_INS_SCAN_DDR  = 4,
	TAP_INS_READ_DOSR  = 5,
	TAP_INS_SCAN_DCR = 6,
	TAP_INS_LOAD_WDI = 7,
};

#define XTENSA_NUM_TAP_INS 8

static const struct xtensa_tap_instr tap_instrs[] = {
	{ TAP_INS_ENABLE_OCD, 0x11, 1,                    "EnableOCD" },
	{ TAP_INS_DEBUG_INT,  0x12, 1,                    "DebugInt"  },
	{ TAP_INS_EXECUTE_DI, 0x15, 1,                    "ExecuteDI" },
	{ TAP_INS_LOAD_DI,    0x16, XT_INS_NUM_BITS, "LoadDI"    },
	{ TAP_INS_SCAN_DDR,   0x17, 32,                   "ScanDDR"   },
	{ TAP_INS_READ_DOSR,  0x18, 8,                    "ReadDOSR"  },
	{ TAP_INS_SCAN_DCR,   0x19, 8,                    "ScanDCR"   },
	{ TAP_INS_LOAD_WDI,   0x1a, XT_INS_NUM_BITS, "LoadWDI"   },
};
/* Static buffer that holds the tap instructions ready for sending to JTAG */
static uint8_t tap_instr_buf[XTENSA_NUM_TAP_INS*4];

/* Debug Output Status Register (DOSR) fields */
#define DOSR_NEXT_DI     (1<<0)
#define DOSR_EXCEPTION   (1<<1)
#define DOSR_IN_OCD_MODE (1<<2)
#define DOSR_DOD_READY   (1<<3)

/*******************************************************************
 *                        Xtensa CPU registers                     *
 *******************************************************************/

/* Xtensa register list currently taken from
   ./overlays/xtensa_lx106/gdb/gdb/xtensa-config.c - probably entirely
   overlay-specific :(. Maybe eventually should live in config file,
   or least a mapping in config file from openocd's set of all
   possible Xtensa registers to the register index used by gdb for the
   current overlay (currently this is a 1:1 mapping to lx106 regs only)?
*/
enum xtensa_reg_idx {
	XT_REG_IDX_A0 = 0,
	XT_REG_IDX_A1,
	XT_REG_IDX_A2,
	XT_REG_IDX_A3,
	XT_REG_IDX_A4,
	XT_REG_IDX_A5,
	XT_REG_IDX_A6,
	XT_REG_IDX_A7,
	XT_REG_IDX_A8,
	XT_REG_IDX_A9,
	XT_REG_IDX_A10,
	XT_REG_IDX_A11,
	XT_REG_IDX_A12,
	XT_REG_IDX_A13,
	XT_REG_IDX_A14,
	XT_REG_IDX_A15,
	XT_REG_IDX_PC,
	XT_REG_IDX_SAR,
	XT_REG_IDX_LITBASE,
	XT_REG_IDX_SR176,
	XT_REG_IDX_SR208,
	XT_REG_IDX_PS,
	XT_REG_IDX_MMID,
	XT_REG_IDX_IBREAKENABLE,
	XT_REG_IDX_DDR,
	XT_REG_IDX_IBREAKA0,
	XT_REG_IDX_DBREAKA0,
	XT_REG_IDX_DBREAKC0,
	XT_REG_IDX_EPC1,
	XT_REG_IDX_EPC2,
	XT_REG_IDX_EPC3,
	XT_REG_IDX_DEPC,
	XT_REG_IDX_EPS2,
	XT_REG_IDX_EPS3,
	XT_REG_IDX_EXCSAVE1,
	XT_REG_IDX_EXCSAVE2,
	XT_REG_IDX_EXCSAVE3,
	XT_REG_IDX_INTERRUPT,
	XT_REG_IDX_INTSET,
	XT_REG_IDX_INTCLEAR,
	XT_REG_IDX_INTENABLE,
	XT_REG_IDX_VECBASE,
	XT_REG_IDX_EXCCAUSE,
	XT_REG_IDX_DEBUGCAUSE,
	XT_REG_IDX_CCOUNT,
	XT_REG_IDX_PRID,
	XT_REG_IDX_ICOUNT,
	XT_REG_IDX_ICOUNTLEVEL,
	XT_REG_IDX_EXCVADDR,
	XT_REG_IDX_CCOMPARE0,
};

#define XT_NUM_REGS 50

/* PC register isn't directly accessible, it's EPC(DEBUGLEVEL) instead */
#define XT_REG_IDX_PC_ALIAS (XT_REG_IDX_EPC1-1+XT_DEBUGLEVEL)

static const struct xtensa_core_reg xt_regs[] = {
	{ XT_REG_IDX_A0, "a0",                     0x00, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A1, "a1",                     0x01, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A2, "a2",                     0x02, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A3, "a3",                     0x03, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A4, "a4",                     0x04, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A5, "a5",                     0x05, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A6, "a6",                     0x06, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A7, "a7",                     0x07, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A8, "a8",                     0x08, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A9, "a9",                     0x09, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A10, "a10",                   0x0a, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A11, "a11",                   0x0b, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A12, "a12",                   0x0c, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A13, "a13",                   0x0d, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A14, "a14",                   0x0e, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_A15, "a15",                   0x0f, XT_REG_GENERAL, 0 },
	{ XT_REG_IDX_PC,  "PC",                    0x20, XT_REG_PC     , 0 },
	{ XT_REG_IDX_SAR, "SAR",                   0x03, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_LITBASE,      "LITBASE",      0x05, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_SR176,        "SR176",        0xb0, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_SR208,        "SR208",        0xd0, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_PS,           "PS",           0xe6, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_MMID,         "MMID",         0x59, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_IBREAKENABLE, "IBREAKENABLE", 0x60, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_DDR,          "DDR",          0x68, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_IBREAKA0,     "IBREAKA0",     0x80, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_DBREAKA0,     "DBREAKA0",     0x90, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_DBREAKC0,     "DBREAKC0",     0xa0, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_EPC1,         "EPC1",         0xb1, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_EPC2,         "EPC2",         0xb2, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_EPC3,         "EPC3",         0xb3, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_DEPC,         "DEPC",         0xc0, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_EPS2,         "EPS2",         0xc2, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_EPS3,         "EPS3",         0xc3, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_EXCSAVE1,     "EXCSAVE1",     0xd1, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_EXCSAVE2,     "EXCSAVE2",     0xd2, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_EXCSAVE3,     "EXCSAVE3",     0xd3, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_INTERRUPT,    "INTERRUPT",    0xe2, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_INTSET,       "INTSET",       0xe2, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_INTCLEAR,     "INTCLEAR",     0xe3, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_INTENABLE,    "INTENABLE",    0xe4, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_VECBASE,      "VECBASE",      0xe7, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_EXCCAUSE,     "EXCCAUSE",     0xe8, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_DEBUGCAUSE,   "DEBUGCAUSE",   0xe9, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_CCOUNT,       "CCOUNT",       0xea, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_PRID,         "PRID",         0xeb, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_ICOUNT,       "ICOUNT",       0xec, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_ICOUNTLEVEL,  "ICOUNTLEVEL",  0xed, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_EXCVADDR,     "EXCVADDR",     0xee, XT_REG_SPECIAL, 0 },
	{ XT_REG_IDX_CCOMPARE0,    "CCOMPARE0",    0xf0, XT_REG_SPECIAL, 0 },
};

/*******************************************************************
 *                       Xtensa CPU instructions                   *
 *******************************************************************/

#define _XT_INS_FORMAT_RSR(OPCODE,SR,T) (OPCODE			\
					 | ((SR & 0xFF) << 8)	\
					 | ((T & 0x0F) << 4))

#define _XT_INS_FORMAT_RRI8(OPCODE,R,S,T,IMM8) (OPCODE			\
						| ((IMM8 & 0xFF) << 16) \
						| ((R & 0x0F) << 12 )	\
						| ((S & 0x0F) << 8 )	\
						| ((T & 0x0F) << 4 ))
/* Special register indexes
   These can be derived from the regnum field in the register table, above,
   but because they're frequently used in the debugger code we've extracted
   them here to their own constants.

   TODO: Make this a smart macro or something
*/
#define XT_SR_DDR         0x68
#define XT_SR_ICOUNT      0xec
#define XT_SR_ICOUNTLEVEL 0xed

/* Xtensa processor instruction opcodes
   TODO: Almost certainly host-endianness issues here
*/
/* "Return From Debug Operation" to Normal */
#define XT_INS_RFDO_0      0xf1e000
/* "Return From Debug Operation" to OCD Run */
#define XT_INS_RFDO_1      0xf1e100

/* Load 32-bit Indirect from A(S)+4*IMM8 to A(T) */
#define XT_INS_L32I(S,T,IMM8)  _XT_INS_FORMAT_RRI8(0x002002,0,S,T,IMM8)
/* Load 16-bit Unsigned from A(S)+2*IMM8 to A(T) */
#define XT_INS_L16UI(S,T,IMM8) _XT_INS_FORMAT_RRI8(0x001002,0,S,T,IMM8)
/* Load 8-bit Unsigned from A(S)+IMM8 to A(T) */
#define XT_INS_L8UI(S,T,IMM8)  _XT_INS_FORMAT_RRI8(0x000002,0,S,T,IMM8)

/* Store 32-bit Indirect to A(S)+4*IMM8 from A(T) */
#define XT_INS_S32I(S,T,IMM8) _XT_INS_FORMAT_RRI8(0x006002,0,S,T,IMM8)
/* Store 16-bit to A(S)+2*IMM8 from A(T) */
#define XT_INS_S16I(S,T,IMM8) _XT_INS_FORMAT_RRI8(0x005002,0,S,T,IMM8)
/* Store 8-bit to A(S)+IMM8 from A(T) */
#define XT_INS_S8I(S,T,IMM8)  _XT_INS_FORMAT_RRI8(0x004002,0,S,T,IMM8)

/* Read Special Register */
#define XT_INS_RSR(SR,T) _XT_INS_FORMAT_RSR(0x030000,SR,T)
/* Write Special Register */
#define XT_INS_WSR(SR,T) _XT_INS_FORMAT_RSR(0x130000,SR,T)
/* Swap Special Register */
#define XT_INS_XSR(SR,T) _XT_INS_FORMAT_RSR(0x610000,SR,T)

/* Forward declarations */
static int xtensa_get_core_reg(struct reg *reg);
static int xtensa_set_core_reg(struct reg *reg, uint8_t *buf);
static int xtensa_save_context(struct target *target);
static int xtensa_restore_context(struct target *target);


/* Add an Xtensa OCD TAP instruction to the JTAG queue */
static int xtensa_tap_queue(struct target *target, int inst_idx, const uint8_t *data_out, uint8_t *data_in)
{
	const struct xtensa_tap_instr *ins;
	static const uint8_t dummy_out[] = {0,0,0,0};

	if ((inst_idx < 0 || inst_idx >= XTENSA_NUM_TAP_INS))
		return ERROR_COMMAND_SYNTAX_ERROR;
	ins = &tap_instrs[inst_idx];

	if(!data_out)
		data_out = dummy_out;

	if (!target->tap)
		return ERROR_FAIL;

	jtag_add_plain_ir_scan(target->tap->ir_length, tap_instr_buf+inst_idx*4,
			       NULL, TAP_IDLE);
	jtag_add_plain_dr_scan(ins->data_len, data_out, data_in, TAP_IDLE);

	return ERROR_OK;
}

/* Execute an Xtensa OCD TAP instruction immediately */
static int xtensa_tap_exec(struct target *target, int inst_idx, uint32_t data_out, uint32_t *data_in)
{
	uint8_t out[4] = { 0 }, in[4] = { 0 };
	int res;
	if(data_out)
		buf_set_u32(out, 0, 32, data_out);

	res = xtensa_tap_queue(target, inst_idx, out, in);
	if(res != ERROR_OK)
		return res;

	res = jtag_execute_queue();
	if(res != ERROR_OK) {
		LOG_ERROR("failed to scan tap instruction");
		return res;
	}


	if(data_in) {
		*data_in = buf_get_u32(in, 0, 32);
		LOG_DEBUG("Executed %s, data_out=0x%" PRIx32 " data_in=0x%" PRIx32,
			  tap_instrs[inst_idx].name, data_out, *data_in);
	} else {
		LOG_DEBUG("Executed %s, data_out=0x%" PRIx32,
			  tap_instrs[inst_idx].name, data_out);
	}

	return ERROR_OK;
}

/* Set up a register we intend to use for scratch purposes */
static int xtensa_setup_scratch_reg(struct target *target, int reg_idx)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	int res;
	if (reg_idx < 0 || reg_idx >= XT_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!reg_list[reg_idx].valid) {
		res = xtensa_get_core_reg(&reg_list[reg_idx]);
		if (res != ERROR_OK)
			return res;
	}
	reg_list[reg_idx].dirty = 1;
	return ERROR_OK;
}


/* Queue an Xtensa CPU instruction via the TAP's LoadDI function */
static inline int xtensa_tap_queue_cpu_inst(struct target *target, uint32_t inst)
{
	uint8_t inst_buf[4];
	buf_set_u32(inst_buf, 0, 32, inst);
	return xtensa_tap_queue(target, TAP_INS_LOAD_DI, inst_buf, NULL);
}


/* Queue a load to a general register aX, via DDR  */
static inline int xtensa_tap_queue_load_general_reg(struct target *target, uint8_t general_reg_num, uint32_t value)
{
	uint8_t value_buf[4];
	int res;
	buf_set_u32(value_buf, 0, 32, value);
	res = xtensa_tap_queue(target, TAP_INS_SCAN_DDR, value_buf, NULL);
	if(res != ERROR_OK)
		return res;
	return xtensa_tap_queue_cpu_inst(target, XT_INS_RSR(XT_SR_DDR, general_reg_num));
}


/* Queue a write to an Xtensa special register, via the WSR instruction.

   This function does not go through the gdb-facing register cache.
*/
static int xtensa_tap_queue_write_sr(struct target *target, int idx, uint32_t value)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	struct reg *reg;
	struct xtensa_core_reg *xt_reg;
	int res;

	if(idx < 0 || idx >= XT_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg = &reg_list[idx];
	xt_reg = reg->arch_info;

	if(xt_reg->type != XT_REG_SPECIAL)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Use a0 as working register */
	xtensa_setup_scratch_reg(target, XT_REG_IDX_A0);

	/* Push new value into a0 via DDR */
	res = xtensa_tap_queue_load_general_reg(target, 0, value);
	if(res != ERROR_OK)
		return res;
	/* load Special Register from a0 */
	res = xtensa_tap_queue_cpu_inst(target, XT_INS_WSR(xt_reg->reg_num, 0));

	return res;
}

static int xtensa_target_create(struct target *target, Jim_Interp *interp)
{
	struct xtensa_common *xtensa = calloc(1, sizeof(struct xtensa_common));

	if (!xtensa)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target->arch_info = xtensa;
	xtensa->tap = target->tap;

	return ERROR_OK;
}

static void xtensa_build_reg_cache(struct target *target);

static int xtensa_init_target(struct command_context *cmd_ctx, struct target *target)
{
	int i;
	LOG_DEBUG("%s", __func__);
	struct xtensa_common *xtensa = target_to_xtensa(target);

	xtensa_build_reg_cache(target);

	xtensa->state = XT_NORMAL; // Assume normal state until we examine

	/* TODO: Reset breakpoint state and build reg cache */

	/* pre-seed TAP instruction buffer with tap instruction opcodes */
	for(i = 0; i < XTENSA_NUM_TAP_INS; i++)
		buf_set_u32(tap_instr_buf+4*i, 0, 32, tap_instrs[i].inst);

	return ERROR_OK;
}

static int xtensa_poll(struct target *target)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *pc;
	uint32_t dosr;
	int res;

	/* OCD guide 2.9.2 points out no reliable way to detect core reset.

	   So, even though this ENABLE_OCD is nearly always a No-Op, we send it
	   on every poll just in case the target has reset and gone back to Running state
	   (in which case this moves it to OCD Run State. */
	res = xtensa_tap_queue(target, TAP_INS_ENABLE_OCD, NULL, NULL);
	if(res != ERROR_OK) {
		LOG_ERROR("Failed to queue EnableOCD instruction.");
		return ERROR_FAIL;
	}

	res = xtensa_tap_exec(target, TAP_INS_READ_DOSR, 0, &dosr);
	if(res != ERROR_OK) {
		LOG_ERROR("Failed to read DOSR. Not Xtensa OCD?");
		return ERROR_FAIL;
	}

	if(dosr & (DOSR_IN_OCD_MODE)) {
		if(target->state != TARGET_HALTED) {
			if(target->state != TARGET_UNKNOWN && (dosr & DOSR_EXCEPTION) == 0) {
				LOG_WARNING("%s: DOSR has set InOCDMode without the Exception flag. Unexpected. DOSR=%02x",
					    __func__, dosr);
			}
			if(target->state == TARGET_DEBUG_RUNNING) {
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
			} else {
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			}

			xtensa->state = XT_OCD_HALT;
			target->state = TARGET_HALTED;
			xtensa_save_context(target);

			LOG_DEBUG("target->state: %s", target_state_name(target));
			pc = &xtensa->core_cache->reg_list[XT_REG_IDX_PC];
			LOG_INFO("halted: PC: 0x%" PRIx32, buf_get_u32(pc->value, 0, 32));
		}
	} else if(target->state != TARGET_RUNNING && target->state != TARGET_DEBUG_RUNNING){
		xtensa->state = XT_OCD_RUN;
		target->state = TARGET_RUNNING;
	}
	return ERROR_OK;
}

static int xtensa_examine(struct target *target)
{
	int res;

	if (!target_was_examined(target)) {
		target_set_examined(target);

		/* without IDCODE, there isn't actually a lot we can
		   do here apart from try to poll and check that the
		   TAP responds to it, ie has some known Xtensa
		   registers. */
		res = xtensa_poll(target);
		if(res != ERROR_OK) {
			LOG_ERROR("Failed to examine target.");
			return ERROR_FAIL;
		}

		/* TODO: Clear breakpoint state here as much as possible. */
	}

	return ERROR_OK;
}


static int xtensa_halt(struct target *target)
{
	int res;

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	res = xtensa_tap_exec(target, TAP_INS_DEBUG_INT, 0, 0);
	if(res != ERROR_OK) {
		LOG_ERROR("Failed to issue DebugInt instruction. Can't halt.");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int xtensa_resume(struct target *target,
			 int current,
			 uint32_t address,
			 int handle_breakpoints,
			 int debug_execution)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	uint8_t buf[4];
	int res;

	LOG_INFO("%s current=%d address=%04" PRIx32, __func__, current, address);

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	if(address && !current) {
		buf_set_u32(buf, 0, 32, address);
		xtensa_set_core_reg(&xtensa->core_cache->reg_list[XT_REG_IDX_PC], buf);
	}
	xtensa_restore_context(target);
	register_cache_invalidate(xtensa->core_cache);

	res = xtensa_tap_exec(target, TAP_INS_LOAD_DI, XT_INS_RFDO_1, 0);
	if(res != ERROR_OK) {
		LOG_ERROR("Failed to issue LoadDI instruction. Can't resume.");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int xtensa_step(struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	int res;
	uint32_t dosr;
	static const uint32_t icount_val = -2; /* ICOUNT value to load for 1 step */
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("%s: target not halted", __func__);
		return ERROR_TARGET_NOT_HALTED;
	}

	/* working reg a0 */
	xtensa_setup_scratch_reg(target, XT_REG_IDX_A0);

	/* Load debug level into ICOUNTLEVEL */
	res = xtensa_tap_queue_write_sr(target, XT_REG_IDX_ICOUNTLEVEL, XT_DEBUGLEVEL);
	if(res != ERROR_OK)
		return res;

	/* load ICOUNT step count value */
	res = xtensa_tap_queue_write_sr(target, XT_REG_IDX_ICOUNT, icount_val);
	if(res != ERROR_OK)
		return res;

	res = jtag_execute_queue();
	if(res != ERROR_OK)
		return res;

	/* Now ICOUNT is set, we can resume as if we were going to run */
	res = xtensa_resume(target, current, address, 0, 0);
	if(res != ERROR_OK) {
		LOG_ERROR("%s: Failed to resume after setting up single step", __func__);
		return res;
	}

	/* Wait for stepping to complete */
	do {
		res = xtensa_tap_exec(target, TAP_INS_READ_DOSR, 0, &dosr);
		if(res != ERROR_OK) {
			LOG_ERROR("Failed to read DOSR. Not Xtensa OCD?");
			return ERROR_FAIL;
		}
	} while((dosr & (DOSR_IN_OCD_MODE|DOSR_EXCEPTION)) == 0);

	xtensa_save_context(target);
	target->debug_reason = DBG_REASON_SINGLESTEP;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	LOG_INFO("halted: PC: 0x%" PRIx32, buf_get_u32(reg_list[XT_REG_IDX_PC].value, 0, 32));
	return ERROR_OK;
}


static int xtensa_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int xtensa_assert_reset(struct target *target)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_HAS_SRST) {
		/* default to asserting srst */
		if (jtag_reset_config & RESET_SRST_PULLS_TRST)
			jtag_add_reset(1, 1);
		else
			jtag_add_reset(0, 1);
	}

	target->state = TARGET_RESET;
	jtag_add_sleep(5000);

	register_cache_invalidate(xtensa->core_cache);

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int xtensa_deassert_reset(struct target *target)
{
	int res;

	/* deassert reset lines */
	jtag_add_reset(0, 0);

	usleep(100000);
	res = xtensa_poll(target);
	if (res != ERROR_OK)
		return res;

	if (target->reset_halt) {
		/* TODO: work out if possible to halt on reset (I think "no" */
		res = target_halt(target);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: failed to halt afte reset", __func__);
			return res;
		}
		LOG_WARNING("%s: 'reset halt' is not supported for Xtensa. "
			    "Have halted some time after resetting (not the same thing!)", __func__);
	}

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int xtensa_read_memory_inner(struct target *target,
				    uint32_t address,
				    uint32_t size,
				    uint32_t count,
				    uint8_t *buffer)
{
	int res;
	uint32_t inst;
	uint8_t imm8;
	static const uint8_t zeroes[4] = {0};

	/* Load DDR with base address, save to register a0 */
	/* Push base base address to a0 via DDR */
	res = xtensa_tap_queue_load_general_reg(target, 0, address);
	if(res != ERROR_OK)
		return res;

	for(imm8 = 0; imm8 < count; imm8++) {
		/* determine the load instruction (based on size) */
		switch(size) {
		case 4:
			inst = XT_INS_L32I(0, 1, imm8); break;
		case 2:
			inst = XT_INS_L16UI(0, 1, imm8); break;
		case 1:
			inst = XT_INS_L8UI(0, 1, imm8); break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		/* queue the load instruction to the address register */
		res = xtensa_tap_queue_cpu_inst(target, inst);
		if(res != ERROR_OK)
			return res;

		/* queue the load instruction from the address register to DDR */
		res = xtensa_tap_queue_cpu_inst(target, XT_INS_WSR(XT_SR_DDR, 1));
		if(res != ERROR_OK)
			return res;

		/* queue the read of DDR - note specific length to avoid buffer overrun */
		jtag_add_plain_ir_scan(target->tap->ir_length,
				       tap_instr_buf+TAP_INS_SCAN_DDR*4,
				       NULL, TAP_IDLE);
		jtag_add_plain_dr_scan(8*size, zeroes, buffer+imm8*size, TAP_IDLE);
	}
	res = jtag_execute_queue();
	if(res != ERROR_OK) {
		LOG_ERROR("%s: JTAG scan failed", __func__);
		return res;
	}
	return ERROR_OK;
}


static int xtensa_read_memory(struct target *target,
			      uint32_t address,
			      uint32_t size,
			      uint32_t count,
			      uint8_t *buffer)
{
	int res;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* Read & dirty a0 & a1 as we're going to use them as working regs */
	xtensa_setup_scratch_reg(target, XT_REG_IDX_A0);
	xtensa_setup_scratch_reg(target, XT_REG_IDX_A1);

	/* NB: if we were supporting the ICACHE option, we would need
	 * to invalidate it here */

	/* All the LxxI instructions support up to 255 offsets per
	   instruction, so we break each read up into blocks of at
	   most this size.
	*/
	while(count > 255) {
		LOG_DEBUG("%s: splitting read from 0x%" PRIx32 " size %d count 255",__func__,
			  address,size);
		res = xtensa_read_memory_inner(target, address, size, 255, buffer);
		if(res != ERROR_OK) {
			LOG_ERROR("%s: inner read failed at address 0x%" PRIx32, __func__, address);
			return res;
		}
		count -= 255;
		address += (255 * size);
		buffer += (255 * size);
	}
	res = xtensa_read_memory_inner(target, address, size, count, buffer);
	if(res != ERROR_OK) {
		LOG_ERROR("%s: final read failed at address 0x%" PRIx32, __func__, address);
	}

	return res;
}

static int xtensa_write_memory_inner(struct target *target,
				     uint32_t address,
				     uint32_t size,
				     uint32_t count,
				     const uint8_t *buffer)
{
	int res;
	uint32_t inst;
	uint8_t imm8;

	/* Push base address to a0 via DDR */
	res = xtensa_tap_queue_load_general_reg(target, 0, address);
	if(res != ERROR_OK)
		return res;

	for(imm8 = 0; imm8 < count; imm8++) {
		/* load next word from buffer into a1, via DDR */
		res = xtensa_tap_queue_load_general_reg(target, 1,
							buf_get_u32(buffer+imm8*size, 0, 8*size));
		if(res != ERROR_OK)
			return res;

		/* determine the store instruction (based on size) */
		switch(size) {
		case 4:
			inst = XT_INS_S32I(0, 1, imm8); break;
		case 2:
			inst = XT_INS_S16I(0, 1, imm8); break;
		case 1:
			inst = XT_INS_S8I(0, 1, imm8); break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		/* queue the store instruction to the address register */
		res = xtensa_tap_queue_cpu_inst(target, inst);
		if(res != ERROR_OK)
			return res;
	}
	res = jtag_execute_queue();
	if(res != ERROR_OK) {
		LOG_ERROR("%s: JTAG scan failed", __func__);
		return res;
	}

	return ERROR_OK;
}

static int xtensa_write_memory(struct target *target,
			       uint32_t address,
			       uint32_t size,
			       uint32_t count,
			       const uint8_t *buffer)
{
	/* NOTE FOR LATER: This function is almost identical to
	   xtensa_read_memory, and can possibly be converted into a common
	   wrapper function. The only problem is the 'const uint8_t
	   *buffer' rather than the non-const read function version... :(.
	   */
	int res;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* Read & dirty a0 & a1 as we're going to use them as working regs */
	xtensa_setup_scratch_reg(target, XT_REG_IDX_A0);
	xtensa_setup_scratch_reg(target, XT_REG_IDX_A1);

	/* All the LxxI instructions support up to 255 offsets per
	   instruction, so we break each read up into blocks of at
	   most this size.
	*/
	while(count > 255) {
		LOG_DEBUG("%s: splitting read from 0x%" PRIx32 " size %d count 255",__func__,
			  address,size);
		res = xtensa_write_memory_inner(target, address, size, 255, buffer);
		if(res != ERROR_OK) {
			LOG_ERROR("%s: inner write failed at address 0x%" PRIx32, __func__, address);

			return res;
		}
		count -= 255;
		address += (255 * size);
		buffer += (255 * size);
	}
	res = xtensa_write_memory_inner(target, address, size, count, buffer);
	if(res != ERROR_OK) {
		LOG_ERROR("%s: final write failed at address 0x%" PRIx32, __func__, address);
	}

	/* NB: if we were supporting the ICACHE option, we would need
	 * to invalidate it here */

	return res;
}

/* Read register value from target. This function goes via the gdb-facing register cache. */
static int xtensa_read_register(struct target *target, int idx, int force)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	struct reg *reg;
	struct xtensa_core_reg *xt_reg;
	uint32_t reg_value;
	uint8_t read_reg_buf[4];
	int res;

	if(idx < 0 || idx >= XT_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg = &reg_list[idx];
	xt_reg = reg->arch_info;

	LOG_DEBUG("%s %s valid=%d dirty=%d force=%d", __func__, reg->name,
		  reg->valid, reg->dirty, force);

	if((reg->valid && !force) || reg->dirty)
		return ERROR_OK; /* Still OK */

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if(xt_reg->type == XT_REG_GENERAL) {
		/* Access via WSR from general reg Ax to DDR */
		res = xtensa_tap_queue_cpu_inst(target, XT_INS_WSR(XT_SR_DDR,xt_reg->reg_num));
		if(res != ERROR_OK)
			return res;
		xtensa_tap_queue(target, TAP_INS_SCAN_DDR, NULL, read_reg_buf);
		if(res != ERROR_OK)
			return res;
	}
	else {
		/* Otherwise, access is via a special register read via a0 */

		/* Store a0 as being used as scratch reg */
		xtensa_setup_scratch_reg(target, XT_REG_IDX_A0);

		if(xt_reg->type == XT_REG_PC) {
			/* Replace PC register with the EPC reg we use as alias */
			xt_reg = reg_list[XT_REG_IDX_PC_ALIAS].arch_info;
		}

		/* RSR to read special reg to a0, then WSR to DDR, then scan via OCD */
		res = xtensa_tap_queue_cpu_inst(target, XT_INS_RSR(xt_reg->reg_num, 0));
		if(res != ERROR_OK)
			return res;
		res = xtensa_tap_queue_cpu_inst(target, XT_INS_WSR(XT_SR_DDR, 0));
		res = xtensa_tap_queue(target, TAP_INS_SCAN_DDR, NULL, read_reg_buf);
		if(res != ERROR_OK)
			return res;
	}

	res = jtag_execute_queue();
	if(res != ERROR_OK)
		return res;

	reg_value = buf_get_u32(read_reg_buf, 0, 32);
	buf_set_u32(reg->value, 0, 32, reg_value);

	LOG_DEBUG("%s: read %s type %d num %d value 0x%" PRIx32, __func__, xt_reg->name,
		 xt_reg->type, xt_reg->reg_num, reg_value);

	reg->valid = 1;
	reg->dirty = 0;
	return ERROR_OK;
}

static int xtensa_write_register(struct target *target, int idx)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	struct reg *reg;
	struct xtensa_core_reg *xt_reg;
	uint32_t value;
	int res;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if(idx < 0 || idx >= XT_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	reg = &reg_list[idx];
	xt_reg = reg->arch_info;

	LOG_DEBUG("%s %s dirty=%d value=%04"PRIx32, __func__, reg->name,
		  reg->dirty, buf_get_u32(reg->value, 0, 32));

	if(!reg->dirty)
		return ERROR_OK;

	if(xt_reg->type == XT_REG_GENERAL) {
		/* Load new register value to general register Ax via DDR */
		res = xtensa_tap_queue_load_general_reg(target, xt_reg->reg_num,
							buf_get_u32(reg->value, 0, 32));
		if(res != ERROR_OK)
			return res;
	}
	else {
		/* Special register load */
		if(xt_reg->type == XT_REG_PC) {
			/* Replace PC with its EPC alias register (the one we can actually touch) */
			xt_reg = reg_list[XT_REG_IDX_PC_ALIAS].arch_info;
		}
		value = buf_get_u32(reg->value, 0, 32);
		res = xtensa_tap_queue_write_sr(target, xt_reg->reg_num, value);
		if(res != ERROR_OK)
			return res;
	}

	res = jtag_execute_queue();
	if(res != ERROR_OK)
		return res;

	/* If we just wrote to the PC register or the EPC register we
	   use as its alias, make sure to invalidate the other aliased
	   register */
	if(idx == XT_REG_IDX_PC) {
		reg_list[XT_REG_IDX_PC_ALIAS].valid = 0;
		reg_list[XT_REG_IDX_PC_ALIAS].dirty = 0;
	} else if(idx == XT_REG_IDX_PC_ALIAS) {
		reg_list[XT_REG_IDX_PC].valid = 0;
		reg_list[XT_REG_IDX_PC].dirty = 0;
	}

	reg->valid = 1;
	reg->dirty = 0;

	xtensa_read_register(target, idx, 1); /* TODO: this is a debug check */

	return ERROR_OK;
}

static int xtensa_get_core_reg(struct reg *reg)
{
	struct xtensa_core_reg *xt_reg = reg->arch_info;
	return xtensa_read_register(xt_reg->target, xt_reg->idx, 1);
}

static int xtensa_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct xtensa_core_reg *xt_reg = reg->arch_info;
	struct target *target = xt_reg->target;
	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->dirty = 1;
	reg->valid = 1;
	return ERROR_OK;
}

/* Save context from target */
static int xtensa_save_context(struct target *target)
{
	int i;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	for(i = 0; i < XT_NUM_REGS; i++)
		xtensa_read_register(target, i, 1);
	return ERROR_OK;
}

/* Restore context to target */
static int xtensa_restore_context(struct target *target)
{
	int i;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	for(i = 0; i < XT_NUM_REGS; i++) {
		xtensa_write_register(target, i);
	}
	return ERROR_OK;
}


static const struct reg_arch_type xtensa_reg_type = {
	.get = xtensa_get_core_reg,
	.set = xtensa_set_core_reg,
};

static void xtensa_build_reg_cache(struct target *target)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(XT_NUM_REGS, sizeof(struct reg));
	struct xtensa_core_reg *arch_info = malloc(
			sizeof(struct xtensa_core_reg) * XT_NUM_REGS);
	int i;

	cache->name = "Xtensa registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = XT_NUM_REGS;
	(*cache_p)= cache;
	xtensa->core_cache = cache;

	for(i = 0; i < XT_NUM_REGS; i++) {
		arch_info[i] = xt_regs[i];
		arch_info[i].target = target;
		reg_list[i].name = arch_info[i].name;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1,4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &xtensa_reg_type;
		reg_list[i].arch_info = &arch_info[i];
	}
}

static int xtensa_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class)
{
	int i;
	struct xtensa_common *xtensa = target_to_xtensa(target);

	*reg_list_size = XT_NUM_REGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	if (!*reg_list)
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (i = 0; i < XT_NUM_REGS; i++)
		(*reg_list)[i] = &xtensa->core_cache->reg_list[i];

	return ERROR_OK;
}

/** Holds methods for Xtensa targets. */
struct target_type xtensa_target = {
	.name = "xtensa",

	.poll = xtensa_poll,
	.arch_state = xtensa_arch_state,

	.halt = xtensa_halt,
	.resume = xtensa_resume,
	.step = xtensa_step,

	.assert_reset = xtensa_assert_reset,
	.deassert_reset = xtensa_deassert_reset,

	.read_memory = xtensa_read_memory,
	.write_memory = xtensa_write_memory,

	.get_gdb_reg_list = xtensa_get_gdb_reg_list,

	/*
	  .run_algorithm = xtensa_run_algorithm,

	  .add_breakpoint = xtensa_add_breakpoint,
	  .remove_breakpoint = xtensa_remove_breakpoint,
	  .add_watchpoint = xtensa_add_watchpoint,
	  .remove_watchpoint = xtensa_remove_watchpoint,

	  .commands = xtensa_command_handlers,
	*/
	.target_create = xtensa_target_create,
	.init_target = xtensa_init_target,
	.examine = xtensa_examine,
};
