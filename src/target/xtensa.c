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
 * XTensa arch target implementation. Very preliminary & restricted in scope:
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

#include "xtensa.h"

/* XTensa config options
 *
 * This is stuff which is currently hard-coded but ideally would
 * eventually be per-target configured.
 */
#define XTENSA_INST_NUM_BITS 24


/* XTensa OCD TAP instructions */
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
	{ TAP_INS_LOAD_DI,    0x16, XTENSA_INST_NUM_BITS, "LoadDI"    },
	{ TAP_INS_SCAN_DDR,   0x17, 32,                   "ScanDDR"   },
	{ TAP_INS_READ_DOSR,  0x18, 8,                    "ReadDOSR"  },
	{ TAP_INS_SCAN_DCR,   0x19, 8,                    "ScanDCR"   },
	{ TAP_INS_LOAD_WDI,   0x1a, XTENSA_INST_NUM_BITS, "LoadWDI"   },
};
/* Static buffer that holds the tap instructions ready for sending to JTAG */
static uint8_t tap_instr_buf[XTENSA_NUM_TAP_INS*4];

/* Debug Output Status Register (DOSR) fields */
#define DOSR_NEXT_DI     (1<<0)
#define DOSR_EXCEPTION   (1<<1)
#define DOSR_IN_OCD_MODE (1<<2)
#define DOSR_DOD_READY   (1<<3)

/* Instruction field offsets & masks */
#define XT_MASK_IMM8 0xFF
#define XT_MASK_R 0x0F
#define XT_MASK_S 0x0F
#define XT_MASK_T 0x0F
#define XT_MASK_SR 0xFF
/* RRI8 */
#define XT_OFFS_RRI8_IMM8 16
#define XT_OFFS_RRI8_S 8
#define XT_OFFS_RRI8_T 4
/* RSR */
#define _XT_INS_FORMAT_RSR(OPCODE,SR,T) (OPCODE			\
					 | ((SR & 0xFF) << 8)	\
					 | ((T & 0x0F) << 4))

#define _XT_INS_FORMAT_RRI8(OPCODE,R,S,T,IMM8) (OPCODE			\
						| ((IMM8 & 0xFF) << 16) \
						| ((R & 0x0F) << 12 )	\
						| ((S & 0x0F) << 8 )	\
						| ((T & 0x0F) << 4 ))


/* Special register indexes */
#define XT_SR_DDR  104

/* XTensa processor instruction opcodes
   TODO: Almost certainly host-endianness issues here
*/
/* "Return From Debug Operation" to Normal */
#define XT_INS_RFDO_0      (0xf1e000)
/* "Return From Debug Operation" to OCD Run */
#define XT_INS_RFDO_1      (0xf1e100)

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

/* Rear Special Register */
#define XT_INS_RSR(SR,T) _XT_INS_FORMAT_RSR(0x030000,SR,T)
/* Write Special Register */
#define XT_INS_WSR(SR,T) _XT_INS_FORMAT_RSR(0x130000,SR,T)
/* Swap Special Register */
#define XT_INS_XSR(SR,T) _XT_INS_FORMAT_RSR(0x610000,SR,T)

/* Add an XTensa OCD TAP instruction to the JTAG queue */
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

	LOG_DEBUG("Queueing TAP inst %s has_data_out=%d has_data_in=%d",
		  ins->name, data_out?1:0, data_in?1:0);

	jtag_add_plain_ir_scan(target->tap->ir_length, tap_instr_buf+inst_idx*4,
			       NULL, TAP_IDLE);
	jtag_add_plain_dr_scan(ins->data_len, data_out, data_in, TAP_IDLE);

	return ERROR_OK;
}

/* Execute an XTensa OCD TAP instruction immediately */
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

static int xtensa_target_create(struct target *target, Jim_Interp *interp)
{
	struct xtensa_common *xtensa = calloc(1, sizeof(struct xtensa_common));

	if (!xtensa)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target->arch_info = xtensa;
	xtensa->tap = target->tap;

	return ERROR_OK;
}

static int xtensa_init_target(struct command_context *cmd_ctx, struct target *target)
{
	int i;
	LOG_DEBUG("%s", __func__);
	struct xtensa_common *xtensa = target_to_xtensa(target);

	xtensa->state = XT_NORMAL; // Assume normal state until we examine

	/* TODO: Reset breakpoint state and build reg cache */

	for(i = 0; i < XTENSA_NUM_TAP_INS; i++)
		buf_set_u32(tap_instr_buf+4*i, 0, 32, tap_instrs[i].inst);

	return ERROR_OK;
}

static int xtensa_poll(struct target *target)
{
	struct xtensa_common *xtensa = target_to_xtensa(target);
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
		xtensa->state = XT_OCD_HALT;
		target->state = TARGET_HALTED;
	} else {
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
		   TAP responds to it, ie has some known XTensa
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
	return xtensa_poll(target);
}

static int xtensa_resume(struct target *target,
			 int current,
			 uint32_t address,
			 int handle_breakpoints,
			 int debug_execution)
{
	uint8_t didr[4];
	buf_set_u32(didr, 0, 32, 0x1);
	int res = xtensa_tap_queue(target, TAP_INS_SCAN_DDR, didr, NULL);
	res = xtensa_tap_exec(target, TAP_INS_LOAD_DI, XT_INS_RFDO_1, 0);
	if(res != ERROR_OK) {
		LOG_ERROR("Failed to issue LoadDI instruction. Can't resume.");
		return ERROR_FAIL;
	}
	return xtensa_poll(target);
}

static int xtensa_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

static int xtensa_assert_reset(struct target *target)
{
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

	/* TODO: invalidate register cache, once we have one */

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
		LOG_WARNING("%s: 'reset halt' is not supported for XTensa. "
			    "Have halted some time after resetting (not the same thing!)", __func__);
	}

	LOG_DEBUG("%s", __func__);
	return ERROR_OK;
}

/* Swap the contents of an array of register values with
   the address register contents.

   Useful for saving/restoring state.

   "args" contains "count" values to be loaded into address registers. After succesful execution,
   "args" contains the values that were loaded in these register addresses.
*/
static int xtensa_swap_address_regs(struct target *target, uint32_t *args, uint32_t count)
{
	uint8_t val_buf[4*(count+1)];
	uint8_t inst_buf[4*count];
	uint32_t i;
	int res;

	if(count == 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Load values into buffer */
	for(i = 0; i < count; i++)
		buf_set_u32(val_buf+i*4, 0, 32, args[i]);
	buf_set_u32(val_buf+count*4, 0, 32, 0);

	/* Load DDR with first value from value buffer */
	res = xtensa_tap_queue(target, TAP_INS_SCAN_DDR,
			       val_buf, NULL);
	if(res != ERROR_OK)
		return res;

	for(i = 0; i < count; i++) {
		/* queue XSR swap instruction */
		buf_set_u32(inst_buf+i*4, 0, 32, XT_INS_XSR(XT_SR_DDR, i));
		res = xtensa_tap_queue(target, TAP_INS_LOAD_DI,
				       inst_buf+i*4, 0);
		if(res != ERROR_OK)
			return res;

		/* Read DDR (now with swapped value of address reg), while scanning in next value */
		res = xtensa_tap_queue(target, TAP_INS_SCAN_DDR,
				       val_buf+(i+1)*4, val_buf+i*4);
		if(res != ERROR_OK)
			return res;
	}

	res = jtag_execute_queue();
	if(res != ERROR_OK)
		LOG_ERROR("%s: failed to swap %d register addresses (error on i=%d)", __func__, count, i);

	for(i = 0; i < count; i++) {
		args[i] = buf_get_u32(val_buf+i*4, 0, 32);
	}
	return res;
}

static int xtensa_read_memory_inner(struct target *target,
				    uint32_t address,
				    uint32_t size,
				    uint32_t count,
				    uint8_t *buffer)
{
	int res;
	uint32_t inst;
	uint8_t inst_buf[4];
	uint8_t load_ddr_buf[4];
	uint32_t address_regs[2] = {0};
	uint8_t imm8;
	static const uint8_t zeroes[4] = {0};

	/* Set up a0 with the base address */
	address_regs[0] = address;
	xtensa_swap_address_regs(target, address_regs, 1);

	/* Opcode to copy address register 1 into DDR */
	buf_set_u32(load_ddr_buf, 0, 32, XT_INS_WSR(XT_SR_DDR, 1));

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
		buf_set_u32(inst_buf, 0, 32, inst);

		/* queue the load instruction to the address register */
		res = xtensa_tap_queue(target, TAP_INS_LOAD_DI, inst_buf, NULL);
		if(res != ERROR_OK)
			return res;

		/* queue the load instruction from the address register to DDR */
		res = xtensa_tap_queue(target, TAP_INS_LOAD_DI, load_ddr_buf, NULL);
		if(res != ERROR_OK)
			return res;

		/* queue the read of DDR */
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
	uint32_t address_regs[2] = {0}; /* a0, a1 */

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* Save a0, a1 values to swap back when completed. */
	xtensa_swap_address_regs(target, address_regs, 2);

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
		if(res != ERROR_OK)
			goto cleanup;
		count -= 255;
		address += (255 * size);
		buffer += 255;
	}
	res = xtensa_read_memory_inner(target, address, size, count, buffer);

 cleanup:
	/* Restore a0, a1 */
	xtensa_swap_address_regs(target, address_regs, 2);

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
	uint8_t inst_buf[4];
	uint8_t load_ddr_buf[4];
	uint8_t data_buf[4] = {0};
	uint32_t address_regs[2] = {0};
	uint8_t imm8;

	/* Set up a0 with the base address */
	address_regs[0] = address;
	xtensa_swap_address_regs(target, address_regs, 1);

	/* Opcode to copy address DDR into register 1 */
	buf_set_u32(load_ddr_buf, 0, 32, XT_INS_RSR(XT_SR_DDR, 1));

	for(imm8 = 0; imm8 < count; imm8++) {
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
		buf_set_u32(inst_buf, 0, 32, inst);

		/* queue the data to store to DDR (always write 32 bits to DDR) */
		memcpy(data_buf, buffer+imm8*size, size);
		res = xtensa_tap_queue(target, TAP_INS_SCAN_DDR, data_buf, NULL);
		if(res != ERROR_OK)
			return res;

		/* queue the load instruction DDR to the address register */
		res = xtensa_tap_queue(target, TAP_INS_LOAD_DI, load_ddr_buf, NULL);
		if(res != ERROR_OK)
			return res;

		/* queue the store instruction to the address register */
		res = xtensa_tap_queue(target, TAP_INS_LOAD_DI, inst_buf, NULL);
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
	uint32_t address_regs[2] = {0}; /* a0, a1 */

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;

	/* Save a0, a1 values to swap back when completed. */
	xtensa_swap_address_regs(target, address_regs, 2);

	/* All the LxxI instructions support up to 255 offsets per
	   instruction, so we break each read up into blocks of at
	   most this size.
	*/
	while(count > 255) {
		LOG_DEBUG("%s: splitting read from 0x%" PRIx32 " size %d count 255",__func__,
			  address,size);
		res = xtensa_write_memory_inner(target, address, size, 255, buffer);
		if(res != ERROR_OK)
			goto cleanup;
		count -= 255;
		address += (255 * size);
		buffer += 255;
	}
	res = xtensa_write_memory_inner(target, address, size, count, buffer);

 cleanup:
	/* NB: if we were supporting the ICACHE option, we would need
	 * to invalidate it here */

	/* Restore a0, a1 */
	xtensa_swap_address_regs(target, address_regs, 2);

	return res;
}

/** Holds methods for Xtensa targets. */
struct target_type xtensa_target = {
	.name = "xtensa",

	.poll = xtensa_poll,
	.arch_state = xtensa_arch_state,

	.halt = xtensa_halt,
	.resume = xtensa_resume,

	.assert_reset = xtensa_assert_reset,
	.deassert_reset = xtensa_deassert_reset,

	.read_memory = xtensa_read_memory,
	.write_memory = xtensa_write_memory,

	/*
	  .get_gdb_reg_list = xtensa_get_gdb_reg_list,

	  .step = xtensa_step,

	  .read_buffer = xtensa_read_buffer_default,
	  .write_buffer = xtensa_write_buffer_default,

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
