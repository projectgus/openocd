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
 * - Assumes "DIR Array option" not configured.
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

/* Debug Output Status Register (DOSR) fields */
#define DOSR_NEXT_DI     (1<<0)
#define DOSR_EXCEPTION   (1<<1)
#define DOSR_IN_OCD_MODE (1<<2)
#define DOSR_DOD_READY   (1<<3)

/* XTensa processor instruction opcodes
   TODO: Almost certainly host-endianness issues here
*/
#define XT_INS_RFDO_0      (0xf1e000)    /* "Return From Debug Operation" to Normal */
#define XT_INS_RFDO_1      (0xf1e100)    /* "Return From Debug Operation" to OCD Run */

/* Add an XTensa OCD TAP instruction to the JTAG queue */
static int xtensa_tap_queue(struct target *target, int inst_idx, const uint8_t *data_out, uint8_t *data_in)
{
	uint8_t ins_resp;
	const struct xtensa_tap_instr *ins;

	if ((inst_idx < 0 || inst_idx >= XTENSA_NUM_TAP_INS))
		return ERROR_COMMAND_SYNTAX_ERROR;
	ins = &tap_instrs[inst_idx];

	if(ins->data_len > 0 && !data_out)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!target->tap)
		return ERROR_FAIL;

	LOG_DEBUG("Queueing TAP inst %s has_data_out=%d has_data_in=%d",
		  ins->name, data_out?1:0, data_in?1:0);

	jtag_add_plain_ir_scan(target->tap->ir_length, &ins->inst, &ins_resp, TAP_IDLE);
	if(data_out) {
		jtag_add_plain_dr_scan(ins->data_len, data_out, data_in, TAP_IDLE);
	}

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
	LOG_DEBUG("%s", __func__);
	struct xtensa_common *xtensa = target_to_xtensa(target);

	xtensa->state = XT_NORMAL; // Assume normal state until we examine

	/* TODO: Reset breakpoint state and build reg cache */

	return ERROR_OK;
}

static int xtensa_examine(struct target *target)
{
	int res;
	struct xtensa_common *xtensa = target_to_xtensa(target);

	if (!target_was_examined(target)) {
		target_set_examined(target);

		/* without IDCODE, the best we can do to check this is a
		   valid XTensa core is to try and transition
		   from "Normal" mode to OCDRunMode... */
		res = xtensa_tap_exec(target, TAP_INS_ENABLE_OCD, 0, NULL);
		if(res != ERROR_OK) {
			LOG_ERROR("Failed to execute EnableOCD instruction. Not XTensa OCD?");
			return ERROR_FAIL;
		}

		uint32_t dosr;
		res = xtensa_tap_exec(target, TAP_INS_READ_DOSR, 0, &dosr);
		if(res != ERROR_OK) {
			LOG_ERROR("Failed to read DOSR. Not Xtensa OCD?");
			return ERROR_FAIL;
		}

		if(dosr & (DOSR_NEXT_DI|DOSR_IN_OCD_MODE)) {
			xtensa->state = XT_OCD_HALT;
		} else {
			xtensa->state = XT_OCD_RUN;
		}

		/* TODO: Clear breakpoint state here as much as possible. */
	}

	return ERROR_OK;
}


static int xtensa_poll(struct target *target)
{
    struct xtensa_common *xtensa = target_to_xtensa(target);
    uint32_t dosr;
    int res;

    usleep(500*1000); /* TODO: Remove this after confident polling is reliable!! */
    
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


/** Holds methods for Xtensa targets. */
struct target_type xtensa_target = {
	.name = "xtensa",

	.poll = xtensa_poll,
	.arch_state = xtensa_arch_state,

	.halt = xtensa_halt,
	.resume = xtensa_resume,

	/*
	.get_gdb_reg_list = xtensa_get_gdb_reg_list,

	.step = xtensa_step,

	.assert_reset = xtensa_assert_reset,
	.deassert_reset = xtensa_deassert_reset,

	.read_memory = xtensa_read_memory_default,
	.write_memory = xtensa_write_memory_default,

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
