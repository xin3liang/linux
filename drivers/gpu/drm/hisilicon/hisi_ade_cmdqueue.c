/* Copyright (c) 2008-2010, Hisilicon Tech. Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/bitops.h>
#include "hisi_ade_cmdqueue.h"
#include <linux/fs.h>

#define  ADE_INVAL_REG      (0xffff)

struct write_cmd_head  wr_cmd;

/* BOARD TYPE */
enum {
    ASIC_BOARD = 0,
    SFT_BOARD  = 1,
};

/********** ADE Register Union Struct ***********/
// Define the union U_RD_CH1_PE
union U_RD_CH1_PE {
    // Define the struct bits
    struct
    {
        unsigned int    rd_ch1_qos            : 4   ; // [3..0] 
        unsigned int    rd_ch1_qos_sec        : 4   ; // [7..4] 
        unsigned int    rd_ch1_qos_thd        : 4   ; // [11..8] 
        unsigned int    rd_ch1_qos_cfg        : 1   ; // [12] 
        unsigned int    Reserved_77           : 11  ; // [23..13] 
        unsigned int    rd_ch1_min_burst_len  : 4   ; // [27..24] 
        unsigned int    Reserved_76           : 4   ; // [31..28] 
    } bits;

    // Define an unsigned member
    unsigned int    u32;

} ;

void ade_cmdq_wr_eof_cmd(void *cmdbuff_vaddr, u32 *cmd_len)
{
    NULL_EOF_CMD  eof_cmd;
    NULL_EOF_CMD  null_cmd;
    u32 align_count = 0;
    unsigned   i = 0;

    eof_cmd.ul32 = 0;
    eof_cmd.bits.cmd_type = ADE_CMD_TYPE_EOF;

    null_cmd.ul32 = 0;
    null_cmd.bits.cmd_type = ADE_CMD_TYPE_NULL;

    align_count = (*cmd_len + sizeof(eof_cmd.ul32)) % ADE_CMD_ALIGN_BYTE_LEN ;
    if (align_count != 0) {
        /* align up to 16*4 byte */
        align_count = (((*cmd_len / ADE_CMD_ALIGN_BYTE_LEN + 1) * ADE_CMD_ALIGN_BYTE_LEN) - sizeof(eof_cmd.ul32) - *cmd_len) / 4;
        for (i = 0; i < align_count; i++) {
            memcpy((char*)cmdbuff_vaddr + *cmd_len, &null_cmd, sizeof(null_cmd.ul32));
            *cmd_len += sizeof(null_cmd.ul32);
        }
    }

    memcpy((char*)cmdbuff_vaddr + *cmd_len, &eof_cmd.ul32, sizeof(eof_cmd.ul32));
    *cmd_len += sizeof(eof_cmd.ul32);

}

void ade_cmdq_wr_cmd2buff(void* cmdbuff_vaddr, u32 *cmd_len)
{
    u32   wr_cmd_len = 0;

    if (wr_cmd.cmd_head.bits.first_reg_offset == ADE_INVAL_REG) {
        return;
    }

    wr_cmd_len = sizeof(wr_cmd.cmd_head.ul32) * (wr_cmd.cmd_head.bits.arg_len + 1);
    wr_cmd.cmd_head.bits.arg_len -= 1;

    memcpy((char*)cmdbuff_vaddr + *cmd_len, &wr_cmd, wr_cmd_len);
    *cmd_len += wr_cmd_len;

    wr_cmd.cmd_head.bits.cmd_type = ADE_CMD_TYPE_WRITE;
    wr_cmd.cmd_head.bits.arg_len  = 0;
    wr_cmd.cmd_head.bits.first_reg_offset = ADE_INVAL_REG;
}

/************************* RDMA *********************************/
inline void ade_cmdq_wr_cmd(u32 reg_addr, u32 val)
{
    u32 last_reg;

    if (reg_addr == ADE_INVAL_REG) {
        return;
    }

    last_reg = wr_cmd.cmd_head.bits.arg_len * 4 + wr_cmd.cmd_head.bits.first_reg_offset;

    wr_cmd.cmd_head.bits.cmd_type = ADE_CMD_TYPE_WRITE;
    if ((last_reg == reg_addr) && (wr_cmd.cmd_head.bits.arg_len < ADE_CMD_WITE_REG_MAX)) {
        wr_cmd.reg_val[wr_cmd.cmd_head.bits.arg_len] = val;
        wr_cmd.cmd_head.bits.arg_len++;
    } else {
        wr_cmd.cmd_head.bits.first_reg_offset = reg_addr;
        wr_cmd.cmd_head.bits.arg_len = 1;
        wr_cmd.reg_val[0] = val;
    }
}

void ade_cmdq_wr_rdma_pe_cmd(u32 reg_addr, u32 ch_type, u32 rotation)
{
    volatile union U_RD_CH1_PE rdma_pe;

    rdma_pe.u32 = 0;

    /* the min burst len is 16 when no rotation, is 4 when have online rotation */
    rdma_pe.bits.rd_ch1_min_burst_len = 0xf;
    if (OVERLAY_PIPE_TYPE_ONLINE == ch_type) {
	    rdma_pe.bits.rd_ch1_qos = 4;
	    if (rotation != ADE_ROT_NOP) {
            rdma_pe.bits.rd_ch1_min_burst_len = 0x3;
	    }
    } else {
	    rdma_pe.bits.rd_ch1_qos = 2;
    }
#if 0
    if (SFT_BOARD == fb_get_board_type()) {
        rdma_pe.bits.rd_ch1_min_burst_len = 0x3;
    }
#endif
    ade_cmdq_wr_cmd(reg_addr, rdma_pe.u32);

}
