/* Copyright (c) 2008-2011, Hisilicon Tech. Co., Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above
 *	   copyright notice, this list of conditions and the following
 *	   disclaimer in the documentation and/or other materials provided
 *	   with the distribution.
 *	 * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *	   contributors may be used to endorse or promote products derived
 *	   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __ADE_CMDQUEUE_H__
#define __ADE_CMDQUEUE_H__

#define ADE_CMD_WITE_REG_MAX            (128) 
#define ADE_CMD_ALIGN_BYTE_LEN           (64)   /* 16 * 4, 16 words */
/*************************enum***************************/
enum {
	ADE_CMD_TYPE_NULL  = 0x0,
	ADE_CMD_TYPE_WRITE = 0x1,
	ADE_CMD_TYPE_READ  = 0x2,
	ADE_CMD_TYPE_WAIT  = 0x3,
	ADE_CMD_TYPE_EOF   = 0x4,
	ADE_CMD_TYPE_INVAL
}; 

enum ADE_ROT{
	ADE_ROT_NOP = 0,
	ADE_ROT_90,
	ADE_ROT_180,
	ADE_ROT_270,
    ADE_ROT_H_MIRROR,
    ADE_ROT_V_MIRROR,
    ADE_ROT_90_H_MIRROR,
    ADE_ROT_90_V_MIRROR,
    ADE_ROT_INVALID
};
enum OVERLAY_PIPE_NUM{
	OVERLAY_PIPE_ADE_CH1 = 0,
	OVERLAY_PIPE_ADE_CH2,
	OVERLAY_PIPE_ADE_CH3,
	OVERLAY_PIPE_ADE_CH4,
	OVERLAY_PIPE_ADE_CH5,
	OVERLAY_PIPE_ADE_CH6,
	OVERLAY_PIPE_ADE_DISP,

	OVERLAY_PIPE_ADE_MAX
};

enum OVERLAY_PIPE_TYPE {
    OVERLAY_PIPE_TYPE_ONLINE = 0,
    OVERLAY_PIPE_TYPE_OFFLINE,
    OVERLAY_PIPE_TYPE_COPYBIT,
    OVERLAY_PIPE_TYPE_DISPLAY,
    OVERLAY_PIPE_TYPE_INVAL
};

typedef union {
	struct {
		u32     first_reg_offset : 16;
		u32     arg_len  : 8;
		u32     cmd_type : 8;
		}bits;
	u32     ul32;
}WRITE_CMD_HEAD;

typedef union {
    struct {
        u32     reserved : 24;
        u32     cmd_type : 8;
    }bits;
    u32     ul32;
}NULL_EOF_CMD;

struct write_cmd_head {
	WRITE_CMD_HEAD   cmd_head;
	u32              reg_val[ADE_CMD_WITE_REG_MAX];
	}; 

extern void ade_cmdq_wr_rdma_pe_cmd(u32 reg_addr, u32 ch_type, u32 rotation);
extern void ade_cmdq_wr_cmd(u32 reg_addr, u32 val);
extern void ade_cmdq_wr_cmd2buff(void *cmdbuff_vaddr, u32 *cmd_len);
extern void ade_cmdq_wr_eof_cmd(void *cmdbuff_vaddr, u32 *cmd_len); 
#endif  /*  ADE_CMDQUEUE_H  */
