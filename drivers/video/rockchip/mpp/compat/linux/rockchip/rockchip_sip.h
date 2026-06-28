/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Forward-port compat shim for RK3588 rkvdec2 decoder on mainline 6.18.
 *
 * mpp_rkvdec2.h:23 does `#include <linux/rockchip/rockchip_sip.h>`, a BSP-only
 * header that exists in neither the running-kernel build tree nor the worktree
 * (confirmed: no include/linux/rockchip/ on mainline).  Resolved via
 * `ccflags-y += -I$(src)/compat`.
 *
 * The only symbol the decoder needs from it is sip_smc_vpu_reset(), called in
 * rkvdec2_sip_reset() (mpp_rkvdec2.c:1441) and the link reset paths
 * (mpp_rkvdec2_link.c:1795,:2414).  All three sites ignore the return value;
 * the real ATF SMC reset is unavailable on this firmware, and the live reset
 * path is the CRU-based rkvdec2_reset() fallback.  Signature matches the BSP
 * declaration verbatim:
 *   struct arm_smccc_res sip_smc_vpu_reset(u32 arg0, u32 arg1, u32 arg2);
 *
 * NOTE: the separately-included <soc/rockchip/rockchip_sip.h> (which DOES exist
 * upstream) does not declare this symbol, so there is no conflicting prototype.
 */
#ifndef _MPP_COMPAT_LINUX_ROCKCHIP_SIP_H
#define _MPP_COMPAT_LINUX_ROCKCHIP_SIP_H

#include <linux/arm-smccc.h>

static inline struct arm_smccc_res
sip_smc_vpu_reset(u32 arg0, u32 arg1, u32 arg2)
{
	struct arm_smccc_res res = { 0 };

	return res;
}

#endif /* _MPP_COMPAT_LINUX_ROCKCHIP_SIP_H */
