/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Forward-port compat shim for RK3588 rkvdec2 decoder on mainline 6.18.
 *
 * The BSP header <soc/rockchip/rockchip_dmc.h> (DMC = dynamic memory
 * controller / DDR devfreq) does not exist upstream.  Resolved via
 * `ccflags-y += -I$(src)/compat`, so the unchanged
 * `#include <soc/rockchip/rockchip_dmc.h>` in mpp_rkvdec2.c picks up this file.
 *
 * The decoder calls only rockchip_dmcfreq_lock()/_unlock() (mpp_rkvdec2.c:1440
 * /:1442), inside rkvdec2_sip_reset()'s DMC-quiesce window.  On mainline there
 * is no DDR devfreq governor to coordinate with, so these are no-ops.  The BSP
 * header itself provided identical no-op inlines in its !CONFIG_ROCKCHIP_DMC
 * #else branch, so this matches vendor behaviour when DMC is absent.
 */
#ifndef _MPP_COMPAT_SOC_ROCKCHIP_DMC_H
#define _MPP_COMPAT_SOC_ROCKCHIP_DMC_H

static inline void rockchip_dmcfreq_lock(void) { }
static inline void rockchip_dmcfreq_lock_nested(void) { }
static inline void rockchip_dmcfreq_unlock(void) { }

#endif /* _MPP_COMPAT_SOC_ROCKCHIP_DMC_H */
