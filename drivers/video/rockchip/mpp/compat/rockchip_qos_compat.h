/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Forward-port compat shim for RK3588 rkvdec2 decoder on mainline 6.18.
 *
 * mpp_rkvdec2_link.c:648/:653 call rockchip_save_qos()/rockchip_restore_qos()
 * unconditionally (rkvdec2_link_reset(), an always-compiled, referenced path).
 * The BSP declared these in <soc/rockchip/pm_domains.h>, but that header DOES
 * exist upstream and only declares rockchip_pmu_block()/_unblock() -- the QoS
 * helpers are absent.  Because the real pm_domains.h lives in $(srctree)/include
 * and LINUXINCLUDE is searched BEFORE `-I$(src)/compat`, a compat copy of
 * pm_domains.h would be silently shadowed.  This file is therefore pulled in via
 * an explicit `ccflags-y += -include $(src)/compat/rockchip_qos_compat.h` in the
 * mpp Makefile, guaranteeing the symbols resolve regardless of header search
 * order.
 *
 * No-op stubs: QoS register save/restore around a reset keeps the hardware's
 * power-on defaults across the CRU reset, which is correct for the SIP-off live
 * path.  Return 0 (success) -- matches the BSP's !CONFIG_ROCKCHIP_PM_DOMAINS
 * inline behaviour.
 */
#ifndef _MPP_COMPAT_ROCKCHIP_QOS_COMPAT_H
#define _MPP_COMPAT_ROCKCHIP_QOS_COMPAT_H

#include <linux/device.h>

static inline int rockchip_save_qos(struct device *dev) { return 0; }
static inline int rockchip_restore_qos(struct device *dev) { return 0; }

#endif /* _MPP_COMPAT_ROCKCHIP_QOS_COMPAT_H */
