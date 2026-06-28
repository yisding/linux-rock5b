/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Forward-port compat shim for RK3588 VEPU580 (rkvenc2) on mainline 6.18.
 *
 * Unlike the other BSP soc/rockchip headers, <soc/rockchip/pm_domains.h> DOES
 * exist upstream -- but it only declares rockchip_pmu_block()/_unblock().  The
 * BSP-only rockchip_pmu_idle_request() is absent (confirmed: `git -C linux grep
 * rockchip_pmu_idle_request v6.18` is empty).
 *
 * Because the real pm_domains.h lives in $(srctree)/include and LINUXINCLUDE is
 * searched BEFORE ccflags `-I $(src)/compat`, we cannot shadow it via the -I
 * trick; this stub is pulled in by an explicit `#include "compat/..."` placed
 * right after the pm_domains.h include in mpp_common.h.
 *
 * The only caller is the mpp_pmu_idle_request() inline (mpp_common.h), which
 * short-circuits and returns 0 when mpp->skip_idle is set.  The rkvenc2 DT nodes
 * carry "rockchip,skip-pmu-idle-request", so skip_idle==1 and this stub is
 * effectively never reached on RK3588.  Return success regardless.
 */
#ifndef _MPP_COMPAT_ROCKCHIP_PMU_IDLE_H
#define _MPP_COMPAT_ROCKCHIP_PMU_IDLE_H

#include <linux/types.h>

struct device;

static inline int rockchip_pmu_idle_request(struct device *dev, bool idle)
{
	return 0;
}

#endif /* _MPP_COMPAT_ROCKCHIP_PMU_IDLE_H */
