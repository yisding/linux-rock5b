/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Forward-port compat shim for RK3588 VEPU580 (rkvenc2) on mainline 6.18.
 *
 * The BSP header <soc/rockchip/rockchip_iommu.h> does not exist upstream
 * (confirmed: `git -C linux ls-tree v6.18 include/soc/rockchip/rockchip_iommu.h`
 * is empty).  Resolved via `ccflags-y += -I $(src)/compat` in the mpp Makefile,
 * so the unchanged `#include <soc/rockchip/rockchip_iommu.h>` in mpp_iommu.c /
 * mpp_rkvenc2.c picks up this file.
 *
 * The minimal rkvenc2 module only ever calls:
 *   - rockchip_iommu_mask_irq()  (fault-storm guard in the pagefault handler;
 *                                 mpp_iommu.c, mpp_rkvenc2.c)
 *   - rockchip_iommu_enable()/_disable()  (mpp_iommu_refresh() error-recovery
 *                                          re-attach; retval ignored)
 *
 * IMPORTANT: the BSP header declared these as real externs behind
 * `IS_REACHABLE(CONFIG_ROCKCHIP_IOMMU)` and only provided inline stubs in the
 * #else.  Mainline DOES define CONFIG_ROCKCHIP_IOMMU (drivers/iommu/
 * rockchip-iommu.c), so reusing the BSP header verbatim would leave these
 * symbols as UNRESOLVED externs at modpost/link time.  We therefore provide
 * UNCONDITIONAL inline stubs here.  See "Residual TODOs" for the production
 * (real fault-mask) path.
 */
#ifndef _MPP_COMPAT_SOC_ROCKCHIP_IOMMU_H
#define _MPP_COMPAT_SOC_ROCKCHIP_IOMMU_H

#include <linux/types.h>
#include <linux/errno.h>

struct device;

static inline void rockchip_iommu_mask_irq(struct device *dev) { }
static inline void rockchip_iommu_unmask_irq(struct device *dev) { }
static inline int rockchip_iommu_enable(struct device *dev) { return -ENODEV; }
static inline int rockchip_iommu_disable(struct device *dev) { return -ENODEV; }
static inline bool rockchip_iommu_is_enabled(struct device *dev) { return false; }
static inline int rockchip_iommu_force_reset(struct device *dev) { return -ENODEV; }
static inline int rockchip_pagefault_done(struct device *master_dev) { return 0; }

#endif /* _MPP_COMPAT_SOC_ROCKCHIP_IOMMU_H */
