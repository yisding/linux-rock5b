/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Forward-port compat shim for RK3588 VEPU580 (rkvenc2) on mainline 6.18.
 *
 * The BSP header <soc/rockchip/rockchip_system_monitor.h> does not exist
 * upstream.  Every symbol/type below is referenced ONLY from the DVFS island in
 * mpp_rkvenc2.c (CONFIG_ROCKCHIP_MPP_RKVENC2_DEVFREQ, default n).
 *
 * rockchip_system_monitor_register() returns ERR_PTR(-ENODEV); the driver
 * checks IS_ERR(), logs "without system monitor", sets mdev_info = NULL and
 * continues -- so the encoder runs without SoC-wide thermal/voltage monitoring
 * (see Residual TODOs: mainline thermal-cooling device covers the venc role).
 *
 * struct monitor_dev_profile is filled by the driver (venc_mdevp), so the
 * touched fields must match: type, data, low/high_temp_adjust, check_rate_volt,
 * opp_info.  struct monitor_dev_info is only ever used as an opaque pointer.
 */
#ifndef _MPP_COMPAT_SOC_ROCKCHIP_SYSTEM_MONITOR_H
#define _MPP_COMPAT_SOC_ROCKCHIP_SYSTEM_MONITOR_H

#include <linux/types.h>
#include <linux/err.h>

struct device;
struct rockchip_opp_info;

enum monitor_dev_type {
	MONITOR_TYPE_CPU,
	MONITOR_TYPE_DEV,	/* GPU, NPU, DMC, VENC, and so on */
};

struct monitor_dev_info;

struct monitor_dev_profile {
	enum monitor_dev_type type;
	void *data;
	int (*low_temp_adjust)(struct monitor_dev_info *info, bool is_low);
	int (*high_temp_adjust)(struct monitor_dev_info *info, bool is_high);
	int (*check_rate_volt)(struct monitor_dev_info *info);
	struct rockchip_opp_info *opp_info;
};

static inline struct monitor_dev_info *
rockchip_system_monitor_register(struct device *dev,
				 struct monitor_dev_profile *devp)
{
	return ERR_PTR(-ENODEV);
}

static inline void
rockchip_system_monitor_unregister(struct monitor_dev_info *info)
{
}

static inline int
rockchip_monitor_check_rate_volt(struct monitor_dev_info *info)
{
	return 0;
}

static inline int
rockchip_monitor_dev_low_temp_adjust(struct monitor_dev_info *info, bool is_low)
{
	return 0;
}

static inline int
rockchip_monitor_dev_high_temp_adjust(struct monitor_dev_info *info, bool is_high)
{
	return 0;
}

#endif /* _MPP_COMPAT_SOC_ROCKCHIP_SYSTEM_MONITOR_H */
