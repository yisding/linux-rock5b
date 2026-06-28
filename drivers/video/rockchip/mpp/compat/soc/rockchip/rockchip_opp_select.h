/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Forward-port compat shim for RK3588 VEPU580 (rkvenc2) on mainline 6.18.
 *
 * The BSP header <soc/rockchip/rockchip_opp_select.h> does not exist upstream.
 * Every symbol/type below is referenced ONLY from the DVFS island in
 * mpp_rkvenc2.c, which the forward-port gates behind
 * CONFIG_ROCKCHIP_MPP_RKVENC2_DEVFREQ (default n, W6).  When DVFS is off these
 * inlines are unreferenced and elided; the #include still resolves.  When DVFS
 * is on, the stubs make the encoder fall back to fixed assigned-clock-rates
 * with NO OPP voltage/leakage management (see Residual TODOs / W15).
 *
 * struct rockchip_opp_info is embedded BY VALUE in struct rkvenc_dev
 * (mpp_rkvenc2.c:342), so a COMPLETE type is required.  Only these members are
 * touched by the driver -- keep them, drop the rest of the (large) BSP struct:
 *   grf, volt_rm_tbl, current_rm, is_rate_volt_checked, is_runtime_active.
 */
#ifndef _MPP_COMPAT_SOC_ROCKCHIP_OPP_SELECT_H
#define _MPP_COMPAT_SOC_ROCKCHIP_OPP_SELECT_H

#include <linux/types.h>
#include <linux/errno.h>

struct device;
struct of_device_id;
struct regmap;
struct volt_rm_table;

#define VOLT_RM_TABLE_END	(~1)

struct rockchip_opp_info {
	struct regmap *grf;
	struct volt_rm_table *volt_rm_tbl;
	u32 current_rm;
	bool is_rate_volt_checked;
	bool is_runtime_active;
};

struct rockchip_opp_data {
	int (*set_read_margin)(struct device *dev,
			       struct rockchip_opp_info *info,
			       u32 rm);
};

static inline void rockchip_get_opp_data(const struct of_device_id *matches,
					 struct rockchip_opp_info *info)
{
}

static inline void rockchip_opp_dvfs_lock(struct rockchip_opp_info *info) { }
static inline void rockchip_opp_dvfs_unlock(struct rockchip_opp_info *info) { }

static inline int rockchip_init_opp_table(struct device *dev,
					  struct rockchip_opp_info *info,
					  char *clk_name, char *reg_name)
{
	/* Non-zero -> rkvenc_devfreq_init() bails out, devfreq stays disabled. */
	return -EOPNOTSUPP;
}

static inline void rockchip_uninit_opp_table(struct device *dev,
					     struct rockchip_opp_info *info)
{
}

#endif /* _MPP_COMPAT_SOC_ROCKCHIP_OPP_SELECT_H */
