/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Forward-port compat shim for RK3588 VEPU580 (rkvenc2) on mainline 6.18.
 *
 * The BSP header <soc/rockchip/rockchip_ipa.h> does not exist upstream.
 * mpp_rkvenc2.c #includes it (line ~31) but references NO rockchip_ipa_*
 * symbol whatsoever -- it is a dead include.  This empty stub exists only so
 * the #include resolves; the include line may be deleted outright (W6).
 *
 * Minimal struct + no-op stubs are provided in case a future refactor pulls
 * the IPA static-power model back in; they degrade to "no static power model".
 */
#ifndef _MPP_COMPAT_SOC_ROCKCHIP_IPA_H
#define _MPP_COMPAT_SOC_ROCKCHIP_IPA_H

#include <linux/types.h>

struct device;

struct ipa_power_model_data {
	u32 static_coefficient;
	s32 ts[4];
	struct thermal_zone_device *tz;
	bool valid;
};

static inline struct ipa_power_model_data *
rockchip_ipa_power_model_init(struct device *dev, char *lkg_name)
{
	return NULL;
}

static inline unsigned long
rockchip_ipa_get_static_power(struct ipa_power_model_data *model_data,
			      unsigned long voltage)
{
	return 0;
}

#endif /* _MPP_COMPAT_SOC_ROCKCHIP_IPA_H */
