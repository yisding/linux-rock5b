# SPDX-License-Identifier: GPL-2.0
#
# Rockchip MPP service framework - VEPU580 / rkvenc2 (H.264/H.265) encoder.
#
# compat/ holds small inline shims for BSP-only <soc/rockchip/*.h> services
# that are still absent on mainline. The Rockchip IOMMU helpers now live in
# include/soc/rockchip and drivers/iommu/rockchip-iommu.c.
# MPP_VERSION replaces the donor's git-shellout version string.

ccflags-y += -I$(src)/compat
ccflags-y += -DMPP_VERSION=\"6.18-rkvenc-fwport\"

# rkvdec2 link.c calls rockchip_save_qos()/_restore_qos(): the real upstream
# <soc/rockchip/pm_domains.h> lacks them and shadows any compat copy (LINUXINCLUDE
# beats -I$(src)/compat), so force-include the no-op shim instead.
ccflags-y += -include $(src)/compat/rockchip_qos_compat.h

obj-$(CONFIG_ROCKCHIP_MPP_SERVICE) += rk_vcodec.o

rk_vcodec-y := mpp_service.o mpp_common.o mpp_iommu.o
rk_vcodec-$(CONFIG_ROCKCHIP_MPP_RKVENC2) += mpp_rkvenc2.o
rk_vcodec-$(CONFIG_ROCKCHIP_MPP_RKVDEC2) += mpp_rkvdec2.o mpp_rkvdec2_link.o
