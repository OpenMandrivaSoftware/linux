# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_DRM_CDNS_MHDP8546) += cdns-mhdp8546.o
cdns-mhdp8546-y := cdns-mhdp8546-core.o
cdns-mhdp8546-$(CONFIG_DRM_CDNS_MHDP8546_J721E) += cdns-mhdp8546-j721e.o
obj-$(CONFIG_DRM_CDNS_MHDP)		+= cdns_mhdp_drmcore.o
cdns_mhdp_drmcore-y := cdns-mhdp-common.o cdns-mhdp-audio.o cdns-mhdp-dp.o cdns-mhdp-hdmi.o
cdns_mhdp_drmcore-$(CONFIG_DRM_CDNS_DP) += cdns-dp-core.o
cdns_mhdp_drmcore-$(CONFIG_DRM_CDNS_HDMI) += cdns-hdmi-core.o
