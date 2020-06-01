// SPDX-License-Identifier: GPL-2.0-only

#ifndef _CDNS_MHDP_H_
#define _CDNS_MHDP_H_

#include <drm/bridge/cdns-mhdp.h>

/* mhdp common */
u32 cdns_mhdp_bus_read(struct cdns_mhdp_device *mhdp, u32 offset);
void cdns_mhdp_bus_write(u32 val, struct cdns_mhdp_device *mhdp, u32 offset);
int cdns_mhdp_mailbox_send(struct cdns_mhdp_device *mhdp, u8 module_id,
				  u8 opcode, u16 size, u8 *message);
int cdns_mhdp_reg_read(struct cdns_mhdp_device *mhdp, u32 addr);
int cdns_mhdp_mailbox_read_receive(struct cdns_mhdp_device *mhdp,
					  u8 *buff, u16 buff_size);
int cdns_mhdp_mailbox_validate_receive(struct cdns_mhdp_device *mhdp,
					      u8 module_id, u8 opcode, u16 req_size);
int cdns_mhdp_reg_read(struct cdns_mhdp_device *mhdp, u32 addr);
int cdns_mhdp_reg_write(struct cdns_mhdp_device *mhdp, u32 addr, u32 val);
int cdns_mhdp_reg_write_bit(struct cdns_mhdp_device *mhdp, u16 addr,
				   u8 start_bit, u8 bits_no, u32 val);

/* Audio */
int cdns_mhdp_register_audio_driver(struct device *dev);
void cdns_mhdp_unregister_audio_driver(struct device *dev);

/* HDMI */
int cdns_hdmi_scdc_write(struct cdns_mhdp_device *mhdp, u8 addr, u8 value);
void cdns_mhdp_infoframe_set(struct cdns_mhdp_device *mhdp,
					u8 entry_id, u8 packet_len, u8 *packet, u8 packet_type);
int cdns_hdmi_ctrl_init(struct cdns_mhdp_device *mhdp,
					int protocol, u32 char_rate);
int cdns_hdmi_enable_gcp(struct cdns_mhdp_device *mhdp);
int cdns_hdmi_disable_gcp(struct cdns_mhdp_device *mhdp);
int cdns_hdmi_mode_config(struct cdns_mhdp_device *mhdp,
			struct drm_display_mode *mode, struct video_info *video_info);
int cdns_hdmi_get_edid_block(void *data, u8 *edid,
			  u32 block, size_t length);

#endif
