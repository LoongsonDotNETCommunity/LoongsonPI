// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2008 - 2023 Xel Technology. */

#ifndef __XLNID_LODESTAR_H__
#define __XLNID_LODESTAR_H__

extern s32 xlnid_reset_hw_lodestar(struct xlnid_hw *hw);
extern enum xlnid_media_type xlnid_get_media_type_lodestar(struct xlnid_hw *hw);
extern s32 xlnid_get_link_capabilities_lodestar(struct xlnid_hw *hw,
                xlnid_link_speed *speed,
                bool *autoneg);
extern s32 xlnid_check_mac_link_lodestar(struct xlnid_hw *hw, xlnid_link_speed *speed,
               bool *link_up, bool link_up_wait_to_complete);

#endif /* __XLNID_LODESTAR_H__ */


