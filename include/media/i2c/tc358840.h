/* SPDX-License-Identifier: GPL-2.0 */
/*
 * tc358840.h - Toshiba UH2C/D HDMI-CSI bridge driver
 *
 * Copyright (c) 2015, Armin Weiss <weii@zhaw.ch>
 * Copyright 2015-2020 Cisco Systems, Inc. and/or its affiliates. All rights reserved.
 */

#ifndef _TC358840_
#define _TC358840_

enum tc358840_csi_port {
	CSI_TX_NONE = 0,
	CSI_TX_0,
	CSI_TX_1,
	CSI_TX_BOTH
};

enum tc358840_clock_mode {
	CSI_NON_CONT_CLK = 0,
	CSI_CONT_CLK
};

enum tc358840_ddc5v_delays {
	DDC5V_DELAY_0MS,
	DDC5V_DELAY_50MS,
	DDC5V_DELAY_100MS,
	DDC5V_DELAY_200MS,
	DDC5V_DELAY_MAX = DDC5V_DELAY_200MS,
};

/* custom controls */
#define V4L2_CID_USER_TC358840_BASE (V4L2_CID_BASE + 0x1000)

/* Audio sample rate in Hz */
#define TC358840_CID_AUDIO_SAMPLING_RATE (V4L2_CID_USER_TC358840_BASE + 1)
/* Audio present status */
#define TC358840_CID_AUDIO_PRESENT       (V4L2_CID_USER_TC358840_BASE + 2)
/* Splitter width */
#define TC358840_CID_SPLITTER_WIDTH      (V4L2_CID_USER_TC358840_BASE + 3)
/* TMDS present status */
#define TC358840_CID_TMDS_PRESENT        (V4L2_CID_USER_TC358840_BASE + 4)
/* EQ_BYPS mode */
#define TC358840_CID_EQ_BYPS_MODE        (V4L2_CID_USER_TC358840_BASE + 5)
#define TC358840_CID_EQ_BYPS_AUTO 0
#define TC358840_CID_EQ_BYPS_CLR  1
#define TC358840_CID_EQ_BYPS_SET  2

#endif /* _TC358840_ */
