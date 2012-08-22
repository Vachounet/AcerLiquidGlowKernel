/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MIPI_CHIMEI_H
#define MIPI_CHIMEI_H

#define CHIMEI_WVGA_TWO_LANE

int mipi_chimei_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel);

#ifdef CONFIG_FB_MSM_BACKLIGHT_HX8363A

#define MIPI_CHIMEI_PWM_LEVEL 255 
#else
#define MIPI_CHIMEI_PWM_FREQ_HZ 5000  
#define MIPI_CHIMEI_PWM_PERIOD_USEC (USEC_PER_SEC / MIPI_CHIMEI_PWM_FREQ_HZ)
#define MIPI_CHIMEI_PWM_LEVEL 100
#define MIPI_CHIMEI_PWM_DUTY_LEVEL \
	(MIPI_CHIMEI_PWM_PERIOD_USEC / MIPI_CHIMEI_PWM_LEVEL)
#define  ANDROID_MAX_BACKLIGHT_BRIGHTNESS 255
#endif

#endif  
