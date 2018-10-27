/*
 * Copyright (C) 2014 Fastwel, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _INCLUDE_LINUX_PLATFORM_DATA_BQ27541_H_
#define _INCLUDE_LINUX_PLATFORM_DATA_BQ27541_H_

struct bq27541_platform_data {
	int n_charger_prsnt_gpio;
	int n_battery_low_gpio;
	int charging_led_gpio;
};

#endif /* _INCLUDE_LINUX_PLATFORM_DATA_BQ27541_H_ */
