/* imu-v3-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * bno055.c: Driver for BNO055
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "bno055.h"

#include "configs/config_bno055.h"

#include "bricklib2/os/coop_task.h"
#include "bricklib2/logging/logging.h"

BNO055 bno055;
CoopTask bno055_task;

void bno055_tick_task(void) {
	while(true) {
		coop_task_yield();
	}
}

void bno055_tick(void) {
	coop_task_tick(&bno055_task);
}

void bno055_init_i2c(void) {

}
void bno055_init(void) {
	memset(&bno055, 0, sizeof(BNO055));

	bno055_init_i2c();
	coop_task_init(&bno055_task, bno055_tick_task);
}