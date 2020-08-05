/* imu-v3-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * config_bno055.h: Config for BNO055 driver
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

#ifndef CONFIG_BNO055_H
#define CONFIG_BNO055_H

#include "xmc_gpio.h"
#include "xmc_i2c.h"

#define BNO055_I2C_BAUDRATE         400000

#define BNO055_I2C_ADDRESS          0b0101001
#define BNO055_I2C                  XMC_I2C0_CH1

#define BNO055_SCL_PORT             XMC_GPIO_PORT1
#define BNO055_SCL_PIN              3
#define BNO055_SCL_PIN_MODE         XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT6
#define BNO055_SCL_INPUT            XMC_USIC_CH_INPUT_DX1
#define BNO055_SCL_SOURCE           0
#define BNO055_SCL_FIFO_SIZE        XMC_USIC_CH_FIFO_SIZE_16WORDS
#define BNO055_SCL_FIFO_POINTER     32

#define BNO055_SDA_PORT             XMC_GPIO_PORT1
#define BNO055_SDA_PIN              2
#define BNO055_SDA_PIN_MODE         XMC_GPIO_MODE_OUTPUT_OPEN_DRAIN_ALT7
#define BNO055_SDA_INPUT            XMC_USIC_CH_INPUT_DX0
#define BNO055_SDA_SOURCE           1
#define BNO055_SDA_FIFO_SIZE        XMC_USIC_CH_FIFO_SIZE_16WORDS
#define BNO055_SDA_FIFO_POINTER     48

#define BNO055_ADDRESS_SELECTION    P0_7
#define BNO055_INT                  P0_8
#define BNO055_NRESET               P0_9
#define BNO055_NBOOT                P0_12

#endif