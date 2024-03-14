/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-12-17     Rbb666        first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "hal_data.h"

#define LED_PIN0    BSP_IO_PORT_01_PIN_02 /* Onboard LED pins */
#define LED_PIN1    BSP_IO_PORT_10_PIN_03 /* Onboard LED pins */
void hal_entry(void)
{
    while (1)
    {
        rt_pin_write(LED_PIN0, PIN_LOW);
		rt_pin_write(LED_PIN1, PIN_LOW);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN0, PIN_HIGH);
		rt_pin_write(LED_PIN1, PIN_HIGH);
        rt_thread_mdelay(500);
	}

}



