/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2021-01-04     iysheng           first version
 */

#include <board.h>
#include <drivers/adc.h>
#include <rtdbg.h>
#include <stdio.h>
#include <rtthread.h>
#include "board.h"

#define LED_G  GET_PIN(E, 15)

int main(void)
{
    rt_pin_mode(LED_G, PIN_MODE_OUTPUT);

    while (1)
    {
        rt_pin_write(LED_G, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_G, PIN_LOW);
        rt_thread_mdelay(500);
    }

    return 0;
}
