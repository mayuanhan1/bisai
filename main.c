/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-29     Rbb666       first version
 */
/*
#include <rtthread.h>
#include <rtdevice.h>

#include "drv_gpio.h"

#define LED_PIN GET_PIN(0, 1)

int main(void)
{
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);

    for (;;)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
*/

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_gpio.h"
#define LED_PIN GET_PIN(0, 1)  // 定义LED引脚
#define BREATHE_CYCLE 20       // 呼吸周期(ms)
#define STEP_SIZE 1            // 亮度变化步长(1-100)

void breath_led_entry(void *parameter)
{
    rt_uint8_t brightness = 0;  // 当前亮度值(0-100)
    rt_int8_t direction = 1;    // 亮度变化方向(1增加/-1减少)

    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);  // 初始化GPIO为输出模式

    while (1) {
        // 计算当前占空比时间
        rt_uint32_t on_time = brightness * BREATHE_CYCLE / 100;  // 高电平时间
        rt_uint32_t off_time = BREATHE_CYCLE - on_time;          // 低电平时间

        // 输出当前PWM波形
        if(on_time > 0) {
            rt_pin_write(LED_PIN, PIN_HIGH);
            rt_thread_mdelay(on_time);  // 保持高电平
        }
        if(off_time > 0) {
            rt_pin_write(LED_PIN, PIN_LOW);
            rt_thread_mdelay(off_time); // 保持低电平
        }
        // 更新亮度值(渐变效果)
        brightness += direction;
        if(brightness >= 100 || brightness <= 0) {
            direction = -direction;  // 到达边界时反转变化方向
        }
    }
}
int main(void)
{
    // 创建呼吸灯线程
    rt_thread_t tid = rt_thread_create("breath",
                                     breath_led_entry,
                                     RT_NULL,
                                     512,
                                     RT_THREAD_PRIORITY_MAX/2,
                                     20);
    if(tid) rt_thread_startup(tid);

    return 0;
}


