/*********************************************************************************************************************
 * TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC264 开源库的一部分
 *
 * TC264 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          gpio
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.8.0
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2023-02-01       pudding             first version
 ********************************************************************************************************************/

#include "zf_driver_pwm.h"
#include "zf_driver_delay.h"

#include "gpio.h"

uint8 key_status = 1;  // 当前按键状态
uint8 key_last_status; // 上一次按键状态
uint8 key_flag;        // 按键触发标志位
uint8 board_check_id = 0;

void board_check_init()
{
    gpio_init(EN_PIN, GPI, 1, GPI_PULL_UP);

    if (!gpio_get_level(EN_PIN))
    {
        board_check_id = 0;  // en board
        zero_reval = 4.41;
    }
    else
    {
        board_check_id = 1;  // no en board
        zero_reval = 3.41;
    }
}


void led_init()
{
    gpio_init(LED_RUN_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(LED_ERR_PIN, GPO, 1, GPO_PUSH_PULL);
    // gpio_init(LED_MODEL_PIN, GPO, 1, GPO_PUSH_PULL); 
}

void led_output()
{
    if (ccu61_get_trap_flag())
    {
        gpio_set_level(LED_ERR_PIN, 0); // 开启故障灯
        gpio_set_level(LED_RUN_PIN, 1); // 关闭运行灯
    }
    else
    {
        gpio_set_level(LED_ERR_PIN, 1); // 关闭故障灯
        if (timer_1ms >= 50)
            gpio_set_level(LED_RUN_PIN, 0); // 开启运行灯
        else
            gpio_set_level(LED_RUN_PIN, 1); // 关闭运行灯
    }
    // if (model_state)
    // if (0)
    // {
    //     gpio_set_level(LED_MODEL_PIN, 1);
    // }
    // else
    // {
    //     gpio_set_level(LED_MODEL_PIN, 0);
    // }
}
