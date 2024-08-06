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
 * 文件名称          adc
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

#include "zf_driver_adc.h"

#include "adc.h"

adc_struct adc_information;

void adc_collection_init(void)
{
    adc_init(BOARD_POTENTIOMET_PORT, ADC_GATHER_RESOLUTION); // 初始化板载电位器采集端口
    adc_init(BATTERY_PHASE_PORT, ADC_GATHER_RESOLUTION);     // 初始化电源电压采集端口
    adc_init(A_PHASE_PORT, ADC_GATHER_RESOLUTION);           // 初始化A相电流采集端口
    adc_init(B_PHASE_PORT, ADC_GATHER_RESOLUTION);           // 初始化B相电流采集端口
    adc_init(C_PHASE_PORT, ADC_GATHER_RESOLUTION);           // 初始化C相电流采集端口
    adc_init(BUS_PHASE_PORT, ADC_GATHER_RESOLUTION);         // 初始化母线电流采集端口
}

void adc_read(void)
{
    uint16 adc_value_a_phase = 0;
    uint16 adc_value_b_phase = 0;
    uint16 adc_value_c_phase = 0;
    uint16 adc_value_bus_phase = 0;
    uint16 adc_value_battery_phase = 0;

    adc_value_a_phase = adc_convert(A_PHASE_PORT);                                                // 获取A相电流采集端口的值
    adc_value_b_phase = adc_convert(B_PHASE_PORT);                                                // 获取A相电流采集端口的值
    adc_value_c_phase = adc_convert(C_PHASE_PORT);                                                // 获取A相电流采集端口的值
    adc_value_bus_phase = adc_convert(BUS_PHASE_PORT);                                            // 获取母线电流采集端口的值
    adc_value_battery_phase = adc_convert(BATTERY_PHASE_PORT);                                    // 获取电源电压采集端口的值
    adc_information.current_board = adc_convert(BOARD_POTENTIOMET_PORT);                          // 获取板载电位器电压
    adc_information.current_a = (float)(adc_value_a_phase - 2048) * CURRENT_TRANSITION_VALUE;     // 计算A相电流
    adc_information.current_b = (float)(adc_value_b_phase - 2048) * CURRENT_TRANSITION_VALUE;     // 计算B相电流
    adc_information.current_c = (float)(adc_value_c_phase - 2048) * CURRENT_TRANSITION_VALUE;     // 计算C相电流
    adc_information.voltage_bus = (float)(adc_value_bus_phase - 2048) * CURRENT_TRANSITION_VALUE; // 计算母线电流
    adc_information.battery_voltage = (float)adc_value_battery_phase / 4096 * 3.3 * 11;           // 计算电池电压
}

#define BATTERY_PROTECT_VALUE (3.7f)
void battery_check(void)
{
    static uint32 check_count = 0;

    if (adc_information.battery_voltage > 5.0)
    {
        if (adc_information.battery_voltage < 9.0)
        {
            if (adc_information.battery_voltage < (BATTERY_PROTECT_VALUE * 2)) // 2s电池电压不得低于7.4V
            {
                check_count++;
            }
            else
            {
                // 若电压过低，自锁1秒后再恢复检测
                check_count = (check_count > 100 && check_count < 300) ? (check_count + 1) : 0;
            }
        }
        else if (adc_information.battery_voltage < 13.0)
        {
            if (adc_information.battery_voltage < (BATTERY_PROTECT_VALUE * 3)) // 3s电池电压不得低于11.1V
            {
                check_count++;
            }
            else
            {
                check_count = (check_count > 100 && check_count < 300) ? (check_count + 1) : 0;
            }
        }
        else if (adc_information.battery_voltage < 26.0)
        {
            if (adc_information.battery_voltage < (BATTERY_PROTECT_VALUE * 6)) // 6s电池电压不得低于22.2V
            {
                check_count++;
            }
            else
            {
                check_count = (check_count > 100 && check_count < 300) ? (check_count + 1) : 0;
            }
        }
    }
    //
    // else
    // {
    //     motor_control.battery_state = NO_BATTERY;
    // }

    // if(check_count == 0)
    // {
    //     motor_control.battery_state = NORMAL_VOLTAGE;
    // }
    // if(check_count > 100)                                       // 电压过低持续500ms则确认过低
    // {
    //     motor_control.battery_state = LOW_VOLTAGE;

    // }
}
