/*
 * foc.c
 *
 *  Created on: 2023年3月30日
 *      Author: 11474
 */
#include "zf_common_typedef.h"
#include "ccu6_pwm.h"
#include "foc.h"
#include "as5047p.h"
#include "move_filter.h"

#pragma section all "cpu1_dsram"

// double theta2;


int slow_startup_count = 0;
int dianliu = 0;

float error_sum_d = 0;
float error_sum_q = 0;

extern float data_send[16];
//-------------------------------------------------------------------------------------------------------------------
//  @brief      克拉克变换
//  @param      void
//  @return     void
//  @since      none
//-------------------------------------------------------------------------------------------------------------------
clark_variable clark_cacl(adc_struct current)
{
    clark_variable clark;
    clark.Alpha = 1.5 * current.current_a * 2 / 3;
    clark.Beta = (sqrt3 / 2.0) * current.current_a * 2 / 3 + sqrt3 * current.current_b * 2 / 3;

    return clark;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      帕克变换
//  @param      clark:克拉克结构体
//  @param      theta:角度
//  @return     帕克变换计算结果
//  @since
//-------------------------------------------------------------------------------------------------------------------
park_variable park_cacl(clark_variable clark, double theta)
{
    park_variable park;

    park.id_ref = clark.Alpha * fast_cos(theta) + clark.Beta * fast_sin(theta);
    park.iq_ref = -clark.Alpha * fast_sin(theta) + clark.Beta * fast_cos(theta);
    move_filter_double_calc(&id_ref_filter, park.id_ref);
    move_filter_double_calc(&iq_ref_filter, park.iq_ref);
    park.id_ref = id_ref_filter.data_average;
    park.iq_ref = iq_ref_filter.data_average;
    return park;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      帕克逆变换
//  @param      park:帕克结构体
//  @param      theta:角度
//  @return     帕克逆变换计算结果
//  @since
//-------------------------------------------------------------------------------------------------------------------
out_variable iPark_Calc(ipark_variable park, double theta)
{
    out_variable u_in;

    u_in.u_alpha = park.u_d * fast_cos(theta) - park.u_q * fast_sin(theta);
    u_in.u_beta = park.u_d * fast_sin(theta) + park.u_q * fast_cos(theta);
    // look3 =park.u_q*10000;
    return u_in;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      中间变量计算
//  @param      clark: Alpha & Beta
//  @return     中间变量计算结果
//  @since      x,y,z为中间变量，无实际意义
//-------------------------------------------------------------------------------------------------------------------
Instrument_Typedef Tool_Calc(out_variable V_Clark)
{
    Instrument_Typedef tool;

    tool.x = V_Clark.u_beta;
    tool.y = V_Clark.u_alpha * sqrt3 / 2.0 - V_Clark.u_beta / 2.0;
    tool.z = -V_Clark.u_alpha * sqrt3 / 2.0 - V_Clark.u_beta / 2.0;

    return tool;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      电角度扇区判断
//  @param      temp:用于辅助计算的中间变量
//  @return     N = （1~6）
//              3 1 5 4 6 2 3（N值）
//              1 2 3 4 5 6 1（扇区）
//  @since      x,y,z为中间变量，无实际意义
//-------------------------------------------------------------------------------------------------------------------
uint8 Electrical_Sector_Judge(Instrument_Typedef tool)
{
    uint8 N = 0;

    if (tool.x > 0)
        N = N + 1;
    if (tool.y > 0)
        N = N + 2;
    if (tool.z > 0)
        N = N + 4;

    return N;
}
// int PID[3]={0,0,0};
// 函数合起来

//-------------------------------------------------------------------------------------------------------------------
//  @brief      每个扇区两基矢量作用时间计算
//  @param      tool:用于辅助计算的中间变量
//  @param      N:判断扇区的N值
//  @param      Udc:母线电压
//  @param      T:PWM周期（装载值）
//  @return     各扇区时间计算结果
//  @since      根据扇区分别计算在各扇区中两基矢量的作用时间
//-------------------------------------------------------------------------------------------------------------------
VectorTime_Typedef Vector_Calc(Instrument_Typedef tool, uint8 N, uint8 Udc, uint16 T)
{
    VectorTime_Typedef vector;

    double temp = sqrt3 * T / Udc; // 为了等幅值变换，已乘以2/3

    switch (N)
    {
    case 3: // 扇区1
        vector.ta = temp * tool.y;
        vector.tb = temp * tool.x;
        break;
    case 1: // 扇区2
        vector.ta = -temp * tool.y;
        vector.tb = -temp * tool.z;
        break;
    case 5: // 扇区3
        vector.ta = temp * tool.x;
        vector.tb = temp * tool.z;
        break;
    case 4: // 扇区4
        vector.ta = -temp * tool.x;
        vector.tb = -temp * tool.y;
        break;
    case 6: // 扇区5
        vector.ta = temp * tool.z;
        vector.tb = temp * tool.y;
        break;
    case 2: // 扇区6
        vector.ta = -temp * tool.z;
        vector.tb = -temp * tool.x;
        break;
    default:
        vector.ta = 0;
        vector.tb = 0;
        break;
    }

    return vector;
}

Period_Typedef PeriodCal(VectorTime_Typedef vector, uint8 N, uint16 T)
{
    Period_Typedef period;
    uint16 value1, value2, value3;
    double Ttemp = vector.ta + vector.tb;

    if (Ttemp > T)
    {
        vector.ta = vector.ta / Ttemp * (double)T;
        vector.tb = vector.tb / Ttemp * (double)T;
    }
    value1 = (uint16)(((double)T - vector.ta - vector.tb) / 4.0);
    value2 = (uint16)(value1 + vector.ta / 2.0);
    value3 = (uint16)(value2 + vector.tb / 2.0);
    switch (N)
    {
    case 3:
        period.AH = value1;
        period.BH = value2;
        period.CH = value3;
        break;
    case 1:
        period.AH = value2;
        period.BH = value1;
        period.CH = value3;
        break;
    case 5:
        period.AH = value3;
        period.BH = value1;
        period.CH = value2;
        break;
    case 4:
        period.AH = value3;
        period.BH = value2;
        period.CH = value1;
        break;
    case 6:
        period.AH = value2;
        period.BH = value3;
        period.CH = value1;
        break;
    case 2:
        period.AH = value1;
        period.BH = value3;
        period.CH = value2;
        break;
    default:
        period.AH = 2500;
        period.BH = 2500;
        period.CH = 2500;
        break;
    }

    return period;
}

FOC_Parm_Typedef FOC_S;
// FOC_Parm_Typedef ccu6_pwm;
//-------------------------------------------------------------------------------------------------------------------
//   @brief      d、q轴PI控制器
//   @param      ref_park:d、q目标值
//   @param      I_park  :d、q实际值
//   @return     d、q输入值
//   @since
//-------------------------------------------------------------------------------------------------------------------
ipark_variable Current_Close_Loop(ipark_variable ref_park, park_variable I_park)
{
    ipark_variable Park_in;
    float error_d, error_q;

    // int slow_startup_count = 0;

    float kp_foc_id = 120;  // 10
    float ki_foc_id = 0.08; // 0.06
    float kp_foc_iq = 7;    // 20
    float ki_foc_iq = 0.08; // 0.06
    // if(slow_startup_count<=100000)
    //{
    // slow_startup_count++;
    // }
    // if(slow_startup_count>=100000)
    //{
    //     kp_foc_id = 7;
    //     ki_foc_id = 0.17;
    //     kp_foc_iq = 7;    ///计时，防止未打开开关前积分累加
    //     ki_foc_iq = 0.17;
    //     slow_startup_count = 199999;
    // }
    // if(slow_startup_count<100000)
    //{
    //      kp_foc_id = 0;
    //      ki_foc_id = 0;
    //      kp_foc_iq = 0;
    //      ki_foc_iq = 0;
    //     error_sum_d=0;
    //     error_sum_q=0;
    // }
    error_d = FOC_S.Ref_Park.u_d - I_park.id_ref;
    error_q = FOC_S.Ref_Park.u_q - I_park.iq_ref;

    // look5 = error_d * 10000;
    // look6 = error_q * 10000; // look1-5都是用于上位机查看波形

    // look4 = (error_q)*10000;
    // look3 =I_park.id_ref*10000;
    error_sum_d = error_sum_d + error_d;
    error_sum_q = error_sum_q + error_q;

    if (error_sum_d > 70)
        error_sum_d = 70; // 积分限幅
    if (error_sum_d < -70)
        error_sum_d = -70; // 积分限幅
    if (error_sum_q > 70)
        error_sum_q = 70; // 积分限幅
    if (error_sum_q < -70)
        error_sum_q = -70; // 积分限幅
    //  look5 =error_sum_d*100;
    // look6 =error_sum_q*100;
    Park_in.u_d = kp_foc_id * error_d + ki_foc_id * error_sum_d;
    Park_in.u_q = kp_foc_iq * error_q + ki_foc_iq * error_sum_q;
    if (Park_in.u_d >= 4)
    {
        Park_in.u_d = 4;
    }
    if (Park_in.u_d <= 0 - 4)
    {
        Park_in.u_d = 0 - 4;
    }
    if (Park_in.u_q <= 0 - 6)
    {
        Park_in.u_q = 0 - 6;
    }
    if (Park_in.u_q >= 6)
    {
        Park_in.u_q = 6;
    }

    // if(Park_in.u_q>=6.5)
    //{
    //     Park_in.u_q=6.5;
    // }
    // if(Park_in.u_d>=1)
    //{
    //     Park_in.u_d=1;
    // }
    // if(Park_in.u_q<=0-6.5)
    //{
    //     Park_in.u_q=0-6.5;
    // }
    // if(Park_in.u_d<=0-1)
    //{
    //     Park_in.u_d=0-1;
    // }
    // look1 = I_park.id_ref*1000;

    // look3 =I_park.iq_ref*1000;
    // Park_in.u_d=1;
    // Park_in.u_q=0;
    return Park_in;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      开启所有上桥     关闭所有下桥
//  @param      periodAH:       A上桥PWM占空比
//  @param      periodBH:       B上桥PWM占空比
//  @param      periodCH:       C上桥PWM占空比
//  @return     void
//  @since      采用中心对齐模式，装载值越大，高电平时间越短
//-------------------------------------------------------------------------------------------------------------------
void Mos_All_High_Open(uint16 periodAH, uint16 periodBH, uint16 periodCH)
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x3F; // 0011 1111                   //用户手册27.8章节自己看

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, periodAH); // 设置比较值
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, periodBH); // 设置比较值
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, periodCH); // 设置比较值
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);                  // 使能
}
void mos_close()
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x2A;

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, 0);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, 0);
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, 0);
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);
}

float ang = 0;
clark_variable adcd_struct;
park_variable I_out;
ipark_variable Park_in;
void foc_commutation()
{
    if (slow_startup_count <= 200000)
    {
        // mos_close();
        slow_startup_count++;
    }
    if (slow_startup_count >= 100000)
    // if (0)
    {
        // dianliu++;

        // get_rotor_angle();
        // test
        ang += 0.01;
        if (ang >= 360.f)
            ang = 0.f;
        theta = ANGLE_TO_RAD((int16)ang);

#ifdef CURRENTLOOP

        adc_read();
        data_send[1] = (float)adc_information.current_a;
        data_send[2] = (float)adc_information.current_b;
        data_send[3] = (float)adc_information.current_c;

        /*----------*/
        adcd_struct = clark_cacl(adc_information);
        /*----------*/
        I_out = park_cacl(adcd_struct, theta);

        data_send[4] = (float)I_out.id_ref * 10000;
        data_send[5] = (float)I_out.iq_ref * 10000;

        // look5 =  adc_information.current_a *1000;
        //  look6 =  adc_information.current_b *1000;
        // data_send[3]=(int16) I_out.id_ref*1000;
        // data_send[4]=(int16)  I_out.iq_ref*1000;
        FOC_S.Ref_Park.u_d = 0;
        FOC_S.Ref_Park.u_q = 0.3; // 期望值
        if (slow_startup_count >= 200000)
        {
            FOC_S.Ref_Park.u_q = 0.2; // 期望值//0.01
        }

        Current_Close_Loop(FOC_S.Ref_Park, I_out);

        data_send[6] = Park_in.u_q * 1000;
        data_send[7] = Park_in.u_d * 1000;
#else
        // 开环
        Park_in.u_d = 0;
        Park_in.u_q = 6; // 6

#endif

        FOC_S.V_Clark = iPark_Calc(Park_in, theta);

        FOC_S.tool = Tool_Calc(FOC_S.V_Clark);                                        // 中间变量计算
        FOC_S.N = Electrical_Sector_Judge(FOC_S.tool);                                // 电角度扇区判断
        FOC_S.Vector = Vector_Calc(FOC_S.tool, FOC_S.N, BUS_VOLTAGE, PWM_PRIOD_LOAD); // 矢量作用时间计算
        FOC_S.Period = PeriodCal(FOC_S.Vector, FOC_S.N, PWM_PRIOD_LOAD);              // 各桥PWM占空比计算
        Mos_All_High_Open(FOC_S.Period.AH, FOC_S.Period.BH, FOC_S.Period.CH);
        data_send[1] = (float)FOC_S.Period.AH;
        data_send[2] = (float)FOC_S.Period.BH;
        data_send[3] = (float)FOC_S.Period.CH;
    }
}

#pragma section all restore