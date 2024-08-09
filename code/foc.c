/*
 * foc.c
 *
 *  Created on: 2023��3��30��
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
//  @brief      �����˱任
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
//  @brief      ���˱任
//  @param      clark:�����˽ṹ��
//  @param      theta:�Ƕ�
//  @return     ���˱任������
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
//  @brief      ������任
//  @param      park:���˽ṹ��
//  @param      theta:�Ƕ�
//  @return     ������任������
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
//  @brief      �м��������
//  @param      clark: Alpha & Beta
//  @return     �м����������
//  @since      x,y,zΪ�м��������ʵ������
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
//  @brief      ��Ƕ������ж�
//  @param      temp:���ڸ���������м����
//  @return     N = ��1~6��
//              3 1 5 4 6 2 3��Nֵ��
//              1 2 3 4 5 6 1��������
//  @since      x,y,zΪ�м��������ʵ������
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
// ����������

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ÿ����������ʸ������ʱ�����
//  @param      tool:���ڸ���������м����
//  @param      N:�ж�������Nֵ
//  @param      Udc:ĸ�ߵ�ѹ
//  @param      T:PWM���ڣ�װ��ֵ��
//  @return     ������ʱ�������
//  @since      ���������ֱ�����ڸ�����������ʸ��������ʱ��
//-------------------------------------------------------------------------------------------------------------------
VectorTime_Typedef Vector_Calc(Instrument_Typedef tool, uint8 N, uint8 Udc, uint16 T)
{
    VectorTime_Typedef vector;

    double temp = sqrt3 * T / Udc; // Ϊ�˵ȷ�ֵ�任���ѳ���2/3

    switch (N)
    {
    case 3: // ����1
        vector.ta = temp * tool.y;
        vector.tb = temp * tool.x;
        break;
    case 1: // ����2
        vector.ta = -temp * tool.y;
        vector.tb = -temp * tool.z;
        break;
    case 5: // ����3
        vector.ta = temp * tool.x;
        vector.tb = temp * tool.z;
        break;
    case 4: // ����4
        vector.ta = -temp * tool.x;
        vector.tb = -temp * tool.y;
        break;
    case 6: // ����5
        vector.ta = temp * tool.z;
        vector.tb = temp * tool.y;
        break;
    case 2: // ����6
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
//   @brief      d��q��PI������
//   @param      ref_park:d��qĿ��ֵ
//   @param      I_park  :d��qʵ��ֵ
//   @return     d��q����ֵ
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
    //     kp_foc_iq = 7;    ///��ʱ����ֹδ�򿪿���ǰ�����ۼ�
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
    // look6 = error_q * 10000; // look1-5����������λ���鿴����

    // look4 = (error_q)*10000;
    // look3 =I_park.id_ref*10000;
    error_sum_d = error_sum_d + error_d;
    error_sum_q = error_sum_q + error_q;

    if (error_sum_d > 70)
        error_sum_d = 70; // �����޷�
    if (error_sum_d < -70)
        error_sum_d = -70; // �����޷�
    if (error_sum_q > 70)
        error_sum_q = 70; // �����޷�
    if (error_sum_q < -70)
        error_sum_q = -70; // �����޷�
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
//  @brief      ������������     �ر���������
//  @param      periodAH:       A����PWMռ�ձ�
//  @param      periodBH:       B����PWMռ�ձ�
//  @param      periodCH:       C����PWMռ�ձ�
//  @return     void
//  @since      �������Ķ���ģʽ��װ��ֵԽ�󣬸ߵ�ƽʱ��Խ��
//-------------------------------------------------------------------------------------------------------------------
void Mos_All_High_Open(uint16 periodAH, uint16 periodBH, uint16 periodCH)
{
    ccu6SFR->MODCTR.B.T12MODEN = 0x3F; // 0011 1111                   //�û��ֲ�27.8�½��Լ���

    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_0, periodAH); // ���ñȽ�ֵ
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_1, periodBH); // ���ñȽ�ֵ
    IfxCcu6_setT12CompareValue(ccu6SFR, IfxCcu6_T12Channel_2, periodCH); // ���ñȽ�ֵ
    IfxCcu6_enableShadowTransfer(ccu6SFR, TRUE, FALSE);                  // ʹ��
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
        FOC_S.Ref_Park.u_q = 0.3; // ����ֵ
        if (slow_startup_count >= 200000)
        {
            FOC_S.Ref_Park.u_q = 0.2; // ����ֵ//0.01
        }

        Current_Close_Loop(FOC_S.Ref_Park, I_out);

        data_send[6] = Park_in.u_q * 1000;
        data_send[7] = Park_in.u_d * 1000;
#else
        // ����
        Park_in.u_d = 0;
        Park_in.u_q = 6; // 6

#endif

        FOC_S.V_Clark = iPark_Calc(Park_in, theta);

        FOC_S.tool = Tool_Calc(FOC_S.V_Clark);                                        // �м��������
        FOC_S.N = Electrical_Sector_Judge(FOC_S.tool);                                // ��Ƕ������ж�
        FOC_S.Vector = Vector_Calc(FOC_S.tool, FOC_S.N, BUS_VOLTAGE, PWM_PRIOD_LOAD); // ʸ������ʱ�����
        FOC_S.Period = PeriodCal(FOC_S.Vector, FOC_S.N, PWM_PRIOD_LOAD);              // ����PWMռ�ձȼ���
        Mos_All_High_Open(FOC_S.Period.AH, FOC_S.Period.BH, FOC_S.Period.CH);
        data_send[1] = (float)FOC_S.Period.AH;
        data_send[2] = (float)FOC_S.Period.BH;
        data_send[3] = (float)FOC_S.Period.CH;
    }
}

#pragma section all restore