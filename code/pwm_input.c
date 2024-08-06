/*********************************************************************************************************************
 * TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� TC264 ��Դ���һ����
 *
 * TC264 ��Դ�� ��������
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
 * �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          pwm_input
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          ADS v1.8.0
 * ����ƽ̨          TC264D
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2023-02-01       pudding             first version
 ********************************************************************************************************************/

#include "ifxGtm_Tim.h"
#include "ccu6_pwm.h"
#include "pwm_input.h"

uint16 pwm_in_duty;
IfxGtm_Tim_In driver;
IfxGtm_Tim_In driver_back;
IfxGtm_Tim_In_Config config;
IfxGtm_Tim_In_Config config_back;
uint16 i;

//-------------------------------------------------------------------------------------------------------------------
// �������     ���벶���ʼ��
// ����˵��
// ����˵��
// ʹ��ʾ��
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void pwm_input_init(void)
{
    IfxGtm_enable(&MODULE_GTM);

    if (!(MODULE_GTM.CMU.CLK_EN.U & 0x2))
    {
        IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, (float)FCY);
        IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK1);
    }

    IfxGtm_Tim_In_initConfig(&config, &MODULE_GTM);
    config.timIndex = IfxGtm_Tim_1;
    config.channelIndex = IfxGtm_Tim_Ch_6;
    config.isrPriority = GTM_PWM_IN_PRIORITY;
    config.capture.irqOnNewVal = TRUE;
    config.capture.irqOnCntOverflow = TRUE;
    config.timeout.clock = IfxGtm_Cmu_Clk_0;
    config.filter.inputPin = &IfxGtm_TIM1_6_TIN6_P02_6_IN;
    config.filter.inputPinMode = IfxPort_InputMode_pullDown;
    driver.periodTick = FPWM;
    IfxGtm_Tim_In_init(&driver, &config);

    gpio_init(MOTOR_DIR_IN_PIN, GPI, 0, GPI_PULL_DOWN); // ��ʼ��������������
    
    IfxGtm_Tim_In_initConfig(&config_back, &MODULE_GTM);
    config_back.timIndex = IfxGtm_Tim_1;
    config_back.channelIndex = IfxGtm_Tim_Ch_2;
    config_back.capture.irqOnNewVal = TRUE;
    config_back.capture.irqOnCntOverflow = TRUE;
    config_back.timeout.clock = IfxGtm_Cmu_Clk_0;
    config_back.filter.inputPin = &IfxGtm_TIM1_2_TIN55_P21_4_IN;
    config_back.filter.inputPinMode = IfxPort_InputMode_pullDown;
    driver_back.periodTick = FPWM;
    IfxGtm_Tim_In_init(&driver_back, &config_back);

    gpio_init(MOTOR_back_DIR_IN_PIN, GPI, 0, GPI_PULL_DOWN); // ��ʼ��������������
}
// void pwm_back_input_init(void)
// {
//     IfxGtm_enable(&MODULE_GTM);

//     if (!(MODULE_GTM.CMU.CLK_EN.U & 0x2))
//     {
//         IfxGtm_Cmu_setClkFrequency(&MODULE_GTM, IfxGtm_Cmu_Clk_0, (float)FCY);
//         IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK1);
//     }

//     config_back.timIndex = IfxGtm_Tim_1;
//     config_back.channelIndex = IfxGtm_Tim_Ch_2;
//     config_back.isrPriority = GTM_PWM_IN_PRIORITY_back;
//     config_back.capture.irqOnNewVal = TRUE;
//     config_back.capture.irqOnCntOverflow = TRUE;
//     config_back.timeout.clock = IfxGtm_Cmu_Clk_0;
//     config_back.filter.inputPin = &IfxGtm_TIM1_2_TIN55_P21_4_IN;
//     config_back.filter.inputPinMode = IfxPort_InputMode_pullDown;
//     driver_back.periodTick = FPWM;
//     IfxGtm_Tim_In_init(&driver_back, &config_back);
// }

IfxGtm_Tim_In g_driverTIM[24]; /* TIM driver structure                                     */
IfxGtm_Tim_In_Config configTIM;
void TIM_InitConfig(IfxGtm_Tim_TinMap pin)
{
    static uint8 count = 0;

    if (count == 0)
    {
        IfxGtm_enable(&MODULE_GTM);                                  /* Enable the GTM                           */
        IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_CLK0); // Enable the CMU clock 0

        count++;
    }

    uint8 tempNum = (uint8)pin.tim * 8 + (uint8)pin.channel;

    IfxGtm_Tim_In_initConfig(&configTIM, &MODULE_GTM);          /* Initialize default parameters            */
    configTIM.filter.inputPin = &pin;                           /* Select input port pin                    */
    configTIM.filter.inputPinMode = IfxPort_InputMode_pullDown; /* Select input port pin mode               */
    IfxGtm_Tim_In_init(&g_driverTIM[tempNum], &configTIM);      /* Initialize the TIM                       */
}

/*************************************************************************
 *  �������ƣ�void TIM_GetPwm(void)
 *  ����˵������ȡTIM�����źŵ����ں�ռ�ձ�
 *  ����˵����pin        TIM�ܽ�
 *  ����˵����Period     ����
 *  ����˵����Duty       ռ�ձ�
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��22��
 *  ��    ע��
 *************************************************************************/
void TIM_GetPwm(IfxGtm_Tim_TinMap pin, float32 *Duty)
{
    uint8 g_dataCoherent = FALSE;
    uint8 tempNum = (uint8)pin.tim * 8 + (uint8)pin.channel;
    IfxGtm_Tim_In_update(&g_driverTIM[tempNum]); /* Update the measured data         */
    //*PwmFreq_Hz = 1.0f/IfxGtm_Tim_In_getPeriodSecond(&g_driverTIM[tempNum]);             /* Get the period of the PWM signal */
    if (gpio_get_level(MOTOR_PWM_IN_PIN) == 0)
        *Duty = IfxGtm_Tim_In_getDutyPercent(&g_driverTIM[tempNum], &g_dataCoherent); /* Get the duty cycle     */
}
