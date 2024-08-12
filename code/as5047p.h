#ifndef _AS5047P_H
#define _AS5047P_H

#include "zf_common_headfile.h"

// ��ģʽ�������ƿ���
#define AS5047P_ABI_Part_EN 1
#define angle_offset 0.1;

#define MAX_BAUD 50000000

//====================================================Ӳ�� SPI ����====================================================
#define AS5047P_SPI_SPEED (10 * 1000 * 1000) // Ӳ�� SPI ����
#define AS5047P_SPI (SPI_0)                  // Ӳ�� SPI ��
#define AS5047P_SPC_PIN (SPI0_SCLK_P20_11)   // Ӳ�� SPI SCK ����
#define AS5047P_SDI_PIN (SPI0_MOSI_P20_14)   // Ӳ�� SPI MOSI ����
#define AS5047P_SDO_PIN (SPI0_MISO_P20_12)   // Ӳ�� SPI MISO ����
#define AS5047P_CS_PIN (SPI0_CS2_P20_13)     // CS Ƭѡ ����
//====================================================Ӳ�� SPI ����====================================================
// #define AS5047P_CS_PIN             (P20_13)                                    // CS Ƭѡ����
// #define AS5047P_CS(x)              ((x) ? (gpio_high(AS5047P_CS_PIN)) : (gpio_low(AS5047P_CS_PIN)))
// #define AS5047P_TIMEOUT_COUNT      (0x00FF)                                    // ICM20602 ��ʱ����

// --- Volatile registers
#define AS5047P_NOP 0x0000   // �޲���ָ��
#define AS5047P_ERRFL 0x0001 //
#define AS5047P_PROG 0x0003  // ��оƬ���б�̵ļĴ�����ַ
#define AS5047P_DIAAGC 0x3FFC
#define AS5047P_MAG 0x3FFD      //
#define AS5047P_ANGLEUNC 0x3FFE //
#define AS5047P_ANGLECOM 0x3FFF // ��ž��ж�̬�Ƕ������Ƕ���Ϣ�ļĴ�����ַ
// --- Non-volatile registers
#define AS5047P_ZPOSM 0x0016 // ��Ŵ�����ʼλ�õĸ�8λ��ַ
#define AS5047P_ZPOSL 0x0017 // ��Ŵ�����ʼλ�õĵ�8λ��ַ
#define AS5047P_SETTINGS1 0x0018
#define AS5047P_SETTINGS2 0x0019
// --- Fields in registers
#define AS5047P_ERRFL_PARERR (1 << 2)
#define AS5047P_ERRFL_INVCOMM (1 << 1)
#define AS5047P_ERRFL_FRERR (1 << 0)
#define AS5047P_PROG_PROGVER (1 << 6)
#define AS5047P_PROG_PROGOTP (1 << 3)
#define AS5047P_PROG_OTPREF (1 << 2)
#define AS5047P_PROG_PROGEN (1 << 0)
#define AS5047P_DIAAGC_MAGL (1 << 11)
#define AS5047P_DIAAGC_MAGH (1 << 10)
#define AS5047P_DIAAGC_COF (1 << 9)
#define AS5047P_DIAAGC_LF (1 << 8)
#define AS5047P_DIAAGC_AGC (0x00FF << 0)
#define AS5047P_MAG_CMAG (0x3FFF << 0)
#define AS5047P_ANGLEUNC_CORDICANG (0x3FFF << 0)
#define AS5047P_ANGLECOM_DAECANG (0x3FFF << 0)
#define AS5047P_ZPOSM_ZPOSM (0x00FF << 0)
#define AS5047P_ZPOSL_COMP_H_ERR_EN (1 << 7)
#define AS5047P_ZPOSL_COMP_I_ERR_EN (1 << 6)
#define AS5047P_ZPOSL_ZPOSL (0x003F << 0)
#define AS5047P_SETTINGS1_BIT0 (1 << 0)
#define AS5047P_SETTINGS1_NOISESET (1 << 1)
#define AS5047P_SETTINGS1_DIR (1 << 2)
#define AS5047P_SETTINGS1_UVW_ABI (1 << 3)
#define AS5047P_SETTINGS1_DAECDIS (1 << 4)
#define AS5047P_SETTINGS1_ABIBIN (1 << 5)
#define AS5047P_SETTINGS1_DATASEL (1 << 6)
#define AS5047P_SETTINGS1_PWMON (1 << 7)
#define AS5047P_SETTINGS2_UVWPP (0x0007 << 0)
#define AS5047P_SETTINGS2_HYS (0x0003 << 3)
#define AS5047P_SETTINGS2_ABIRES (0x0007 << 5)
// --- R & W command
#define AS5047P_SPI_R (1 << 14)
#define AS5047P_SPI_W (0) //( 0 << 14)

#ifdef AS5047P_ABI_Part_EN
#define AS5047P_TIM (TIM4_ENCODER)
#define AS5047P_B (TIM4_ENCODER_CH1_P02_8) // ��ģ��B
#define AS5047P_A (TIM4_ENCODER_CH2_P00_9) // ��ģ��A
#endif

uint16 get_magnet_val();
double get_magnet_angle(uint16 val);
double get_elec_angle(uint16 val);
double calc_elec_angle_by_magnet(double reval);
float _normalizeAngle(float angle);
void as5047p_printf_test(void);
void as5047p_init(void);
extern double theta;
extern uint16 theta_val;
extern double theta_elec;
extern double theta_magnet;
extern double zero_val;
#endif /*_AS5047P_H*/
