// #ifndef _MT6701CT_H
// #define _MT6701CT_H

// #include "zf_common_headfile.h"

// #define MT6701CT_SPI_SPEED            (10 * 1000 * 1000)                        // Ӳ�� SPI ����
// #define MT6701CT_SPI                  (SPI_0)                                   // Ӳ�� SPI ��
// #define MT6701CT_SPC_PIN              (SPI0_SCLK_P20_11)                        // Ӳ�� SPI SCK ����
// #define MT6701CT_SDI_PIN              (SPI0_MOSI_P20_14)                        // Ӳ�� SPI MOSI ����
// #define MT6701CT_SDO_PIN              (SPI0_MISO_P20_12)                        // Ӳ�� SPI MISO ����
// //====================================================Ӳ�� SPI ����====================================================

// #define MT6701CT_CS_PIN               (P20_13)                                  // CS Ƭѡ����
// #define MT6701CT_CS(x)                ((x) ? (gpio_high(MT6701CT_CS_PIN)) : (gpio_low(MT6701CT_CS_PIN)))

// //================================================���� MT6701CT �ڲ���ַ================================================
// #define MT6701CT_DEV_ADDR             (0x69)                                    // SA0�ӵأ�0x68 SA0������0x69 ģ��Ĭ������
// #define MT6701CT_SPI_W                (0x00)
// #define MT6701CT_SPI_R                (0x80)


// // --- Volatile registers
// #define MT6701CT_NOP 0x0000   // �޲���ָ��
// #define MT6701CT_ERRFL 0x0001 //
// #define MT6701CT_PROG 0x0003  // ��оƬ���б�̵ļĴ�����ַ
// #define MT6701CT_DIAAGC 0x3FFC
// #define MT6701CT_MAG 0x3FFD      //
// #define MT6701CT_ANGLEUNC 0x3FFE //
// #define MT6701CT_ANGLECOM 0x3FFF // ��ž��ж�̬�Ƕ������Ƕ���Ϣ�ļĴ�����ַ

// double get_rotor_angle_mt(void);
// void mt6701ct_printf_test(void);
// void mt6701ct_init(void);

// #endif /* _MT6701CT_H */