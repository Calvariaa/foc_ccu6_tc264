// #ifndef _MT6701CT_H
// #define _MT6701CT_H

// #include "zf_common_headfile.h"

// #define MT6701CT_SPI_SPEED            (10 * 1000 * 1000)                        // 硬件 SPI 速率
// #define MT6701CT_SPI                  (SPI_0)                                   // 硬件 SPI 号
// #define MT6701CT_SPC_PIN              (SPI0_SCLK_P20_11)                        // 硬件 SPI SCK 引脚
// #define MT6701CT_SDI_PIN              (SPI0_MOSI_P20_14)                        // 硬件 SPI MOSI 引脚
// #define MT6701CT_SDO_PIN              (SPI0_MISO_P20_12)                        // 硬件 SPI MISO 引脚
// //====================================================硬件 SPI 驱动====================================================

// #define MT6701CT_CS_PIN               (P20_13)                                  // CS 片选引脚
// #define MT6701CT_CS(x)                ((x) ? (gpio_high(MT6701CT_CS_PIN)) : (gpio_low(MT6701CT_CS_PIN)))

// //================================================定义 MT6701CT 内部地址================================================
// #define MT6701CT_DEV_ADDR             (0x69)                                    // SA0接地：0x68 SA0上拉：0x69 模块默认上拉
// #define MT6701CT_SPI_W                (0x00)
// #define MT6701CT_SPI_R                (0x80)


// // --- Volatile registers
// #define MT6701CT_NOP 0x0000   // 无操作指令
// #define MT6701CT_ERRFL 0x0001 //
// #define MT6701CT_PROG 0x0003  // 对芯片进行编程的寄存器地址
// #define MT6701CT_DIAAGC 0x3FFC
// #define MT6701CT_MAG 0x3FFD      //
// #define MT6701CT_ANGLEUNC 0x3FFE //
// #define MT6701CT_ANGLECOM 0x3FFF // 存放具有动态角度误差补偿角度信息的寄存器地址

// double get_rotor_angle_mt(void);
// void mt6701ct_printf_test(void);
// void mt6701ct_init(void);

// #endif /* _MT6701CT_H */