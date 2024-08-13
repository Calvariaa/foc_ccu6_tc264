/*
 * foc.h
 *
 *  Created on: 2023年3月30日
 *      Author: 11474
 */
#include "zf_common_headfile.h"

#ifndef CODE_FOC_H_
#define CODE_FOC_H_
#define sqrt3 (double)1.732050807568877
#define pi (double)3.141592653589793
#define pi_2 (double)6.283185307179586
#define CLARK_ONEbySQRT3 (double)0.57735026918963f /* 1/sqrt(3) */
#define CLARK_ONEbyTHREE (double)0.33333333333333f /* 1/3 */

#define pi (double)3.141592653589793
#define pi_2 (double)6.283185307179586

#define ANGLE_TO_RAD(x)     ( (x) * PI / 180.0 )                                // 角度转换为弧度
#define RAD_TO_ANGLE(x)     ( (x) * 180.0 / PI )                                // 弧度转换为角度

// #define degrees_0   (double)0
// #define degrees_15  (double)0.261799387799149
// #define degrees_30  (double)0.523598775598299
// #define degrees_45  (double)0.785398163397448
// #define degrees_60  (double)1.047197551196598
// #define degrees_75  (double)1.308996938995747
// #define degrees_90  (double)1.570796326794896
// #define degrees_105 (double)1.832595714594046
// #define degrees_120 (double)2.094395102393195
// #define degrees_135 (double)2.356194490192345
// #define degrees_150 (double)2.617993877991495
// #define degrees_165 (double)2.879793265790643
// #define degrees_180 (double)pi
// #define degrees_195 (double)3.403392041388943
// #define degrees_210 (double)3.665191429188092
// #define degrees_225 (double)3.926990816987241
// #define degrees_240 (double)4.188790204786391
// #define degrees_255 (double)4.450589592585540
// #define degrees_270 (double)4.712388980384690
// #define degrees_285 (double)4.974188368183839
// #define degrees_300 (double)5.235987755982989
// #define degrees_315 (double)5.4977871437821382
// #define degrees_330 (double)5.7595865315812876
// #define degrees_345 (double)6.0213859193804370
// #define degrees_360 (double)pi_2
#define BUS_VOLTAGE 12 // 母线电压

// clark变换输出值
typedef struct
{
        float Alpha;
        float Beta;

} clark_variable;
// park变换输出值
typedef struct
{
        double id_ref;
        double iq_ref;

} park_variable;

// ipark变换输入值
typedef struct
{
        float u_d;
        float u_q;

} ipark_variable;

// 反park输出变量
typedef struct
{
        float u_alpha;
        float u_beta;

} out_variable;

typedef struct
{
        uint16 AH; // A上桥定时器比较值
        uint16 AL; // A下桥定时器比较值
        uint16 BH;
        uint16 BL;
        uint16 CH;
        uint16 CL;
} Period_Typedef;

typedef struct
{
        float ta; // 基矢量作用时间a
        float tb; // 基矢量作用时间b
} VectorTime_Typedef;
typedef struct
{
        float x;
        float y;
        float z;
} Instrument_Typedef; // 为了减少计算量的中间变量结构体

typedef struct
{
        // BLmotor_Typedef BLmotor;        //电机参数
        // ADC_Typedef Adc;                //adc采集
        out_variable V_Clark; // Alpha、Beta输入
        // CLARK_Typedef I_Clrak;          //Alpha、Beta反馈
        ipark_variable Ref_Park;   // d、q目标值
        park_variable I_Park;      // d、q返回值
        ipark_variable Park_in;    // d、q输入值
        Instrument_Typedef tool;   // SVPWM算法中间量
        VectorTime_Typedef Vector; // 矢量作用时间
        // HALL_Typedef hall;              //霍尔传感器数据
        uint8 N; // 电角度扇区
        // double theta;                   //电角度
        Period_Typedef Period; // 各桥定时器比较值
        // Current_CL_Typedef Current_CL;  //电流环PID参数
} FOC_Parm_Typedef;

// extern FOC_Parm_Typedef FOC;
clark_variable clark_cacl(adc_struct current);
park_variable park_cacl(clark_variable clark, float theta);
out_variable iPark_Calc(ipark_variable park, double theta);
Period_Typedef PeriodCal(VectorTime_Typedef vector, uint8 N, uint16 T);
VectorTime_Typedef Vector_Calc(Instrument_Typedef tool, uint8 N, uint8 Udc, uint16 T);
uint8 Electrical_Sector_Judge(Instrument_Typedef tool);
Instrument_Typedef Tool_Calc(out_variable clark2);

// extern int slow_startup_count;

extern double zero_reval;
#endif /* CODE_FOC_H_ */
