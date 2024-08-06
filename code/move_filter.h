#ifndef _MOVE_FILTER_H
#define _MOVE_FILTER_H

#include "zf_common_headfile.h"

#define MOVE_AVERAGE_SIZE 200 // ���建������С

typedef struct
{
    uint8 index;                          // �±�
    uint8 buffer_size;                    // buffer��С
    int32 data_buffer[MOVE_AVERAGE_SIZE]; // ������
    int32 data_sum;                       // ���ݺ�
    int32 data_average;                   // ����ƽ��ֵ
} move_filter_struct;

extern move_filter_struct speed_filter;

void move_filter_init(move_filter_struct *move_average);
void move_filter_calc(move_filter_struct *move_average, int32 new_data);

typedef struct
{
    uint8 index;                           // �±�
    uint8 buffer_size;                     // buffer��С
    double data_buffer[MOVE_AVERAGE_SIZE]; // ������
    double data_sum;                       // ���ݺ�
    double data_average;                   // ����ƽ��ֵ
} move_filter_double_struct;

extern move_filter_double_struct current_a_filter;
extern move_filter_double_struct current_b_filter;
extern move_filter_double_struct current_c_filter;
extern move_filter_double_struct iq_ref_filter;
extern move_filter_double_struct id_ref_filter;
void move_filter_double_init(move_filter_double_struct *move_average);
void move_filter_double_calc(move_filter_double_struct *move_average, double new_data);

#endif /* CODE_MOVE_FILTER_H_ */
