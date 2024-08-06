/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2021,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����ϵ�Ա��ͷ�
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            move_filter
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ3184284598)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        ADS v1.5.2
 * @Target core     TC264D
 * @Taobao          https://seekfree.taobao.com/
 * @date            2021-12-10
 ********************************************************************************************************************/

#include "move_filter.h"

move_filter_struct speed_filter;
move_filter_double_struct current_a_filter;
move_filter_double_struct current_b_filter;
move_filter_double_struct current_c_filter;
move_filter_double_struct iq_ref_filter;
move_filter_double_struct id_ref_filter;
// move_filter_double_struct
//-------------------------------------------------------------------------------------------------------------------
//   @brief      ����ƽ���˲�����
//   @param      void
//   @return     void
//   @since      ��Ҫ���ڶ������˲����洢Ŀ�����������n�����ݣ������ƽ��ֵ
//-------------------------------------------------------------------------------------------------------------------
void move_filter_calc(move_filter_struct *move_filter, int32 new_data)
{
    // �����µ���ֵ ��ȥ��ĩβ����ֵ ������µĺ�
    move_filter->data_sum = move_filter->data_sum + new_data - move_filter->data_buffer[move_filter->index];
    // ������ƽ��ֵ
    move_filter->data_average = move_filter->data_sum / move_filter->buffer_size;

    // ������д�뻺����
    move_filter->data_buffer[move_filter->index] = new_data;
    move_filter->index++;
    if (move_filter->buffer_size <= move_filter->index)
    {
        move_filter->index = 0;
    }
}

void move_filter_double_calc(move_filter_double_struct *move_filter, double new_data)
{
    // �����µ���ֵ ��ȥ��ĩβ����ֵ ������µĺ�
    move_filter->data_sum = move_filter->data_sum + new_data - move_filter->data_buffer[move_filter->index];
    // ������ƽ��ֵ
    move_filter->data_average = move_filter->data_sum / move_filter->buffer_size;

    // ������д�뻺����
    move_filter->data_buffer[move_filter->index] = new_data;
    move_filter->index++;
    if (move_filter->buffer_size <= move_filter->index)
    {
        move_filter->index = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����ƽ���˲���ʼ��
//  @param      void
//  @return     void
//  @since      ��Ҫ���ڶ������˲����洢Ŀ�����������n�����ݣ������ƽ��ֵ
//-------------------------------------------------------------------------------------------------------------------
void move_filter_init(move_filter_struct *move_filter)
{
    move_filter->data_average = 0;
    move_filter->data_sum = 0;
    move_filter->index = 0;
    // ���û�������С
    move_filter->buffer_size = MOVE_AVERAGE_SIZE;

    uint8 i;
    for (i = 0; i < move_filter->buffer_size; i++)
    {
        move_filter->data_buffer[i] = 0;
    }
}

void move_filter_double_init(move_filter_double_struct *move_filter)
{
    move_filter->data_average = 0;
    move_filter->data_sum = 0;
    move_filter->index = 0;
    // ���û�������С
    move_filter->buffer_size = MOVE_AVERAGE_SIZE;

    uint8 i;
    for (i = 0; i < move_filter->buffer_size; i++)
    {
        move_filter->data_buffer[i] = 0;
    }
}
