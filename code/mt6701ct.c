#include "mt6701ct.h"

const uint8 crc_6[] = {0, 3, 6, 5, 12, 15, 10, 9, 24, 27, 30, 29, 20, 23, 18, 17, 48, 51, 54, 53, 60, 63, 58, 57, 40, 43, 46,
                       45, 36, 39, 34, 33, 35, 32, 37, 38, 47, 44, 41, 42, 59, 56, 61, 62, 55, 52, 49, 50, 19, 16, 21, 22, 31,
                       28, 25, 26, 11, 8, 13, 14, 7, 4, 1, 2};

uint8 Encoder_Data[3] = {0}; // �������ݻ���

// �Ƕ�int���ݼ���ʹ��
volatile uint16 _angle_last_dat = 0;
volatile uint16 _angle_this_dat = 0;
volatile int16 _angle_dangle = 0;

// �ٶ�int���ݼ���ʹ��
volatile uint32 _speed_last_time = 0;
volatile uint32 _speed_this_time = 0;
volatile long _speed_dtime = 0; // 0.1us�̶�
bool _speed_dat_flag = 0;        // ���ٶ�����æ��־λ

// λ��int���ݼ���ʹ��
volatile long _postion_add_dat = 0; // λ�û�������,��Ȧ��
volatile long __postion_add_dat = 0; // λ�û�������,��Ȧ��,�ش�ר�ã���������

uint8 crc6_itu(void)
{
    uint8 crc = 0;
    crc = crc_6[crc ^ (u8)(Encoder_Data[0] >> 2)];
    crc = crc_6[crc ^ (((u8)(Encoder_Data[0] & 0x03) << 4) | (Encoder_Data[1] >> 4))];
    crc = crc_6[crc ^ (((u8)(Encoder_Data[1] & 0x0f) << 2) | (Encoder_Data[2] >> 6))];
    if (crc == Encoder_Data[2])
        return 1;
    else
        return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     MT6701CT д�Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     mt6701ct_write_register(MT6701CT_PWR_CONF, 0x00);                   // �رո߼�ʡ��ģʽ
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void mt6701ct_write_register(uint8 reg, uint8 data)
{
    MT6701CT_CS(0);
    spi_write_8bit_register(MT6701CT_SPI, reg | MT6701CT_SPI_W, data);
    MT6701CT_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     MT6701CT д����
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     mt6701ct_write_registers(MT6701CT_INIT_DATA, mt6701ct_config_file, sizeof(mt6701ct_config_file));
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void mt6701ct_write_registers(uint8 reg, const uint8 *data, uint32 len)
{
    MT6701CT_CS(0);
    spi_write_8bit_registers(MT6701CT_SPI, reg | MT6701CT_SPI_W, data, len);
    MT6701CT_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     MT6701CT ���Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ���ز���     uint8           ����
// ʹ��ʾ��     mt6701ct_read_register(MT6701CT_CHIP_ID);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static uint8 mt6701ct_read_register(uint8 reg)
{
    uint8 data[2];
    MT6701CT_CS(0);
    spi_read_8bit_registers(MT6701CT_SPI, reg | MT6701CT_SPI_R, data, 2);
    MT6701CT_CS(1);
    return data[1];
}

//-------------------------------------------------------------------------------------------------------------------
// �������     MT6701CT ������
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ���ݻ�����
// ����˵��     len             ���ݳ���
// ���ز���     void
// ʹ��ʾ��     mt6701ct_read_registers(MT6701CT_ACC_ADDRESS, dat, 6);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void mt6701ct_read_registers(uint8 reg, uint8 *data, uint32 len)
{
    uint8 temp_data[8];
    MT6701CT_CS(0);
    spi_read_8bit_registers(MT6701CT_SPI, reg | MT6701CT_SPI_R, temp_data, len + 1);
    MT6701CT_CS(1);
    for(int i = 0; i < len; i ++)
    {
        *(data ++) = temp_data[i + 1];
    }
}

void mt6701ct_init()
{

}
