#include "as5047p.h"
#include "IFXQSPI_REGDEF.h"
#include "IfxQspi_SpiMaster.h"
#include "IfxQspi.h"

uint16 theta_val;

double theta_elec;

double theta_magnet;
int32 full_rotations;

double theta_magnet_last = 0;
int32 full_rotations_last = 0;

double zero_reval = 0;
double zero_angle = 0;

//-------------------------------------------------------------------------------------------------------------------
//  �������      spi_init_16��ʼ��
//  ����˵��      spi_n           ѡ��SPIģ��(SPI_1-SPI_4)
//  ����˵��      mode            SPIģʽ 0��CPOL=0 CPHA=0    1��CPOL=0 CPHA=1   2��CPOL=1 CPHA=0   3��CPOL=1 CPHA=1  // ����ϸ�ڿ����в�������
//  ����˵��      baud            ����SPI�Ĳ�����
//  ����˵��      cs_pin          ѡ��SPIƬѡ����
//  ����˵��      sck_pin         ѡ��SPIʱ������
//  ����˵��      mosi_pin        ѡ��SPI MOSI����
//  ����˵��      miso_pin        ѡ��SPI MISO����
//  ���ز���      void
//  ʹ��ʾ��      spi_init(SPI_2, SPI_MODE0, 1*1000*1000, SPI2_SCLK_P15_3, SPI2_MOSI_P15_5, SPI2_MISO_P15_4, SPI2_CS0_P15_2); // Ӳ��SPI��ʼ��  ģʽ0 ������Ϊ1Mhz
//  ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void spi_16_init(spi_index_enum spi_n, spi_sck_pin_enum sck_pin, spi_mosi_pin_enum mosi_pin, spi_miso_pin_enum miso_pin, spi_cs_pin_enum cs_pin, uint8 mode, uint32 baud)
{

    IfxQspi_SpiMaster_Config MasterConfig;
    IfxQspi_SpiMaster MasterHandle;
    IfxQspi_SpiMaster_Channel MasterChHandle;
    IfxQspi_SpiMaster_Pins MasterPins;
    IfxQspi_SpiMaster_Output SlsoPin;
    volatile Ifx_QSPI* moudle;

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);

    spi_mux(spi_n, sck_pin, mosi_pin, miso_pin, cs_pin, &MasterPins, &SlsoPin);

    IfxQspi_SpiMaster_initModuleConfig(&MasterConfig, moudle);
    MasterConfig.base.mode = SpiIf_Mode_master;
    MasterConfig.base.maximumBaudrate = MAX_BAUD;
    MasterConfig.base.isrProvider = IfxSrc_Tos_cpu0;

    MasterConfig.pins = &MasterPins;
    IfxQspi_SpiMaster_initModule(&MasterHandle, &MasterConfig);

    IfxQspi_SpiMaster_ChannelConfig MasterChConfig;
    IfxQspi_SpiMaster_initChannelConfig(&MasterChConfig, &MasterHandle);

    MasterChConfig.base.baudrate = (float)baud;
    switch (mode)
    {
    case 0:
    {
        MasterChConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleLow;                   // CPOL
        MasterChConfig.base.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge; // CPHA
    }
    break;
    case 1:
    {
        MasterChConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleLow;
        MasterChConfig.base.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnLeadingEdge;
    }
    break;
    case 2:
    {
        MasterChConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleHigh;
        MasterChConfig.base.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnTrailingEdge;
    }
    break;
    case 3:
    {
        MasterChConfig.base.mode.clockPolarity = SpiIf_ClockPolarity_idleHigh;
        MasterChConfig.base.mode.shiftClock = SpiIf_ShiftClock_shiftTransmitDataOnLeadingEdge;
    }
    break;
    }

    MasterChConfig.base.mode.dataHeading = SpiIf_DataHeading_msbFirst;
    MasterChConfig.base.mode.dataWidth = 16;

    MasterChConfig.base.mode.csActiveLevel = Ifx_ActiveState_low;
    MasterChConfig.sls.output = SlsoPin;
    IfxQspi_SpiMaster_initChannel(&MasterChHandle, &MasterChConfig);
}

uint8 get_spi_cs_pin(spi_cs_pin_enum cs_pin)
{
    if (SPI0_CS0_P20_8 == cs_pin)
        return gpio_get_level(P20_8);
    else if (SPI0_CS1_P20_9 == cs_pin)
        return gpio_get_level(P20_9);
    else if (SPI0_CS2_P20_13 == cs_pin)
        return gpio_get_level(P20_13);
    else if (SPI0_CS3_P11_10 == cs_pin)
        return gpio_get_level(P11_10);
    else if (SPI0_CS4_P11_11 == cs_pin)
        return gpio_get_level(P11_11);
    else if (SPI0_CS5_P11_2 == cs_pin)
        return gpio_get_level(P11_2);
    else if (SPI0_CS6_P20_10 == cs_pin)
        return gpio_get_level(P20_10);
    else if (SPI0_CS7_P33_5 == cs_pin)
        return gpio_get_level(P33_5);
    else if (SPI0_CS8_P20_6 == cs_pin)
        return gpio_get_level(P20_6);
    else if (SPI0_CS9_P20_3 == cs_pin)
        return gpio_get_level(P20_3);
    else if (SPI0_CS13_P15_0 == cs_pin)
        return gpio_get_level(P15_0);
    else if (SPI1_CS0_P20_8 == cs_pin)
        return gpio_get_level(P20_8);
    else if (SPI1_CS1_P20_9 == cs_pin)
        return gpio_get_level(P20_9);
    else if (SPI1_CS2_P20_13 == cs_pin)
        return gpio_get_level(P20_13);
    else if (SPI1_CS3_P11_10 == cs_pin)
        return gpio_get_level(P11_10);
    else if (SPI1_CS4_P11_11 == cs_pin)
        return gpio_get_level(P11_11);
    else if (SPI1_CS5_P11_2 == cs_pin)
        return gpio_get_level(P11_2);
    else if (SPI1_CS6_P33_10 == cs_pin)
        return gpio_get_level(P33_10);
    else if (SPI1_CS7_P33_5 == cs_pin)
        return gpio_get_level(P33_5);
    else if (SPI1_CS8_P10_4 == cs_pin)
        return gpio_get_level(P10_4);
    else if (SPI1_CS9_P10_5 == cs_pin)
        return gpio_get_level(P10_5);
    else if (SPI2_CS0_P15_2 == cs_pin)
        return gpio_get_level(P15_2);
    else if (SPI2_CS1_P14_2 == cs_pin)
        return gpio_get_level(P14_2);
    else if (SPI2_CS2_P14_6 == cs_pin)
        return gpio_get_level(P14_6);
    else if (SPI2_CS3_P14_3 == cs_pin)
        return gpio_get_level(P14_3);
    else if (SPI2_CS5_P15_1 == cs_pin)
        return gpio_get_level(P15_1);
    else if (SPI2_CS6_P33_13 == cs_pin)
        return gpio_get_level(P33_13);
    else if (SPI2_CS7_P20_10 == cs_pin)
        return gpio_get_level(P20_10);
    else if (SPI2_CS8_P20_6 == cs_pin)
        return gpio_get_level(P20_6);
    else if (SPI2_CS9_P20_3 == cs_pin)
        return gpio_get_level(P20_3);

    else if (SPI3_CS0_P02_4 == cs_pin)
        return gpio_get_level(P02_4);
    else if (SPI3_CS1_P02_0 == cs_pin)
        return gpio_get_level(P02_0);
    else if (SPI3_CS1_P33_9 == cs_pin)
        return gpio_get_level(P33_9);
    else if (SPI3_CS2_P02_1 == cs_pin)
        return gpio_get_level(P02_1);
    else if (SPI3_CS2_P33_8 == cs_pin)
        return gpio_get_level(P33_8);
    else if (SPI3_CS3_P02_2 == cs_pin)
        return gpio_get_level(P02_2);
    else if (SPI3_CS4_P02_3 == cs_pin)
        return gpio_get_level(P02_3);
    else if (SPI3_CS5_P02_8 == cs_pin)
        return gpio_get_level(P02_8);
    else if (SPI3_CS6_P00_8 == cs_pin)
        return gpio_get_level(P00_8);
    else if (SPI3_CS7_P00_9 == cs_pin)
        return gpio_get_level(P00_9);
    else if (SPI3_CS7_P33_7 == cs_pin)
        return gpio_get_level(P33_7);
    else if (SPI3_CS8_P10_5 == cs_pin)
        return gpio_get_level(P10_5);
    else if (SPI3_CS11_P33_10 == cs_pin)
        return gpio_get_level(P33_10);
    else if (SPI3_CS12_P22_2 == cs_pin)
        return gpio_get_level(P22_2);
    else if (SPI3_CS13_P23_1 == cs_pin)
        return gpio_get_level(P23_1);
    else
    {
        zf_assert(FALSE);
        return 0; // �����������˵��CS���Ŵ���
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      16λSPI���ͽ��պ���(�û��Զ����,����AS5047��ȡ)
//-------------------------------------------------------------------------------------------------------------------
void spi_16_mosi(spi_index_enum spi_n, spi_cs_pin_enum cs_pin, uint16* modata, uint16* midata, uint8 continuous)
{
    uint32 i;
    Ifx_QSPI_BACON bacon;
    volatile Ifx_QSPI* moudle;

    moudle = IfxQspi_getAddress((IfxQspi_Index)spi_n);

    bacon.U = moudle->BACON.U;

    bacon.B.DL = 15;
    bacon.B.IDLE = 1;
    bacon.B.IPRE = 1;
    bacon.B.LEAD = 1;
    bacon.B.LPRE = 1;
    bacon.B.MSB = 1;
    bacon.B.PARTYP = 0;
    bacon.B.BYTE = 0;
    bacon.B.TRAIL = 1;
    bacon.B.TPRE = 1;
    bacon.B.CS = cs_pin % 102 / 6 - 3;

    if (continuous)
        IfxQspi_writeBasicConfigurationBeginStream(moudle, bacon.U); // �������ݺ�CS��������Ϊ��
    else
        IfxQspi_writeBasicConfigurationEndStream(moudle, bacon.U); // ÿ����һ���ֽ�CS�ź�����һ��

    if (midata)
    {
        // ��֮ǰfifo�е�����ȫ����ȡ����
        i = moudle->STATUS.B.RXFIFOLEVEL;
        while (i--)
        {
            (uint16)IfxQspi_readReceiveFifo(moudle);
        }
    }

    if (continuous)
        IfxQspi_writeBasicConfigurationEndStream(moudle, bacon.U);
    IfxQspi_writeTransmitFifo(moudle, *modata);
    while (moudle->STATUS.B.TXFIFOLEVEL != 0)
        ; // �ȴ��������

    if (midata)
    {
        while (moudle->STATUS.B.RXFIFOLEVEL == 0)
            ;
        *midata = (uint16)IfxQspi_readReceiveFifo(moudle);
    }

    while (!get_spi_cs_pin(cs_pin))
        ;
}

////-------------------------------------------------------------------------------------------------------------------
//  @brief      �ж�ʮ��λ�ĵ�������1�ĸ���
//  @param      cmd     �Ĵ�����ַ
//  @param      val     ��Ҫд�������
//  @return     0:ż�� 1:������
//  @since      none
////-------------------------------------------------------------------------------------------------------------------
uint16 parity(uint16_t x)
{
    uint16_t parity = 0;

    while (x != 0)
    {
        parity ^= x;
        x >>= 1;
    }

    return (parity & 0x1);
}

////-------------------------------------------------------------------------------------------------------------------
//  @brief      AS5047P SPIд�Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      val     ��Ҫд�������
//  @return     void
//  @since      none
////-------------------------------------------------------------------------------------------------------------------
void AS5047_W_Reg(uint16 cmd, uint16 val)
{
    if (parity(cmd & 0x3FFF) == 1)
        cmd |= 0x8000; // ����żУ��λ
    if (parity(val & 0x3FFF) == 1)
        val |= 0x8000; // ����żУ��λ
    spi_16_mosi(AS5047P_SPI, AS5047P_CS_PIN, &cmd, NULL, 1);
    spi_16_mosi(AS5047P_SPI, AS5047P_CS_PIN, &val, NULL, 1);
}
////-------------------------------------------------------------------------------------------------------------------
//  @brief      AS5047P SPI���Ĵ���
//  @param      cmd     �Ĵ�����ַ
//  @param      *val    �������ݵĵ�ַ
//  @return     void
//  @since      none
////-------------------------------------------------------------------------------------------------------------------
void AS5047_R_Reg(uint16 cmd, uint16* val)
{
    if (parity(cmd | AS5047P_SPI_R) == 1)
        cmd |= 0x8000; // ����żУ��λ
    cmd |= AS5047P_SPI_R;
    spi_16_mosi(AS5047P_SPI, AS5047P_CS_PIN, &cmd, val, 0);
    *val &= 0x3FFF;
}

////-------------------------------------------------------------------------------------------------------------------
//  @brief      ������λ�Ƕ�
//  @param      none
//  @return     void
//  @since      ����ʵ��������
////-------------------------------------------------------------------------------------------------------------------
void AS5047P_Set_Zero_Position(void)
{
    AS5047_W_Reg(AS5047P_ZPOSM, 0x0069); // 5B
    AS5047_W_Reg(AS5047P_ZPOSL, 0x003D); // 06
}
////-------------------------------------------------------------------------------------------------------------------
//  @brief      ����AS5047P
//  @param      none
//  @return     void
//  @since      ��������������ѯ�ֲ�
////-------------------------------------------------------------------------------------------------------------------
void AS5047P_Configuration(void)
{

    AS5047_W_Reg(AS5047P_SETTINGS1, 0x0021); // 1000101,����ABI�ӿ�
    AS5047_W_Reg(AS5047P_SETTINGS2, 0x0000);
    AS5047P_Set_Zero_Position();
}
////-------------------------------------------------------------------------------------------------------------------
////  @brief      AS5047P������ݴ�ӡ
////  @param      void
////  @return     void
////  @since      ����ʱ����main.cʹ�ã���ʹ��ʱ�ǵùر�
////-------------------------------------------------------------------------------------------------------------------
void as5047p_printf_test(void)
{
    uint16 val;
    AS5047_R_Reg(AS5047P_ANGLECOM, &val);
    val &= 0x3FFF;
    printf("%d\n", val);
}

void AS5047P_SPI_Init()
{
    spi_16_init(AS5047P_SPI, AS5047P_SPC_PIN, AS5047P_SDI_PIN, AS5047P_SDO_PIN, AS5047P_CS_PIN, 1, 10 * 1000 * 1000);
}

#if AS5047P_ABI_Part_EN
void AS5047P_ABI_MODE_Init(void)
{
    encoder_quad_init(AS5047P_TIM, AS5047P_B, AS5047P_A);
}
#endif

void as5047p_init(void)
{
    AS5047P_SPI_Init();
    AS5047P_Configuration();

#if AS5047P_ABI_Part_EN
    AS5047P_ABI_MODE_Init();
#endif
}


////-------------------------------------------------------------------------------------------------------------------
//  @brief      �õ�ת�ӽǶ�
//  @param      none
//  @return     ת�ӽǶȣ����ȣ�
//  @since      none
////-------------------------------------------------------------------------------------------------------------------
extern float data_send[32];

uint16 get_magnet_val()
{
    uint16 val;
    AS5047_R_Reg(AS5047P_ANGLECOM, &val);
    val &= 0x3FFF;

    return val;
}

double get_magnet_angle(uint16 val)
{
    double reval;
    val = val % 16384;
    reval = (double)val / 16384 * pi_2;
    reval -= zero_angle;
    if (reval < 0)
    {
        reval += pi_2;
    }
    return reval;
}

double angle_prev = 0.0;
int32 angle_rot_dat = 0;
int32 get_magnet_angle_rot(double reval)
{
    double d_angle = reval - angle_prev;
    if (fabs(d_angle) > (0.8f * pi_2)) angle_rot_dat += (d_angle > 0.f) ? -1 : 1;

    angle_prev = reval;
    return angle_rot_dat;
}

void reset_rotations()
{
    angle_rot_dat = 0;
}

double get_elec_angle(uint16 val)
{
    double reval;
    val = val % 2340;
    reval = (double)val / 2340 * pi_2;
    reval -= zero_reval;
    if (reval < 0)
    {
        reval += pi_2;
    }
    return reval;
}

// ��һ���Ƕ�
float _normalizeAngle(float angle)
{
    float a = fmod(angle, pi_2);
    return a >= 0 ? a : (a + pi_2);
}

double get_magnet_speed(double reval, int32 reval_rot, double reval_last, int32 reval_rot_last, uint16 T)
{
    return (double)(((reval_rot - reval_rot_last) * pi_2) + (reval - reval_last)) * T;
}

void set_zero_angle(float angle)
{
    zero_angle = angle;
}