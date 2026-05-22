#include "bh1750.h"
#include "Delay.h"

/* ???? - PB12/PB13/PB14 */
#define BH_SCL_PORT   GPIOB
#define BH_SCL_PIN    GPIO_Pin_12

#define BH_SDA_PORT   GPIOB
#define BH_SDA_PIN    GPIO_Pin_13

#define BH_ADDR_PORT  GPIOB
#define BH_ADDR_PIN   GPIO_Pin_14

#define BH_ADDR_WRITE  0x46
#define BH_ADDR_READ   0x47

static void bh_sda_out(void)
{
    GPIO_InitTypeDef g;
    g.GPIO_Pin   = BH_SDA_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BH_SDA_PORT, &g);
}

static void bh_sda_in(void)
{
    GPIO_InitTypeDef g;
    g.GPIO_Pin  = BH_SDA_PIN;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BH_SDA_PORT, &g);
}

static void bh_start(void)
{
    bh_sda_out();
    GPIO_SetBits(BH_SDA_PORT, BH_SDA_PIN);
    GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);
    Delay_us(5);
    GPIO_ResetBits(BH_SDA_PORT, BH_SDA_PIN);
    Delay_us(5);
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN);
}

static void bh_stop(void)
{
    bh_sda_out();
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN);   // ??? SCL ?
    GPIO_ResetBits(BH_SDA_PORT, BH_SDA_PIN);
    Delay_us(5);
    GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);
    Delay_us(5);
    GPIO_SetBits(BH_SDA_PORT, BH_SDA_PIN);
}

static u8 bh_wait_ack(void)
{
    u8 timeout = 0;
    bh_sda_in();
    GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);
    Delay_us(5);
    while (GPIO_ReadInputDataBit(BH_SDA_PORT, BH_SDA_PIN))
    {
        if (++timeout > 250)
        {
            bh_stop();
            return 1;
        }
        Delay_us(1);
    }
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN);
    return 0;
}

static void bh_ack(void)
{
    bh_sda_out();
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN);   // ??? SCL
    Delay_us(2);
    GPIO_ResetBits(BH_SDA_PORT, BH_SDA_PIN);    // SDA LOW = ACK
    Delay_us(2);
    GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);
    Delay_us(5);
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN);
}

static void bh_nack(void)
{
    bh_sda_out();
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN);   // ??? SCL
    Delay_us(2);
    GPIO_SetBits(BH_SDA_PORT, BH_SDA_PIN);      // SDA HIGH = NACK
    Delay_us(2);
    GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);
    Delay_us(5);
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN);
}

static void bh_write_byte(u8 data)
{
    u8 i;
    bh_sda_out();
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN);
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
            GPIO_SetBits(BH_SDA_PORT, BH_SDA_PIN);
        else
            GPIO_ResetBits(BH_SDA_PORT, BH_SDA_PIN);
        data <<= 1;
        Delay_us(2);
        GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);
        Delay_us(5);
        GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN);
        Delay_us(2);
    }
}

static u8 bh_read_byte(u8 ack)
{
    u8 i, data = 0;
    bh_sda_in();
    for (i = 0; i < 8; i++)
    {
        GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN);
        Delay_us(5);
        GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);
        data <<= 1;
        if (GPIO_ReadInputDataBit(BH_SDA_PORT, BH_SDA_PIN))
            data |= 0x01;
        Delay_us(5);
    }
    /* ack/nack ?????? SCL ??? SDA */
    if (ack) bh_ack();
    else     bh_nack();
    return data;
}

static void bh_cmd(u8 cmd)
{
    bh_start();
    bh_write_byte(BH_ADDR_WRITE);
    bh_wait_ack();
    bh_write_byte(cmd);
    bh_wait_ack();
    bh_stop();
}

void bh1750_init(void)
{
    GPIO_InitTypeDef g;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;

    g.GPIO_Pin = BH_SCL_PIN;
    GPIO_Init(BH_SCL_PORT, &g);

    g.GPIO_Pin = BH_SDA_PIN;
    GPIO_Init(BH_SDA_PORT, &g);

    g.GPIO_Pin = BH_ADDR_PIN;
    GPIO_Init(BH_ADDR_PORT, &g);
    GPIO_ResetBits(BH_ADDR_PORT, BH_ADDR_PIN);

    bh_cmd(BH1750_POWER_ON);
    bh_cmd(BH1750_POWER_RESET);
    bh_cmd(BH1750_CONTINUOUS_HIGH_RES);
    Delay_ms(180);
}

float bh1750_get_lux(void)
{
    u8 buf[2];
    u16 raw;

    bh_start();
    bh_write_byte(BH_ADDR_READ);
    bh_wait_ack();
    buf[0] = bh_read_byte(1);    // ???,? ACK
    buf[1] = bh_read_byte(0);    // ???,? NACK
    bh_stop();

    raw = ((u16)buf[0] << 8) | buf[1];
    return (float)raw / 1.2f;
}
