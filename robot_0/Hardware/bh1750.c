/**
 * ============================================================================
 * BH1750光照传感器驱动
 * ============================================================================
 * @brief 基于软件模拟I2C的BH1750光照强度传感器驱动
 * @details BH1750是一款数字型光照传感器，测量范围0-65535 lux
 *          采用I2C接口通信，地址可配置（本驱动使用地址0x46/0x47）
 * @author 机器人控制项目组
 * @date 2026-05-25
 * @version v1.1
 */

#include "stm32f10x_adc.h"
#include "bh1750.h"
#include "Delay.h"

/* ==================== 硬件引脚定义 ==================== */

/** @brief SCL时钟线端口 */
#define BH_SCL_PORT   GPIOB
/** @brief SCL时钟线引脚 */
#define BH_SCL_PIN    GPIO_Pin_12

/** @brief SDA数据线端口 */
#define BH_SDA_PORT   GPIOB
/** @brief SDA数据线引脚 */
#define BH_SDA_PIN    GPIO_Pin_13

/** @brief ADDR地址选择端口 */
#define BH_ADDR_PORT  GPIOB
/** @brief ADDR地址选择引脚 */
#define BH_ADDR_PIN   GPIO_Pin_14

/** @brief BH1750写地址（ADDR接地时） */
#define BH_ADDR_WRITE  0x46
/** @brief BH1750读地址（ADDR接地时） */
#define BH_ADDR_READ   0x47

/* ==================== I2C通信内部函数 ==================== */

/**
 * @brief 设置SDA为输出模式
 * @details 将SDA引脚配置为推挽输出，用于发送数据
 */
static void bh_sda_out(void)
{
    GPIO_InitTypeDef g;
    g.GPIO_Pin   = BH_SDA_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BH_SDA_PORT, &g);
}

/**
 * @brief 设置SDA为输入模式
 * @details 将SDA引脚配置为浮空输入，用于接收数据
 */
static void bh_sda_in(void)
{
    GPIO_InitTypeDef g;
    g.GPIO_Pin  = BH_SDA_PIN;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(BH_SDA_PORT, &g);
}

/**
 * @brief 发送I2C起始信号
 * @details 起始条件：SCL为高电平时，SDA由高变低
 */
static void bh_start(void)
{
    bh_sda_out();                        /* SDA设为输出 */
    GPIO_SetBits(BH_SDA_PORT, BH_SDA_PIN);  /* SDA = 1 */
    GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);  /* SCL = 1 */
    Delay_us(5);                         /* 延时确保稳定 */
    GPIO_ResetBits(BH_SDA_PORT, BH_SDA_PIN); /* SDA由1变0 → 起始条件 */
    Delay_us(5);                         /* 延时确保稳定 */
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN); /* SCL = 0 */
}

/**
 * @brief 发送I2C停止信号
 * @details 停止条件：SCL为高电平时，SDA由低变高
 */
static void bh_stop(void)
{
    bh_sda_out();                        /* SDA设为输出 */
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN); /* SCL = 0 */
    GPIO_ResetBits(BH_SDA_PORT, BH_SDA_PIN); /* SDA = 0 */
    Delay_us(5);                         /* 延时确保稳定 */
    GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);   /* SCL = 1 */
    Delay_us(5);                         /* 延时确保稳定 */
    GPIO_SetBits(BH_SDA_PORT, BH_SDA_PIN);   /* SDA由0变1 → 停止条件 */
}

/**
 * @brief 等待从设备ACK应答
 * @return 0-成功接收到ACK，1-超时未收到ACK
 * @details 在SCL高电平期间检测SDA是否为低电平
 */
static u8 bh_wait_ack(void)
{
    u8 timeout = 0;
    bh_sda_in();                         /* SDA设为输入 */
    GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);   /* SCL = 1 */
    Delay_us(5);                         /* 延时等待应答 */
    
    /* 等待SDA变为低电平（ACK） */
    while (GPIO_ReadInputDataBit(BH_SDA_PORT, BH_SDA_PIN))
    {
        if (++timeout > 250)            /* 超时检测 */
        {
            bh_stop();                   /* 超时未收到ACK，发送停止信号 */
            return 1;
        }
        Delay_us(1);
    }
    
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN); /* SCL = 0 */
    return 0;
}

/**
 * @brief 向从设备写入一个字节
 * @param data 要发送的字节数据
 * @details 采用标准I2C写时序，高位在前
 */
static void bh_write_byte(u8 data)
{
    u8 i;
    bh_sda_out();                        /* SDA设为输出 */
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN); /* SCL = 0 */
    
    for (i = 0; i < 8; i++)             /* 逐位发送 */
    {
        if (data & 0x80)                /* 判断最高位 */
            GPIO_SetBits(BH_SDA_PORT, BH_SDA_PIN);
        else
            GPIO_ResetBits(BH_SDA_PORT, BH_SDA_PIN);
        
        data <<= 1;                     /* 左移一位 */
        Delay_us(2);                    /* 延时确保稳定 */
        GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN); /* SCL = 1 */
        Delay_us(5);                    /* 保持高电平 */
        GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN); /* SCL = 0 */
        Delay_us(2);                    /* 延时确保稳定 */
    }
}

/**
 * @brief 从设备读取一个字节
 * @param ack 是否发送ACK：1-发送ACK，0-发送NACK
 * @return 读取到的字节数据
 * @details 采用标准I2C读时序，高位在前
 */
static u8 bh_read_byte(u8 ack)
{
    u8 i, data = 0;
    bh_sda_in();                        /* SDA设为输入 */
    
    for (i = 0; i < 8; i++)             /* 逐位接收 */
    {
        GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN); /* SCL = 0 */
        Delay_us(5);                    /* 延时等待数据稳定 */
        GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);   /* SCL = 1 */
        
        data <<= 1;                     /* 左移一位 */
        if (GPIO_ReadInputDataBit(BH_SDA_PORT, BH_SDA_PIN))
            data |= 0x01;               /* 读取当前位 */
        
        Delay_us(5);                    /* 保持高电平 */
    }
    
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN); /* SCL = 0 */
    Delay_us(2);
    
    bh_sda_out();                        /* SDA设为输出，准备发送ACK/NACK */
    Delay_us(2);
    
    if (ack)
    {
        GPIO_ResetBits(BH_SDA_PORT, BH_SDA_PIN); /* ACK: SDA = 0 */
    }
    else
    {
        GPIO_SetBits(BH_SDA_PORT, BH_SDA_PIN);   /* NACK: SDA = 1 */
    }
    
    Delay_us(2);
    GPIO_SetBits(BH_SCL_PORT, BH_SCL_PIN);       /* SCL = 1 */
    Delay_us(5);
    GPIO_ResetBits(BH_SCL_PORT, BH_SCL_PIN);     /* SCL = 0 */
    
    return data;
}

/**
 * @brief 向BH1750发送命令
 * @param cmd 命令字节
 * @details 发送起始信号→发送写地址→等待ACK→发送命令→等待ACK→发送停止信号
 */
static void bh_cmd(u8 cmd)
{
    bh_start();
    bh_write_byte(BH_ADDR_WRITE);
    bh_wait_ack();
    bh_write_byte(cmd);
    bh_wait_ack();
    bh_stop();
}

/* ==================== 公共API函数 ==================== */

/**
 * @brief BH1750传感器初始化
 * @details 配置GPIO引脚，发送初始化命令，进入连续高分辨率模式
 */
void bh1750_init(void)
{
    GPIO_InitTypeDef g;

    /* 使能GPIOB时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* 配置SCL、SDA、ADDR为推挽输出 */
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;

    g.GPIO_Pin = BH_SCL_PIN;
    GPIO_Init(BH_SCL_PORT, &g);

    g.GPIO_Pin = BH_SDA_PIN;
    GPIO_Init(BH_SDA_PORT, &g);

    g.GPIO_Pin = BH_ADDR_PIN;
    GPIO_Init(BH_ADDR_PORT, &g);
    GPIO_ResetBits(BH_ADDR_PORT, BH_ADDR_PIN);  /* ADDR接地，选择地址0x46 */

    /* 发送初始化命令序列 */
    bh_cmd(BH1750_POWER_ON);             /* 上电命令 */
    bh_cmd(BH1750_POWER_RESET);          /* 复位命令 */
    bh_cmd(BH1750_CONTINUOUS_HIGH_RES);  /* 连续高分辨率模式 */
    
    Delay_ms(180);                       /* 等待首次测量完成（典型120ms，留有余量） */
}

/**
 * @brief 获取光照强度值
 * @return 光照强度值（单位：lux）
 * @details 读取传感器输出的16位数据，转换为lux值
 */
float bh1750_get_lux(void)
{
    u8 buf[2];                           /* 存储高字节和低字节 */
    u16 raw;                             /* 原始16位数据 */

    /* 发送读取命令 */
    bh_start();
    bh_write_byte(BH_ADDR_READ);         /* 发送读地址 */
    bh_wait_ack();
    
    buf[0] = bh_read_byte(1);            /* 读取高字节，发送ACK */
    buf[1] = bh_read_byte(0);            /* 读取低字节，发送NACK */
    
    bh_stop();                           /* 发送停止信号 */

    /* 组合为16位原始值并转换为lux */
    raw = ((u16)buf[0] << 8) | buf[1];
    return (float)raw / 1.2f;            /* BH1750转换公式：lux = raw / 1.2 */
}
