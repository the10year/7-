/**
  * 函 数 库：OLED显示屏驱动（精简版）
  * 说 明：仅保留主程序实际使用的函数
  * 使 用：STM32F10x + I2C接口（PB8-SCL, PB9-SDA）
  */

#include "stm32f10x.h"
#include "OLED.h"
#include <string.h>

/* 显存数组，8行×128列，每个字节代表8个像素点 */
uint8_t OLED_DisplayBuf[8][128];

/************************* I2C通信底层函数 *************************/

/**
  * 函    数：OLED写SCL高低电平
  * 参    数：BitValue 电平值，0=低电平，1=高电平
  * 返 回 值：无
  */
void OLED_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)BitValue);
}

/**
  * 函    数：OLED写SDA高低电平
  * 参    数：BitValue 电平值，0=低电平，1=高电平
  * 返 回 值：无
  */
void OLED_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)BitValue);
}

/**
  * 函    数：OLED引脚初始化
  * 参    数：无
  * 返 回 值：无
  * 说 明：将SCL(PB8)和SDA(PB9)初始化为开漏输出模式
  */
void OLED_GPIO_Init(void)
{
	uint32_t i, j;
	
	/* 延时等待OLED供电稳定 */
	for (i = 0; i < 1000; i ++)
	{
		for (j = 0; j < 1000; j ++);
	}
	
	/* 使能GPIOB时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* 配置GPIO为开漏输出模式 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	/* 配置SCL引脚(PB8) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 配置SDA引脚(PB9) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 释放总线，初始化为高电平 */
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * 函    数：I2C起始信号
  * 参    数：无
  * 返 回 值：无
  * 说 明：SCL高电平时，SDA从高拉低，产生起始信号
  */
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);	// 先释放SDA
	OLED_W_SCL(1);	// 释放SCL
	OLED_W_SDA(0);	// SCA从高拉低，产生起始信号
	OLED_W_SCL(0);	// 拉低SCL占用总线
}

/**
  * 函    数：I2C终止信号
  * 参    数：无
  * 返 回 值：无
  * 说 明：SCL高电平时，SDA从低拉高，产生终止信号
  */
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);	// 确保SDA为低
	OLED_W_SCL(1);	// 释放SCL
	OLED_W_SDA(1);	// SDA从低拉高，产生终止信号
}

/**
  * 函    数：I2C发送一个字节
  * 参    数：Byte 要发送的数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	
	/* 循环8次，逐位发送数据 */
	for (i = 0; i < 8; i++)
	{
		/* 取出最高位写入SDA（!!将非零值转换为1） */
		OLED_W_SDA(!!(Byte & (0x80 >> i)));
		OLED_W_SCL(1);	// 产生时钟，从机读取数据
		OLED_W_SCL(0);	// 拉低时钟，准备发送下一位
	}
	
	/* 额外时钟周期，忽略从机的应答信号 */
	OLED_W_SCL(1);
	OLED_W_SCL(0);
}

/**
  * 函    数：OLED写命令
  * 参    数：Command 命令值，范围：0x00~0xFF
  * 返 回 值：无
  */
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();				// 起始信号
	OLED_I2C_SendByte(0x78);		// 发送OLED从机地址（写）
	OLED_I2C_SendByte(0x00);		// 控制字节：0x00表示写命令
	OLED_I2C_SendByte(Command);		// 发送命令
	OLED_I2C_Stop();				// 终止信号
}

/**
  * 函    数：OLED写数据
  * 参    数：Data 数据数组指针
  * 参    数：Count 要发送的数据个数
  * 返 回 值：无
  */
void OLED_WriteData(uint8_t *Data, uint8_t Count)
{
	uint8_t i;
	
	OLED_I2C_Start();				// 起始信号
	OLED_I2C_SendByte(0x78);		// 发送OLED从机地址（写）
	OLED_I2C_SendByte(0x40);		// 控制字节：0x40表示写数据
	for (i = 0; i < Count; i ++)
	{
		OLED_I2C_SendByte(Data[i]);	// 逐个发送数据
	}
	OLED_I2C_Stop();				// 终止信号
}

/************************* 硬件配置函数 *************************/

/**
  * 函    数：OLED初始化
  * 参    数：无
  * 返 回 值：无
  * 说 明：使用前必须调用此函数
  */
void OLED_Init(void)
{
	OLED_GPIO_Init();			// 初始化GPIO
	
	/* OLED驱动芯片SSD1306初始化命令序列 */
	OLED_WriteCommand(0xAE);	// 关闭显示
	OLED_WriteCommand(0xD5);	// 设置显示时钟分频比
	OLED_WriteCommand(0x80);	// 建议值0x80
	OLED_WriteCommand(0xA8);	// 设置多路复用率
	OLED_WriteCommand(0x3F);	// 64行
	OLED_WriteCommand(0xD3);	// 设置显示偏移
	OLED_WriteCommand(0x00);	// 无偏移
	OLED_WriteCommand(0x40);	// 设置显示开始行
	OLED_WriteCommand(0xA1);	// 设置段复用（列地址映射）
	OLED_WriteCommand(0xC8);	// 设置COM输出方向
	OLED_WriteCommand(0xDA);	// 设置COM引脚硬件配置
	OLED_WriteCommand(0x12);
	OLED_WriteCommand(0x81);	// 设置对比度
	OLED_WriteCommand(0xCF);	// 对比度值
	OLED_WriteCommand(0xD9);	// 设置预充电周期
	OLED_WriteCommand(0xF1);
	OLED_WriteCommand(0xDB);	// 设置VCOMH电压
	OLED_WriteCommand(0x30);
	OLED_WriteCommand(0xA4);	// 设置显示内容（0xA4=正常显示）
	OLED_WriteCommand(0xA6);	// 设置正常/反色显示（0xA6=正常）
	OLED_WriteCommand(0x8D);	// 设置充电泵
	OLED_WriteCommand(0x14);	// 启用充电泵
	OLED_WriteCommand(0xAF);	// 开启显示
}

/**
  * 函    数：设置光标位置
  * 参    数：Page 页地址，范围：0~7（每页8个像素高度）
  * 参    数：X 列地址，范围：0~127
  * 返 回 值：无
  * 说 明：X坐标+2是为适配1.3寸OLED的偏移
  */
void OLED_SetCursor(uint8_t Page, uint8_t X)
{
	X += 2;	// 1.3寸OLED起始列偏移2列
	
	OLED_WriteCommand(0xB0 | Page);					// 设置页地址
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	// 设置列地址高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			// 设置列地址低4位
}

/************************* 工具函数 *************************/

/**
  * 函    数：次方函数
  * 参    数：X 底数
  * 参    数：Y 指数
  * 返 回 值：X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

/************************* 核心显示函数 *************************/

/**
  * 函    数：更新显存到屏幕
  * 参    数：无
  * 返 回 值：无
  * 说 明：所有显示函数只修改显存，需调用此函数才真正显示
  */
void OLED_Update(void)
{
	uint8_t j;
	
	/* 遍历8页（每页8像素高，共64像素） */
	for (j = 0; j < 8; j ++)
	{
		OLED_SetCursor(j, 0);				// 设置光标到当前页起始列
		OLED_WriteData(OLED_DisplayBuf[j], 128);	// 写入整页数据
	}
}

/**
  * 函    数：清空显存
  * 参    数：无
  * 返 回 值：无
  * 说 明：调用后需调用OLED_Update()才能清屏
  */
void OLED_Clear(void)
{
	uint8_t i, j;
	
	for (j = 0; j < 8; j ++)		// 遍历8页
	{
		for (i = 0; i < 128; i ++)	// 遍历128列
		{
			OLED_DisplayBuf[j][i] = 0x00;	// 全部清零
		}
	}
}

/**
  * 函    数：显示图像到显存
  * 参    数：X 左上角X坐标，范围：0~127
  * 参    数：Y 左上角Y坐标，范围：0~63
  * 参    数：Width 图像宽度（像素）
  * 参    数：Height 图像高度（像素）
  * 参    数：Image 图像数据数组
  * 返 回 值：无
  */
void OLED_ShowImage(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
	uint8_t i, j;
	
	/* 边界检查 */
	if (X > 127) {return;}
	if (Y > 63) {return;}
	
	/* 遍历图像涉及的所有页 */
	for (j = 0; j < (Height - 1) / 8 + 1; j ++)
	{
		for (i = 0; i < Width; i ++)
		{
			/* 边界检查 */
			if (X + i > 127) {break;}
			if (Y / 8 + j > 7) {return;}
			
			/* 写入当前页的数据 */
			OLED_DisplayBuf[Y / 8 + j][X + i] |= Image[j * Width + i] << (Y % 8);
			
			/* 如果图像跨越两页，写入下一页数据 */
			if (Y / 8 + j + 1 > 7) {continue;}
			OLED_DisplayBuf[Y / 8 + j + 1][X + i] |= Image[j * Width + i] >> (8 - Y % 8);
		}
	}
}

/**
  * 函    数：显示一个字符
  * 参    数：X 左上角X坐标
  * 参    数：Y 左上角Y坐标
  * 参    数：Char 要显示的字符
  * 参    数：FontSize 字体大小：OLED_8X16 或 OLED_6X8
  * 返 回 值：无
  */
void OLED_ShowChar(uint8_t X, uint8_t Y, char Char, uint8_t FontSize)
{
	if (FontSize == OLED_8X16)		// 8×16字体
	{
		OLED_ShowImage(X, Y, 8, 16, OLED_F8x16[Char - ' ']);
	}
	else if(FontSize == OLED_6X8)	// 6×8字体
	{
		OLED_ShowImage(X, Y, 6, 8, OLED_F6x8[Char - ' ']);
	}
}

/**
  * 函    数：显示字符串
  * 参    数：X 左上角X坐标
  * 参    数：Y 左上角Y坐标
  * 参    数：String 要显示的字符串
  * 参    数：FontSize 字体大小：OLED_8X16 或 OLED_6X8
  * 返 回 值：无
  */
void OLED_ShowString(uint8_t X, uint8_t Y, char *String, uint8_t FontSize)
{
	uint8_t i;
	
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(X + i * FontSize, Y, String[i], FontSize);
	}
}

/**
  * 函    数：显示数字（十进制正整数）
  * 参    数：X 左上角X坐标
  * 参    数：Y 左上角Y坐标
  * 参    数：Number 要显示的数字，范围：0~4294967295
  * 参    数：Length 显示位数（不足补前导零）
  * 参    数：FontSize 字体大小：OLED_8X16 或 OLED_6X8
  * 返 回 值：无
  */
void OLED_ShowNum(uint8_t X, uint8_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
	uint8_t i;
	
	for (i = 0; i < Length; i++)
	{
		/* 逐位提取数字并显示 */
		OLED_ShowChar(X + i * FontSize, Y, 
			Number / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
	}
}

/**
  * 函    数：显示汉字串
  * 参    数：X 左上角X坐标
  * 参    数：Y 左上角Y坐标
  * 参    数：Chinese 要显示的汉字串（UTF-8编码）
  * 返 回 值：无
  * 说 明：汉字字模需在OLED_Data.c中定义
  */
void OLED_ShowChinese(uint8_t X, uint8_t Y, char *Chinese)
{
	uint8_t pChinese = 0;			// 当前汉字字符计数
	uint8_t pIndex;					// 字模索引
	uint8_t i;						// 循环变量
	char SingleChinese[OLED_CHN_CHAR_WIDTH + 1] = {0};	// 单个汉字缓冲区
	
	for (i = 0; Chinese[i] != '\0'; i ++)
	{
		SingleChinese[pChinese] = Chinese[i];	// 收集汉字字符
		pChinese ++;
		
		/* 收集满一个汉字（UTF-8下3个字节） */
		if (pChinese >= OLED_CHN_CHAR_WIDTH)
		{
			pChinese = 0;
			
			/* 在字模库中查找匹配的汉字 */
			for (pIndex = 0; strcmp(OLED_CF16x16[pIndex].Index, "") != 0; pIndex ++)
			{
				if (strcmp(OLED_CF16x16[pIndex].Index, SingleChinese) == 0)
				{
					break;	// 找到匹配的汉字
				}
			}
			
			/* 显示找到的汉字（16×16像素） */
			OLED_ShowImage(X + ((i + 1) / OLED_CHN_CHAR_WIDTH - 1) * 16, 
				Y, 16, 16, OLED_CF16x16[pIndex].Data);
		}
	}
}
