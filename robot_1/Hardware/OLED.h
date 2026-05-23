#ifndef __OLED_H
#define __OLED_H
/**
  * OLED_8X16：宽8像素，高16像素的字体
  * OLED_6X8：宽6像素，高8像素的字体
  */
#define OLED_8X16		8
#define OLED_6X8			6

/**
  * OLED_CHN_CHAR_WIDTH：汉字宽度（字节数）
  * UTF-8编码下，一个中文字符占用3个字节
  */
#define OLED_CHN_CHAR_WIDTH	3

/**
  * 结构体：汉字字模
  * Index：汉字内码（UTF-8编码，字符串形式）
  * Data：字模数据，16×16像素共32字节
  */
typedef struct {
	char Index[OLED_CHN_CHAR_WIDTH + 1];	// 汉字内码（末尾加'\0'）
	uint8_t Data[32];						// 16×16字模数据
} OLED_CF16x16_t;

/**
  * 初始化OLED屏幕（必须在使用前调用）
  */
void OLED_Init(void);

/**
  * 清空显存（调用OLED_Update后才真正清屏）
  */
void OLED_Clear(void);

/**
  * 将显存数据更新到屏幕
  * 调用所有显示函数后，需调用此函数才能看到效果
  */
void OLED_Update(void);

/**
  * 显示字符串
  *  X 左上角X坐标（0~127）
  * Y 左上角Y坐标（0~63）
  * String 要显示的字符串
  * FontSize 字体大小：OLED_8X16 或 OLED_6X8
  */
void OLED_ShowString(uint8_t X, uint8_t Y, char *String, uint8_t FontSize);

/**
  * 显示数字（十进制正整数）
  *  X 左上角X坐标（0~127）
  *  Y 左上角Y坐标（0~63）
  *  Number 要显示的数字（0~4294967295）
  *  Length 显示位数（不足时补前导零）
  *  FontSize 字体大小：OLED_8X16 或 OLED_6X8
  *  OLED_ShowNum(0, 0, 123, 3, OLED_8X16); // 显示"123"
  */
void OLED_ShowNum(uint8_t X, uint8_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize);

/**
  * 显示汉字串
  *  X 左上角X坐标（0~127）
  *  Y 左上角Y坐标（0~63）
  *  Chinese 要显示的汉字串（UTF-8编码）
  *  需要先在OLED_Data.c中定义对应的汉字字模
  *  OLED_ShowChinese(30, 0, "运动控制"); // 显示"运动控制"
  */
void OLED_ShowChinese(uint8_t X, uint8_t Y, char *Chinese);

/************************* 外部字模数据声明 *************************/

/**
  * 英文字体：8×16 ASCII字模库
  * 包含95个可见字符（空格~~）
  * 每个字符16字节
  */
extern const uint8_t OLED_F8x16[][16];

/**
  * 英文字体：6×8 ASCII字模库
  * 包含95个可见字符（空格~）
  * 每个字符6字节
  */
extern const uint8_t OLED_F6x8[][6];

/**
  * 中文字体：16×16汉字字模库
  * 需要在OLED_Data.c中定义需要显示的汉字
  */
extern const OLED_CF16x16_t OLED_CF16x16[];

#endif
