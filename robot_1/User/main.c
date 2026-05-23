#include "stm32f10x.h"                 
#include "OLED.h"
#include "Delay.h"
#include "Key.h"
#include "Serial.h"
#include "string.h"
#include <stdio.h>

//蓝牙控制程序以状态机为核心框架，菜单形式的结构。
//通过全局状态变量记录程序运行阶段，结合按键输入触发状态切换。
//OLED程序采用江协科技原始程序。


//硬件连接：
//OLED   SCL--PB8    SDA--PB9    
//HC05蓝牙模块   TX--PA10   RX--PA9
//可调电位器   OUT--PA0
//菜单按键  PA3,PA4,PA5,PA6       运动按键  PB0,PB1,PB10,PB11

//程序仅用于学习交流
//ID:我的STM32又烧了 抖音B站同名



// 系统状态枚举
typedef enum {
    SYSTEM_STARTUP = 0,  // 启动画面状态（上电初始）
    SYSTEM_RUNNING       // 正常运行状态（进入主页面后）
} SystemState_t;

// 全局系统状态变量
SystemState_t systemState = SYSTEM_STARTUP;

// 运动控制页面显示标记：-1=无S值显示，0=S4(前进)，1=S3(后退)，2=S2(左移)，3=S1(右移)
int8_t currentShowS = -1;

// 页面枚举
typedef enum {
    PAGE_MAIN = 0,         // 主页面
    PAGE_MOTION_CONTROL,   // 运动控制页面
    PAGE_COMMAND_CONTROL,  // 指令控制子页面
    PAGE_SYSTEM_INFO      // 系统信息页面
} Page_t;

// 子页面分页枚举
typedef enum {
    SUBPAGE_1 = 0,  // 子页面第一页
    SUBPAGE_2 = 1   // 子页面第二页
} SubPage_t;

// 全局页面状态变量
Page_t currentPage = PAGE_MAIN;    // 当前显示的页面
uint8_t cursorPos = 0;             // 光标位置
uint8_t mainCursorPos = 0;         // 记录主页面进入子页时的光标位置
SubPage_t subPage = SUBPAGE_1;     // 子页面分页索引

// 函数声明
void ShowStartupScreen(void);      // 显示上电启动画面
void ShowMainPage(void);           // 显示主页面
void ShowMotionControlPage(void);  // 显示运动控制页面
void ShowCommandControlPage(void); // 显示指令控制子页面
void ShowSystemInfoPage(void);     // 显示系统信息页面
void ProcessKey(uint8_t keyNum);   // 按键处理逻辑
void ShowSubPage(Page_t page);     // 子页面显示函数

// 显示启动画面
void ShowStartupScreen(void) {
    OLED_Clear();
    OLED_ShowString(5, 10, "STM32", OLED_8X16);
    OLED_ShowChinese(45, 10, "四足");
	OLED_ShowChinese(77, 10, "机器人");
    OLED_ShowString(30, 32, "2026.5.23", OLED_8X16);
    OLED_Update();
}

// 显示主页面
void ShowMainPage() {
    OLED_Clear();
    
    // 主页面四个选项
    OLED_ShowChinese(30, 0, "运动控制");
    OLED_ShowChinese(30, 16, "指令控制");
    OLED_ShowChinese(30, 32, "系统信息");
    
    // 显示光标
    switch(cursorPos) {
        case 0: OLED_ShowString(0, 0, "->", OLED_8X16); break;
        case 1: OLED_ShowString(0, 16, "->", OLED_8X16); break;
        case 2: OLED_ShowString(0, 32, "->", OLED_8X16); break;
    }
    
    OLED_Update();
}

//运动控制前进后退左移右移框架已经写好，需自行扩展，四足机器人那边也需要自行编写动作程序
//这里的按键是单独新增的四个按键。
//因为前后左右移动程序我也在调试中，所以如果你写好了记得发我一份，谢谢，嘻嘻。

void ShowMotionControlPage() {
    OLED_Clear();
    OLED_ShowChinese(28, 0, "运动控制");
    
    // 固定显示标签
    OLED_ShowChinese(0, 20, "前进:");
    OLED_ShowChinese(65, 20, "后退:");
    OLED_ShowChinese(0, 40, "左移:");
    OLED_ShowChinese(65, 40, "右移:");
    
    // 仅显示当前选中的S值（一次显示一个）
    switch(currentShowS) {
        case 0: OLED_ShowString(30, 20, ":S4", OLED_8X16); break;
        case 1: OLED_ShowString(95, 20, ":S3", OLED_8X16); break;
        case 2: OLED_ShowString(30, 40, ":S2", OLED_8X16); break;
        case 3: OLED_ShowString(95, 40, ":S1", OLED_8X16); break;
        default: break;
    }
    
    OLED_Update();
}

// 显示指令控制子页面
void ShowCommandControlPage() {
    OLED_Clear();
    OLED_ShowChinese(25, 0, "指令控制");
    OLED_ShowNum(90, 0, subPage + 1, 1, OLED_6X8);  // 显示当前分页（1/2）
    
    // 根据当前分页显示对应动作选项
    if (subPage == SUBPAGE_1) {
        OLED_ShowChinese(40, 16, "复位"); 
        OLED_ShowChinese(40, 32, "站立");
        OLED_ShowChinese(40, 48, "蹲下");
    } else {
        OLED_ShowChinese(40, 16, "摇摆");
        OLED_ShowChinese(40, 32, "招手");
        OLED_ShowChinese(40, 48, "抬腿");
    }
    
    // 显示当前光标位置（对应动作选项）
    OLED_ShowString(0, 16 + cursorPos * 16, "->", OLED_8X16);
    OLED_Update();
}

// 显示系统信息页面
void ShowSystemInfoPage() {
    OLED_Clear();
    OLED_ShowChinese(28, 0, "系统信息");
    OLED_ShowString(48, 20, "ID:", OLED_8X16);
    OLED_ShowChinese(16, 40, "四足");
	OLED_ShowChinese(48, 40, "机器人");
    OLED_Update();
}

// 按键处理
void ProcessKey(uint8_t keyNum) {
     if (keyNum == 0) return;  // 无按键按下，直接返回
    
    if (systemState == SYSTEM_STARTUP) {  // 启动状态下不处理其他按键
        return;
    }
    
    // 根据当前页面，执行不同的按键逻辑
    switch (currentPage) {
        case PAGE_MAIN:  // 主页面按键逻辑
            if (keyNum == 1) {  // 按键1：光标下移
                cursorPos = (cursorPos + 1) % 3;
                ShowMainPage();
            }
            else if (keyNum == 2) {  // 按键2：进入当前光标对应的子页面
                mainCursorPos = cursorPos;  // 记录主页面光标位置
                // 根据光标位置切换到对应子页面
                switch(cursorPos) {
                    case 0: currentPage = PAGE_MOTION_CONTROL; break;
                    case 1: currentPage = PAGE_COMMAND_CONTROL; break;
                    case 2: currentPage = PAGE_SYSTEM_INFO; break;
                }
                cursorPos = 0;       // 子页面光标重置为0
                subPage = SUBPAGE_1; // 子页面分页重置为第一页
                ShowSubPage(currentPage); // 显示目标子页面
            }
            break;
            
        case PAGE_MOTION_CONTROL:  // 运动控制页面按键逻辑
            switch(keyNum) {
                case 5: // 按键5：前进（显示S4，发送S4指令）
                    currentShowS = 0; 
                    Serial_SendString("@UP#*");
                    ShowMotionControlPage();
                    break;
                case 6: // 按键6：后退（显示S3，发送S3指令）
                    currentShowS = 1;
                    Serial_SendString("@BACK#*");
                    ShowMotionControlPage();
                    break;
                case 7: // 按键7：左移（显示S2，发送S2指令）
                    currentShowS = 2;
                    Serial_SendString("@LEFT#*");
                    ShowMotionControlPage();
                    break;
                case 8: // 按键8：右移（显示S1，发送S1指令）
                    currentShowS = 3;
                    Serial_SendString("@RIGHT#*");
                    ShowMotionControlPage();
                    break;
                case 3: // 按键3：退出（返回主页面）
                    currentShowS = -1;  // 重置S值显示标记
                    currentPage = PAGE_MAIN;
                    cursorPos = mainCursorPos;  // 恢复主页面光标位置
                    ShowMainPage();
                    break;
            }
            break;
            
        case PAGE_COMMAND_CONTROL:  // 指令控制页面按键逻辑
            if (keyNum == 1) {  // 按键1：光标下移
                cursorPos = (cursorPos + 1) % 3;
                if (cursorPos == 0) {  // 光标回到第0位时，切换分页
                    subPage = (SubPage_t)((subPage + 1) % 2);
                }
                ShowCommandControlPage();
            }
            else if (keyNum == 2) { /* 预留下一级 */ }
            else if (keyNum == 3) {  // 按键3：退出（返回主页面）
                currentPage = PAGE_MAIN;
                cursorPos = mainCursorPos;
                ShowMainPage();
            }
            else if (keyNum == 4) {  // 按键4：发送当前光标对应的动作指令
                if (subPage == SUBPAGE_1) {
                    switch(cursorPos) {
                        case 0: Serial_SendString("@REST#*"); break; // 复位
                        case 1: Serial_SendString("@ZK#*"); break;   // 站立
                        case 2: Serial_SendString("@ZX#*"); break;   // 蹲下
                    }
                } else {
                    switch(cursorPos) {
                        case 0: Serial_SendString("@YB#*"); break;   // 摇摆
                        case 1: Serial_SendString("@ZS#*"); break;   // 招手
                        case 2: Serial_SendString("@TJ#*"); break;   // 抬腿
                    }
                }
            }
            break;
            
            
        case PAGE_SYSTEM_INFO:  // 系统信息页面按键逻辑
            if (keyNum == 3) {  // 按键3：退出（返回主页面）
                currentPage = PAGE_MAIN;
                cursorPos = mainCursorPos;
                ShowMainPage();
            }
            break;
    }
}

// 显示子页面
void ShowSubPage(Page_t page) {
    switch (page) {
        case PAGE_MOTION_CONTROL: ShowMotionControlPage(); break;
        case PAGE_COMMAND_CONTROL: ShowCommandControlPage(); break;
        case PAGE_SYSTEM_INFO: ShowSystemInfoPage(); break;
        default: break;
    }
}

// 主函数
int main(void) {
    
    OLED_Init();
    Key_Init();
    Serial_Init();  
    
    // 上电后显示启动画面，并设置系统状态为启动状态
    ShowStartupScreen();  
    systemState = SYSTEM_STARTUP;
    
   //这个程序菜单启动画面，只要按下按键后才进入控制界面，可以自定义菜单界面内容，纯花哨程序
    while (systemState == SYSTEM_STARTUP) {
     
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == 0) {
            Delay_ms(20);  
            if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == 0) {
               
                systemState = SYSTEM_RUNNING;
                currentPage = PAGE_MAIN;
                cursorPos = 0;
                ShowMainPage();
            
                while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == 0);
                break;
            }
        }
        Delay_ms(10);  
    }
    
   
    while (1) {
        Key_Scan();                  // 按键扫描（非阻塞）
        uint8_t keyNum = Key_GetNum();// 获取按键值
        ProcessKey(keyNum);          // 处理按键逻辑       
    }
}
