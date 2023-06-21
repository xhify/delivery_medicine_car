#pragma once

#include "Basic.hpp"

//超时时间
#define TIMEOUT 2.0*configTICK_RATE_HZ
	
/*RGB565颜色*/
#define  BLACK	0x0000			// 黑色
#define  NAVY	0x000F				// 深蓝色
#define  DGREEN  0x03E0     // 深绿色
#define  DCYAN   0x03EF     // 深青色
#define  MAROON  0x7800     // 深红色
#define  PURPLE  0x780F     // 紫色
#define  OLIVE   0x7BE0     // 橄榄绿
#define  LGRAY   0xC618     // 灰白色
#define  DGRAY   0x7BEF     // 深灰色
#define  BLUE   0x001F      // 蓝色
#define  GREEN   0x07E0     // 绿色
#define  CYAN   0x07FF      // 青色
#define  RED    0xF800      // 红色
#define  MAGENTA  0xF81F    // 品红
#define  YELLOW  0xFFE0     // 黄色
#define  WHITE   0xFFFF     // 白色
/*RGB565颜色*/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);//指定区域填充颜色
void LCD_DrawPoint(u16 x,u16 y,u16 color);//在指定位置画一个点
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color);//在指定位置画一条线
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);//在指定位置画一个矩形
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color);//在指定位置画一个圆

//中文显示有问题，为c++ c 编译器的冲突
//void LCD_ShowChinese(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);//显示汉字串
//void LCD_ShowChinese12x12(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);//显示单个12x12汉字
//void LCD_ShowChinese16x16(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);//显示单个16x16汉字
//void LCD_ShowChinese24x24(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);//显示单个24x24汉字
//void LCD_ShowChinese32x32(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode);//显示单个32x32汉字

void LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode);//显示一个字符
void LCD_ShowString(u16 x,u16 y,const char *p,u16 fc,u16 bc,u8 sizey,u8 mode);//显示字符串
u32 mypow(u8 m,u8 n);//求幂
void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey);//显示整数变量
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey);//显示两位小数变量
void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[]);//显示图片
void init_GUI();

extern int picture_id;