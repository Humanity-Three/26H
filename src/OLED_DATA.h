#ifndef __OLED_DATA_H
#define __OLED_DATA_H

#include <stdint.h>

/*中文字符字节宽度*/
#define OLED_CHN_CHAR_WIDTH			3		//UTF-8编码格式给3，GB2312编码格式给2

/*字模基本单元*/
typedef struct 
{
	char Index[OLED_CHN_CHAR_WIDTH + 1];	//汉字索引
	uint8_t Data[32];						//字模数据
} ChineseCell16x16_t;
typedef struct 
{
    char Index[OLED_CHN_CHAR_WIDTH + 1];   // 汉字索引
    uint8_t Data[24];                      // 字模数据，12*12的汉字需要24字节
} ChineseCell12x12_t;



/*ASCII字模数据声明*/
extern const uint8_t OLED_F8x16[][16];
extern const uint8_t OLED_F7x12[][14];
extern const uint8_t OLED_F6x8[][6];

/*汉字字模数据声明*/
extern const ChineseCell16x16_t OLED_CF16x16[];
extern const ChineseCell12x12_t OLED_CF12x12[];  // 声明12x12字模数组
/*图像数据声明*/
extern const uint8_t Diode[];
extern const uint8_t Battery[];
extern const uint8_t Win11Wallpaper[];
/*按照上面的格式，在这个位置加入新的图像数据声明*/
//...

#endif


/*****************江协科技|版权所有****************/
/*****************jiangxiekeji.com*****************/
