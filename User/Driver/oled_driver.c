#include "oled_driver.h"

/**
 * @brief	使用类似printf的方式显示字符串，显示6x8大小的ASCII字符
 * @param x  String start position on the X-axis  range：0 - 127
 * @param y  String start position on the Y-axis  range：0 - 3
 * @param {const char *} *format 待显示的格式化字符串
 * 例如：Oled_Printf(0, 0, 12, 0, "Data = %d", dat);
**/
int Oled_Printf(uint8_t x, uint8_t y, const char *format, ...)
{
	char buffer[128]; // 临时存储格式化后的字符串
	va_list arg;      // 处理可变参数
	int len;          // 最终字符串长度

	va_start(arg, format);
	// 安全地格式化字符串到 buffer
	len = vsnprintf(buffer, sizeof(buffer), format, arg);
	va_end(arg);

	OLED_ShowStr(x, y, buffer, 8);

	return len;
}

void menu_show_string(unsigned char line, char* str)
{
  OLED_ShowStr(0, line, str, 8);
}
