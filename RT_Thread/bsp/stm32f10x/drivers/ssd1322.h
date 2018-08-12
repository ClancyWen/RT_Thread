#ifndef SSD1322_H_INCLUDED
#define SSD1322_H_INCLUDED

#include <rtthread.h>

// Compatible list:
// ssd1289

/* LCD color */


/*---------------------- Graphic LCD size definitions ------------------------*/
#define OLED_WIDTH       256                 /* Screen Width (in pixels)           */
#define OLED_HEIGHT      64                 /* Screen Hight (in pixels)           */
#define BPP             16                  /* Bits per pixel                     */
#define BYPP            ((BPP+7)/8)         /* Bytes per pixel                    */


extern rt_uint8_t char_1[];
extern rt_uint8_t char_2[];
extern rt_uint8_t char_3[];
extern rt_uint8_t oled_buff[OLED_WIDTH * OLED_HEIGHT /2];
extern struct rt_device _oled_device;
//#define _ILI_REVERSE_DIRECTION_

rt_size_t oled_ssd1322_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size);
rt_size_t oled_ssd1322_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size);

void ssd1322_oled_set_pixel(rt_uint8_t pixel, rt_uint32_t x, rt_uint32_t y);
void ssd1322_oled_draw_hline(rt_uint8_t pixel, rt_uint32_t x_start, 
	                           rt_uint32_t x_end, rt_uint32_t y);
void ssd1322_oled_draw_vline(rt_uint8_t pixel, rt_uint32_t x, 
	                           rt_uint32_t y_start, rt_uint32_t y_end);
void ssd1322_oled_draw_block(rt_uint8_t pixel, rt_uint32_t x_start, rt_uint32_t x_end, 
	                           rt_uint32_t y_start, rt_uint32_t y_end);
void ssd1322_oled_draw_frame(rt_uint8_t pixel, rt_uint32_t x_start,
                             rt_uint32_t x_end, rt_uint32_t y_start, rt_uint32_t y_end);
void ssd1322_oled_draw_chinese(rt_uint8_t pixel, const rt_uint8_t * chr, rt_uint32_t x, rt_uint32_t y);

void ssd1322_oled_draw_char(rt_uint8_t pixel, const rt_uint8_t * chr, rt_uint32_t x, rt_uint32_t y);

void ssd1322_oled_draw_picture(rt_uint8_t pixel, const rt_uint8_t * chr, 
	                            rt_uint32_t x, rt_uint32_t y,
														  rt_uint32_t pixel_x, rt_uint32_t pixel_y);

#endif // SSD1322_H_INCLUDED
