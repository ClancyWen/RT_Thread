/**********************************************************************************************************
*
*	模块名称 : TFT驱动模块
*	文件名称 : ssd1289.c
*	版    本 : V1.0
*	说    明 :
*	修改记录 :
*		版本号  日期        作者                       说明
*
*		v1.0    2013-09-8  jiezhi320(UP MCU工作室)    根据rtt的原版进行了改动
*                                                      支持：ssd1289
*
*	Copyright (C),
*   淘宝店：   http://shop73275611.taobao.com
*   QQ交流群： 258043068
*
**********************************************************************************************************/
#include "ssd1289.h"
#include "stm32f10x.h"
#include "board.h"

#include "rtthread.h"
#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#endif
//竖屏  不需  _ILI_REVERSE_DIRECTION_
//横屏  需要  _ILI_REVERSE_DIRECTION_

//#define _ILI_REVERSE_DIRECTION_

//输出重定向
#define printf               rt_kprintf //使用rt_kprintf来输出
//#define printf(...)

/* LCD is connected to the FSMC_Bank1_NOR/SRAM4 and NE4 is used as ship select signal */
/* RS <==> A0 */
#define LCD_REG              (*((volatile unsigned short *) 0x64000000)) /* RS = 0 */
#define LCD_RAM              (*((volatile unsigned short *) 0x64000002)) /* RS = 1 */
#define NEX_SELECT            FSMC_Bank1_NORSRAM2


//内联函数定义,用以提高性能
#ifdef __CC_ARM                			 /* ARM Compiler 	*/
#define lcd_inline   				static __inline
#elif defined (__ICCARM__)        		/* for IAR Compiler */
#define lcd_inline 					inline
#elif defined (__GNUC__)        		/* GNU GCC Compiler */
#define lcd_inline 					static __inline
#else
#define lcd_inline                  static
#endif



#define rw_data_prepare()               write_cmd(34)

static rt_uint16_t deviceid;     //设置一个静态变量用来保存LCD的ID
struct rt_device _lcd_device;	 //设备框架结构体  RTGRAPHIC_PIXEL_FORMAT_RGB565

static void delay(rt_uint32_t cnt)
{
    volatile rt_uint32_t dl;
    while(cnt--)
    {
        for(dl=0; dl<500; dl++);
    }
}
/* 总线配置*/
static void LCD_FSMCConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  Timing_read,Timing_write;;

    /* Enable FSMC, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |
                           RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);

    /* Set PD.00(D2), PD.01(D3), PD.04(NOE), PD.05(NWE), PD.08(D13), PD.09(D14),
       PD.10(D15), PD.14(D0), PD.15(D1) as alternate
       function push pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |
                                  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Set PE.07(D4), PE.08(D5), PE.09(D6), PE.10(D7), PE.11(D8), PE.12(D9), PE.13(D10),
       PE.14(D11), PE.15(D12) as alternate function push pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
                                  GPIO_Pin_15;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* Set PF.00(A0 (RS)) as alternate function push pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    /* Set PG.9(NE2 (LCD/CS)) as alternate function push pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    /* Set PA.8 打开背光 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);

    /*-- FSMC Configuration -------------------------------------------------*/
    Timing_read.FSMC_AddressSetupTime = 12;	 //地址建立时间（ADDSET）为1个HCLK
    Timing_read.FSMC_AddressHoldTime = 0x00;	 //地址保持时间（A
    Timing_read.FSMC_DataSetupTime = 23;		 ////数据保存时间为4个HCLK
    Timing_read.FSMC_BusTurnAroundDuration = 0x00;
    Timing_read.FSMC_CLKDivision = 0x00;
    Timing_read.FSMC_DataLatency = 0x00;
    Timing_read.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A

    Timing_write.FSMC_AddressSetupTime = 0x03;	 //地址建立时间（ADDSET）为1个HCLK
    Timing_write.FSMC_AddressHoldTime = 0x00;	 //地址保持时间（A
    Timing_write.FSMC_DataSetupTime = 0x03;		 ////数据保存时间为4个HCLK
    Timing_write.FSMC_BusTurnAroundDuration = 0x00;
    Timing_write.FSMC_CLKDivision = 0x00;
    Timing_write.FSMC_DataLatency = 0x00;
    Timing_write.FSMC_AccessMode = FSMC_AccessMode_A;	 //模式A

    FSMC_NORSRAMInitStructure.FSMC_Bank = NEX_SELECT;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &Timing_read;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &Timing_write;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);
    /* BANK 4 (of NOR/SRAM Bank 1~4) is enabled */
    FSMC_NORSRAMCmd(NEX_SELECT, ENABLE);
}

static void lcd_port_init(void)
{
    LCD_FSMCConfig();
}

lcd_inline void write_cmd(rt_uint16_t cmd)
{
    LCD_REG = cmd;
}

lcd_inline rt_uint16_t read_data(void)
{
    return LCD_RAM;
}

lcd_inline void write_data(rt_uint16_t data_code )
{
    LCD_RAM = data_code;
}

lcd_inline void write_reg(rt_uint16_t reg_addr, rt_uint16_t reg_val)
{
    write_cmd(reg_addr);
    write_data(reg_val);
}

lcd_inline unsigned short read_reg(rt_uint16_t reg_addr)
{
    rt_uint16_t val=0;
    write_cmd(reg_addr);
    val = read_data();
    return (val);
}


//返回LCD的ID
rt_uint32_t lcd_getdeviceid(void)
{
    return deviceid;
}

/*
static rt_uint16_t BGR2RGB(rt_uint16_t c)
{
    u16  r, g, b, rgb;

    b = (c>>0)  & 0x1f;
    g = (c>>5)  & 0x3f;
    r = (c>>11) & 0x1f;

    rgb =  (b<<11) + (g<<5) + (r<<0);
    return( rgb );
}
*/

/* 设置光标位置 */
static void lcd_SetCursor(rt_uint32_t x, rt_uint32_t y)
{
#if defined(_ILI_HORIZONTAL_DIRECTION_)
    write_reg(0x4e,y);    /* 0-239 */
    write_reg(0x4f,x);    /* 0-319 */
#else
    write_reg(0x4e,x);    /* 0-239 */
    write_reg(0x4f,y);    /* 0-319 */
#endif
}

/* 读取指定地址的GRAM */
static unsigned short lcd_read_gram(rt_uint32_t x, rt_uint32_t y)
{
    rt_uint16_t temp;

    lcd_SetCursor(x,y);
    rw_data_prepare();
    /* dummy read */
    temp = read_data();
    temp = read_data();
    return temp;
}
#if 0
static void lcd_clear(rt_uint16_t Color)
{
    rt_uint32_t index=0;

    lcd_SetCursor(0,0);
    rw_data_prepare();         /* Prepare to write GRAM */
    for (index=0; index<(LCD_WIDTH*LCD_HEIGHT); index++)
    {
        write_data(Color);
    }
}
#endif


static void lcd_data_bus_test(void)
{
    unsigned short temp1;
    unsigned short temp2;

    write_reg(0x0011,0x6030 | (0<<3)); // AM=0 hline
    /* wirte */
    lcd_SetCursor(0,0);
    rw_data_prepare();
    write_data(0x5566);

    lcd_SetCursor(1,0);
    rw_data_prepare();
    write_data(0xAAbb);

    /* read */
    lcd_SetCursor(0,0);
    temp1 = lcd_read_gram(0,0);
    temp2 = lcd_read_gram(1,0);

    if( (temp1 == 0x5566) && (temp2 == 0xAAbb) )
    {
        printf(" data bus test pass!");
    }
    else
    {
        printf(" data bus test error: %04X %04X",temp1,temp2);
    }
}

static void lcd_gram_test(void)
{
    unsigned short temp;
    unsigned int test_x;
    unsigned int test_y;

    printf(" LCD GRAM test....");

    /* write */
    temp=0;
    write_reg(0x0011,0x6030 | (0<<3)); // AM=0 hline

    lcd_SetCursor(0,0);
    rw_data_prepare();
    for(test_y=0; test_y<76800; test_y++)
    {
        write_data(temp);
        temp++;
    }/* write */

    /* read */
    /*temp=0;
    {
        for(test_y=0; test_y<320; test_y++)
        {
            for(test_x=0; test_x<240; test_x++)
            {
                if( lcd_read_gram(test_x,test_y) != temp++)
                {
                    printf("  LCD GRAM ERR!!");
                    return ;
                }
            }
        }
        printf("  TEST PASS!\r\n");
    } read */
}


void lcd_Initializtion(void)
{
    lcd_port_init();
    delay(50);
    write_reg(0x0000,0x0001);
    delay(50);
    deviceid = read_reg(0x00);

    /* deviceid check */
    if( deviceid != 0x8989 )
    {
        printf("Invalid LCD ID:%08X\r\n",deviceid);
        printf("Please check you hardware and configure.");
    }
    else
    {
        printf("\r\nLCD Device ID : %04X ",deviceid);
    }

    // power supply setting
    // set R07h at 0021h (GON=1,DTE=0,D[1:0]=01)
    write_reg(0x0007,0x0021);
    // set R00h at 0001h (OSCEN=1)
    write_reg(0x0000,0x0001);
    // set R07h at 0023h (GON=1,DTE=0,D[1:0]=11)
    write_reg(0x0007,0x0023);
    // set R10h at 0000h (Exit sleep mode)
    write_reg(0x0010,0x0000);
    // Wait 30ms
    delay(3000);
    // set R07h at 0033h (GON=1,DTE=1,D[1:0]=11)
    write_reg(0x0007,0x0033);
    // Entry mode setting (R11h)
    // R11H Entry mode
    // vsmode DFM1 DFM0 TRANS OEDef WMode DMode1 DMode0 TY1 TY0 ID1 ID0 AM LG2 LG2 LG0
    //   0     1    1     0     0     0     0      0     0   1   1   1  *   0   0   0
    write_reg(0x0011,0x6070);
    // LCD driver AC setting (R02h)
    write_reg(0x0002,0x0600);
    // power control 1
    // DCT3 DCT2 DCT1 DCT0 BT2 BT1 BT0 0 DC3 DC2 DC1 DC0 AP2 AP1 AP0 0
    // 1     0    1    0    1   0   0  0  1   0   1   0   0   1   0  0
    // DCT[3:0] fosc/4 BT[2:0]  DC{3:0] fosc/4
    write_reg(0x0003,0x0804);//0xA8A4
    write_reg(0x000C,0x0000);//
    write_reg(0x000D,0x080C);//
    // power control 4
    // 0 0 VCOMG VDV4 VDV3 VDV2 VDV1 VDV0 0 0 0 0 0 0 0 0
    // 0 0   1    0    1    0    1    1   0 0 0 0 0 0 0 0
    write_reg(0x000E,0x2900);
    write_reg(0x001E,0x00B8);

#if defined(_ILI_REVERSE_DIRECTION_)
    write_reg(0x0001,0x293F);//驱动输出控制320*240  0x6B3F
#else
    write_reg(0x0001,0x2B3F);//驱动输出控制320*240  0x6B3F
#endif

    write_reg(0x0010,0x0000);
    write_reg(0x0005,0x0000);
    write_reg(0x0006,0x0000);
    write_reg(0x0016,0xEF1C);
    write_reg(0x0017,0x0003);
    write_reg(0x0007,0x0233);//0x0233
    write_reg(0x000B,0x0000|(3<<6));
    write_reg(0x000F,0x0000);//扫描开始地址
    write_reg(0x0041,0x0000);
    write_reg(0x0042,0x0000);
    write_reg(0x0048,0x0000);
    write_reg(0x0049,0x013F);
    write_reg(0x004A,0x0000);
    write_reg(0x004B,0x0000);
    write_reg(0x0044,0xEF00);
    write_reg(0x0045,0x0000);
    write_reg(0x0046,0x013F);
    write_reg(0x0030,0x0707);
    write_reg(0x0031,0x0204);
    write_reg(0x0032,0x0204);
    write_reg(0x0033,0x0502);
    write_reg(0x0034,0x0507);
    write_reg(0x0035,0x0204);
    write_reg(0x0036,0x0204);
    write_reg(0x0037,0x0502);
    write_reg(0x003A,0x0302);
    write_reg(0x003B,0x0302);
    write_reg(0x0023,0x0000);
    write_reg(0x0024,0x0000);
    write_reg(0x0025,0x8000);   // 65hz
    write_reg(0x004f,0);        // 行首址0
    write_reg(0x004e,0);        // 列首址0

    //数据总线测试,用于测试硬件连接是否正常.
    lcd_data_bus_test();
    //GRAM测试,此测试可以测试LCD控制器内部GRAM.测试通过保证硬件正常
    lcd_gram_test();

    //清屏
    //lcd_clear( Blue );
}


void rt_hw_lcd_update(rtgui_rect_t *rect)
{
    /* nothing for none-DMA mode driver */
}

rt_uint8_t * rt_hw_lcd_get_framebuffer(void)
{
    return RT_NULL; /* no framebuffer driver */
}

/*  设置像素点 颜色,X,Y */
void rt_hw_lcd_set_pixel(const char* c, rt_base_t x, rt_base_t y)
{
    rt_uint16_t p;
    p =  *(uint16_t *)c;
    //p = rtgui_color_to_565p(*c);
    lcd_SetCursor(x,y);

    rw_data_prepare();
    write_data(p);
}

/* 获取像素点颜色 */
void rt_hw_lcd_get_pixel(char* c, rt_base_t x, rt_base_t y)
{
    rt_uint16_t p;

    p = (lcd_read_gram(x,y));

    *(rt_uint16_t*)c = p;
}


/* 画水平线 */
void rt_hw_lcd_draw_hline(const char* c, int x1, int x2, int y)
{
    rt_uint16_t p;

    p = *(uint16_t *)c;
    //p = rtgui_color_to_565(*pixel);
    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
#if defined(_ILI_HORIZONTAL_DIRECTION_)
    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    write_reg(0x0011,0x6070 | (1<<3)); // AM=0 hline
#else
    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    write_reg(0x0011,0x6030 | (0<<3)); // AM=0 hline
#endif

    lcd_SetCursor(x1, y);
    rw_data_prepare(); /* Prepare to write GRAM */
    while (x1 < x2)
    {
        write_data(p);
        x1++;
    }
}

/* 垂直线 */
void rt_hw_lcd_draw_vline(const char* c, int x, int y1, int y2)
{
    rt_uint16_t p;

    p = *(uint16_t *)c;
    //p = rtgui_color_to_565p(*pixel);

    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
#if defined(_ILI_HORIZONTAL_DIRECTION_)
    write_reg(0x0011,0x6030 | (0<<3)); // AM=0 hline
#else
    write_reg(0x0011,0x6070 | (1<<3)); // AM=0 hline
#endif

    lcd_SetCursor(x, y1);
    rw_data_prepare(); /* Prepare to write GRAM */
    while (y1 < y2)
    {
        write_data(p);
        y1++;
    }
}

/* ?? */
void rt_hw_lcd_draw_blit_line(const char* c, int x, int y, rt_size_t size)
{
    rt_uint16_t *ptr;
    ptr = (rt_uint16_t*)c;

#if defined(_ILI_HORIZONTAL_DIRECTION_)
    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    write_reg(0x0011,0x6070 | (1<<3)); // AM=0 hline
#else
    /* [5:4]-ID~ID0 [3]-AM-1垂直-0水平 */
    write_reg(0x0011,0x6030 | (0<<3)); // AM=0 hline
#endif

    lcd_SetCursor(x, y);
    rw_data_prepare(); /* Prepare to write GRAM */
    while (size)
    {
        write_data(*ptr ++);
        size--;
    }
}

/* 下面的东东，就有linux 设备驱动框架的味道了 */
struct rt_device_graphic_ops lcd_ili_ops =
{
    rt_hw_lcd_set_pixel,
    rt_hw_lcd_get_pixel,
    rt_hw_lcd_draw_hline,
    rt_hw_lcd_draw_vline,
    rt_hw_lcd_draw_blit_line
};


static rt_err_t lcd_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t lcd_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t lcd_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t lcd_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    switch (cmd)
    {
    case RTGRAPHIC_CTRL_GET_INFO:
    {
        struct rt_device_graphic_info *info;

        info = (struct rt_device_graphic_info*) args;
        RT_ASSERT(info != RT_NULL);

        info->bits_per_pixel = 16;
        info->pixel_format = RTGRAPHIC_PIXEL_FORMAT_RGB565P;
        info->framebuffer = RT_NULL;
#if defined(_ILI_HORIZONTAL_DIRECTION_)
        info->width = LCD_HEIGHT;
        info->height = LCD_WIDTH;
#else
        info->width = LCD_WIDTH;
        info->height = LCD_HEIGHT;
#endif
    }
    break;

    case RTGRAPHIC_CTRL_RECT_UPDATE:
        /* nothong to be done */
        break;

    default:
        break;
    }

    return RT_EOK;
}

/* 需直接调用的 用于硬件初始化和注册设备 */
int rt_hw_lcd_init(void)
{
    lcd_Initializtion();

    /* register lcd device */
    _lcd_device.type  = RT_Device_Class_Graphic;
    _lcd_device.init  = lcd_init;
    _lcd_device.open  = lcd_open;
    _lcd_device.close = lcd_close;
    _lcd_device.control = lcd_control;
    _lcd_device.read  = RT_NULL;
    _lcd_device.write = RT_NULL;

    _lcd_device.user_data = &lcd_ili_ops;

    /* register graphic device driver */
    rt_device_register(&_lcd_device, "lcd",
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

    return 0;
}

INIT_DEVICE_EXPORT(rt_hw_lcd_init);

/*以下提供finsh接口，用于调屏*/
#ifdef RT_USING_FINSH
#include "finsh.h"
void tft_write_reg(unsigned char reg_addr,unsigned short reg_val)
{
    write_cmd(reg_addr);
    write_data(reg_val);
}

FINSH_FUNCTION_EXPORT(tft_write_reg, addr & val ) ;
#endif //RT_USING_FINSH


