#include "ssd1322.h"

// Compatible list:
// ssd1322

//内联函数定义,用以提高性能
#ifdef __CC_ARM                			 /* ARM Compiler 	*/
#define oled_inline   				static __inline
#elif defined (__ICCARM__)        		/* for IAR Compiler */
#define oled_inline 					inline
#elif defined (__GNUC__)        		/* GNU GCC Compiler */
#define oled_inline 					static __inline
#else
#define oled_inline                  static
#endif

#define rw_data_prepare()               write_cmd(34)


/********* control ***********/
#include "stm32f10x.h"
#include "board.h"

//输出重定向.当不进行重定向时.
#define printf               rt_kprintf //使用rt_kprintf来输出
//#define printf(...)                       //无输出

/* OLED is connected to the FSMC_Bank1_NOR/SRAM1 and NE1 is used as ship select signal */
/* RS <==> A17 */
#define OLED_REG              (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define OLED_RAM              (*((volatile unsigned short *) 0x60040000)) /* RS = 1 */
#define Set_Column_Address   0x15
#define Set_Row_Address		   0x75
#define Read_RAM_Command     0x5d
#define Write_RAM_Command    0x5c

rt_uint8_t oled_buff[OLED_WIDTH * OLED_HEIGHT /2]={0}; 
rt_uint8_t char_1[24]={
0x04,0x00,0x08,0x00,0x3F,0xF0,0xC0,0x00,0x08,0x40,0xF3,0x80,0x20,0x10,0x2F,0xF0,
0x20,0x00,0x2A,0x00,0x31,0xC0,0x00,0x00

};
rt_uint8_t char_2[]={
0x00,0x40,0x07,0xC0,0x39,0x00,0x0F,0x00,0x01,0xC0,0x00,0x40
};
rt_uint8_t char_3[]=
{
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x00,0x01,
0x10,0x00,0x02,0x08,0x00,0x02,0x0C,0x00,0x06,0x04,0x00,0x04,0x04,0x00,0x04,0x06,
0x00,0x04,0x02,0x00,0x04,0x02,0x00,0x02,0x02,0x00,0x02,0x02,0x00,0x02,0x04,0x00,
0x02,0x04,0x00,0x02,0x08,0x00,0x01,0xB0,0x00,0x00,0xE0,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

};






static void OLED_FSMCConfig(void)
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  Timing_read,Timing_write;

    /* FSMC GPIO configure */
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF
                               | RCC_APB2Periph_GPIOG, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
			  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

			
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

        /*
        FSMC_D0 ~ FSMC_D3
        PD14 FSMC_D0   PD15 FSMC_D1   PD0  FSMC_D2   PD1  FSMC_D3
        */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_14 | GPIO_Pin_15;
        GPIO_Init(GPIOD,&GPIO_InitStructure);

        /*
        FSMC_D4 ~ FSMC_D12
        PE7 ~ PE15  FSMC_D4 ~ FSMC_D12
        */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
        GPIO_Init(GPIOE,&GPIO_InitStructure);

        /* FSMC_D13 ~ FSMC_D15   PD8 ~ PD10 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_Init(GPIOD,&GPIO_InitStructure);

//        /*
//        FSMC_A0 ~ FSMC_A5   FSMC_A6 ~ FSMC_A9
//        PF0     ~ PF5       PF12    ~ PF15
//        */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
//                                      | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//        GPIO_Init(GPIOF,&GPIO_InitStructure);

//        /* FSMC_A10 ~ FSMC_A15  PG0 ~ PG5 */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
//        GPIO_Init(GPIOG,&GPIO_InitStructure);

//        /* FSMC_A16 ~ FSMC_A18  PD11 ~ PD13 */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
//        GPIO_Init(GPIOD,&GPIO_InitStructure);

        /* RD-PD4 WR-PD5 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
        GPIO_Init(GPIOD,&GPIO_InitStructure);

//        /* NBL0-PE0 NBL1-PE1 */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//        GPIO_Init(GPIOE,&GPIO_InitStructure);

        /* NE1/NCE2 */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        GPIO_Init(GPIOD,&GPIO_InitStructure);
//        /* NE2 */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//        GPIO_Init(GPIOG,&GPIO_InitStructure);
//        /* NE3 */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//        GPIO_Init(GPIOG,&GPIO_InitStructure);
//        /* NE4 */
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//        GPIO_Init(GPIOG,&GPIO_InitStructure);
    }
    /* FSMC GPIO configure */

    /*-- FSMC Configuration -------------------------------------------------*/
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &Timing_read;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &Timing_write;
    FSMC_NORSRAMStructInit(&FSMC_NORSRAMInitStructure);

    Timing_read.FSMC_AddressSetupTime = 8;             /* 地址建立时间  */
    Timing_read.FSMC_AddressHoldTime  = 8;             /* 地址保持时间  */
    Timing_read.FSMC_DataSetupTime = 8;                /* 数据建立时间  */
    Timing_read.FSMC_AccessMode = FSMC_AccessMode_A;    /* FSMC 访问模式 */

    Timing_write.FSMC_AddressSetupTime = 0;             /* 地址建立时间  */
    Timing_write.FSMC_AddressHoldTime  = 0;             /* 地址保持时间  */
    Timing_write.FSMC_DataSetupTime = 1;                /* 数据建立时间  */
    Timing_write.FSMC_AccessMode = FSMC_AccessMode_A;   /* FSMC 访问模式 */

    /* Color oled configuration ------------------------------------
       oled configured as follow:
          - Data/Address MUX = Disable
          - Memory Type = SRAM
          - Data Width = 16bit
          - Write Operation = Enable
          - Extended Mode = Enable
          - Asynchronous Wait = Disable */
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
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
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure);
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
}


static void oled_port_init(void)
{
    OLED_FSMCConfig();
}

oled_inline void write_cmd(unsigned short cmd)  //写命令
{
    OLED_REG = cmd;
}

oled_inline unsigned short read_data(void)      //读数据
{
    return OLED_RAM;
}

oled_inline void write_data(unsigned short data_code )  //写数据
{
    OLED_RAM = data_code;
}

oled_inline void write_reg(unsigned char reg_addr,unsigned short reg_val) //写寄存器
{
    write_cmd(reg_addr);
    write_data(reg_val);
}

oled_inline unsigned short read_reg(unsigned char reg_addr)   //读寄存器
{
    unsigned short val=0;
    write_cmd(reg_addr);
    val = read_data();
    return (val);
}



/********* control <只移植以上函数即可> ***********/

//static unsigned short deviceid=0;//设置一个静态变量用来保存oled的ID

//static unsigned short BGR2RGB(unsigned short c)
//{
//    u16  r, g, b, rgb;
//
//    b = (c>>0)  & 0x1f;
//    g = (c>>5)  & 0x3f;
//    r = (c>>11) & 0x1f;
//
//    rgb =  (b<<11) + (g<<5) + (r<<0);
//
//    return( rgb );
//}
/* 设置光标地址 */
static void oled_SetCursor(unsigned int x, unsigned int y)
{
//    write_reg(Set_Column_Address,x);  
//    write_reg(Set_Row_Address,y);     
	write_cmd(Set_Column_Address); /* 0-127 */
	write_data(x+28);
	write_data(x+29);
	
	write_cmd(Set_Row_Address); /* 0-64 */
	write_data(y+28);
	write_data(y+29);
}

///* 读取指定地址的GRAM */
//static unsigned short oled_read_gram(unsigned int x,unsigned int y)
//{
//    unsigned short temp;
//    oled_SetCursor(x,y);
//    write_cmd(Read_RAM_Command); //从RAM读数据之前，必须先写该命令
//    /* dummy read */
//    temp = read_data();
//    temp = read_data();
//    return temp;
//}

static void oled_clear(unsigned short Color)
{
    unsigned int index=0;
    write_cmd(Set_Column_Address);
	  write_data(0x1c);
		write_data(0x1c+63);
	
		write_cmd(Set_Row_Address);
		write_data(0);
		write_data(63);
    write_cmd(Write_RAM_Command);                     /* Prepare to write GRAM */
    for (index=0; index<(64*128); index++)
    {
        write_data(Color);
    }
}


/*  测试OLED连接是否正常 */
static void oled_data_bus_test(void)
{
    unsigned char temp1=0x55;
    unsigned char temp2=0xaa;


    /* wirte */
    oled_SetCursor(0,0);
    write_cmd(Write_RAM_Command);
    write_data(temp1);
    write_data(temp2);

	  temp1 = 0;
	  temp2 = 0;
    /* read */
    oled_SetCursor(0,0);
	  write_cmd(Read_RAM_Command);
	  temp1 = read_data();
	  temp1 = read_data();
		temp2 = read_data();

    if( (temp1 == 0x55) && (temp2 == 0xaa) )
    {
        rt_kprintf(" data bus test pass!\r\n");
    }
    else
    {
        rt_kprintf(" data bus test error: %04X %04X\r\n",temp1,temp2);
    }
}

void ssd1322_init(void)
{
	   
    oled_port_init();
		write_cmd(0xfd);
		write_data(0x12);
		
		write_cmd(0xae);

		write_cmd(0xb3);
		write_data(0x91);
		
		write_cmd(0xca);
		write_data(0x3f);
		
		write_cmd(0xa2);
		write_data(0x00);
		
		write_cmd(0xa1);
		write_data(0x00);
		
		write_cmd(0xa0);
		write_data(0x14);
		write_data(0x11);
		
		write_cmd(0xB5);
		write_data(0x00);
		
		write_cmd(0xab);
		write_data(0x01);
		
		write_cmd(0xb4);
		write_data(0xa0);
		write_data(0xfd);
		
		write_cmd(0xc1);
		write_data(0x9f);
		
		write_cmd(0xc7);
		write_data(0x0f);
		
		write_cmd(0xb9);
		
		write_cmd(0xb1);
		write_data(0xe2);
		
		write_cmd(0xd1);
		write_data(0x82);
		write_data(0x20);
		
		write_cmd(0xbb);
		write_data(0x1f);
		
		write_cmd(0xb6);
		write_data(0x08);
		
		write_cmd(0xbe);
		write_data(0x07);
		
		write_cmd(0xa6);

		write_cmd(0xaf);
			
    //数据总线测试,用于测试硬件连接是否正常.
    oled_data_bus_test();
    //GRAM测试,此测试可以测试oled控制器内部GRAM.测试通过保证硬件正常
//    oled_gram_test();
    //清屏
    oled_clear(0);

}

void oled_dma_init(const rt_uint8_t	*src)
{
		DMA_InitTypeDef DMA_InitStructure;
	
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能 DMA 时钟
		DMA_DeInit(DMA1_Channel1); //将 DMA 的通道 1 寄存器重设为缺省值
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)src; //DMA 外设 ADC 基地址
		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0x60040000; //DMA 内存基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //数据传输方向，外设到内存
		DMA_InitStructure.DMA_BufferSize = 8192; //DMA 通道的 DMA 缓存的大小
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable; //外设地址不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址寄存器递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//数据宽度为 8 位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte; //数据宽度
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //工作在正常缓存模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA 通道拥有中优先级
		DMA_InitStructure.DMA_M2M = DMA_M2M_Enable; //非内存到内存传输
		DMA_Init(DMA1_Channel1, &DMA_InitStructure); //初始化 DMA 的通道
	
		DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
		DMA_ClearFlag(DMA1_FLAG_TC1);
		DMA_Cmd(DMA1_Channel1, ENABLE);
}


/* 设置像素点 颜色,X,Y */
/* 以下操作又多种不同方式 */
void ssd1322_oled_set_pixel(rt_uint8_t pixel, rt_uint32_t x, rt_uint32_t y)
{
//	  oled_buff[(y*OLED_WIDTH *4)/8+x*4/8] = pixel << ((x%2)?4:0) ;
	
		/* 方式一：pixel 在0x00~0xf0 16种值*/
	  pixel &= 0xf0;
	  oled_buff[y*OLED_WIDTH/2 + x/2] &= ~((0xf0  >> ((x&0x01)?4:0))) ;  
	  oled_buff[y*OLED_WIDTH/2 + x/2] |= (pixel >> ((x&0x01)?4:0)) ;
	
//		/* 方式二：pixel 在0x00~0x0f 16种值,计算不正确*/
//    pixel &= 0x0f;
//		oled_buff[y*OLED_WIDTH/2 + x/2] &= (0x0f  << ((x&0x01)?4:0)) ;  
//	  oled_buff[y*OLED_WIDTH/2 + x/2] |= (pixel << ((x&0x01)?0:4)) ;
}



/* 画水平线 */
void ssd1322_oled_draw_hline(rt_uint8_t pixel, rt_uint32_t x_start, 
	                           rt_uint32_t x_end, rt_uint32_t y)
{
		uint32_t  i;
	  if(x_start>x_end)
		{
			rt_kprintf("hline:x_start>x_end \n");
			return; 
		}
		pixel &= 0xf0;
    write_cmd(Write_RAM_Command);  /* Prepare to write GRAM */
	
		for(i=x_start; i<x_end; i++)
		{
			oled_buff[ i/2 + y * OLED_WIDTH /2] &= ~ (0xf0 >> ((i&0x01)?0x0f:0xf0));
			oled_buff[ i/2 + y * OLED_WIDTH /2] |= (pixel >> ( (i&0x01)?4:0));
		}
}

/* 垂直线 */
void ssd1322_oled_draw_vline(rt_uint8_t pixel, rt_uint32_t x, 
	                           rt_uint32_t y_start, rt_uint32_t y_end)
{		
	  rt_uint16_t  i;
    if(y_start>y_end)
		{
			rt_kprintf("vline:y_start>y_end \n");
			return; 
		}

	  pixel &= 0xf0;
    write_cmd(Write_RAM_Command);  /* Prepare to write GRAM */
	
		for(i=y_start; i<y_end; i++)
		{
			oled_buff[ x/2 + i * OLED_WIDTH /2] &= ~ ( 0xf0 >> ((x&0x01)?4:0));
			oled_buff[ x/2 + i * OLED_WIDTH /2] |=  pixel >>((x&0x01)?4:0);
		}
}

/* 矩形块 */
void ssd1322_oled_draw_block(rt_uint8_t pixel, rt_uint32_t x_start, rt_uint32_t x_end, 
	                           rt_uint32_t y_start, rt_uint32_t y_end)
{		
		rt_uint16_t  x,y;
	
		if(x_start>x_end || y_start>y_end)
		{
			rt_kprintf("x_start>x2 or y_start>y_end \n");
			return; 
		}
    write_cmd(Write_RAM_Command);  /* Prepare to write GRAM */
		
		for(y=y_start;y<y_end;y++)
		{
			for(x=x_start; x<x_end; x++)
			{
				oled_buff[ x/2 + y * OLED_WIDTH /2] &= ~ (0xf0 >> ((x&0x01)?0x0f:0xf0));
				oled_buff[ x/2 + y * OLED_WIDTH /2] |= (pixel >> ( (x&0x01)?4:0));
			}
	  }
}

/* 矩形框 */
void ssd1322_oled_draw_frame(rt_uint8_t pixel, rt_uint32_t x_start,
                             rt_uint32_t x_end, rt_uint32_t y_start, rt_uint32_t y_end)
{
		if((x_start>x_end) || (y_start>y_end))
		{
			rt_kprintf("x_start>x_end or y_start>y_end \n");
			return; 
		}
		ssd1322_oled_draw_hline(pixel, x_start, x_end, y_start);
		ssd1322_oled_draw_hline(pixel, x_start, x_end+1, y_end);
		ssd1322_oled_draw_vline(pixel, x_start, y_start, y_end);
		ssd1322_oled_draw_vline(pixel, x_end, y_start, y_end);
}
/* 汉字 */
/* 取模方式 12*12 逐列式顺向*/
void ssd1322_oled_draw_chinese(rt_uint8_t pixel, const rt_uint8_t * chr, rt_uint32_t x, rt_uint32_t y)
{
	rt_uint8_t i,j;
	rt_uint16_t mid=0;
	
	for(i=0;i<12;i++)
	{
		mid = 0;
		mid |= (*(chr+2*i) >>4) &0x0f;
		mid = mid<<4;
		mid |= *(chr+2*i) &0x0f;
		mid = mid<<4;
		mid |= (*(chr+2*i+1) >>4) &0x0f;
		mid = mid<<4;
		mid |= *(chr+2*i+1) &0x0f;
		

		for(j=0; j<12; j++)
		{
			if((mid & (0x0010 << j)) == 0)
			{
				ssd1322_oled_set_pixel(0x00, x+i, y+12-j);
			}
			else
			{
				ssd1322_oled_set_pixel(pixel, x+i, y+12-j);
			}
		}/* for(j=0; j<12; j++) */
	}/* for(i=0;i<12;i++) */
}

/* 字符 */
/* 取模方式 12*12 逐列式顺向*/
void ssd1322_oled_draw_char(rt_uint8_t pixel, const rt_uint8_t * chr, rt_uint32_t x, rt_uint32_t y)
{
	rt_uint8_t i,j;
	rt_uint16_t mid=0;
	
	for(i=0;i<6;i++)
	{
		mid = 0;
		mid |= (*(chr+2*i) >>4) &0x0f;
		mid = mid<<4;
		mid |= *(chr+2*i) &0x0f;
		mid = mid<<4;
		mid |= (*(chr+2*i+1) >>4) &0x0f;
		mid = mid<<4;
		mid |= *(chr+2*i+1) &0x0f;
		

		for(j=0; j<12; j++)
		{
			if((mid & (0x0010 << j)) == 0)
			{
				ssd1322_oled_set_pixel(0x00, x+i, y+12-j);
			}
			else
			{
				ssd1322_oled_set_pixel(pixel, x+i, y+12-j);
			}
		}
	}
}

/* 图片 */
/* 取模方式 12*12 逐列式顺向*/
void ssd1322_oled_draw_picture(rt_uint8_t pixel, const rt_uint8_t * chr, 
	                            rt_uint32_t x, rt_uint32_t y,
														  rt_uint32_t pixel_x, rt_uint32_t pixel_y)
{
	rt_uint8_t i,j,k;

	if(pixel_y%8 == 0)
	{
		pixel_y=pixel_y/8;
	}
	else
	{
		pixel_y=pixel_y/8+1;
	}
	for(k=0;k<pixel_x;k++)
	{
		for(j=0;j<pixel_y;j++)
		{
			for(i=0;i<8;i++)
			{
				if(((0x80 >> i) & *(chr+(j+pixel_y*k)) )== 0)
				{
					ssd1322_oled_set_pixel(0x00, x+k, y+i+8*j);
				}
				else
			  {
				  ssd1322_oled_set_pixel(pixel, x+k, y+i+8*j);
			  }
			}
		}
	}

	
}




struct rt_device_graphic_ops ssd1322_ops =
{
	ssd1322_oled_set_pixel,
//	ssd1322_oled_get_pixel,
	ssd1322_oled_draw_hline,
	ssd1322_oled_draw_vline,
	ssd1322_oled_draw_block,
	ssd1322_oled_draw_frame,
	ssd1322_oled_draw_chinese,
	ssd1322_oled_draw_char,
	ssd1322_oled_draw_picture

};

struct rt_device _oled_device;
static rt_err_t oled_init(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t oled_open(rt_device_t dev, rt_uint16_t oflag)
{
	return RT_EOK;
}


static rt_size_t oled_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
	write_cmd(Write_RAM_Command);
	oled_dma_init(buffer);
	return RT_EOK;
}
static rt_err_t oled_close(rt_device_t dev)
{
	return RT_EOK;
}

static rt_err_t oled_control(rt_device_t dev, int cmd, void *args)
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
			info->width = 240;
			info->height = 320;
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

void rt_hw_oled_init(void)
{
    /* OLED RESET */
    /* PF10 : OLED RESET */
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_Init(GPIOF,&GPIO_InitStructure);

        GPIO_ResetBits(GPIOF,GPIO_Pin_10);
        GPIO_SetBits(GPIOF,GPIO_Pin_10);
        /* wait for oled reset */
        rt_thread_delay(1);
    }
   ssd1322_init();
	/* register oled device */
	_oled_device.type  = RT_Device_Class_Graphic;
	_oled_device.init  = oled_init;
	_oled_device.open  = oled_open;
	_oled_device.close = oled_close;
	_oled_device.control = oled_control;
	_oled_device.read  = RT_NULL;
	_oled_device.write = oled_write;

	_oled_device.user_data = &ssd1322_ops;

    /* register graphic device driver */
	rt_device_register(&_oled_device, "oled",
		RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);
}

