/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2013-07-12     aozima       update for auto initial.
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>
#include <string.h>

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#include <rtgui/calibration.h>
#endif

#ifdef RT_USING_SPI
#include "spi_flash.h"
#include "spi_flash_w25qxx.h"
#include "spi.h"
#endif

#include "variable.h"
#include "ssd1322.h"
#include "led.h"
#define THREAD_STACK_SIZE 512
#define THREAD_PRIORITY 25
#define THREAD_TIMESLICE 5

u8 uart422_tx_buff1[]="hello world!";
u8 uart422_tx_buff2[]="HELLO WORLD!";


ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t oled_stack[ 512 ];
static struct rt_thread oled_thread;
static void oled_thread_entry(void *parameter)
{
    rt_device_t oled_device;
	  rt_err_t result = RT_EOK;
    struct rt_device_graphic_ops *oled_ops;
	
    rt_hw_oled_init();
	
	  oled_device = (rt_device_t) rt_device_find("oled");
    if(oled_device == NULL)
		{
			rt_kprintf("Don't find oled deivice !");
		}
		
    oled_ops = (struct rt_device_graphic_ops *)oled_device->user_data;
    while (1)
    {
			
			oled_ops->set_pixel(0xf0, 0, 0);
			oled_ops->draw_hline(0xf0,0,10,5);
			oled_ops->draw_vline(0xf0,10,5,10);
			oled_ops->draw_block(0xf0, 11,13,11,13);
			oled_ops->draw_frame(0xf0, 20,30,20,30);
			oled_ops->draw_chinese(0xf0,char_1,55,20);
			oled_ops->draw_char(0xf0,char_2,40,20);
			oled_ops->draw_picture(0xf0,char_3, 150,10,26,21);
			oled_device->write(oled_device, 0, oled_buff, sizeof(oled_buff));
			rt_thread_delay(RT_TICK_PER_SECOND);
    }
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t uart1_stack[ 512 ];
static struct rt_thread uart1_thread;


/* 消息队列控制块 */
static struct rt_messagequeue mq;
/* 消息队列中用到的放置消息的内存池 */
static char msg_pool[2048];
rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    struct rx_msg msg;
    msg.dev = dev;
    msg.size = size;


			/*发送消息到消息队列中*/
			rt_mq_send(&mq, &msg, sizeof(struct rx_msg));
    return RT_EOK;
}


static char uart_rx_buffer[64];

static	struct rt_semaphore sem1; //定义一个信号量
int semaphore_static_init()
{
	rt_err_t result;
	/* 初始化信号量，初始值是0 */
	
	result = rt_sem_init(&sem1, "sem", 0, RT_IPC_FLAG_FIFO);
	if(result != RT_EOK)
	{
		return 0;
	}
	return 0;
}
static void uart1_thread_entry(void* parameter)
{
    rt_device_t device;
	  rt_err_t result = RT_EOK;
	  rt_tick_t tick; 
    struct rx_msg msg;
    int count = 0;

	  device = rt_device_find("uart1");
    if(device != RT_NULL )
	  {
			 /* set callback function and open device */
			 rt_device_set_rx_indicate(device, uart_input);
			 rt_device_open(device, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);
		}

    while (1)
    {
//				result = rt_mq_recv(&mq, &msg, sizeof(struct rx_msg), RT_WAITING_FOREVER);
//				if (result == -RT_ETIMEOUT)
//        {
//          rt_kprintf("timeout count:%d\n", ++count);
//        }
//				if(result == RT_EOK)
//				{
//					rt_uint32_t rx_length;
//          rx_length = (sizeof(uart_rx_buffer) - 1) > msg.size ?
//          msg.size : sizeof(uart_rx_buffer) - 1;
//					
//				
//          rx_length = rt_device_read(msg.dev, 0, &uart_rx_buffer[0],rx_length);
//          uart_rx_buffer[rx_length] = '\0';
//					rt_device_write(device, 0, &uart_rx_buffer[0],rx_length);
//					/* 发送不需要标志位的UART422参数 */
////					rt_device_write(device, 0, &uart422_tx_buff1, sizeof(uart422_tx_buff1));
////					rt_thread_delay( RT_TICK_PER_SECOND/10);
//					rt_kprintf("aaaa : %d",rx_length);
					/* 发送需要标志位的UART422参数 */
//					{
//					/*获取当前的OS TICK*/
//					tick = rt_tick_get();

//					/*试图持有一个信号量，如果10个OS Tick依然没有拿到，则超时返回*/
//					result = rt_sem_take(&sem1, 10);
//					if(result == -RT_ETIMEOUT)
//					{
//						/* 超时后判断是否刚好是10个OS Tick */
//						if(rt_tick_get() - tick != 10 )
//						{
//							rt_sem_delete(&sem1);
//							return;
//						}
//						rt_kprintf("take semaphore timeout\n");
//					}
//					else
//					{
//						if(sem1.value == 1)
//						{
//							rt_sem_release(&sem1);
//							rt_device_write(device, 0, &uart422_tx_buff2, sizeof(uart422_tx_buff2));
//						}
//						rt_sem_release(&sem1);
//					}
//				 }
//				}
    }
}



rt_err_t messageq_simple_init(void)
{
	rt_err_t result;
	
	/*初始化消息队列*/
	result = rt_mq_init(&mq,"mqt",
	           &msg_pool[0],   /* 内存池指向msg_pool*/
	            128-sizeof(void*),/*每个消息的大小是128- void*.void*表示消息头的大小*/
							sizeof(msg_pool),/* 内存池的大小是msg_pool的大小*/
								RT_IPC_FLAG_FIFO); /* 如果有多个线程等待，按照先来先得的方法分配消息*/
							
	if(result != RT_EOK)
	{
		rt_kprintf("init message queue failed.\n");
	}
}
ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t led_stack[ 512 ];
static struct rt_thread led_thread;
static void led_thread_entry(void* parameter)
{
    unsigned int count=0;

    rt_hw_led_init();

    while (1)
    {
        /* led1 on */
#ifndef RT_USING_FINSH
        rt_kprintf("led on, count : %d\r\n",count);
#endif
        count++;
        rt_hw_led_on(0);
        rt_thread_delay( RT_TICK_PER_SECOND/2 ); /* sleep 0.5 second and switch to other thread */

        /* led1 off */
#ifndef RT_USING_FINSH
        rt_kprintf("led off\r\n");
#endif
        rt_hw_led_off(0);
        rt_thread_delay( RT_TICK_PER_SECOND/2 );
    }
}

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t spi2_stack[ 512 ];
static struct rt_thread spi2_thread;
static void spi2_thread_entry(void* parameter)
{


}

#ifdef RT_USING_RTGUI
rt_bool_t cali_setup(void)
{
    rt_kprintf("cali setup entered\n");
    return RT_FALSE;
}

void cali_store(struct calibration_data *data)
{
    rt_kprintf("cali finished (%d, %d), (%d, %d)\n",
               data->min_x,
               data->max_x,
               data->min_y,
               data->max_y);
}
#endif /* RT_USING_RTGUI */

void rt_init_thread_entry(void* parameter)
{
#ifdef RT_USING_COMPONENTS_INIT
    /* initialization RT-Thread Components */
    rt_components_init();
#endif
	
#ifdef RT_USING_SPI
	  rt_hw_spi2_init();


	  

#endif /* #ifdef RT_USING_SPI*/		
#ifdef RT_USING_DFS
		dfs_init();
	  elm_init();
    /* Filesystem Initialization */
	  if (dfs_mount("w25q128", "/", "elm", 0, 0) == 0)
			{
					rt_kprintf("File System initialized!\n");
			}
			else
			{
					rt_kprintf("File System initialzation failed!\n");
			}
#endif  /* RT_USING_DFS */
		
		
#ifdef RT_USING_RTGUI
    {
        extern void rt_hw_lcd_init();
        extern void rtgui_touch_hw_init(void);

        rt_device_t lcd;

        /* init lcd */
        rt_hw_lcd_init();

        /* init touch panel */
        rtgui_touch_hw_init();

        /* find lcd device */
        lcd = rt_device_find("lcd");

        /* set lcd device as rtgui graphic driver */
        rtgui_graphic_set_device(lcd);

#ifndef RT_USING_COMPONENTS_INIT
        /* init rtgui system server */
        rtgui_system_server_init();
#endif

        calibration_set_restore(cali_setup);
        calibration_set_after(cali_store);
        calibration_init();
    }
#endif /* #ifdef RT_USING_RTGUI */
		


    messageq_simple_init();
		semaphore_static_init();

}

int rt_application_init(void)
{
    rt_thread_t init_thread;

    rt_err_t result;

    /* init led thread */
    result = rt_thread_init(&led_thread,
                            "led",
                            led_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&led_stack[0],
                            sizeof(led_stack),
                            20,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&led_thread);
    }
		
		/* init uart1 thread */
    result = rt_thread_init(&uart1_thread,
                            "uart1",
                            uart1_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&uart1_stack[0],
                            sizeof(uart1_stack),
                            21,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&uart1_thread);
    }
		
		/* init oled thread */
    result = rt_thread_init(&oled_thread,
                            "oled",
                            oled_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&oled_stack[0],
                            sizeof(oled_stack),
                            21,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&oled_thread);
    }
		
#ifdef RT_USING_SPI2
	  /* init spi1 thread */
    result = rt_thread_init(&spi2_thread,
                            "spi2",
                            spi2_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&spi2_stack[0],
                            sizeof(spi2_stack),
                            22,
                            5);
    if (result == RT_EOK)
    {
        rt_thread_startup(&spi2_thread);
    }
#endif

#if (RT_THREAD_PRIORITY_MAX == 32)
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 8, 20);
#else
    init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, 80, 20);
#endif

	
		
    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    return 0;
}

/*@}*/
