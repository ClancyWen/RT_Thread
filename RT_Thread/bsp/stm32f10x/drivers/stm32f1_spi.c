#include <stdint.h>
#include <rtdevice.h>

//#include "stm32f10x.h"
#include "spi_flash.h"
#include "spi_flash_w25qxx.h"
#include "spi.h"


rt_inline uint16_t get_spi_BaudRatePrescaler(rt_uint32_t max_hz)
{
    uint16_t SPI_BaudRatePrescaler;
    if(max_hz>= SystemCoreClock/2 && SystemCoreClock/2 <=18000000)
    {
        SPI_BaudRatePrescaler= SPI_BaudRatePrescaler_2;
    }
    else if(max_hz>= SystemCoreClock/4)
    {
        SPI_BaudRatePrescaler= SPI_BaudRatePrescaler_4;
    }
    else if(max_hz>= SystemCoreClock/8)
    {
        SPI_BaudRatePrescaler= SPI_BaudRatePrescaler_8;
    }
    else if(max_hz>= SystemCoreClock/16)
    {
        SPI_BaudRatePrescaler= SPI_BaudRatePrescaler_16;
    }
    else if(max_hz>= SystemCoreClock/32)
    {
        SPI_BaudRatePrescaler= SPI_BaudRatePrescaler_32;
    }
    else if(max_hz>= SystemCoreClock/64)
    {
        SPI_BaudRatePrescaler= SPI_BaudRatePrescaler_64;
    }
    else if(max_hz>= SystemCoreClock/128)
    {
        SPI_BaudRatePrescaler= SPI_BaudRatePrescaler_128;
    }
    else
    {
        SPI_BaudRatePrescaler= SPI_BaudRatePrescaler_256;
    }
    return SPI_BaudRatePrescaler;
}

rt_uint32_t xfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    struct stm32_spi_bus *stm32_spi_bus=(struct stm32_spi_bus*)device->bus;
    struct rt_spi_configuration *config=&device->config;
    SPI_TypeDef* SPI=stm32_spi_bus->SPI;
    struct stm32_spi_cs* stm32_spi_cs=device->parent.user_data;
    rt_uint32_t size=message->length;
    
    /* take cs */
    if(message->cs_take)
    {
        GPIO_ResetBits(stm32_spi_cs->GPIOx, stm32_spi_cs->GPIO_Pin);
    }
    
       if(config->data_width <=8)
       {
           const rt_uint8_t *send_ptr= message->send_buf;
           rt_uint8_t *recv_ptr= message->recv_buf;
           while(size--)
           {
               rt_uint8_t data= 0xff;
               if(send_ptr != RT_NULL)
               {
                   data=*send_ptr++;
               }
               /* Wait until the transmit buffer is empty */
               while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE)== RESET);
               /* Send byte */
               SPI_I2S_SendData(SPI, data);
               /* Wait until a data is received */
               while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE)== RESET);
               /* Get the receive data */
               data= SPI_I2S_ReceiveData(SPI);
               if(recv_ptr != RT_NULL)
               {
                   *recv_ptr++ =data;
               }
            }
       }
       else if(config->data_width <=16)
       {
            const rt_uint16_t *send_ptr= message->send_buf;
            rt_uint16_t *recv_ptr= message->recv_buf;
            while(size--)
            {
                rt_uint16_t data=0xffff;
                if(send_ptr != RT_NULL)
                {
                    data= *send_ptr++;
                }
                /* Wait until the transmit buffer is empty */
                while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE)== RESET);
                /* Send the byte*/
                SPI_I2S_SendData(SPI, data);
                /* Wait until a data is received */
                while(SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE)== RESET);
                data= SPI_I2S_ReceiveData(SPI);
                if(recv_ptr != RT_NULL )
                {
                    *recv_ptr++= data;
                }
            }
       }
        /* release CS */
        if(message->cs_release)
        {
            GPIO_SetBits(stm32_spi_cs->GPIOx, stm32_spi_cs-> GPIO_Pin);
        }
     return message->length;
}

rt_err_t configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration)
{
    struct stm32_spi_bus *stm32_spi_bus=(struct stm32_spi_bus*)device->bus;
    SPI_InitTypeDef SPI_InitStructure;
    
    SPI_Init(SPI2, &SPI_InitStructure);
   /* data width */
   if(configuration->data_width <= 8)
   {   
       SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;
   }
    else if( configuration->data_width <= 16)
    {
       SPI_InitStructure.SPI_DataSize=SPI_DataSize_16b;
    }
    else
    {
        return RT_EIO;
    }
    /* baudrete */
    SPI_InitStructure.SPI_BaudRatePrescaler= get_spi_BaudRatePrescaler(configuration->max_hz);
    
    /* CPOL */
    if(configuration->mode & RT_SPI_CPOL)
    {
        SPI_InitStructure.SPI_CPOL=SPI_CPOL_High;
    }
    else
    {
        SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;
    }
    /* CPHA */
    if(configuration->mode & RT_SPI_CPHA)
    {
        SPI_InitStructure.SPI_CPHA=SPI_CPHA_2Edge;
    }
    else
    {
        SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;
    }
    /* MSB OR LSB */
    if(configuration->mode & RT_SPI_MSB)
    {
        SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;
    }
    else
    {
        SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_LSB;
    }
    SPI_InitStructure.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode= SPI_Mode_Master;
    SPI_InitStructure.SPI_NSS= SPI_NSS_Soft;
    
    /* init SPI */
    SPI_I2S_DeInit(stm32_spi_bus-> SPI);
    SPI_Init(stm32_spi_bus->SPI, &SPI_InitStructure);
    /* Enable SPI_MASTER */
    SPI_Cmd(stm32_spi_bus->SPI, ENABLE);
    SPI_CalculateCRC(stm32_spi_bus->SPI, DISABLE);
    
    return RT_EOK;
}

rt_err_t stm32_spi_register(SPI_TypeDef* SPI,
                           struct stm32_spi_bus* stm32_spi,
                           const char *spi_bus_name)
{
    if(SPI== SPI1)
    {
        stm32_spi-> SPI= SPI1;
        RCC_APB1PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //84MHZ
    }
		
    else if(SPI== SPI2)
    {
        stm32_spi->SPI= SPI2;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//42MHZ
    }
    else if(SPI == SPI3)
    {
        stm32_spi->SPI = SPI3;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//42MHZ     
    }
    else
    {
        return RT_ENOSYS;
    }
		return RT_EOK;
}
int rt_hw_spi_init(void)
{
    static struct stm32_spi_bus stm32_spi;    //it must be add static
    static struct rt_spi_device rt_spi_device_20;
    static struct stm32_spi_cs stm32_spi_cs_20;
    struct rt_spi_configuration cfg;   
    
    /* Initialize the SPI2 control pin */
    GPIO_InitTypeDef GPIO_InitStructure;
    /* SPI2 */
    /* Pin13 SCK*/
    /* Pin14 MISO */
    /* Pin15 MOSI */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15);
    stm32_spi_register(SPI2, &stm32_spi, "spi2");
    
    /* attach spi2*/
    stm32_spi_cs_20.GPIOx= GPIOB;
    stm32_spi_cs_20.GPIO_Pin= GPIO_Pin_12;
    


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
   
    rt_spi_bus_attach_device(&rt_spi_device_20, "spi2","spi",(void*) &stm32_spi_cs_20);//set spi_device->bus
    
    cfg.data_width= 8;
    cfg.mode = RT_SPI_MODE_3| RT_SPI_MSB;
    cfg.max_hz= 2625000;
    rt_spi_configure(&rt_spi_device_20, &cfg);
    
    return 0;
}



