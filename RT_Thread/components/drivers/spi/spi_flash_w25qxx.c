/*
 * File      : spi_flash_w25qxx.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2011, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-12-16     aozima       the first version
 * 2012-05-06     aozima       can page write.
 * 2012-08-23     aozima       add flash lock.
 * 2012-08-24     aozima       fixed write status register BUG.
 */

#include <stdint.h>
#include <rtdevice.h>

#include "spi_flash.h"
#include "spi_flash_w25qxx.h"
#include "spi.h"

#define FLASH_DEBUG

#ifdef FLASH_DEBUG
#define FLASH_TRACE         rt_kprintf
#else
#define FLASH_TRACE(...)
#endif /* #ifdef FLASH_DEBUG */

#define PAGE_SIZE           4096   //sector size

/* JEDEC Manufacturer��s ID */
#define MF_ID           (0xEF)

/* JEDEC Device ID: Memory type and Capacity */
#define MTC_W25Q80_BV         (0x4014) /* W25Q80BV */
#define MTC_W25Q16_BV_CL_CV   (0x4015) /* W25Q16BV W25Q16CL W25Q16CV  */
#define MTC_W25Q16_DW         (0x6015) /* W25Q16DW  */
#define MTC_W25Q32_BV         (0x4016) /* W25Q32BV */
#define MTC_W25Q32_DW         (0x6016) /* W25Q32DW */
#define MTC_W25Q64_BV_CV      (0x4017) /* W25Q64BV W25Q64CV */
#define MTC_W25Q64_DW         (0x4017) /* W25Q64DW */
#define MTC_W25Q128_BV        (0x4018) /* W25Q128BV */
#define MTC_W25Q256_FV        (TBD)    /* W25Q256FV */

/* command list */
#define CMD_WRSR                    (0x01)  /* Write Status Register */
#define CMD_PP                      (0x02)  /* Page Program */
#define CMD_READ                    (0x03)  /* Read Data */
#define CMD_WRDI                    (0x04)  /* Write Disable */
#define CMD_RDSR1                   (0x05)  /* Read Status Register-1 */
#define CMD_WREN                    (0x06)  /* Write Enable */
#define CMD_FAST_READ               (0x0B)  /* Fast Read */
#define CMD_ERASE_4K                (0x20)  /* Sector Erase:4K */
#define CMD_RDSR2                   (0x35)  /* Read Status Register-2 */
#define CMD_ERASE_32K               (0x52)  /* 32KB Block Erase */
#define CMD_JEDEC_ID                (0x9F)  /* Read JEDEC ID */
#define CMD_ERASE_full              (0xC7)  /* Chip Erase */
#define CMD_ERASE_64K               (0xD8)  /* 64KB Block Erase */

#define DUMMY                       (0xFF)


rt_err_t configure(struct rt_spi_device *device, struct rt_spi_configuration *configuration);
rt_uint32_t xfer(struct rt_spi_device *device, struct rt_spi_message *message);

static struct spi_flash_device  spi_flash_device;
static struct rt_spi_ops stm32_spi_ops=
{
	configure,
	xfer
};
static void flash_lock(struct spi_flash_device * flash_device)
{
    rt_mutex_take(&flash_device->lock, RT_WAITING_FOREVER);
}

static void flash_unlock(struct spi_flash_device * flash_device)
{
    rt_mutex_release(&flash_device->lock);
}

static uint8_t w25qxx_read_status(void)
{
    return rt_spi_sendrecv8(spi_flash_device.rt_spi_device, CMD_RDSR1);
}

static void w25qxx_wait_busy(void)
{
    while( w25qxx_read_status() & (0x01));
}

/** \brief read [size] byte from [offset] to [buffer]
 *
 * \param offset uint32_t unit : byte
 * \param buffer uint8_t*
 * \param size uint32_t   unit : byte
 * \return uint32_t byte for read
 *
 */
static uint32_t w25qxx_read(uint32_t offset, uint8_t * buffer, uint32_t size)
{
    uint8_t send_buffer[4];

    send_buffer[0] = CMD_WRDI;
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 1);

    send_buffer[0] = CMD_READ;
    send_buffer[1] = (uint8_t)(offset>>16);
    send_buffer[2] = (uint8_t)(offset>>8);
    send_buffer[3] = (uint8_t)(offset);

    rt_spi_send_then_recv(spi_flash_device.rt_spi_device,
                          send_buffer, 4,
                          buffer, size);

    return size;
}

/** \brief write N page on [page]
 *
 * \param page_addr uint32_t unit : byte (4096 * N,1 page = 4096byte)
 * \param buffer const uint8_t*
 * \return uint32_t
 *
 */
uint32_t w25qxx_page_write(uint32_t page_addr, const uint8_t* buffer)
{
    uint32_t index;
    uint8_t send_buffer[4];

    RT_ASSERT((page_addr&0xFF) == 0); /* page addr must align to 256byte. */

    send_buffer[0] = CMD_WREN;
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 1);

    send_buffer[0] = CMD_ERASE_4K;
    send_buffer[1] = (page_addr >> 16);
    send_buffer[2] = (page_addr >> 8);
    send_buffer[3] = (page_addr);
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 4);

    w25qxx_wait_busy(); // wait erase done.

    for(index=0; index < (PAGE_SIZE / 256); index++)
    {
        send_buffer[0] = CMD_WREN;
        rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 1);

        send_buffer[0] = CMD_PP;
        send_buffer[1] = (uint8_t)(page_addr >> 16);
        send_buffer[2] = (uint8_t)(page_addr >> 8);
        send_buffer[3] = (uint8_t)(page_addr);

        rt_spi_send_then_send(spi_flash_device.rt_spi_device,
                              send_buffer,
                              4,
                              buffer,
                              256);

        buffer += 256;
        page_addr += 256;
        w25qxx_wait_busy();
    }

    send_buffer[0] = CMD_WRDI;
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 1);

    return PAGE_SIZE;
}

/* RT-Thread device interface */
static rt_err_t w25qxx_flash_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t w25qxx_flash_open(rt_device_t dev, rt_uint16_t oflag)
{
    uint8_t send_buffer[3];

    flash_lock((struct spi_flash_device *)dev);

    send_buffer[0] = CMD_WREN;
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 1);

    send_buffer[0] = CMD_WRSR;
    send_buffer[1] = 0;
    send_buffer[2] = 0;
    rt_spi_send(spi_flash_device.rt_spi_device, send_buffer, 3);

    w25qxx_wait_busy();

    flash_unlock((struct spi_flash_device *)dev);

    return RT_EOK;
}

static rt_err_t w25qxx_flash_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t w25qxx_flash_control(rt_device_t dev, int cmd, void *args)
{
    RT_ASSERT(dev != RT_NULL);

    if (cmd == RT_DEVICE_CTRL_BLK_GETGEOME)
    {
        struct rt_device_blk_geometry *geometry;

        geometry = (struct rt_device_blk_geometry *)args;
        if (geometry == RT_NULL) return -RT_ERROR;

        geometry->bytes_per_sector = spi_flash_device.geometry.bytes_per_sector;
        geometry->sector_count = spi_flash_device.geometry.sector_count;
        geometry->block_size = spi_flash_device.geometry.block_size;
    }

    return RT_EOK;
}

static rt_size_t w25qxx_flash_read(rt_device_t dev,
                                   rt_off_t pos,
                                   void* buffer,
                                   rt_size_t size)
{
    flash_lock((struct spi_flash_device *)dev);

    w25qxx_read(pos*spi_flash_device.geometry.bytes_per_sector,
                buffer,
                size*spi_flash_device.geometry.bytes_per_sector);

    flash_unlock((struct spi_flash_device *)dev);

    return size;
}

static rt_size_t w25qxx_flash_write(rt_device_t dev,
                                    rt_off_t pos,
                                    const void* buffer,
                                    rt_size_t size)
{
    rt_size_t i = 0;
    rt_size_t block = size;
    const uint8_t * ptr = buffer;

    flash_lock((struct spi_flash_device *)dev);

    while(block--)
    {
        w25qxx_page_write((pos + i)*spi_flash_device.geometry.bytes_per_sector,
                          ptr);
        ptr += PAGE_SIZE;
        i++;
    }

    flash_unlock((struct spi_flash_device *)dev);

    return size;
}

rt_err_t w25qxx_init(const char * flash_device_name, const char * spi_device_name)
{
    struct rt_spi_device * rt_spi_device;
    /* initialize mutex */
    if (rt_mutex_init(&spi_flash_device.lock, spi_device_name, RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("init sd lock mutex failed\n");
        return -RT_ENOSYS;
    }

    rt_spi_device = (struct rt_spi_device *)rt_device_find(spi_device_name);
    if(rt_spi_device == RT_NULL)
    {
        FLASH_TRACE("spi device %s not found!\r\n", spi_device_name);
        return -RT_ENOSYS;
    }
    spi_flash_device.rt_spi_device = rt_spi_device;

    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */
        cfg.max_hz = 50 * 1000 * 1000; /* 50M */
        rt_spi_configure(spi_flash_device.rt_spi_device, &cfg);
    }


    /* init flash */
    {
        rt_uint8_t cmd;
        rt_uint8_t id_recv[3];
        uint16_t memory_type_capacity;

        flash_lock(&spi_flash_device);

        cmd = 0xFF; /* reset SPI FLASH, cancel all cmd in processing. */
        rt_spi_send(spi_flash_device.rt_spi_device, &cmd, 1);

        cmd = CMD_WRDI;
        rt_spi_send(spi_flash_device.rt_spi_device, &cmd, 1);

        /* read flash id */
        cmd = CMD_JEDEC_ID;
        rt_spi_send_then_recv(spi_flash_device.rt_spi_device, &cmd, 1, id_recv, 3);

        flash_unlock(&spi_flash_device);

        if(id_recv[0] != MF_ID)
        {
            FLASH_TRACE("Manufacturers ID error!\r\n");
            FLASH_TRACE("JEDEC Read-ID Data : %02X %02X %02X\r\n", id_recv[0], id_recv[1], id_recv[2]);
            return -RT_ENOSYS;
        }

        spi_flash_device.geometry.bytes_per_sector = 4096;
        spi_flash_device.geometry.block_size = 4096; /* block erase: 4k */

        /* get memory type and capacity */
        memory_type_capacity = id_recv[1];
        memory_type_capacity = (memory_type_capacity << 8) | id_recv[2];

        if(memory_type_capacity == MTC_W25Q128_BV)
        {
            FLASH_TRACE("W25Q128BV detection\r\n");
            spi_flash_device.geometry.sector_count = 4096;
        }
        else if(memory_type_capacity == MTC_W25Q64_BV_CV)
        {
            FLASH_TRACE("W25Q64BV or W25Q64CV detection\r\n");
            spi_flash_device.geometry.sector_count = 2048;
        }
        else if(memory_type_capacity == MTC_W25Q64_DW)
        {
            FLASH_TRACE("W25Q64DW detection\r\n");
            spi_flash_device.geometry.sector_count = 2048;
        }
        else if(memory_type_capacity == MTC_W25Q32_BV)
        {
            FLASH_TRACE("W25Q32BV detection\r\n");
            spi_flash_device.geometry.sector_count = 1024;
        }
        else if(memory_type_capacity == MTC_W25Q32_DW)
        {
            FLASH_TRACE("W25Q32DW detection\r\n");
            spi_flash_device.geometry.sector_count = 1024;
        }
        else if(memory_type_capacity == MTC_W25Q16_BV_CL_CV)
        {
            FLASH_TRACE("W25Q16BV or W25Q16CL or W25Q16CV detection\r\n");
            spi_flash_device.geometry.sector_count = 512;
        }
        else if(memory_type_capacity == MTC_W25Q16_DW)
        {
            FLASH_TRACE("W25Q16DW detection\r\n");
            spi_flash_device.geometry.sector_count = 512;
        }
        else if(memory_type_capacity == MTC_W25Q80_BV)
        {
            FLASH_TRACE("W25Q80BV detection\r\n");
            spi_flash_device.geometry.sector_count = 256;
        }
        else
        {
            FLASH_TRACE("Memory Capacity error!\r\n");
            return -RT_ENOSYS;
        }
    }

    /* register device */
    spi_flash_device.flash_device.type    = RT_Device_Class_Block;
    spi_flash_device.flash_device.init    = w25qxx_flash_init;
    spi_flash_device.flash_device.open    = w25qxx_flash_open;
    spi_flash_device.flash_device.close   = w25qxx_flash_close;
    spi_flash_device.flash_device.read    = w25qxx_flash_read;
    spi_flash_device.flash_device.write   = w25qxx_flash_write;
    spi_flash_device.flash_device.control = w25qxx_flash_control;
    /* no private */
    spi_flash_device.flash_device.user_data = RT_NULL;

    rt_device_register(&spi_flash_device.flash_device, flash_device_name,
                       RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE);

    return RT_EOK;
}
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
    
//    SPI_Init(SPI2, &SPI_InitStructure);
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
//    SPI_I2S_DeInit(stm32_spi_bus-> SPI);
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
		return rt_spi_bus_register(&stm32_spi->parent, spi_bus_name, &stm32_spi_ops);
}
int rt_hw_spi2_init(void)
{
    static struct stm32_spi_bus stm32_spi;    //it must be add static
    static struct rt_spi_device rt_spi_device_2;
    static struct stm32_spi_cs stm32_spi_cs_2;

	 

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
    stm32_spi_register(SPI2, &stm32_spi, "spi");
    
    /* attach spi2*/
    stm32_spi_cs_2.GPIOx= GPIOB;
    stm32_spi_cs_2.GPIO_Pin= GPIO_Pin_12;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
		

		
    rt_spi_bus_attach_device(&rt_spi_device_2, "spi2","spi",(void*) &stm32_spi_cs_2);//set spi_device->bus
    
	
		w25qxx_init("w25q128","spi2");
		
    return 0;
}
