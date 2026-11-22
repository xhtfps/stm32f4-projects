/**
 * @file Drivers/TFT_LCD/W25Q64.c
 * @brief W25Q64 SPI Flash 设备对象驱动模块。
 * @details
 * - 属于显示子系统，分别负责 LCD、字库 Flash、字体数据或文字渲染中的明确职责。
 * - 模块间通过对象接口或兼容 API 协作，避免 UI 层直接操作显示控制器寄存器。
 * - 字体/字模数据文件以数组尺寸、字号和索引关系一致性为维护重点。
 * - 本文件已移除原子/ST 示例模板，统一使用项目自定义 UTF-8 文件模板。
 * @encoding UTF-8
 */

#include "W25Q64.h"
#include "spi.h"
#include "TFT_LCD.h"

#define W25Q64_PAGE_SIZE    256U
#define W25Q64_SECTOR_SIZE  4096U

static void W25q64Device_Init(W25q64Device *self);
static uint16_t W25q64Device_ReadId(W25q64Device *self);
static uint8_t W25q64Device_ReadStatus(W25q64Device *self);
static void W25q64Device_WriteStatus(W25q64Device *self, uint8_t status);
static void W25q64Device_WriteEnable(W25q64Device *self);
static void W25q64Device_WriteDisable(W25q64Device *self);
static void W25q64Device_Read(W25q64Device *self, uint8_t *buffer, uint32_t read_addr, uint16_t length);
static void W25q64Device_WritePage(W25q64Device *self, uint8_t *buffer, uint32_t write_addr, uint16_t length);
static void W25q64Device_WriteNoCheck(W25q64Device *self, uint8_t *buffer, uint32_t write_addr, uint16_t length);
static void W25q64Device_Write(W25q64Device *self, uint8_t *buffer, uint32_t write_addr, uint16_t length);
static void W25q64Device_EraseChip(W25q64Device *self);
static void W25q64Device_EraseSector(W25q64Device *self, uint32_t sector_index);
static void W25q64Device_WaitBusy(W25q64Device *self);
static void W25q64Device_PowerDown(W25q64Device *self);
static void W25q64Device_Wakeup(W25q64Device *self);
static uint8_t W25q64Device_Transfer(W25q64Device *self, uint8_t tx_data);
static void W25q64Device_Select(void);
static void W25q64Device_Deselect(void);
static void W25q64Device_DelayUs(uint32_t us);
static void W25q64Device_Construct(W25q64Device *self);

static const W25q64DeviceOps s_w25q64_device_ops =
{
    W25q64Device_Init,
    W25q64Device_ReadId,
    W25q64Device_ReadStatus,
    W25q64Device_WriteStatus,
    W25q64Device_WriteEnable,
    W25q64Device_WriteDisable,
    W25q64Device_Read,
    W25q64Device_WriteNoCheck,
    W25q64Device_Write,
    W25q64Device_EraseChip,
    W25q64Device_EraseSector,
    W25q64Device_WaitBusy,
    W25q64Device_PowerDown,
    W25q64Device_Wakeup
};

uint16_t W25Q64_TYPE = W25Q64;
uint8_t W25Q64_BUFFER[W25Q64_SECTOR_SIZE];

W25q64Device g_w25q64_device;

static void W25q64Device_Construct(W25q64Device *self)
{
    if ((self == NULL) || (self->ops != NULL))
    {
        return;
    }

    SPI1_Init();
    self->spi = &g_tft_spi1_bus;
    self->type = W25Q64;
    self->sector_buffer = W25Q64_BUFFER;
    self->page_size = W25Q64_PAGE_SIZE;
    self->sector_size = W25Q64_SECTOR_SIZE;
    self->ops = &s_w25q64_device_ops;
}

static void W25q64Device_Init(W25q64Device *self)
{
    GPIO_InitTypeDef gpio_init_struct;

    if ((self == NULL) || (self->spi == NULL))
    {
        return;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    gpio_init_struct.Pin = W25Q64_CS_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(W25Q64_CS_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOG, &gpio_init_struct);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);

    W25q64Device_Deselect();
    self->spi->ops->init(self->spi);
    self->spi->ops->set_speed(self->spi, SPI_BAUDRATEPRESCALER_2);

    self->type = self->ops->read_id(self);
    W25Q64_TYPE = self->type;
}

static uint8_t W25q64Device_ReadStatus(W25q64Device *self)
{
    uint8_t status;

    W25q64Device_Select();
    W25q64Device_Transfer(self, W25X_ReadStatusReg);
    status = W25q64Device_Transfer(self, 0xFFU);
    W25q64Device_Deselect();

    return status;
}

static void W25q64Device_WriteStatus(W25q64Device *self, uint8_t status)
{
    W25q64Device_Select();
    W25q64Device_Transfer(self, W25X_WriteStatusReg);
    W25q64Device_Transfer(self, status);
    W25q64Device_Deselect();
}

static void W25q64Device_WriteEnable(W25q64Device *self)
{
    W25q64Device_Select();
    W25q64Device_Transfer(self, W25X_WriteEnable);
    W25q64Device_Deselect();
}

static void W25q64Device_WriteDisable(W25q64Device *self)
{
    W25q64Device_Select();
    W25q64Device_Transfer(self, W25X_WriteDisable);
    W25q64Device_Deselect();
}

static uint16_t W25q64Device_ReadId(W25q64Device *self)
{
    uint16_t id = 0U;

    W25q64Device_Select();
    W25q64Device_Transfer(self, 0x90U);
    W25q64Device_Transfer(self, 0x00U);
    W25q64Device_Transfer(self, 0x00U);
    W25q64Device_Transfer(self, 0x00U);
    id |= (uint16_t)W25q64Device_Transfer(self, 0xFFU) << 8;
    id |= W25q64Device_Transfer(self, 0xFFU);
    W25q64Device_Deselect();

    return id;
}

static void W25q64Device_Read(W25q64Device *self, uint8_t *buffer, uint32_t read_addr, uint16_t length)
{
    uint16_t i;

    if ((buffer == NULL) && (length > 0U))
    {
        return;
    }

    W25q64Device_Select();
    W25q64Device_Transfer(self, W25X_ReadData);
    W25q64Device_Transfer(self, (uint8_t)(read_addr >> 16));
    W25q64Device_Transfer(self, (uint8_t)(read_addr >> 8));
    W25q64Device_Transfer(self, (uint8_t)read_addr);

    for (i = 0U; i < length; i++)
    {
        buffer[i] = W25q64Device_Transfer(self, 0xFFU);
    }

    W25q64Device_Deselect();
}

static void W25q64Device_WritePage(W25q64Device *self, uint8_t *buffer, uint32_t write_addr, uint16_t length)
{
    uint16_t i;

    if ((buffer == NULL) && (length > 0U))
    {
        return;
    }

    /* Page Program 只能在同一页内写入，不能跨 256 字节页边界。
     * 上层 WriteNoCheck 会提前按页切片，保证这里收到的是单页片段。
     */
    self->ops->write_enable(self);

    W25q64Device_Select();
    W25q64Device_Transfer(self, W25X_PageProgram);
    W25q64Device_Transfer(self, (uint8_t)(write_addr >> 16));
    W25q64Device_Transfer(self, (uint8_t)(write_addr >> 8));
    W25q64Device_Transfer(self, (uint8_t)write_addr);

    for (i = 0U; i < length; i++)
    {
        W25q64Device_Transfer(self, buffer[i]);
    }

    W25q64Device_Deselect();
    self->ops->wait_busy(self);
}

static void W25q64Device_WriteNoCheck(W25q64Device *self, uint8_t *buffer, uint32_t write_addr, uint16_t length)
{
    uint16_t page_remain;

    if ((self == NULL) || (buffer == NULL))
    {
        return;
    }

    /* “NoCheck” 的含义是：不检查目标区域是否已擦除。
     * NOR Flash 只能把 1 写成 0，不能直接把 0 写回 1；调用者必须保证目标区域原本为 0xFF。
     */
    page_remain = (uint16_t)(self->page_size - (write_addr % self->page_size));
    if (length <= page_remain)
    {
        page_remain = length;
    }

    while (1)
    {
        W25q64Device_WritePage(self, buffer, write_addr, page_remain);
        if (length == page_remain)
        {
            break;
        }

        buffer += page_remain;
        write_addr += page_remain;
        length = (uint16_t)(length - page_remain);
        page_remain = (length > self->page_size) ? self->page_size : length;
    }
}

static void W25q64Device_Write(W25q64Device *self, uint8_t *buffer, uint32_t write_addr, uint16_t length)
{
    uint32_t sector_index;
    uint16_t sector_offset;
    uint16_t sector_remain;
    uint16_t i;

    if ((self == NULL) || (buffer == NULL) || (self->sector_buffer == NULL))
    {
        return;
    }

    /* 高层写入以扇区为保护单位：
     * 1. 先读取整个扇区到 RAM；
     * 2. 如果目标范围内存在非 0xFF 字节，必须擦除整个扇区；
     * 3. 将新数据合并进扇区缓冲，再整扇区写回。
     * 这样可以保护同扇区内未被本次写入覆盖的旧数据。
     */
    sector_index = write_addr / self->sector_size;
    sector_offset = (uint16_t)(write_addr % self->sector_size);
    sector_remain = (uint16_t)(self->sector_size - sector_offset);
    if (length <= sector_remain)
    {
        sector_remain = length;
    }

    while (1)
    {
        self->ops->read(self, self->sector_buffer, sector_index * self->sector_size, self->sector_size);

        for (i = 0U; i < sector_remain; i++)
        {
            if (self->sector_buffer[sector_offset + i] != 0xFFU)
            {
                break;
            }
        }

        if (i < sector_remain)
        {
            /* 命中旧数据，必须走“读-擦-改-写”路径。 */
            self->ops->erase_sector(self, sector_index);
            for (i = 0U; i < sector_remain; i++)
            {
                self->sector_buffer[sector_offset + i] = buffer[i];
            }
            self->ops->write_no_check(self, self->sector_buffer, sector_index * self->sector_size, self->sector_size);
        }
        else
        {
            /* 目标片段保持擦除态，直接按页写入即可。 */
            self->ops->write_no_check(self, buffer, write_addr, sector_remain);
        }

        if (length == sector_remain)
        {
            break;
        }

        sector_index++;
        sector_offset = 0U;
        buffer += sector_remain;
        write_addr += sector_remain;
        length = (uint16_t)(length - sector_remain);
        sector_remain = (length > self->sector_size) ? self->sector_size : length;
    }
}

static void W25q64Device_EraseChip(W25q64Device *self)
{
    self->ops->write_enable(self);
    self->ops->wait_busy(self);

    W25q64Device_Select();
    W25q64Device_Transfer(self, W25X_ChipErase);
    W25q64Device_Deselect();

    self->ops->wait_busy(self);
}

static void W25q64Device_EraseSector(W25q64Device *self, uint32_t sector_index)
{
    uint32_t addr = sector_index * W25Q64_SECTOR_SIZE;

    self->ops->write_enable(self);
    self->ops->wait_busy(self);

    W25q64Device_Select();
    W25q64Device_Transfer(self, W25X_SectorErase);
    W25q64Device_Transfer(self, (uint8_t)(addr >> 16));
    W25q64Device_Transfer(self, (uint8_t)(addr >> 8));
    W25q64Device_Transfer(self, (uint8_t)addr);
    W25q64Device_Deselect();

    self->ops->wait_busy(self);
}

static void W25q64Device_WaitBusy(W25q64Device *self)
{
    while ((self->ops->read_status(self) & 0x01U) == 0x01U)
    {
    }
}

static void W25q64Device_PowerDown(W25q64Device *self)
{
    W25q64Device_Select();
    W25q64Device_Transfer(self, W25X_PowerDown);
    W25q64Device_Deselect();
    W25q64Device_DelayUs(3U);
}

static void W25q64Device_Wakeup(W25q64Device *self)
{
    W25q64Device_Select();
    W25q64Device_Transfer(self, W25X_ReleasePowerDown);
    W25q64Device_Deselect();
    W25q64Device_DelayUs(3U);
}

static uint8_t W25q64Device_Transfer(W25q64Device *self, uint8_t tx_data)
{
    if ((self == NULL) || (self->spi == NULL))
    {
        return 0xFFU;
    }

    return self->spi->ops->transfer(self->spi, tx_data);
}

static void W25q64Device_Select(void)
{
    W25Q64_CS_L;
}

static void W25q64Device_Deselect(void)
{
    W25Q64_CS_H;
}

static void W25q64Device_DelayUs(uint32_t us)
{
    uint32_t ticks;
    uint32_t start;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    ticks = (SystemCoreClock / 1000000U) * us;
    start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

void W25Q64_Init(void)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->init(&g_w25q64_device);
}

uint8_t W25Q64_ReadSR(void)
{
    W25q64Device_Construct(&g_w25q64_device);
    return g_w25q64_device.ops->read_status(&g_w25q64_device);
}

void W25Q64_Write_SR(uint8_t sr)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->write_status(&g_w25q64_device, sr);
}

void W25Q64_Write_Enable(void)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->write_enable(&g_w25q64_device);
}

void W25Q64_Write_Disable(void)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->write_disable(&g_w25q64_device);
}

uint16_t W25Q64_ReadID(void)
{
    W25q64Device_Construct(&g_w25q64_device);
    return g_w25q64_device.ops->read_id(&g_w25q64_device);
}

void W25Q64_Read(uint8_t *pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->read(&g_w25q64_device, pBuffer, ReadAddr, NumByteToRead);
}

void W25Q64_Write_Page(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    W25q64Device_Construct(&g_w25q64_device);
    W25q64Device_WritePage(&g_w25q64_device, pBuffer, WriteAddr, NumByteToWrite);
}

void W25Q64_Write_NoCheck(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->write_no_check(&g_w25q64_device, pBuffer, WriteAddr, NumByteToWrite);
}

void W25Q64_Write(uint8_t *pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->write(&g_w25q64_device, pBuffer, WriteAddr, NumByteToWrite);
}

void W25Q64_Erase_Chip(void)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->erase_chip(&g_w25q64_device);
}

void W25Q64_Erase_Sector(uint32_t Dst_Addr)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->erase_sector(&g_w25q64_device, Dst_Addr);
}

void W25Q64_Wait_Busy(void)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->wait_busy(&g_w25q64_device);
}

void W25Q64_PowerDown(void)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->power_down(&g_w25q64_device);
}

void W25Q64_WAKEUP(void)
{
    W25q64Device_Construct(&g_w25q64_device);
    g_w25q64_device.ops->wakeup(&g_w25q64_device);
}
