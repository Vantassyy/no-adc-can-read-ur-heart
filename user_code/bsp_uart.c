#include "bsp_uart.h"

// ================== 内部数据结构与变量 ==================

typedef struct
{
    zf_uart_index_enum  hw_uart_index;
    zf_uart_tx_pin_enum tx_pin;
    zf_uart_rx_pin_enum rx_pin;
    zf_fifo_obj_struct  rx_fifo;
    uint8_t             *rx_buffer;
    uint16_t            rx_buffer_size;
    bool                is_initialized;
} bsp_uart_instance_t;

static uint8_t g_debug_uart_rx_buffer[BSP_UART_DEBUG_RX_BUF_SIZE];
static uint8_t g_rtk_uart_rx_buffer[BSP_UART_RTK_RX_BUF_SIZE];

static bsp_uart_instance_t g_uart_instances[BSP_UART_NUM_MAX] =
{
    [BSP_UART_DEBUG] = {
        .hw_uart_index = UART_3,
        .tx_pin = UART3_TX_E10,
        .rx_pin = UART3_RX_E9,
        .rx_buffer = g_debug_uart_rx_buffer,
        .rx_buffer_size = BSP_UART_DEBUG_RX_BUF_SIZE,
        .is_initialized = false
    },
    [BSP_UART_RTK] = {
        .hw_uart_index = UART_1,
        .tx_pin = UART1_TX_F3,
        .rx_pin = UART1_RX_F2,
        .rx_buffer = g_rtk_uart_rx_buffer,
        .rx_buffer_size = BSP_UART_RTK_RX_BUF_SIZE,
        .is_initialized = false
    }
};

// ================== 内部中断处理函数 ==================

static void universal_uart_handler(uint32_t event, void* ptr)
{
    bsp_uart_instance_t* instance = (bsp_uart_instance_t*)ptr;

    if (UART_INTERRUPT_STATE_RX & event)
    {
        uint8_t data_byte = 0;
        zf_uart_query_byte(instance->hw_uart_index, &data_byte);
        zf_fifo_write_element(&instance->rx_fifo, data_byte);
    }
}

// ================== API函数实现 ==================

void bsp_uart_init(bsp_uart_e uart_ch, uint32_t baudrate)
{
    if (uart_ch >= BSP_UART_NUM_MAX) return;

    bsp_uart_instance_t* instance = &g_uart_instances[uart_ch];

    if (instance->is_initialized) return;

    zf_fifo_init(&instance->rx_fifo, FIFO_DATA_8BIT, instance->rx_buffer, instance->rx_buffer_size);
    zf_uart_init(instance->hw_uart_index, baudrate, instance->tx_pin, instance->rx_pin);
    zf_uart_set_interrupt_callback(instance->hw_uart_index, universal_uart_handler, (void*)instance);
    zf_uart_set_interrupt_config(instance->hw_uart_index, UART_INTERRUPT_CONFIG_RX_ENABLE);

    // [已删除] 不再需要手动设置debug uart

    instance->is_initialized = true;
}

void bsp_uart_write_byte(bsp_uart_e uart_ch, uint8_t data)
{
    if (uart_ch >= BSP_UART_NUM_MAX) return;
    zf_uart_write_byte(g_uart_instances[uart_ch].hw_uart_index, data);
}

void bsp_uart_write_buffer(bsp_uart_e uart_ch, const uint8_t* buffer, uint32_t length)
{
    if (uart_ch >= BSP_UART_NUM_MAX) return;
    zf_uart_write_buffer(g_uart_instances[uart_ch].hw_uart_index, (uint8_t*)buffer, length);
}

bool bsp_uart_read_byte(bsp_uart_e uart_ch, uint8_t* data)
{
    if (uart_ch >= BSP_UART_NUM_MAX || data == NULL) return false;

    // [已修正] 添加了第三个参数 FIFO_READ_WITH_CLEAN
    if (zf_fifo_read_element(&g_uart_instances[uart_ch].rx_fifo, data, FIFO_READ_WITH_CLEAN))
    {
        return true;
    }

    return false;
}
