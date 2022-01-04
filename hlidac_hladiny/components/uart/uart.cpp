#include "uart.hpp"

#include <esp_err.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <fmt/core.h>

Uart::Uart(const uart_port_t uart_num)
    : m_uart_num{uart_num},
      m_queue{nullptr},
      m_mutex{},
      m_has_peek{false},
      m_peek_byte{0},
      m_timeout{0}
{}

Uart::~Uart() {
    end();
}

void Uart::begin(
    int baud,
    uint32_t config,
    int pin_rx,
    int pin_tx,
    size_t rx_buf,
    size_t tx_buf,
    size_t queue_size )
{
    end();
    const uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = uart_word_length_t((config & 0x0C) >> 2),
        .parity    = uart_parity_t     ((config & 0x03) >> 0),
        .stop_bits = uart_stop_bits_t  ((config & 0x30) >> 4),
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_FIFO_LEN-1,
        .source_clk = UART_SCLK_APB
    };
    std::lock_guard<mutex_t> lock (m_mutex);
    ESP_ERROR_CHECK(uart_driver_install(m_uart_num, rx_buf * 2, tx_buf * 2, queue_size == 0 ? rx_buf : queue_size, &m_queue, ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(uart_param_config  (m_uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin       (m_uart_num, pin_tx, pin_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void Uart::end() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (uart_is_driver_installed(m_uart_num)) {
        ESP_ERROR_CHECK(uart_driver_delete(m_uart_num));
        m_queue = nullptr;
        m_has_peek = false;
    }
}

// Print

size_t Uart::write(uint8_t v) {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!uart_is_driver_installed(m_uart_num))
        return 0;
    return uart_write_bytes(m_uart_num, &v, 1);
}

size_t Uart::write(const uint8_t *buffer, size_t size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!uart_is_driver_installed(m_uart_num))
        return 0;
    return uart_write_bytes(m_uart_num, buffer, size);
}

extern "C" int uart_get_txfifo_len(const uart_port_t uart_num); // See uart_helper.c

int Uart::availableForWrite() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!uart_is_driver_installed(m_uart_num))
        return 0;
    return uart_get_txfifo_len(m_uart_num);
}

// Stream
int Uart::available() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!uart_is_driver_installed(m_uart_num))
        return 0;
    size_t res = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(m_uart_num, &res));
    return m_has_peek ? (res + 1) : res;
}

int Uart::read() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!uart_is_driver_installed(m_uart_num))
        return -1;
    if (m_has_peek) {
        m_has_peek = false;
    } else {
        if (uart_read_bytes(m_uart_num, &m_peek_byte, 1, m_timeout) < 1)
            return -1;
    }
    return m_peek_byte;
}

size_t Uart::read(uint8_t *buffer, size_t size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!uart_is_driver_installed(m_uart_num))
        return 0;
    size_t read_len = 0;
    if (m_has_peek) {
        m_has_peek = false;
        *(buffer++) = m_peek_byte;
        --size;
        ++read_len;
    }
    read_len += uart_read_bytes(m_uart_num, buffer, size, m_timeout);
    return read_len;
}

int Uart::peek() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!uart_is_driver_installed(m_uart_num))
        return -1;
    if (!m_has_peek) {
        if (read() != -1)
            m_has_peek = true;
        else
            return -1;
    }
    return m_peek_byte;
}

void Uart::flush() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!uart_is_driver_installed(m_uart_num))
        return;
    ESP_ERROR_CHECK(uart_flush(m_uart_num));
}

// static
void Uart::process(void* uart_v) {
    Uart& uart = *static_cast<Uart*>(uart_v);
    uart_event_t event;
    if (xQueueReceive(uart.m_queue, &event, 0)) {
        fmt::print("Event ID {}, size {}, timeouf flag {}\n", int(event.type), event.size, event.timeout_flag);
    }
}