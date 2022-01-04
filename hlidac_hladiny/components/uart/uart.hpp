#pragma once

#include <Arduino.h>
#include <driver/uart.h>

#include <mutex>

class Uart: public Stream
{
    typedef std::recursive_mutex mutex_t;
    Uart() = delete;
    Uart(Uart&) = delete;
public:
    Uart(const uart_port_t uart_num);
    virtual ~Uart();
    void begin(
        int baud = 115200,
        uint32_t config = SERIAL_8N1,
        int pin_rx = UART_PIN_NO_CHANGE,
        int pin_tx = UART_PIN_NO_CHANGE,
        size_t rx_buf = 128,
        size_t tx_buf = 0,
        size_t queue_size = 0 );
    void end();
// Print
    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);
    int availableForWrite();
// Stream
    int available();
    int read();
    size_t read(uint8_t *buffer, size_t size);
    int peek();
    void flush();
private:
    const uart_port_t m_uart_num;
    QueueHandle_t m_queue;
    mutex_t m_mutex;
    bool m_has_peek;
    uint8_t m_peek_byte;
    TickType_t m_timeout;
public:
    static void process(void* uart_v);
};
