#pragma once

#include <Arduino.h>
#include <driver/uart.h>

#include <mutex>
#include <functional>

extern "C" int uart_ll_min_wakeup_thresh(void); // See uart_helper.c

class Uart: public Stream
{
    typedef std::recursive_mutex mutex_t;
    typedef std::function<void(Uart&)> callback_t;
    typedef int baudrate_t;
    struct Settings {
        uart_config_t config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = UART_FIFO_LEN-1,
            .source_clk = UART_SCLK_APB
        };
        uart_sw_flowctrl_t sw_flowctrl = {
            .xon_char  = 0x11,
            .xoff_char = 0x13,
            .xon_thrd  = 32,
            .xoff_thrd = 96
        };
        bool sw_flowctrl_en = false;
        uart_mode_t mode = UART_MODE_UART;
        bool loopback = false;
        uart_signal_inv_t signal_inv = UART_SIGNAL_INV_DISABLE;
        int8_t pin_txd = UART_PIN_NO_CHANGE;
        int8_t pin_rxd = UART_PIN_NO_CHANGE;
        int8_t pin_rts = UART_PIN_NO_CHANGE;
        int8_t pin_cts = UART_PIN_NO_CHANGE;
        uint16_t tx_idle = 0;
        TickType_t tx_timeout = 0;
        TickType_t rx_timeout = 0;
        uint8_t rx_timeout_event = 10;   // default value as defined in uart.c
        uint8_t rx_full_threshold = 120; // default value as defined in uart.c
        uint8_t tx_empty_threshold = 10; // default value as defined in uart.c
        int wakeup_threshold = uart_ll_min_wakeup_thresh() + 1;
        int rx_buffer_size = 1024;
        int tx_buffer_size = 0;
        int queue_size = 32;
        uint32_t stack_size = 2048;
        UBaseType_t task_priority = configMAX_PRIORITIES / 2;
    };
    Uart() = delete;
    Uart(Uart&) = delete;
    Uart(const uart_port_t uart_num);
public:
    static Uart& get_port(const uart_port_t uart_num);
    static void free_port(const uart_port_t uart_num);
    virtual ~Uart();
// Settings
    Uart& config(uint32_t cfg);
    Uart& config(baudrate_t baud, uint32_t cfg);
    Uart& baudrate(baudrate_t baud);
    Uart& word_length(uart_word_length_t length);
    Uart& parity(uart_parity_t par);
    Uart& stopbits(uart_stop_bits_t stop_bits);
    Uart& hw_flow_control(uart_hw_flowcontrol_t mode, uint8_t rx_threshold);
    Uart& clock_source(uart_sclk_t source_clk);
    Uart& sw_flow_control(bool en = true);
    Uart& sw_flow_control(uart_sw_flowctrl_t config);
    Uart& sw_flow_control(uint8_t xon_char, uint8_t xoff_char, uint8_t xon_thrd, uint8_t xoff_thrd);
    Uart& sw_flow_control_symbols(uint8_t xon_char, uint8_t xoff_char);
    Uart& sw_flow_control_thresholds(uint8_t xon_thrd, uint8_t xoff_thrd);
    Uart& mode(uart_mode_t _mode);
    Uart& loopback(bool en = true);
    Uart& pins(int8_t pin_txd, int8_t pin_rxd, uart_signal_inv_t inv = UART_SIGNAL_INV_DISABLE);
    Uart& pins(int8_t pin_txd, int8_t pin_rxd, int8_t pin_rts, int8_t pin_cts, uart_signal_inv_t inv = UART_SIGNAL_INV_DISABLE);
    Uart& pin_txd(int8_t pin, bool inv = false);
    Uart& pin_rxd(int8_t pin, bool inv = false);
    Uart& pin_rts(int8_t pin, bool inv = false);
    Uart& pin_cts(int8_t pin, bool inv = false);
    Uart& tx_idle(uint16_t idle_num);
    Uart& tx_timeout(TickType_t timeout);
    Uart& rx_timeout(TickType_t timeout);
    Uart& rx_timeout_event(uint8_t timeout);
    Uart& rx_full_threshold(uint8_t threshold);
    Uart& tx_empty_threshold(uint8_t threshold);
    Uart& wakeup_threshold(int threshold);
    Uart& rx_buffer_size(int size);
    Uart& tx_buffer_size(int size);
    Uart& queue_size(int size);
    Uart& stack_size(uint32_t size);
    Uart& task_priority(UBaseType_t priority);
    Uart& name(const char* _name);

// Callbacks
    Uart& onData(const callback_t& fcn);
    Uart& onBreak(const callback_t& fcn);
    Uart& onBufferFull(const callback_t& fcn);
    Uart& onFIFOoverflow(const callback_t& fcn);
    Uart& onFrameError(const callback_t& fcn);
    Uart& onParityError(const callback_t& fcn);
    Uart& onDataBreak(const callback_t& fcn);
    Uart& onPattern(const callback_t& fcn);

// Opening
    bool open();
    void close();

    bool is_open();
    bool isOpen();

    void begin();
    //void begin(unsigned long baud, uint32_t config=SERIAL_8N1, int8_t rxPin=-1, int8_t txPin=-1, bool invert=false, unsigned long timeout_ms = 20000UL, uint8_t rxfifo_full_thrhd = 112);
    void end();

// Print
    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);
    size_t write(const uint8_t *buffer, size_t size, uint8_t break_len);
    int availableForWrite();
    bool wait(); // Return true if there is nothing to send i.e. everything already sent. Return false on timeout.
    bool wait(TickType_t timeout);

// Stream
    int available();
    int read();
    size_t read(uint8_t *buffer, size_t size);
    int peek();
    void flush();

// State
    bool rx_transfer_timeout();
    bool collision_flag();

private:
    const uart_port_t m_uart_num;
    Settings m_settings;
    mutex_t m_mutex;
    QueueHandle_t m_queue;
    TaskHandle_t m_task;
    bool m_rx_transfer_timeout;
    bool m_has_peek;
    uint8_t m_peek_byte;
    callback_t m_callbacks[UART_EVENT_MAX];
    char m_name[configMAX_TASK_NAME_LEN];

    bool _apply();
    void _pin_config(int inv, int mask);

    static void process(void* uart_v);
};
