#include "uart.hpp"

#include <esp_err.h>
#include <esp_log.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <cstring>

#include <fmt/core.h>

static const char* LOG_TAG = "UART";

Uart::Uart(const uart_port_t uart_num)
    : m_uart_num{uart_num},
      m_settings{},
      m_mutex{},
      m_queue{nullptr},
      m_task{nullptr},
      m_has_peek{false},
      m_peek_byte{0}
{}

Uart::~Uart() {
    close();
}

bool Uart::_apply() {
    if (is_open()) {
        wait();
        return true;
    } else {
        return false;
    }
}

Uart& Uart::config(uint32_t cfg) {
    std::lock_guard<mutex_t> lock (m_mutex);
    word_length(uart_word_length_t((cfg & 0x0C) >> 2));
    parity     (uart_parity_t     ((cfg & 0x03) >> 0));
    stopbits   (uart_stop_bits_t  ((cfg & 0x30) >> 4));
    return *this;
}
Uart& Uart::config(baudrate_t baud, uint32_t cfg) {
    std::lock_guard<mutex_t> lock (m_mutex);
    baudrate(baud);
    config(cfg);
    return *this;
}
Uart& Uart::baudrate(baudrate_t baud) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.config.baud_rate = baud;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_baudrate(m_uart_num, baud));
    return *this;
}
Uart& Uart::word_length(uart_word_length_t length) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.config.data_bits = length;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_word_length(m_uart_num, length));
    return *this;
}
Uart& Uart::parity(uart_parity_t par) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.config.parity = par;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_parity(m_uart_num, par));
    return *this;
}
Uart& Uart::stopbits(uart_stop_bits_t stop_bits) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.config.stop_bits = stop_bits;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_stop_bits(m_uart_num, stop_bits));
    return *this;
}
Uart& Uart::hw_flow_control(uart_hw_flowcontrol_t mode, uint8_t rx_threshold) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.config.flow_ctrl = mode;
    m_settings.config.rx_flow_ctrl_thresh = rx_threshold;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_hw_flow_ctrl(m_uart_num, mode, rx_threshold));
    return *this;
}
Uart& Uart::clock_source(uart_sclk_t source_clk) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.config.source_clk = source_clk;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting clock_source on opened UART %s. Please reopen to take effect.", m_settings.name);
    return *this;
}
Uart& Uart::sw_flow_control(bool en) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.sw_flowctrl_en = en;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_sw_flow_ctrl(m_uart_num, en, m_settings.sw_flowctrl.xon_thrd, m_settings.sw_flowctrl.xoff_thrd));
    return *this;
}
Uart& Uart::sw_flow_control(uart_sw_flowctrl_t config) {
    sw_flow_control(config.xon_char, config.xoff_char, config.xon_thrd, config.xoff_thrd);
    return *this;
}
Uart& Uart::sw_flow_control(uint8_t xon_char, uint8_t xoff_char, uint8_t xon_thrd, uint8_t xoff_thrd) {
    ESP_LOGE(LOG_TAG, "Settings software flow control symbols is not implemented in supported version of IDF (v4.4) [UART %s].", m_settings.name);
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.sw_flowctrl.xon_char = xon_char;
    m_settings.sw_flowctrl.xoff_char = xoff_char;
    m_settings.sw_flowctrl.xon_thrd = xon_thrd;
    m_settings.sw_flowctrl.xoff_thrd = xoff_thrd;
    m_settings.sw_flowctrl_en = true;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_sw_flow_ctrl(m_uart_num, m_settings.sw_flowctrl_en, m_settings.sw_flowctrl.xon_thrd, m_settings.sw_flowctrl.xoff_thrd));
    return *this;
}
Uart& Uart::sw_flow_control_symbols(uint8_t xon_char, uint8_t xoff_char) {
    ESP_LOGE(LOG_TAG, "Settings software flow control symbols is not implemented in supported version of IDF (v4.4) [UART %s].", m_settings.name);
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.sw_flowctrl.xon_char = xon_char;
    m_settings.sw_flowctrl.xoff_char = xoff_char;
    return *this;
}
Uart& Uart::sw_flow_control_thresholds(uint8_t xon_thrd, uint8_t xoff_thrd) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.sw_flowctrl.xon_thrd = xon_thrd;
    m_settings.sw_flowctrl.xoff_thrd = xoff_thrd;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_sw_flow_ctrl(m_uart_num, m_settings.sw_flowctrl_en, m_settings.sw_flowctrl.xon_thrd, m_settings.sw_flowctrl.xoff_thrd));
    return *this;
}
Uart& Uart::mode(uart_mode_t _mode) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.mode = _mode;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_mode(m_uart_num, _mode));
    return *this;
}
void Uart::_pin_config(int inv, int mask) {
    m_settings.signal_inv = uart_signal_inv_t((m_settings.signal_inv & ~mask) | (inv & mask));
    if (_apply()) {
        ESP_ERROR_CHECK(uart_set_pin(m_uart_num,
            m_settings.pin_txd,
            m_settings.pin_rxd,
            m_settings.pin_rts,
            m_settings.pin_cts ));
        ESP_ERROR_CHECK(uart_set_line_inverse(m_uart_num, m_settings.signal_inv));
    }
}
Uart& Uart::pins(int pin_txd, int pin_rxd, uart_signal_inv_t inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.pin_txd = pin_txd;
    m_settings.pin_rxd = pin_rxd;
    _pin_config(inv,  UART_SIGNAL_IRDA_TX_INV
                    | UART_SIGNAL_IRDA_RX_INV
                    | UART_SIGNAL_RXD_INV
                    | UART_SIGNAL_TXD_INV );
    return *this;
}
Uart& Uart::pins(int pin_txd, int pin_rxd, int pin_rts, int pin_cts, uart_signal_inv_t inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.pin_txd = pin_txd;
    m_settings.pin_rxd = pin_rxd;
    m_settings.pin_rxd = pin_rts;
    m_settings.pin_rxd = pin_cts;
    _pin_config(inv,  UART_SIGNAL_IRDA_TX_INV
                    | UART_SIGNAL_IRDA_RX_INV
                    | UART_SIGNAL_RXD_INV
                    | UART_SIGNAL_TXD_INV
                    | UART_SIGNAL_RTS_INV
                    | UART_SIGNAL_CTS_INV );
    return *this;
}
Uart& Uart::pin_txd(int pin, bool inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.pin_txd = pin;
    const int inv_mask = UART_SIGNAL_IRDA_TX_INV | UART_SIGNAL_TXD_INV;
    _pin_config(inv ? inv_mask : 0, inv_mask);
    return *this;
}
Uart& Uart::pin_rxd(int pin, bool inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.pin_rxd = pin;
    const int inv_mask = UART_SIGNAL_IRDA_RX_INV | UART_SIGNAL_RXD_INV;
    _pin_config(inv ? inv_mask : 0, inv_mask);
    return *this;
}
Uart& Uart::pin_rts(int pin, bool inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.pin_rts = pin;
    const int inv_mask = UART_SIGNAL_RTS_INV;
    _pin_config(inv ? inv_mask : 0, inv_mask);
    return *this;
}
Uart& Uart::pin_cts(int pin, bool inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.pin_cts = pin;
    const int inv_mask = UART_SIGNAL_CTS_INV;
    _pin_config(inv ? inv_mask : 0, inv_mask);
    return *this;
}
Uart& Uart::tx_idle(uint16_t idle_num) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.tx_idle = idle_num;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_tx_idle_num(m_uart_num, idle_num));
    return *this;
}
Uart& Uart::tx_timeout(TickType_t timeout) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.tx_timeout = timeout;
    return *this;
}
Uart& Uart::rx_timeout(TickType_t timeout) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.rx_timeout = timeout;
    return *this;
}
Uart& Uart::rx_timeout_event(uint8_t timeout) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.rx_timeout_event = timeout;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_rx_timeout(m_uart_num, timeout));
    return *this;
}
Uart& Uart::rx_full_threshold(int threshold) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.rx_full_threshold = threshold;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_rx_full_threshold(m_uart_num, threshold));
    return *this;
}
Uart& Uart::tx_empty_threshold(int threshold) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.tx_empty_threshold = threshold;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_tx_empty_threshold(m_uart_num, threshold));
    return *this;
}
Uart& Uart::wakeup_threshold(int threshold) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.wakeup_threshold = threshold;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_wakeup_threshold(m_uart_num, threshold));
    return *this;
}
Uart& Uart::rx_buffer_size(int size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.rx_buffer_size = size;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting rx_buffer_size on opened UART %s. Please reopen to take effect.", m_settings.name);
    return *this;
}
Uart& Uart::tx_buffer_size(int size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.tx_buffer_size = size;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting tx_buffer_size on opened UART %s. Please reopen to take effect.", m_settings.name);
    return *this;
}
Uart& Uart::queue_size(int size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.queue_size = size;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting queue_size on opened UART %s. Please reopen to take effect.", m_settings.name);
    return *this;
}
Uart& Uart::stack_size(uint32_t size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.stack_size = size;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting stack_size on opened UART %s. Please reopen to take effect.", m_settings.name);
    return *this;
}
Uart& Uart::task_priority(UBaseType_t priority) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.task_priority = priority;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting task_priority on opened UART %s. Please reopen to take effect.", m_settings.name);
    return *this;
}
Uart& Uart::name(const char* _name) {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (is_open()) {
        ESP_LOGE(LOG_TAG, "Can not change name of opened UART %s. Please close it first.", m_settings.name);
    } else {
        strncpy(m_settings.name, _name, configMAX_TASK_NAME_LEN-1);
        m_settings.name[configMAX_TASK_NAME_LEN-1] = '\0';
    }
    return *this;
}

// Opening
bool Uart::open() {
    std::lock_guard<mutex_t> lock (m_mutex);
    close();
    ESP_ERROR_CHECK(uart_driver_install(m_uart_num, m_settings.rx_buffer_size, m_settings.tx_buffer_size, m_settings.queue_size, &m_queue, ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(uart_param_config(m_uart_num, &m_settings.config));
    _pin_config(m_settings.signal_inv, 0xFFFFFFFF);
    ESP_ERROR_CHECK(uart_set_sw_flow_ctrl(m_uart_num, m_settings.sw_flowctrl_en, m_settings.sw_flowctrl.xon_thrd, m_settings.sw_flowctrl.xoff_thrd));
    ESP_ERROR_CHECK(uart_set_mode(m_uart_num, m_settings.mode));
    ESP_ERROR_CHECK(uart_set_tx_idle_num(m_uart_num, m_settings.tx_idle));
    ESP_ERROR_CHECK(uart_set_rx_timeout(m_uart_num, m_settings.rx_timeout_event));
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(m_uart_num, m_settings.rx_full_threshold));
    ESP_ERROR_CHECK(uart_set_tx_empty_threshold(m_uart_num, m_settings.tx_empty_threshold));
    ESP_ERROR_CHECK(uart_set_wakeup_threshold(m_uart_num, m_settings.wakeup_threshold));
    BaseType_t err = xTaskCreate(process,
                                 m_settings.name,
                                 m_settings.stack_size,
                                 this,
                                 m_settings.task_priority,
                                 &m_task );
    if (err != pdPASS) {
        ESP_LOGE(LOG_TAG, "Error 0x%X occured while starting process task of UART %s.", err, m_settings.name);
        m_task = nullptr;
        close();
        return false;
    }
    return true;
}

void Uart::close() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (_apply()) {
        while (m_task) {
            vTaskDelete(m_task);
            m_task = nullptr;
        }
        ESP_ERROR_CHECK(uart_driver_delete(m_uart_num));
        m_queue = nullptr;
        m_has_peek = false;
    }
}

bool Uart::is_open() {
    return uart_is_driver_installed(m_uart_num);
}

bool Uart::isOpen() {
    return is_open();
}

void Uart::begin() {
    open();
}

void Uart::end() {
    close();
}

// Print

size_t Uart::write(uint8_t v) {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!is_open())
        return 0;
    return uart_write_bytes(m_uart_num, &v, 1);
}

size_t Uart::write(const uint8_t *buffer, size_t size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!is_open())
        return 0;
    return uart_write_bytes(m_uart_num, buffer, size);
}

extern "C" int uart_get_txfifo_len(const uart_port_t uart_num); // See uart_helper.c

int Uart::availableForWrite() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!is_open())
        return 0;
    return uart_get_txfifo_len(m_uart_num);
}

bool Uart::wait() {
    std::lock_guard<mutex_t> lock (m_mutex);
    return wait(m_settings.tx_timeout);
}

bool Uart::wait(TickType_t timeout) {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!is_open())
        return true;
    esp_err_t err = uart_wait_tx_done(m_uart_num, timeout);
    switch (err) {
        case ESP_OK: return true;
        case ESP_ERR_TIMEOUT: return false;
        default: ESP_ERROR_CHECK(err); // uart_wait_tx_done(m_uart_num, timeout);
    }
    return false;
}

// Stream
int Uart::available() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!is_open())
        return 0;
    size_t res = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(m_uart_num, &res));
    return m_has_peek ? (res + 1) : res;
}

int Uart::read() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!is_open())
        return -1;
    if (m_has_peek) {
        m_has_peek = false;
    } else {
        if (uart_read_bytes(m_uart_num, &m_peek_byte, 1, m_settings.rx_timeout) < 1)
            return -1;
    }
    return m_peek_byte;
}

size_t Uart::read(uint8_t *buffer, size_t size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!is_open())
        return 0;
    size_t read_len = 0;
    if (m_has_peek) {
        m_has_peek = false;
        *(buffer++) = m_peek_byte;
        --size;
        ++read_len;
    }
    read_len += uart_read_bytes(m_uart_num, buffer, size, m_settings.rx_timeout);
    return read_len;
}

int Uart::peek() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!is_open())
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
    if (!is_open())
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