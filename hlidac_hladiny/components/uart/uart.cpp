#include "uart.hpp"

#include <esp_err.h>
#include <esp_log.h>
#include <soc/soc_caps.h>
#include <soc/uart_reg.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <cstring>
#include <memory>

#include <fmt/core.h>

static const char* LOG_TAG = "UART";

static std::unique_ptr<Uart> global_instance[UART_NUM_MAX];

Uart& Uart::get_port(const uart_port_t uart_num) {
    if (uart_num >= UART_NUM_MAX) {
        ESP_LOGE(LOG_TAG, "There is no UART %d, only UART0 - UART%d. Aborting.", uart_num, UART_NUM_MAX-1);
        abort();
    }
    if (!global_instance[uart_num])
         global_instance[uart_num].reset(new Uart(uart_num));
    return *global_instance[uart_num];
}

void Uart::free_port(const uart_port_t uart_num) {
    if (uart_num >= UART_NUM_MAX) {
        ESP_LOGW(LOG_TAG, "There is no UART %d for freeing, only UART0 - UART%d. Do nothing.", uart_num, UART_NUM_MAX-1);
    } else if (!global_instance[uart_num]) {
        ESP_LOGW(LOG_TAG, "UART%d is already free.", uart_num);
    } else {
        global_instance[uart_num].reset();
    }
}

Uart::Uart(const uart_port_t uart_num)
    : m_uart_num{uart_num},
      m_settings{},
      m_mutex{},
      m_queue{nullptr},
      m_task{nullptr},
      m_rx_transfer_timeout{true},
      m_has_peek{false},
      m_peek_byte{0}
{
    strncpy(m_name, "UART", configMAX_TASK_NAME_LEN-1);
    if (configMAX_TASK_NAME_LEN > 5)
        m_name[4] = '0' + m_uart_num;
    m_name[configMAX_TASK_NAME_LEN-1] = '\0';
}

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

#define CHECK_MIN(value, minlim, paramname) \
    if (value < minlim) { \
        ESP_LOGE(LOG_TAG, "Unable to set %s %d for %s. Minimum is %d.", paramname, value, m_name, minlim); \
        value = minlim; \
    }
#define CHECK_MAX(value, maxlim, paramname) \
    if (value > maxlim) { \
        ESP_LOGE(LOG_TAG, "Unable to set %s %d for %s. Maximum is %d.", paramname, value, m_name, maxlim); \
        value = maxlim; \
    }
#define CHECK_RANGE(value, minlim, maxlim, paramname) \
    if (value < minlim || value > maxlim) { \
        ESP_LOGE(LOG_TAG, "Unable to set %s %d for %s. It has to be in range <%d; %d>.", paramname, value, m_name, minlim, maxlim); \
        value = value < minlim ? minlim : maxlim; \
    }
#define CHECK_PIN(pin, pinname, check) \
    if (pin != UART_PIN_NO_CHANGE && !check(pin)) { \
        ESP_LOGE(LOG_TAG, "Invalid pin %d as %s of %s. Leaving unchanged.", pin, pinname, m_name); \
        pin = UART_PIN_NO_CHANGE; \
    }

Uart& Uart::baudrate(baudrate_t baud) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_MAX(baud, SOC_UART_BITRATE_MAX, "baudrate");
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
    CHECK_MAX(rx_threshold, SOC_UART_FIFO_LEN - 1, "HW flow control RX threshold");
    m_settings.config.rx_flow_ctrl_thresh = rx_threshold;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_hw_flow_ctrl(m_uart_num, mode, rx_threshold));
    return *this;
}
Uart& Uart::clock_source(uart_sclk_t source_clk) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.config.source_clk = source_clk;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting clock_source on opened UART %s. Please reopen to take effect.", m_name);
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
    ESP_LOGE(LOG_TAG, "Settings software flow control symbols is not implemented in supported version of IDF (v4.4) [UART %s].", m_name);
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_MAX(xon_thrd, SOC_UART_FIFO_LEN - 1, "SW flow control XON threshold");
    CHECK_MAX(xoff_thrd, SOC_UART_FIFO_LEN - 1, "SW flow control XOFF threshold");
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
    ESP_LOGE(LOG_TAG, "Settings software flow control symbols is not implemented in supported version of IDF (v4.4) [UART %s].", m_name);
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.sw_flowctrl.xon_char = xon_char;
    m_settings.sw_flowctrl.xoff_char = xoff_char;
    return *this;
}
Uart& Uart::sw_flow_control_thresholds(uint8_t xon_thrd, uint8_t xoff_thrd) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_MAX(xon_thrd, SOC_UART_FIFO_LEN - 1, "SW flow control XON threshold");
    CHECK_MAX(xoff_thrd, SOC_UART_FIFO_LEN - 1, "SW flow control XOFF threshold");
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
Uart& Uart::loopback(bool en) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.loopback = en;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_loop_back(m_uart_num, en));
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
Uart& Uart::pins(int8_t pin_txd, int8_t pin_rxd, uart_signal_inv_t inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_PIN(pin_txd, "TXD", GPIO_IS_VALID_OUTPUT_GPIO);
    CHECK_PIN(pin_rxd, "RXD", GPIO_IS_VALID_GPIO);
    m_settings.pin_txd = pin_txd;
    m_settings.pin_rxd = pin_rxd;
    _pin_config(inv,  UART_SIGNAL_IRDA_TX_INV
                    | UART_SIGNAL_IRDA_RX_INV
                    | UART_SIGNAL_RXD_INV
                    | UART_SIGNAL_TXD_INV );
    return *this;
}
Uart& Uart::pins(int8_t pin_txd, int8_t pin_rxd, int8_t pin_rts, int8_t pin_cts, uart_signal_inv_t inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_PIN(pin_txd, "TXD", GPIO_IS_VALID_OUTPUT_GPIO);
    CHECK_PIN(pin_rxd, "RXD", GPIO_IS_VALID_GPIO);
    CHECK_PIN(pin_rts, "RTS", GPIO_IS_VALID_OUTPUT_GPIO);
    CHECK_PIN(pin_cts, "CTS", GPIO_IS_VALID_GPIO);
    m_settings.pin_txd = pin_txd;
    m_settings.pin_rxd = pin_rxd;
    m_settings.pin_rts = pin_rts;
    m_settings.pin_cts = pin_cts;
    _pin_config(inv,  UART_SIGNAL_IRDA_TX_INV
                    | UART_SIGNAL_IRDA_RX_INV
                    | UART_SIGNAL_RXD_INV
                    | UART_SIGNAL_TXD_INV
                    | UART_SIGNAL_RTS_INV
                    | UART_SIGNAL_CTS_INV );
    return *this;
}
Uart& Uart::pin_txd(int8_t pin, bool inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_PIN(pin, "TXD", GPIO_IS_VALID_OUTPUT_GPIO);
    m_settings.pin_txd = pin;
    const int inv_mask = UART_SIGNAL_IRDA_TX_INV | UART_SIGNAL_TXD_INV;
    _pin_config(inv ? inv_mask : 0, inv_mask);
    return *this;
}
Uart& Uart::pin_rxd(int8_t pin, bool inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_PIN(pin, "RXD", GPIO_IS_VALID_GPIO);
    m_settings.pin_rxd = pin;
    const int inv_mask = UART_SIGNAL_IRDA_RX_INV | UART_SIGNAL_RXD_INV;
    _pin_config(inv ? inv_mask : 0, inv_mask);
    return *this;
}
Uart& Uart::pin_rts(int8_t pin, bool inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_PIN(pin, "RTS", GPIO_IS_VALID_OUTPUT_GPIO);
    m_settings.pin_rts = pin;
    const int inv_mask = UART_SIGNAL_RTS_INV;
    _pin_config(inv ? inv_mask : 0, inv_mask);
    return *this;
}
Uart& Uart::pin_cts(int8_t pin, bool inv) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_PIN(pin, "CTS", GPIO_IS_VALID_GPIO);
    m_settings.pin_cts = pin;
    const int inv_mask = UART_SIGNAL_CTS_INV;
    _pin_config(inv ? inv_mask : 0, inv_mask);
    return *this;
}
Uart& Uart::tx_idle(uint16_t idle_num) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_MAX(idle_num, UART_TX_IDLE_NUM_V, "TX idle");
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
    // FIX ME: add CHECK_MAX (underlaing driver uses undocumented uart_hal API).
    m_settings.rx_timeout_event = timeout;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_rx_timeout(m_uart_num, timeout));
    return *this;
}
Uart& Uart::rx_full_threshold(uint8_t threshold) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_MAX(threshold, UART_RXFIFO_FULL_THRHD_V - 1, "RX FIFO full threshold");
    m_settings.rx_full_threshold = threshold;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_rx_full_threshold(m_uart_num, threshold));
    return *this;
}
Uart& Uart::tx_empty_threshold(uint8_t threshold) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.tx_empty_threshold = threshold;
    CHECK_MAX(threshold, UART_TXFIFO_EMPTY_THRHD_V - 1, "TX FIFO empty threshold");
    if (_apply())
        ESP_ERROR_CHECK(uart_set_tx_empty_threshold(m_uart_num, threshold));
    return *this;
}
Uart& Uart::wakeup_threshold(int threshold) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_RANGE(threshold, uart_ll_min_wakeup_thresh(), UART_ACTIVE_THRESHOLD_V - 1, "wakeup threshold");
    m_settings.wakeup_threshold = threshold;
    if (_apply())
        ESP_ERROR_CHECK(uart_set_wakeup_threshold(m_uart_num, threshold));
    return *this;
}
Uart& Uart::rx_buffer_size(int size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_MIN(size, SOC_UART_FIFO_LEN + 1, "RX buffer size");
    m_settings.rx_buffer_size = size;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting rx_buffer_size on opened UART %s. Please reopen to take effect.", m_name);
    return *this;
}
Uart& Uart::tx_buffer_size(int size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    CHECK_MIN(size, SOC_UART_FIFO_LEN + 1, "TX buffer size");
    m_settings.tx_buffer_size = size;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting tx_buffer_size on opened UART %s. Please reopen to take effect.", m_name);
    return *this;
}
Uart& Uart::queue_size(int size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.queue_size = size;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting queue_size on opened UART %s. Please reopen to take effect.", m_name);
    return *this;
}
Uart& Uart::stack_size(uint32_t size) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.stack_size = size;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting stack_size on opened UART %s. Please reopen to take effect.", m_name);
    return *this;
}
Uart& Uart::task_priority(UBaseType_t priority) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_settings.task_priority = priority;
    if (_apply())
        ESP_LOGW(LOG_TAG, "Setting task_priority on opened UART %s. Please reopen to take effect.", m_name);
    return *this;
}
Uart& Uart::name(const char* _name) {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (is_open()) {
        ESP_LOGE(LOG_TAG, "Can not change name of opened UART %s. Please close it first.", m_name);
    } else {
        strncpy(m_name, _name, configMAX_TASK_NAME_LEN-1);
        m_name[configMAX_TASK_NAME_LEN-1] = '\0';
    }
    return *this;
}

// Callbacks
Uart& Uart::onData(const callback_t& fcn) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_callbacks[UART_DATA] = fcn;
    return *this;
}
Uart& Uart::onBreak(const callback_t& fcn) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_callbacks[UART_BREAK] = fcn;
    return *this;
}
Uart& Uart::onBufferFull(const callback_t& fcn) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_callbacks[UART_BUFFER_FULL] = fcn;
    return *this;
}
Uart& Uart::onFIFOoverflow(const callback_t& fcn) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_callbacks[UART_FIFO_OVF] = fcn;
    return *this;
}
Uart& Uart::onFrameError(const callback_t& fcn) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_callbacks[UART_FRAME_ERR] = fcn;
    return *this;
}
Uart& Uart::onParityError(const callback_t& fcn) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_callbacks[UART_PARITY_ERR] = fcn;
    return *this;
}
Uart& Uart::onDataBreak(const callback_t& fcn) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_callbacks[UART_DATA_BREAK] = fcn;
    return *this;
}
Uart& Uart::onPattern(const callback_t& fcn) {
    std::lock_guard<mutex_t> lock (m_mutex);
    m_callbacks[UART_PATTERN_DET] = fcn;
    return *this;
}

// Opening
bool Uart::open() {
    std::lock_guard<mutex_t> lock (m_mutex);
    close();

    /*fmt::print("UART{} settings:\n", m_uart_num);
    fmt::print("\t{:.<32}{}\n", "baud_rate", m_settings.config.baud_rate);
    fmt::print("\t{:.<32}{}\n", "data_bits", m_settings.config.data_bits);
    fmt::print("\t{:.<32}{}\n", "parity", m_settings.config.parity);
    fmt::print("\t{:.<32}{}\n", "stop_bits", m_settings.config.stop_bits);
    fmt::print("\t{:.<32}{}\n", "flow_ctrl", m_settings.config.flow_ctrl);
    fmt::print("\t{:.<32}{}\n", "rx_flow_ctrl_thresh", m_settings.config.rx_flow_ctrl_thresh);
    fmt::print("\t{:.<32}{}\n", "source_clk", m_settings.config.source_clk);
    fmt::print("\t{:.<32}{}\n", "xon_char", m_settings.sw_flowctrl.xon_char);
    fmt::print("\t{:.<32}{}\n", "xoff_char", m_settings.sw_flowctrl.xoff_char);
    fmt::print("\t{:.<32}{}\n", "xon_thrd", m_settings.sw_flowctrl.xon_thrd);
    fmt::print("\t{:.<32}{}\n", "xoff_thrd", m_settings.sw_flowctrl.xoff_thrd);
    fmt::print("\t{:.<32}{}\n", "sw_flowctrl_en", m_settings.sw_flowctrl_en);
    fmt::print("\t{:.<32}{}\n", "mode", m_settings.mode);
    fmt::print("\t{:.<32}{}\n", "loopback", m_settings.loopback);
    fmt::print("\t{:.<32}{}\n", "signal_inv", m_settings.signal_inv);
    fmt::print("\t{:.<32}{}\n", "pin_txd", m_settings.pin_txd);
    fmt::print("\t{:.<32}{}\n", "pin_rxd", m_settings.pin_rxd);
    fmt::print("\t{:.<32}{}\n", "pin_rts", m_settings.pin_rts);
    fmt::print("\t{:.<32}{}\n", "pin_cts", m_settings.pin_cts);
    fmt::print("\t{:.<32}{}\n", "tx_idle", m_settings.tx_idle);
    fmt::print("\t{:.<32}{}\n", "tx_timeout", m_settings.tx_timeout);
    fmt::print("\t{:.<32}{}\n", "rx_timeout", m_settings.rx_timeout);
    fmt::print("\t{:.<32}{}\n", "rx_timeout_event", m_settings.rx_timeout_event);
    fmt::print("\t{:.<32}{}\n", "rx_full_threshold", m_settings.rx_full_threshold);
    fmt::print("\t{:.<32}{}\n", "tx_empty_threshold", m_settings.tx_empty_threshold);
    fmt::print("\t{:.<32}{}\n", "wakeup_threshold", m_settings.wakeup_threshold);
    fmt::print("\t{:.<32}{}\n", "rx_buffer_size", m_settings.rx_buffer_size);
    fmt::print("\t{:.<32}{}\n", "tx_buffer_size", m_settings.tx_buffer_size);
    fmt::print("\t{:.<32}{}\n", "queue_size", m_settings.queue_size);
    fmt::print("\t{:.<32}{}\n", "stack_size", m_settings.stack_size);
    fmt::print("\t{:.<32}{}\n", "task_priority", m_settings.task_priority);
    fmt::print("\t{:.<32}{}\n", "name", name);*/

    ESP_ERROR_CHECK(uart_driver_install(m_uart_num, m_settings.rx_buffer_size, m_settings.tx_buffer_size, m_settings.queue_size, &m_queue, ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(uart_param_config(m_uart_num, &m_settings.config));
    ESP_ERROR_CHECK(uart_set_pin(m_uart_num, m_settings.pin_txd, m_settings.pin_rxd, m_settings.pin_rts, m_settings.pin_cts));
    ESP_ERROR_CHECK(uart_set_line_inverse(m_uart_num, m_settings.signal_inv));
    ESP_ERROR_CHECK(uart_set_sw_flow_ctrl(m_uart_num, m_settings.sw_flowctrl_en, m_settings.sw_flowctrl.xon_thrd, m_settings.sw_flowctrl.xoff_thrd));
    ESP_ERROR_CHECK(uart_set_mode(m_uart_num, m_settings.mode));
    ESP_ERROR_CHECK(uart_set_tx_idle_num(m_uart_num, m_settings.tx_idle));
    ESP_ERROR_CHECK(uart_set_rx_timeout(m_uart_num, m_settings.rx_timeout_event));
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(m_uart_num, m_settings.rx_full_threshold));
    ESP_ERROR_CHECK(uart_set_tx_empty_threshold(m_uart_num, m_settings.tx_empty_threshold));
    ESP_ERROR_CHECK(uart_set_wakeup_threshold(m_uart_num, m_settings.wakeup_threshold));
    ESP_ERROR_CHECK(uart_set_loop_back(m_uart_num, m_settings.loopback));
    BaseType_t err = xTaskCreate(process,
                                 m_name,
                                 m_settings.stack_size,
                                 this,
                                 m_settings.task_priority,
                                 &m_task );
    if (err != pdPASS) {
        ESP_LOGE(LOG_TAG, "Error 0x%X occured while starting process task of UART %s.", err, m_name);
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

size_t Uart::write(const uint8_t *buffer, size_t size, uint8_t break_len) {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!is_open())
        return 0;
    return uart_write_bytes_with_break(m_uart_num, buffer, size, break_len);
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

// State
bool Uart::rx_transfer_timeout() {
    std::lock_guard<mutex_t> lock (m_mutex);
    return m_rx_transfer_timeout;
}

bool Uart::collision_flag() {
    std::lock_guard<mutex_t> lock (m_mutex);
    if (!is_open())
        return false;
    bool v = false;
    ESP_ERROR_CHECK(uart_get_collision_flag(m_uart_num, &v));
    return v;
}

// static
void Uart::process(void* uart_v) {
    Uart& uart = *static_cast<Uart*>(uart_v);
    uart_event_t event;
    for (;;) {
        if (xQueueReceive(uart.m_queue, &event, portMAX_DELAY)) {
            //fmt::print("Event ID {}, size {}, timeouf flag {}\n", int(event.type), event.size, event.timeout_flag);
            ESP_LOGD(LOG_TAG, "%s event type %d, size %u, timeouf flag %d", uart.m_name, int(event.type), event.size, int(event.timeout_flag));
            switch (event.type) {
            case UART_DATA:
                uart.m_rx_transfer_timeout = event.timeout_flag;
                break;
            default:
                break;
            }
            if (event.type < UART_EVENT_MAX) {
                if (uart.m_callbacks[event.type]) {
                    uart.m_callbacks[event.type](uart);
                }
            }
        }
    }
    vTaskDelete(nullptr);
}