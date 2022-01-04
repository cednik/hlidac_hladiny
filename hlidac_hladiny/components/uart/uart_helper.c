/* ESP-IDF file hal/uart_ll.h produces lots of errors, when compiled as C++. So this workaround. */

#include <hal/uart_ll.h>

int uart_get_txfifo_len(const uart_port_t uart_num) {
    return uart_ll_get_txfifo_len(UART_LL_GET_HW(uart_num));
}