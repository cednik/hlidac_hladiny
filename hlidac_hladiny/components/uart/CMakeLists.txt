idf_component_register(
    SRCS "uart.cpp" "uart_helper.c"
    INCLUDE_DIRS "."
    REQUIRES arduino fmt
)