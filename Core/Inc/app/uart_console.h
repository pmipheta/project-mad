/* uart_console.h - USART3 console: RX interrupt ring buffer, status panel, command parser. */
#ifndef APP_UART_CONSOLE_H
#define APP_UART_CONSOLE_H

#include "app/app_config.h"

void UartPrint(const char *s);
void UartInputInit(void);
void UartPanel(void);
void UartPoll(void);

#endif /* APP_UART_CONSOLE_H */
