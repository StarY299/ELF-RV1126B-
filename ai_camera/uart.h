#ifndef UART_H
#define UART_H

int uart_open(const char *dev, int baudrate);
int uart_send(const unsigned char *buf, int len);
int uart_recv(unsigned char *buf, int len);
void uart_close(void);

#endif