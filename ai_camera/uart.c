/**
 * uart.c — 串口驱动 (合并 serial.c)
 * 使用: sensor.c → uart_open/uart_recv/uart_close
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/time.h>
#include "uart.h"

#ifndef CMSPAR
#define CMSPAR 0
#endif
#ifndef CRTSCTS
#define CRTSCTS 0
#endif

static int uart_fd = -1;

/* ---- 内部: 串口参数设置 ---- */
static int uart_set_opt(int fd, unsigned long speed, unsigned char bits,
                         unsigned char stop, unsigned char check, unsigned char hardware)
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0) { perror("tcgetattr"); return -1; }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (bits) {
    case 5: newtio.c_cflag |= CS5; break;
    case 6: newtio.c_cflag |= CS6; break;
    case 7: newtio.c_cflag |= CS7; break;
    default: newtio.c_cflag |= CS8; break;
    }
    switch (check) {
    case 'O': newtio.c_cflag |= PARENB | PARODD; newtio.c_iflag |= INPCK; break;
    case 'E': newtio.c_cflag |= PARENB; newtio.c_cflag &= ~PARODD; newtio.c_iflag |= INPCK; break;
    default: newtio.c_cflag &= ~PARENB; break;
    }

    speed_t spd = B9600;
    switch (speed) {
    case 600: spd=B600; break; case 1200: spd=B1200; break;
    case 2400: spd=B2400; break; case 4800: spd=B4800; break;
    case 9600: spd=B9600; break; case 19200: spd=B19200; break;
    case 38400: spd=B38400; break; case 57600: spd=B57600; break;
    case 115200: spd=B115200; break; case 230400: spd=B230400; break;
    case 460800: spd=B460800; break; case 921600: spd=B921600; break;
    case 1000000: spd=B1000000; break; case 1152000: spd=B1152000; break;
    default: spd=B9600; break;
    }
    cfsetispeed(&newtio, spd); cfsetospeed(&newtio, spd);

    if (stop == 1) newtio.c_cflag &= ~CSTOPB;
    else if (stop == 2) newtio.c_cflag |= CSTOPB;
    if (hardware == 1) newtio.c_cflag |= CRTSCTS;

    newtio.c_cc[VTIME] = 0; newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &newtio) != 0) { perror("com set error"); return -1; }
    printf("set done!\n");
    return 0;
}

/* ---- 内部: 串口接收 (select超时500us) ---- */
static int uart_recv_raw(int fd, unsigned char *buf, int count)
{
    fd_set rd; FD_ZERO(&rd); FD_SET(fd, &rd);
    struct timeval timeout = {0, 500};
    memset(buf, 0, count);
    int ret = select(fd + 1, &rd, NULL, NULL, &timeout);
    if (ret == 0) return 0;
    if (ret < 0) { printf("select %s\n", strerror(errno)); return -1; }
    return read(fd, buf, count);
}

/* ---- API ---- */
int uart_open(const char *dev, int baudrate)
{
    uart_fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd < 0) { perror("open uart failed"); return -1; }
    if (uart_set_opt(uart_fd, baudrate, 8, 1, 'N', 0) < 0) {
        close(uart_fd); uart_fd = -1; return -1;
    }
    return 0;
}

int uart_send(const unsigned char *buf, int len)
{
    if (uart_fd < 0) return -1;
    int r = write(uart_fd, buf, len);
    if (r == -1) { perror("write"); return 0; }
    return r;
}

int uart_recv(unsigned char *buf, int len)
{
    if (uart_fd < 0) return -1;
    return uart_recv_raw(uart_fd, buf, len);
}

void uart_close(void)
{
    if (uart_fd >= 0) { close(uart_fd); uart_fd = -1; }
}
