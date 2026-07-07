#ifndef FIFO_RX_H
#define FIFO_RX_H
#include "ai_msg.h"
int  fifo_rx_init(const char *path);
int  fifo_rx_read(ai_msg_t *msg);
void fifo_rx_close(void);
#endif
