#ifndef TRACKER_H
#define TRACKER_H
#include "tcp_server.h"
void tracker_init(void);
int  tracker_update(const ai_coord_msg_t *msg, int ch);
#endif
