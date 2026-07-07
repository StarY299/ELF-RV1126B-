#ifndef TRACKER_H
#define TRACKER_H
#include "ai_msg.h"
void tracker_init(void);
int  tracker_update(const ai_msg_t *msg, int ch);
#endif
