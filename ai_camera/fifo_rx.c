#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "fifo_rx.h"
static int fd = -1;

int fifo_rx_init(const char *path) {
    fd = open(path, O_RDONLY | O_NONBLOCK);
    if (fd < 0) { perror("fifo open"); return -1; }
    printf("[FIFO] ready: %s\n", path);
    return 0;
}
int fifo_rx_read(ai_coord_msg_t *msg) {
    if (fd < 0) return -1;
    int n = read(fd, msg, 40);  /* 覆盖has_target+class_id */
    if (n == 40) return 1;
    if (n < 0 && errno != EAGAIN) { perror("fifo read"); return -1; }
    return 0;
}
void fifo_rx_close(void) { if (fd >= 0) close(fd); }
