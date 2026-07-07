/**
 * rtsp_service.c — mediamtx 进程管理
 *
 * GStreamer 推流已迁移至 gst_pipeline.c (进程内 C API)。
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include "rtsp_service.h"

static pid_t mediamtx_pid = 0;

pid_t get_mediamtx_pid(void) { return mediamtx_pid; }

int start_mediamtx(void)
{
    printf("[RTSP] Starting mediamtx...\n");

    /* 先清理任何残留的 mediamtx (防止端口冲突) */
    system("killall -9 mediamtx 2>/dev/null");
    usleep(500000);  /* 等端口释放 */

    mediamtx_pid = fork();
    if (mediamtx_pid == -1) {
        perror("[RTSP] fork");
        return -1;
    }
    if (mediamtx_pid == 0) {
        execl("/userdata/mediamtx",
              "mediamtx",
              "/userdata/mediamtx.yml",
              (char *)NULL);
        perror("[RTSP] execl mediamtx");
        _exit(1);
    }

    // 等待 mediamtx 就绪
    sleep(2);

    /* 验证 mediamtx 是否存活 */
    if (kill(mediamtx_pid, 0) != 0) {
        fprintf(stderr, "[RTSP] mediamtx died immediately!\n");
        return -1;
    }

    printf("[RTSP] mediamtx PID=%d\n", mediamtx_pid);
    return 0;
}

int stop_mediamtx(void)
{
    printf("[RTSP] Stopping mediamtx...\n");

    if (mediamtx_pid > 0) {
        kill(mediamtx_pid, SIGTERM);
        waitpid(mediamtx_pid, NULL, 0);
        mediamtx_pid = 0;
    }

    printf("[RTSP] Stopped\n");
    return 0;
}
