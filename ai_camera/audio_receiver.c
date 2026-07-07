/**
 * udp_audio.c — UDP 远程喊话接收线程
 * 监听 UDP :9998, 接收 PCM 音频 → pipe 到 aplay 播放
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include "audio_receiver.h"

#define AUDIO_PORT 9998
#define GAIN       8.0f

static pthread_t    g_tid;
static volatile int g_running = 0;

static void amplify(int16_t *samples, int count, float gain) {
    for (int i = 0; i < count; i++) {
        float v = samples[i] * gain;
        if (v > 32767) v = 32767;
        else if (v < -32768) v = -32768;
        samples[i] = (int16_t)v;
    }
}

static void *audio_thread(void *arg) {
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("[AUDIO] socket"); return NULL; }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(AUDIO_PORT),
        .sin_addr.s_addr = INADDR_ANY
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[AUDIO] bind"); close(sock); return NULL;
    }

    struct timeval tv = { .tv_sec = 0, .tv_usec = 500000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    printf("[AUDIO] listening on UDP :%d (gain=%.1fx)\n", AUDIO_PORT, GAIN);

    FILE *aplay = NULL;
    int16_t buf[2048];

    while (g_running) {
        ssize_t n = recv(sock, buf, sizeof(buf), 0);
        if (n <= 0) {
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) break;
            continue;
        }

        amplify(buf, n / 2, GAIN);

        if (!aplay || ferror(aplay)) {
            if (aplay) pclose(aplay);
            aplay = popen("aplay -q -f S16_LE -c 1 -r 16000 - 2>/dev/null", "w");
            if (!aplay) continue;
        }

        fwrite(buf, 1, n, aplay);
        fflush(aplay);
    }

    if (aplay) pclose(aplay);
    close(sock);
    printf("[AUDIO] stopped\n");
    return NULL;
}

int udp_audio_init(void) {
    g_running = 1;
    if (pthread_create(&g_tid, NULL, audio_thread, NULL) != 0) {
        perror("[AUDIO] pthread_create");
        g_running = 0; return -1;
    }
    return 0;
}

void udp_audio_deinit(void) {
    g_running = 0;
    pthread_join(g_tid, NULL);
}
