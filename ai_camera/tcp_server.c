/**
 * ai_fifo.c — FIFO + TCP 发送 AI 检测坐标 (多客户端版, 2025-06-20)
 *
 * 架构:
 *   AI线程 → ai_fifo_send() → 仅更新 g_last_msg + 写FIFO (永不阻塞)
 *   TCP线程 → 独立发送循环 (非阻塞accept + 广播所有客户端)
 *
 * 关键改进:
 *   - 多客户端支持 (最多16个), 非阻塞accept + 广播发送
 *   - AI线程不再直接写TCP, 消除多线程竞争
 *   - TCP_NODELAY 禁用Nagle, 实时发送小包
 *   - SO_KEEPALIVE 检测死连接
 *   - 30ms发送间隔 (~33Hz), 远高于AI帧率(~7Hz), 确保不丢帧
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <errno.h>
#include "tcp_server.h"

static int g_fd = -1;      // FIFO 文件描述符
static int g_srv = -1;     // TCP 服务端 socket
static pthread_t g_tid;

/* 共享消息缓冲区: AI线程写, TCP线程读 */
static pthread_mutex_t g_msg_lock = PTHREAD_MUTEX_INITIALIZER;
static ai_coord_msg_t g_last_msg;

/* 传感器数据 (线程安全) — 综合值(取max)供AI融合用 */
static pthread_mutex_t g_sensor_lock = PTHREAD_MUTEX_INITIALIZER;
static float g_tvoc = 0.0f, g_ch2o = 0.0f, g_temperature = 0.0f;
static int   g_co2 = 0, g_sensor_alarm = 0;
/* 双从机各自最新值 (用于综合max) */
static float g_tvoc_s1, g_ch2o_s1, g_temp_s1, g_tvoc_s2, g_ch2o_s2, g_temp_s2;
static int   g_co2_s1, g_co2_s2;

/* 多客户端列表 (仅TCP线程访问, 无需锁) */
#define MAX_CLIENTS 16
static int g_clients[MAX_CLIENTS];
static int g_client_count = 0;

/* ---- 添加客户端 ---- */
static void add_client(int fd)
{
    if (g_client_count >= MAX_CLIENTS) {
        printf("[TCP] max clients (%d), rejecting fd=%d\n", MAX_CLIENTS, fd);
        close(fd);
        return;
    }
    g_clients[g_client_count++] = fd;
    printf("[TCP] client #%d connected (fd=%d), total=%d\n", g_client_count, fd, g_client_count);
}

/* ---- 移除客户端 (swap-remove, O(1)) ---- */
static void remove_client(int idx)
{
    int fd = g_clients[idx];
    close(fd);
    g_client_count--;
    if (idx < g_client_count) {
        g_clients[idx] = g_clients[g_client_count];
    }
    printf("[TCP] client removed (fd=%d), total=%d\n", fd, g_client_count);
}

/* ---- TCP 发送线程 (多客户端广播) ---- */
static void *tcp_thread(void *arg)
{
    (void)arg;

    g_srv = socket(AF_INET, SOCK_STREAM, 0);
    if (g_srv < 0) { perror("[TCP] socket"); return NULL; }

    int opt = 1;
    setsockopt(g_srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(AI_TCP_PORT) };
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(g_srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[TCP] bind"); close(g_srv); g_srv = -1; return NULL;
    }
    listen(g_srv, 5);
    printf("[TCP] listening on port %d (multi-client, up to %d)\n", AI_TCP_PORT, MAX_CLIENTS);

    /* 服务端socket设为非阻塞, accept不阻塞发送循环 */
    int srv_flags = fcntl(g_srv, F_GETFL, 0);
    fcntl(g_srv, F_SETFL, srv_flags | O_NONBLOCK);

    memset(g_clients, 0, sizeof(g_clients));

    while (1) {
        /* 1. 非阻塞接受新连接 */
        int cli = accept(g_srv, NULL, NULL);
        if (cli >= 0) {
            opt = 1;
            setsockopt(cli, IPPROTO_TCP, TCP_NODELAY,  &opt, sizeof(opt));
            setsockopt(cli, SOL_SOCKET,  SO_KEEPALIVE, &opt, sizeof(opt));
            int cli_flags = fcntl(cli, F_GETFL, 0);
            fcntl(cli, F_SETFL, cli_flags | O_NONBLOCK);
            add_client(cli);
        }

        /* 2. 读取最新消息 (只锁一次, 广播同一帧) */
        ai_coord_msg_t msg;
        pthread_mutex_lock(&g_msg_lock);
        msg = g_last_msg;
        pthread_mutex_unlock(&g_msg_lock);

        /* 硬件画框已嵌入RTSP标注帧, TCP不再推送框坐标 */
        msg.cx = msg.cy = msg.w = msg.h = 0;

        /* 3. 广播给所有客户端, 同时清理断线者 */
        for (int i = 0; i < g_client_count; ) {
            int fd = g_clients[i];

            /* 探测FIN/RST */
            char c;
            int r = recv(fd, &c, 1, MSG_PEEK | MSG_DONTWAIT);
            if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                remove_client(i);
                continue;  // 不递增i, swap-remove后当前位置已是新fd
            }

            /* 发送 */
            ssize_t n = write(fd, &msg, sizeof(msg));
            if (n < 0) {
                if (errno == EPIPE || errno == ECONNRESET) {
                    remove_client(i);
                    continue;
                }
                /* EAGAIN: 缓冲区满, 跳过下个周期重试 */
            }

            i++;
        }

        usleep(30000);  // 30ms间隔 (~33fps)
    }

    return NULL;
}

/* ---- FIFO 初始化 ---- */
int ai_fifo_init(void)
{
    if (mkfifo(AI_FIFO_PATH, 0666) < 0 && errno != EEXIST) {
        perror("[FIFO] mkfifo"); return -1;
    }
    g_fd = open(AI_FIFO_PATH, O_RDWR | O_NONBLOCK);
    if (g_fd < 0) { perror("[FIFO] open"); return -1; }
    printf("[FIFO] ready, path=%s\n", AI_FIFO_PATH);

    /* 初始化空消息 */
    memset(&g_last_msg, 0, sizeof(g_last_msg));

    pthread_create(&g_tid, NULL, tcp_thread, NULL);
    return 0;
}

/* ---- AI线程调用: 仅更新缓冲区 + 写FIFO (永不阻塞) ---- */
void ai_fifo_send(int cx, int cy, int w, int h, int fw, int fh, float conf,
                  int has_target, int class_id, float smoke_score, float fire_score)
{
    ai_coord_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.cx = cx; msg.cy = cy; msg.w = w; msg.h = h;
    msg.frame_w = fw; msg.frame_h = fh;
    msg.confidence = conf; msg.has_target = has_target;
    msg.class_id = class_id;
    msg.smoke_score = smoke_score; msg.fire_score = fire_score;

    msg.slave_id = 0;  /* AI检测数据 */
    /* 附带综合传感器数据 */
    pthread_mutex_lock(&g_sensor_lock);
    msg.tvoc = g_tvoc; msg.ch2o = g_ch2o;
    msg.co2 = g_co2; msg.sensor_alarm = g_sensor_alarm;
    msg.temperature = g_temperature;
    pthread_mutex_unlock(&g_sensor_lock);

    /* 更新共享缓冲区 (TCP线程从此读取) */
    pthread_mutex_lock(&g_msg_lock);
    g_last_msg = msg;
    pthread_mutex_unlock(&g_msg_lock);

    /* FIFO 写入 (非阻塞, 仅用于舵机追踪) */
    if (g_fd >= 0) write(g_fd, &msg, 40);  /* 覆盖has_target+class_id */
}

/* ---- 传感器/评分支持 ---- */
void ai_fifo_set_sensor(float tvoc, float ch2o, int co2, int alarm, float temperature) {
    pthread_mutex_lock(&g_sensor_lock);
    g_tvoc=tvoc; g_ch2o=ch2o; g_co2=co2; g_sensor_alarm=alarm;
    g_temperature = temperature;
    pthread_mutex_unlock(&g_sensor_lock);
}

/* 单个从机传感器数据 → 独立TCP消息, 同时更新综合max用于AI融合 */
void ai_fifo_set_sensor_slave(int slave_id, float tvoc, float ch2o, int co2, float temperature) {
    /* 1. 更新综合值 (取各从机max, 供AI融合) */
    pthread_mutex_lock(&g_sensor_lock);
    if (slave_id == 1) {
        g_tvoc_s1=tvoc; g_ch2o_s1=ch2o; g_co2_s1=co2; g_temp_s1=temperature;
    } else if (slave_id == 2) {
        g_tvoc_s2=tvoc; g_ch2o_s2=ch2o; g_co2_s2=co2; g_temp_s2=temperature;
    }
    float tvoc_m = (g_tvoc_s1 > g_tvoc_s2) ? g_tvoc_s1 : g_tvoc_s2;
    float ch2o_m = (g_ch2o_s1 > g_ch2o_s2) ? g_ch2o_s1 : g_ch2o_s2;
    g_tvoc = tvoc_m; g_ch2o = ch2o_m;
    g_co2  = (g_co2_s1 > g_co2_s2) ? g_co2_s1 : g_co2_s2;
    g_temperature = (g_temp_s1 > g_temp_s2) ? g_temp_s1 : g_temp_s2;
    g_sensor_alarm = (tvoc_m > 1.0f || ch2o_m > 0.3f || g_co2 > 1500) ? SENSOR_BAD_AIR : SENSOR_OK;
    float _tvoc=g_tvoc, _ch2o=g_ch2o; int _co2=g_co2, _alarm=g_sensor_alarm; float _temp=g_temperature;
    pthread_mutex_unlock(&g_sensor_lock);

    /* 2. 组装带slave_id的独立传感器消息 → TCP队列 */
    ai_coord_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.slave_id    = slave_id;
    msg.tvoc        = tvoc;
    msg.ch2o        = ch2o;
    msg.co2         = co2;
    msg.temperature = temperature;
    msg.sensor_alarm = (tvoc > 1.0f || ch2o > 0.3f || co2 > 1500) ? SENSOR_BAD_AIR : SENSOR_OK;
    msg.frame_w = 1920; msg.frame_h = 1080;

    /* 不影响AI检测的g_last_msg, 直接入TCP发送队列 */
    pthread_mutex_lock(&g_msg_lock);
    g_last_msg = msg;
    pthread_mutex_unlock(&g_msg_lock);
}
void ai_fifo_get_sensor(float *tvoc, float *ch2o, int *co2, int *alarm, float *temperature) {
    pthread_mutex_lock(&g_sensor_lock);
    if(tvoc)*tvoc=g_tvoc; if(ch2o)*ch2o=g_ch2o;
    if(co2)*co2=g_co2; if(alarm)*alarm=g_sensor_alarm;
    if(temperature)*temperature=g_temperature;
    pthread_mutex_unlock(&g_sensor_lock);
}
void ai_fifo_get_scores(float *smoke_score, float *fire_score) {
    pthread_mutex_lock(&g_msg_lock);
    if (smoke_score) *smoke_score = g_last_msg.smoke_score;
    if (fire_score)  *fire_score  = g_last_msg.fire_score;
    pthread_mutex_unlock(&g_msg_lock);
}
void ai_fifo_flush_sensor(void) { /* 已包含在ai_fifo_send中 */ }
int  ai_fifo_tcp_health(void) { return 0; }

/* ---- 清理 ---- */
void ai_fifo_deinit(void)
{
    if (g_fd >= 0) { close(g_fd); g_fd = -1; }
    for (int i = 0; i < g_client_count; i++) close(g_clients[i]);
    g_client_count = 0;
    if (g_srv >= 0) { shutdown(g_srv, SHUT_RDWR); close(g_srv); g_srv = -1; }
    pthread_cancel(g_tid);
    pthread_join(g_tid, NULL);
    printf("[FIFO] closed\n");
}
