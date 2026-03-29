#include "web_server.h"
#include "web_page.h"
#include "robot.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "WEB";

static httpd_handle_t s_server = NULL;
static int s_ws_fd = -1;  /* 当前 WebSocket 客户端 fd */

/* ============ 工具函数 ============ */

static const char *mode_str(robot_mode_t m)
{
    switch (m) {
    case ROBOT_MODE_IDLE:        return "idle";
    case ROBOT_MODE_WEB_CONTROL: return "web_control";
    case ROBOT_MODE_TEACHING:    return "teaching";
    case ROBOT_MODE_PLAYBACK:    return "playback";
    case ROBOT_MODE_IMU_CONTROL: return "imu_control";
    default:                     return "unknown";
    }
}

/* 通过 WebSocket 发送文本帧 */
static void ws_send_text(const char *data, int len)
{
    if (s_server == NULL || s_ws_fd < 0) return;

    httpd_ws_frame_t frame = {
        .type    = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)data,
        .len     = len,
        .final   = true,
    };
    esp_err_t ret = httpd_ws_send_frame_async(s_server, s_ws_fd, &frame);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WS send failed: %s", esp_err_to_name(ret));
        s_ws_fd = -1;
    }
}

/* ============ WebSocket 推送回调 (由 servo_task @10Hz 调用) ============ */

static void ws_push_callback(void *arg)
{
    if (s_ws_fd < 0) return;

    int16_t pos[SERVO_NUM_JOINTS];
    robot_get_current_pos(pos);
    robot_mode_t m = robot_get_mode();
    uint32_t rec_n = robot_get_record_count();
    bool imu_conn = robot_get_imu_connected();

    char buf[300];
    int n;

    if (m == ROBOT_MODE_IMU_CONTROL) {
        float ex, ey, ez;
        robot_get_endeffector_pos(&ex, &ey, &ez);
        n = snprintf(buf, sizeof(buf),
            "{\"st\":\"pos\",\"j\":[%d,%d,%d,%d,%d,%d],\"m\":\"%s\",\"n\":%lu,"
            "\"imu\":%s,\"ee\":[%.1f,%.1f,%.1f]}",
            pos[0], pos[1], pos[2], pos[3], pos[4], pos[5],
            mode_str(m), (unsigned long)rec_n,
            imu_conn ? "true" : "false", ex, ey, ez);
    } else {
        n = snprintf(buf, sizeof(buf),
            "{\"st\":\"pos\",\"j\":[%d,%d,%d,%d,%d,%d],\"m\":\"%s\",\"n\":%lu,\"imu\":%s}",
            pos[0], pos[1], pos[2], pos[3], pos[4], pos[5],
            mode_str(m), (unsigned long)rec_n,
            imu_conn ? "true" : "false");
    }

    ws_send_text(buf, n);
}

/* ============ JSON 命令解析 ============ */

static void handle_ws_message(const char *data, int len)
{
    /* 查找 "cmd" 字段 */
    const char *cmd = strstr(data, "\"cmd\":\"");
    if (!cmd) return;
    cmd += 7;  /* 跳过 "cmd":" */

    if (strncmp(cmd, "web_control", 11) == 0) {
        robot_set_mode(ROBOT_MODE_WEB_CONTROL);
    }
    else if (strncmp(cmd, "teach_start", 11) == 0) {
        robot_set_mode(ROBOT_MODE_TEACHING);
    }
    else if (strncmp(cmd, "teach_stop", 10) == 0) {
        robot_set_mode(ROBOT_MODE_IDLE);
    }
    else if (strncmp(cmd, "stop", 4) == 0) {
        robot_set_mode(ROBOT_MODE_IDLE);
    }
    else if (strncmp(cmd, "home", 4) == 0) {
        robot_go_home();
    }
    else if (strncmp(cmd, "play", 4) == 0) {
        /* 解析 loop 字段 */
        bool loop = (strstr(data, "\"loop\":true") != NULL);
        robot_set_playback_loop(loop);
        robot_set_mode(ROBOT_MODE_PLAYBACK);

        if (robot_get_mode() != ROBOT_MODE_PLAYBACK) {
            /* 切换失败(无录制)，通知前端 */
            const char *err = "{\"st\":\"error\",\"msg\":\"No recording\"}";
            ws_send_text(err, strlen(err));
        }
    }
    else if (strncmp(cmd, "pos", 3) == 0) {
        /* 解析位置数组: "j":[v0,v1,v2,v3,v4,v5] */
        const char *jp = strstr(data, "\"j\":[");
        if (!jp) return;
        jp += 5;

        int16_t pos[SERVO_NUM_JOINTS];
        int parsed = sscanf(jp, "%hd,%hd,%hd,%hd,%hd,%hd",
                            &pos[0], &pos[1], &pos[2],
                            &pos[3], &pos[4], &pos[5]);
        if (parsed == SERVO_NUM_JOINTS) {
            robot_set_target(pos);
        }
    }
    else if (strncmp(cmd, "speed", 5) == 0) {
        const char *vp = strstr(data, "\"v\":");
        if (vp) {
            int v = atoi(vp + 4);
            robot_set_speed((uint16_t)v);
        }
    }
    else if (strncmp(cmd, "imu_control", 11) == 0) {
        robot_set_mode(ROBOT_MODE_IMU_CONTROL);
    }
    else if (strncmp(cmd, "imu_sensitivity", 15) == 0) {
        const char *vp = strstr(data, "\"v\":");
        if (vp) {
            float v = (float)atof(vp + 4);
            if (v < 0.5f) v = 0.5f;
            if (v > 5.0f) v = 5.0f;
            robot_set_imu_sensitivity(v);
        }
    }
}

/* ============ HTTP handlers ============ */

/* GET / → 返回嵌入式网页 */
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, PAGE_HTML, strlen(PAGE_HTML));
    return ESP_OK;
}

/* WebSocket handler */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        /* WebSocket 握手完成 */
        s_ws_fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WS client connected (fd=%d)", s_ws_fd);

        /* 发送初始状态 */
        int16_t pos[SERVO_NUM_JOINTS];
        robot_get_current_pos(pos);
        robot_mode_t m = robot_get_mode();

        char buf[200];
        int n = snprintf(buf, sizeof(buf),
            "{\"st\":\"pos\",\"j\":[%d,%d,%d,%d,%d,%d],\"m\":\"%s\",\"n\":%lu}",
            pos[0], pos[1], pos[2], pos[3], pos[4], pos[5],
            mode_str(m), (unsigned long)robot_get_record_count());
        ws_send_text(buf, n);
        return ESP_OK;
    }

    /* 接收 WebSocket 帧 */
    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = HTTPD_WS_TYPE_TEXT;

    /* 先获取帧长度 */
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;

    if (frame.len == 0) return ESP_OK;

    uint8_t *buf = malloc(frame.len + 1);
    if (!buf) return ESP_ERR_NO_MEM;

    frame.payload = buf;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret == ESP_OK) {
        buf[frame.len] = '\0';
        handle_ws_message((const char *)buf, frame.len);
    }

    free(buf);
    return ret;
}

/* ============ 启动服务器 ============ */

void web_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 4;

    esp_err_t ret = httpd_start(&s_server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start httpd: %s", esp_err_to_name(ret));
        return;
    }

    /* 注册 GET / */
    httpd_uri_t root_uri = {
        .uri     = "/",
        .method  = HTTP_GET,
        .handler = root_handler,
    };
    httpd_register_uri_handler(s_server, &root_uri);

    /* 注册 WebSocket /ws */
    httpd_uri_t ws_uri = {
        .uri          = "/ws",
        .method       = HTTP_GET,
        .handler      = ws_handler,
        .is_websocket = true,
    };
    httpd_register_uri_handler(s_server, &ws_uri);

    /* 注册 WebSocket 推送回调给 robot 模块 */
    robot_set_ws_push_cb(ws_push_callback, NULL);

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
}
