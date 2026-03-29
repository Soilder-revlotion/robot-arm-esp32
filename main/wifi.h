#ifndef WIFI_H
#define WIFI_H

/*
 * 以 STA 模式连接 WiFi 路由器。
 * 阻塞直到获取 IP 地址，获取后将 IP 打印到日志。
 * SSID 和密码在 wifi.c 顶部修改。
 */
void wifi_init_sta(void);

/* 获取 IP 地址字符串（连接成功后可用） */
const char *wifi_get_ip_str(void);

#endif
