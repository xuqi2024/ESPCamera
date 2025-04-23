#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_camera.h"
#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_timer.h"
#include "esp_crc.h"
#include "esp_jpg_decode.h"

// 定义图像格式
#define ESP_IMAGE_FORMAT_JPEG 0
#define ESP_RGB_888 3

// 定义MIN和MAX宏
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

// 帧差分阈值
#define PIXEL_DIFF_THRESHOLD 30
#define BLOCK_SIZE 16
#define FRAME_BUFFER_COUNT 2

// WiFi配置
#define WIFI_SSID "CMCC-T6Ey"    // WiFi名称
#define WIFI_PASS "6fu5k2ue"     // WiFi密码
#define WIFI_MAXIMUM_RETRY 5     // 最大重试次数

// 事件组位
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// UDP配置
#define UDP_PORT 8888            // UDP端口
#define PACKET_SIZE 1024         // 增加包大小
#define BROADCAST_ADDR "255.255.255.255"  // 广播地址
#define FRAME_INTERVAL_MS 50    // 初始帧间隔(20fps)
#define MIN_FRAME_INTERVAL_MS 33  // 最小帧间隔(30fps)
#define MAX_FRAME_INTERVAL_MS 100 // 最大帧间隔(10fps)
#define QUALITY_ADJUST_INTERVAL 30 // 每30帧调整一次质量
#define MAX_UDP_BUFFER_SIZE (128 * 1024) // 增加UDP缓冲区
#define MIN_PACKET_INTERVAL_US 100       // 减小最小包间隔
#define INITIAL_PACKET_INTERVAL_US 200   // 减小初始包间隔

// 发送延迟控制
#define MIN_PACKET_DELAY_US 500        // 最小包间延迟（微秒）
#define MAX_PACKET_DELAY_US 2000 // 最大包间延迟（微秒）
#define INITIAL_PACKET_DELAY_US 1000   // 初始包间延迟

// 发送队列配置
#define FRAME_QUEUE_SIZE 4              // 减小帧队列，因为我们要加快处理速度
static QueueHandle_t frame_queue = NULL;

// 帧结构体
typedef struct {
    uint8_t *data;
    size_t len;
    uint32_t frame_id;
} frame_data_t;

typedef struct {
    uint32_t frame_id;          // 帧ID
    uint32_t packet_id;         // 包ID
    uint32_t total_packets;     // 总包数
    uint32_t packet_size;       // 包大小
    uint32_t crc;              // CRC校验值
    uint8_t data[PACKET_SIZE];  // 数据
} camera_packet_t;

// 自适应控制参数
#define MAX_JPEG_QUALITY 30
#define MIN_JPEG_QUALITY 10
#define MAX_FRAME_INTERVAL 100  // 最大帧间隔(10fps)
#define MIN_FRAME_INTERVAL 33   // 最小帧间隔(30fps)
#define HIGH_PACKET_LOSS_THRESHOLD 0.3  // 30%
#define LOW_PACKET_LOSS_THRESHOLD 0.1   // 10%

// 统计窗口大小
#define STATS_WINDOW_SIZE 30  // 30帧的统计窗口

typedef struct {
    uint32_t frame_interval;
    uint8_t jpeg_quality;
    uint32_t packet_delay_us;
    uint32_t total_packets;
    uint32_t lost_packets;
    float packet_loss_rate;
    uint64_t last_adjust_time;
    // 添加统计窗口
    uint32_t packet_stats_window[STATS_WINDOW_SIZE];
    uint32_t lost_stats_window[STATS_WINDOW_SIZE];
    uint8_t stats_window_index;
} adaptive_control_t;

static adaptive_control_t adaptive_ctrl = {
    .frame_interval = 100,             // 初始帧间隔增加到100ms (10fps)
    .jpeg_quality = 20,                // 初始JPEG质量
    .packet_delay_us = INITIAL_PACKET_DELAY_US,
    .total_packets = 0,
    .lost_packets = 0,
    .packet_loss_rate = 0,
    .last_adjust_time = 0,
    .stats_window_index = 0
};

static const char *TAG = "camera_stream";
static int sock = -1;            // UDP socket
static struct sockaddr_in dest_addr;
static struct sockaddr_in feedback_addr;
static bool feedback_received = false;

// 摄像头引脚定义
#define CAM_PIN_PWDN     -1  // 掉电引脚（本开发板不使用）
#define CAM_PIN_RESET    -1  // 复位引脚（本开发板不使用）
#define CAM_PIN_XCLK     15  // XCLK时钟信号
#define CAM_PIN_SIOD     4   // SCCB/I2C数据线
#define CAM_PIN_SIOC     5   // SCCB/I2C时钟线
#define CAM_PIN_D7       16  // 数据位7
#define CAM_PIN_D6       17  // 数据位6
#define CAM_PIN_D5       18  // 数据位5
#define CAM_PIN_D4       12  // 数据位4
#define CAM_PIN_D3       10  // 数据位3
#define CAM_PIN_D2       8   // 数据位2
#define CAM_PIN_D1       9   // 数据位1
#define CAM_PIN_D0       11  // 数据位0
#define CAM_PIN_VSYNC    6   // 垂直同步
#define CAM_PIN_HREF     7   // 水平参考
#define CAM_PIN_PCLK     13  // 像素时钟

// 摄像头配置
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,           // XCLK 20MHz
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,     
    .frame_size = FRAMESIZE_VGA,     // 降回VGA分辨率
    .jpeg_quality = 20,                // 降低JPEG质量
    .fb_count = 4,                     // 增加帧缓冲区数量
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST,   // 总是获取最新帧
    .sccb_i2c_port = 1
};

// 添加帧缓存结构
typedef struct {
    uint8_t *prev_frame;
    size_t prev_frame_len;
    uint16_t width;
    uint16_t height;
    uint8_t *diff_buffer;
    size_t diff_buffer_size;
} frame_buffer_t;

static frame_buffer_t frame_buffer = {0};

// 初始化帧缓存
static esp_err_t init_frame_buffer(uint16_t width, uint16_t height) {
    frame_buffer.width = width;
    frame_buffer.height = height;
    frame_buffer.diff_buffer_size = (width * height * 3) / 2; // 预留50%空间存储差异
    
    frame_buffer.prev_frame = heap_caps_malloc(width * height * 3, MALLOC_CAP_SPIRAM);
    frame_buffer.diff_buffer = heap_caps_malloc(frame_buffer.diff_buffer_size, MALLOC_CAP_SPIRAM);
    
    if (!frame_buffer.prev_frame || !frame_buffer.diff_buffer) {
        ESP_LOGE(TAG, "无法分配帧缓存内存");
        return ESP_ERR_NO_MEM;
    }
    
    memset(frame_buffer.prev_frame, 0, width * height * 3);
    return ESP_OK;
}

// 计算帧差异并压缩
static size_t compute_frame_diff(const uint8_t *current_frame, size_t frame_len) {
    if (!frame_buffer.prev_frame) {
        // 第一帧，完整发送
        memcpy(frame_buffer.prev_frame, current_frame, frame_len);
        memcpy(frame_buffer.diff_buffer, current_frame, frame_len);
        frame_buffer.prev_frame_len = frame_len;
        return frame_len;
    }

    size_t diff_pos = 0;
    uint16_t block_x, block_y;
    
    // 在diff_buffer开头存储元数据
    uint8_t *meta = frame_buffer.diff_buffer;
    diff_pos += sizeof(uint16_t) * 2; // 预留宽度和高度

    // 按块比较并记录差异
    for (block_y = 0; block_y < frame_buffer.height; block_y += BLOCK_SIZE) {
        for (block_x = 0; block_x < frame_buffer.width; block_x += BLOCK_SIZE) {
            bool block_changed = false;
            
            // 检查块内像素是否有显著变化
            for (int y = 0; y < BLOCK_SIZE && (block_y + y) < frame_buffer.height; y++) {
                for (int x = 0; x < BLOCK_SIZE && (block_x + x) < frame_buffer.width; x++) {
                    int pos = ((block_y + y) * frame_buffer.width + (block_x + x)) * 3;
                    
                    int diff_r = abs(current_frame[pos] - frame_buffer.prev_frame[pos]);
                    int diff_g = abs(current_frame[pos+1] - frame_buffer.prev_frame[pos+1]);
                    int diff_b = abs(current_frame[pos+2] - frame_buffer.prev_frame[pos+2]);
                    
                    if (diff_r > PIXEL_DIFF_THRESHOLD || 
                        diff_g > PIXEL_DIFF_THRESHOLD || 
                        diff_b > PIXEL_DIFF_THRESHOLD) {
                        block_changed = true;
                        break;
                    }
                }
                if (block_changed) break;
            }
            
            if (block_changed) {
                // 记录块位置
                uint16_t block_info = (block_x << 8) | block_y;
                memcpy(frame_buffer.diff_buffer + diff_pos, &block_info, sizeof(uint16_t));
                diff_pos += sizeof(uint16_t);
                
                // 复制块数据
                for (int y = 0; y < BLOCK_SIZE && (block_y + y) < frame_buffer.height; y++) {
                    for (int x = 0; x < BLOCK_SIZE && (block_x + x) < frame_buffer.width; x++) {
                        int pos = ((block_y + y) * frame_buffer.width + (block_x + x)) * 3;
                        memcpy(frame_buffer.diff_buffer + diff_pos, current_frame + pos, 3);
                        diff_pos += 3;
                    }
                }
            }
        }
    }
    
    // 更新元数据
    *(uint16_t*)meta = frame_buffer.width;
    *((uint16_t*)meta + 1) = frame_buffer.height;
    
    // 保存当前帧作为下一次比较的基准
    memcpy(frame_buffer.prev_frame, current_frame, frame_len);
    frame_buffer.prev_frame_len = frame_len;
    
    return diff_pos;
}

static void event_handler(void* arg, esp_event_base_t event_base,
                         int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "重试连接到AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"连接到AP失败");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "获取到IP地址:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t init_wifi(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "init_wifi finished.");

    /* 等待连接成功或达到最大重试次数 */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "已连接到AP SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
        return ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return ESP_FAIL;
    }
}

// 初始化UDP
static esp_err_t init_udp(void)
{
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "无法创建socket: errno %d", errno);
        return ESP_FAIL;
    }

    // 设置socket选项，允许广播
    int broadcastEnable = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));

    // 设置目标地址为广播地址
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(BROADCAST_ADDR);

    return ESP_OK;
}

// 计算CRC32
static uint32_t calculate_crc32(const uint8_t *data, size_t length) {
    return esp_crc32_le(0, data, length);
}

// 更新丢包统计
static void update_packet_stats(uint32_t total, uint32_t lost) {
    adaptive_ctrl.stats_window_index = (adaptive_ctrl.stats_window_index + 1) % STATS_WINDOW_SIZE;
    adaptive_ctrl.packet_stats_window[adaptive_ctrl.stats_window_index] = total;
    adaptive_ctrl.lost_stats_window[adaptive_ctrl.stats_window_index] = lost;
    
    // 计算窗口内的总和
    uint32_t window_total = 0;
    uint32_t window_lost = 0;
    for (int i = 0; i < STATS_WINDOW_SIZE; i++) {
        window_total += adaptive_ctrl.packet_stats_window[i];
        window_lost += adaptive_ctrl.lost_stats_window[i];
    }
    
    // 更新统计
    adaptive_ctrl.total_packets = window_total;
    adaptive_ctrl.lost_packets = window_lost;
    adaptive_ctrl.packet_loss_rate = window_total > 0 ? 
        (float)window_lost / window_total : 0;
}

// 处理反馈数据
static void handle_feedback(const uint8_t *data, size_t len) {
    if (len >= 8) {
        uint32_t lost_packets = *(uint32_t*)data;
        uint32_t total_packets = *(uint32_t*)(data + 4);
        
        // 更新丢包统计
        update_packet_stats(total_packets, lost_packets);
        
        // 根据丢包率调整参数
        uint64_t current_time = esp_timer_get_time() / 1000000;
        if (current_time - adaptive_ctrl.last_adjust_time >= 1) {
            bool params_changed = false;
            
            // 调整JPEG质量
            if (adaptive_ctrl.packet_loss_rate > HIGH_PACKET_LOSS_THRESHOLD) {
                if (adaptive_ctrl.jpeg_quality < MAX_JPEG_QUALITY) {
                    adaptive_ctrl.jpeg_quality = MIN(adaptive_ctrl.jpeg_quality + 5, MAX_JPEG_QUALITY);
                    params_changed = true;
                }
            } else if (adaptive_ctrl.packet_loss_rate < LOW_PACKET_LOSS_THRESHOLD) {
                if (adaptive_ctrl.jpeg_quality > MIN_JPEG_QUALITY) {
                    adaptive_ctrl.jpeg_quality = MAX(adaptive_ctrl.jpeg_quality - 5, MIN_JPEG_QUALITY);
                    params_changed = true;
                }
            }
            
            // 调整帧间隔
            if (adaptive_ctrl.packet_loss_rate > HIGH_PACKET_LOSS_THRESHOLD) {
                if (adaptive_ctrl.frame_interval < MAX_FRAME_INTERVAL) {
                    adaptive_ctrl.frame_interval = MIN(adaptive_ctrl.frame_interval + 10, MAX_FRAME_INTERVAL);
                    params_changed = true;
                }
            } else if (adaptive_ctrl.packet_loss_rate < LOW_PACKET_LOSS_THRESHOLD) {
                if (adaptive_ctrl.frame_interval > MIN_FRAME_INTERVAL) {
                    adaptive_ctrl.frame_interval = MAX(adaptive_ctrl.frame_interval - 5, MIN_FRAME_INTERVAL);
                    params_changed = true;
                }
            }
            
            // 调整包间隔
            if (adaptive_ctrl.packet_loss_rate > HIGH_PACKET_LOSS_THRESHOLD) {
                adaptive_ctrl.packet_delay_us = MIN(adaptive_ctrl.packet_delay_us * 1.5, 5000);
                params_changed = true;
            } else if (adaptive_ctrl.packet_loss_rate < LOW_PACKET_LOSS_THRESHOLD) {
                adaptive_ctrl.packet_delay_us = MAX(adaptive_ctrl.packet_delay_us / 1.2, MIN_PACKET_INTERVAL_US);
                params_changed = true;
            }
            
            if (params_changed) {
                // 更新相机参数
                sensor_t *s = esp_camera_sensor_get();
                if (s) {
                    s->set_quality(s, adaptive_ctrl.jpeg_quality);
                }
                
                ESP_LOGI(TAG, "调整参数 - 帧间隔: %"PRIu32" ms, JPEG质量: %u, 丢包率: %.2f%%, 包间延迟: %"PRIu32" us",
                         adaptive_ctrl.frame_interval,
                         adaptive_ctrl.jpeg_quality,
                         adaptive_ctrl.packet_loss_rate * 100,
                         adaptive_ctrl.packet_delay_us);
            }
            
            adaptive_ctrl.last_adjust_time = current_time;
        }
    }
}

// 发送视频帧
static void send_frame(const uint8_t *buf, size_t len)
{
    static uint32_t frame_counter = 0;
    frame_counter++;

    uint32_t total_packets = (len + PACKET_SIZE - 1) / PACKET_SIZE;
    
    // 设置socket选项
    int sndbuf = MAX_UDP_BUFFER_SIZE;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    // 发送数据包
    int64_t last_send_time = 0;
    for (uint32_t i = 0; i < total_packets; i++) {
        camera_packet_t packet = {
            .frame_id = frame_counter,
            .packet_id = i,
            .total_packets = total_packets,
            .packet_size = (i == total_packets - 1) ? (len % PACKET_SIZE) : PACKET_SIZE
        };

        // 复制数据
        size_t offset = i * PACKET_SIZE;
        size_t size = (i == total_packets - 1) ? (len - offset) : PACKET_SIZE;
        memcpy(packet.data, buf + offset, size);
        
        // 计算CRC
        packet.crc = calculate_crc32(packet.data, size);

        // 精确控制发送时间
        int64_t current_time = esp_timer_get_time();
        if (last_send_time > 0) {
            int64_t time_diff = current_time - last_send_time;
            if (time_diff < adaptive_ctrl.packet_delay_us) {
                int64_t delay_time = adaptive_ctrl.packet_delay_us - time_diff;
                if (delay_time > 1000) {
                    vTaskDelay(pdMS_TO_TICKS(delay_time / 1000));
                } else {
                    esp_rom_delay_us(delay_time);
                }
            }
        }

        // 发送包
        if (sendto(sock, &packet, sizeof(camera_packet_t), 0, 
                  (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0) {
            ESP_LOGW(TAG, "发送失败: %d", errno);
            continue;
        }

        last_send_time = esp_timer_get_time();
    }
}

// 接收反馈数据的任务
static void feedback_task(void *pvParameters)
{
    int feedback_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (feedback_sock < 0) {
        ESP_LOGE(TAG, "无法创建反馈socket");
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT + 1),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };

    if (bind(feedback_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "绑定反馈socket失败");
        close(feedback_sock);
        return;
    }

    uint8_t rx_buffer[32];
    while (1) {
        struct sockaddr_in src_addr;
        socklen_t src_addr_len = sizeof(src_addr);
        int len = recvfrom(feedback_sock, rx_buffer, sizeof(rx_buffer), 0,
                          (struct sockaddr *)&src_addr, &src_addr_len);
        
        if (len > 0) {
            handle_feedback(rx_buffer, len);
            if (!feedback_received) {
                feedback_received = true;
                memcpy(&feedback_addr, &src_addr, sizeof(src_addr));
            }
        }
    }
}

// JPEG编码回调
typedef struct {
    uint8_t *outbuf;
    size_t *outlen;
    size_t maxlen;
} jpg_chunking_t;

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len) {
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!data){
        return 0;
    }
    if(j->outlen && *j->outlen + len > j->maxlen){
        return 0;
    }
    if(j->outbuf) {
        memcpy(j->outbuf + *j->outlen, data, len);
    }
    *j->outlen += len;
    return len;
}

// 将图像转换为JPEG
static uint8_t* convert_to_jpg(const uint8_t *src, size_t src_len, size_t *out_len) {
    if (!src || !out_len) {
        return NULL;
    }

    // 分配输出缓冲区
    size_t jpg_buf_len = src_len;  // 预估JPEG大小
    uint8_t *jpg_buf = heap_caps_malloc(jpg_buf_len, MALLOC_CAP_SPIRAM);
    if (!jpg_buf) {
        ESP_LOGE(TAG, "无法分配JPEG缓冲区");
        return NULL;
    }

    // 设置编码参数
    jpg_chunking_t jchunk = {
        .outbuf = jpg_buf,
        .outlen = out_len,
        .maxlen = jpg_buf_len
    };

    esp_err_t ret = fmt2jpg_cb(src, src_len, frame_buffer.width, frame_buffer.height,
                              PIXFORMAT_RGB888, 60, jpg_encode_stream, &jchunk);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG编码失败");
        free(jpg_buf);
        return NULL;
    }

    return jpg_buf;
}

// 发送任务
static void sender_task(void *pvParameters)
{
    frame_data_t frame;
    while (1) {
        if (xQueueReceive(frame_queue, &frame, portMAX_DELAY)) {
            send_frame(frame.data, frame.len);
            free(frame.data);  // 释放帧数据
        }
    }
}

// 视频流任务
static void stream_task(void *pvParameters)
{
    // 设置当前任务的优先级
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 3);
    
    camera_fb_t *fb = NULL;
    uint32_t frame_count = 0;
    int64_t last_fps_time = esp_timer_get_time() / 1000000;
    float fps = 0;

    // 设置相机参数
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_quality(s, adaptive_ctrl.jpeg_quality);
        s->set_framesize(s, FRAMESIZE_VGA);     // 使用VGA分辨率
        s->set_brightness(s, 1);
        s->set_saturation(s, 1);
        s->set_contrast(s, 1);
        s->set_special_effect(s, 0);
        s->set_wb_mode(s, 1);
        s->set_whitebal(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 1);
        s->set_gain_ctrl(s, 1);
        s->set_hmirror(s, 0);
        s->set_vflip(s, 0);
        s->set_awb_gain(s, 1);
        s->set_agc_gain(s, 0);
        s->set_aec_value(s, 300);
        s->set_pixformat(s, PIXFORMAT_JPEG);
    }

    // 创建帧队列
    frame_queue = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(frame_data_t));
    if (!frame_queue) {
        ESP_LOGE(TAG, "无法创建帧队列");
        vTaskDelete(NULL);
        return;
    }

    // 创建发送任务在核心1上运行，优先级设置为高优先级
    TaskHandle_t sender_task_handle = NULL;
    BaseType_t ret = xTaskCreatePinnedToCore(
        sender_task,
        "sender_task",
        8192,
        NULL,
        configMAX_PRIORITIES - 2,
        &sender_task_handle,
        1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建发送任务失败");
        vTaskDelete(NULL);
        return;
    }

    // 主循环
    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(adaptive_ctrl.frame_interval));
            continue;
        }

        frame_count++;
        if (frame_count % 30 == 0) {
            int64_t current_time = esp_timer_get_time() / 1000000;
            int64_t time_diff = current_time - last_fps_time;
            if (time_diff > 0) {
                fps = frame_count / (float)time_diff;
                ESP_LOGI(TAG, "Stream FPS: %.2f, 丢包率: %.2f%%, 延迟: %"PRIu32"us, 队列: %d", 
                        fps, adaptive_ctrl.packet_loss_rate * 100, 
                        adaptive_ctrl.packet_delay_us,
                        uxQueueMessagesWaiting(frame_queue));
                frame_count = 0;
                last_fps_time = current_time;
            }
        }

        // 准备帧数据
        frame_data_t frame = {
            .data = heap_caps_malloc(fb->len, MALLOC_CAP_DMA),
            .len = fb->len,
            .frame_id = frame_count
        };

        if (frame.data) {
            memcpy(frame.data, fb->buf, fb->len);
            
            // 将帧放入队列，如果队列满则丢弃最旧的帧
            frame_data_t old_frame;
            if (xQueueSend(frame_queue, &frame, 0) != pdTRUE) {
                if (xQueueReceive(frame_queue, &old_frame, 0) == pdTRUE) {
                    free(old_frame.data);
                    if (xQueueSend(frame_queue, &frame, 0) != pdTRUE) {
                        free(frame.data);
                    }
                } else {
                    free(frame.data);
                }
            }
        } else {
            ESP_LOGE(TAG, "无法分配帧内存");
        }

        esp_camera_fb_return(fb);

        // 根据队列状态动态调整延迟
        UBaseType_t waiting = uxQueueMessagesWaiting(frame_queue);
        if (waiting > FRAME_QUEUE_SIZE/2) {
            vTaskDelay(pdMS_TO_TICKS(adaptive_ctrl.frame_interval * 1.5));
        } else {
            vTaskDelay(pdMS_TO_TICKS(adaptive_ctrl.frame_interval));
        }
    }
}

void app_main(void)
{
    // 初始化WiFi
    if (init_wifi() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi初始化失败");
        return;
    }

    // 等待TCP/IP堆栈初始化完成
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 初始化摄像头
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    // 初始化UDP
    if (init_udp() != ESP_OK) {
        ESP_LOGE(TAG, "UDP初始化失败");
        return;
    }

    // 创建反馈接收任务
    xTaskCreate(feedback_task, "feedback_task", 4096, NULL, 5, NULL);
    
    // 创建视频流任务在核心0上运行
    TaskHandle_t stream_task_handle = NULL;
    BaseType_t ret = xTaskCreatePinnedToCore(
        stream_task,
        "stream_task",
        8192,
        NULL,
        configMAX_PRIORITIES - 3,
        &stream_task_handle,
        0
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建视频流任务失败");
        return;
    }

    ESP_LOGI(TAG, "Camera streaming started");
}