#include "ASR.h"
#include "MYHAL_MIC.h"
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "SystemState.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ASR";

static char task_id[37];
QueueHandle_t asr_queue = NULL;   /* ASR 识别结果队列(SystemState.h 里 extern) */

/* 生成 32 位 hex 的会话 ID(不按标准 UUID 分组,格式与长度固定,
 * 4×esp_random 保证唯一性;全部用 %08lx + 强转,避免 32/64 位参数不匹配) */
static void uuid_generate(char *uuid)
{
    uint32_t a = esp_random();
    uint32_t b = esp_random();
    uint32_t c = esp_random();
    uint32_t d = esp_random();

    sprintf(uuid, "%08lx%08lx%08lx%08lx",
            (unsigned long)a, (unsigned long)b,
            (unsigned long)c, (unsigned long)d);
}

/* ===== WebSocket 文本帧重组缓冲 =====
 * 服务端 JSON 消息可能跨帧分片到达(payload_offset/fin),逐片拼接,
 * 消息完整(fin)后再解析。static:不占回调栈。 */
static char asr_json[2048];
static int  asr_json_len = 0;

static void asr_handle_json(const char *json)
{
    cJSON *root = cJSON_Parse(json);
    if (root) {
        cJSON *hdr    = cJSON_GetObjectItem(root, "header");
        cJSON *name   = hdr ? cJSON_GetObjectItem(hdr, "name") : NULL;
        cJSON *payload= cJSON_GetObjectItem(root, "payload");
        cJSON *result = payload ? cJSON_GetObjectItem(payload, "result") : NULL;
        if (name && name->valuestring) {
            if (strcmp(name->valuestring, "SentenceEnd") == 0
                && result && result->valuestring) {
                asr_result_t r;
                strncpy(r.cmd, result->valuestring, sizeof(r.cmd) - 1);
                r.cmd[sizeof(r.cmd) - 1] = '\0';
                ESP_LOGI(TAG, "识别: %s", r.cmd);
                xQueueSend(asr_queue, &r, 0);
            }
        }
        cJSON_Delete(root);    /* 一定要释放,否则内存泄漏 */
    } else {
        ESP_LOGW(TAG, "ASR json parse fail: %.200s", json);
    }
}

static void websocket_event_handler(
        void *arg,
        esp_event_base_t event_base,
        int32_t event_id,
        void *event_data)
{

    switch(event_id)
    {
        case WEBSOCKET_EVENT_CONNECTED:{
            ESP_LOGI(TAG,"Connected");
            /* TCP 断开时服务端会话随之失效,重连后直接开新会话(新 task_id)即可 */
            ASR_Start();
            break;
        }
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "WebSocket Disconnected, pause sending");
            ASRSENDDATACJSON_PermitLock = PermitLock_Lock;   // 暂停发送,等重连后恢复
            break;

        case WEBSOCKET_EVENT_DATA:{
            esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
            if (data->op_code != 0x01) break;   /* 只处理文本帧 */

            /* 分片重组:新消息首片复位缓冲,逐片拼接,fin 后整条解析 */
            if (data->payload_offset == 0) asr_json_len = 0;
            if (data->data_len > 0) {
                int room = (int)sizeof(asr_json) - 1 - asr_json_len;
                if (room > 0) {
                    int n = data->data_len < room ? data->data_len : room;
                    memcpy(asr_json + asr_json_len, data->data_ptr, n);
                    asr_json_len += n;
                }
            }
            if (data->fin) {
                asr_json[asr_json_len] = '\0';
                asr_handle_json(asr_json);
                ASRSENDDATACJSON_PermitLock = PermitLock_Free;
            }
            break;
        }
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WebSocket Error");
            ASRSENDDATACJSON_PermitLock = PermitLock_Lock;   /* 出错暂停发送,等重连恢复 */
            break;

        default:
            break;
    }
}

static esp_websocket_client_handle_t ws_client = NULL;

void ASR_Init(void){

    char ws_uri[256];

    snprintf(ws_uri,
         sizeof(ws_uri),
         "wss://nls-gateway.cn-shanghai.aliyuncs.com/ws/v1?token=%s",
         CONFIG_ASR_TOKEN);
    esp_websocket_client_config_t cfg = {
        .uri = ws_uri,
        .crt_bundle_attach = esp_crt_bundle_attach,  // 验证服务器证书
        .buffer_size = 4096,         // 容纳 100ms PCM(3200B),避免分片
        .ping_interval_sec = 10,     // WebSocket 心跳保活(默认即10,显式声明)
        .keep_alive_enable = true,   // TCP 保活,检测半开连接并触发重连
        .keep_alive_idle = 10,
        .keep_alive_interval = 5,
        .keep_alive_count = 3,
        .reconnect_timeout_ms = 3000, // 断连后 3 秒自动重连(默认 10 秒)
    };
    ws_client = esp_websocket_client_init(&cfg);

    esp_websocket_register_events(ws_client,
        WEBSOCKET_EVENT_ANY,
        websocket_event_handler,
        ws_client);

    esp_websocket_client_start(ws_client);

    asr_queue = xQueueCreate(4, sizeof(asr_result_t));   // 深度4 = 缓存4条指令
    if (asr_queue == NULL){
        ESP_LOGE(TAG, "queue create fail");
    }
    if (CONFIG_ASR_TOKEN[0] == '\0' || CONFIG_ASR_APPKEY[0] == '\0') {
        ESP_LOGE(TAG, "ASR token/appkey 未配置!请在 menuconfig 的 'AI 语音控制配置' 中填写");
    }
}

void ASR_Start(void)
{
    char message_id[37];
    char json[512];

    uuid_generate(task_id);
    uuid_generate(message_id);

    snprintf(json,
             sizeof(json),

"{"
"\"header\":{"
"\"message_id\":\"%s\","
"\"task_id\":\"%s\","
"\"namespace\":\"SpeechTranscriber\","
"\"name\":\"StartTranscription\","
"\"appkey\":\"%s\""
"},"

"\"payload\":{"
"\"format\":\"pcm\","
"\"sample_rate\":16000,"
"\"speech_noise_threshold\":0"
"}"
"}",
message_id,
task_id,
CONFIG_ASR_APPKEY);

    esp_websocket_client_send_text(
        ws_client,
        json,
        strlen(json),
        pdMS_TO_TICKS(1000)
    );

    ESP_LOGI(TAG, "StartTranscription sent");
}

/* 发送任务：持续从麦克风流缓冲取 PCM，经 WebSocket 带超时发送。
 * 与采集任务解耦，网络阻塞不会拖累 I2S 采集导致 overrun。*/
void asr_send_task(void *arg)
{
    static int16_t pcm[MIC_SAMPLES];
    const TickType_t send_timeout = pdMS_TO_TICKS(1000);
    uint32_t sent = 0, fail = 0, drop = 0;
    uint32_t last_log = 0;

    for (;;) {
        int got = MYHAL_MIC_Take(pcm, MIC_SAMPLES);
        if (got <= 0) {
            continue;
        }

        /* 转写未就绪时丢弃此帧，防止缓冲堆积 */
        if (ASRSENDDATACJSON_PermitLock != PermitLock_Free) {
            drop++;
            if (drop - last_log >= 50) {   /* 节流打印,避免刷屏 */
                last_log = drop;
                ESP_LOGW(TAG, "ws not ready, dropped %lu frames (fail=%lu sent=%lu)",
                         (unsigned long)drop, (unsigned long)fail, (unsigned long)sent);
            }
            continue;
        }

        int rc = esp_websocket_client_send_bin(ws_client,
                                               (char *)pcm,
                                               got * (int)sizeof(int16_t),
                                               send_timeout);
        if (rc < 0) {
            fail++;
            if ((fail & 0x3F) == 1) {
                ESP_LOGW(TAG, "ws send fail %lu times", (unsigned long)fail);
            }
        } else {
            sent++;
        }
    }
}

void ASR_Stop(void)
{
    char message_id[37];
    char json[256];

    uuid_generate(message_id);

    snprintf(json,
             sizeof(json),
"{"
"\"header\":{"
"\"message_id\":\"%s\","
"\"task_id\":\"%s\","
"\"namespace\":\"SpeechTranscriber\","
"\"name\":\"StopTranscription\","
"\"appkey\":\"%s\""
"},"
"\"payload\":{}"
"}",
message_id,
task_id,
CONFIG_ASR_APPKEY);

    esp_websocket_client_send_text(
        ws_client,
        json,
        strlen(json),
        pdMS_TO_TICKS(1000)
    );

    ESP_LOGI(TAG, "StopTranscription sent");
}
