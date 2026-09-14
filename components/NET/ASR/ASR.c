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
QueueHandle_t asr_queue = NULL;


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
                if (asr_queue == NULL || xQueueSend(asr_queue, &r, 0) != pdPASS) {
                    ESP_LOGW(TAG, "ASR result queue unavailable/full, result dropped");
                }
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
            
            ASR_Start();
            break;
        }
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "WebSocket Disconnected, pause sending");
            ASRSENDDATACJSON_PermitLock = PermitLock_Lock;   
            break;

        case WEBSOCKET_EVENT_DATA:{
            esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
            if (data->op_code != 0x01) break;   /* 只处理文本帧 */


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

    /* 回调可能在 client_start 返回前后立即执行，先创建回调依赖的队列。 */
    asr_queue = xQueueCreate(4, sizeof(asr_result_t));
    if (asr_queue == NULL) {
        ESP_LOGE(TAG, "queue create fail");
        return;
    }

    if (CONFIG_ASR_TOKEN[0] == '\0' || CONFIG_ASR_APPKEY[0] == '\0') {
        ESP_LOGE(TAG, "ASR token/appkey 未配置!请在 menuconfig 的 'AI 语音控制配置' 中填写");
        return;
    }

    char ws_uri[256];

    snprintf(ws_uri,
         sizeof(ws_uri),
         "wss://nls-gateway.cn-shanghai.aliyuncs.com/ws/v1?token=%s",
         CONFIG_ASR_TOKEN);
    esp_websocket_client_config_t cfg = {
        .uri = ws_uri,
        .crt_bundle_attach = esp_crt_bundle_attach, 
        .buffer_size = 4096,        
        .ping_interval_sec = 10,     
        .keep_alive_enable = true,   
        .keep_alive_idle = 10,
        .keep_alive_interval = 5,
        .keep_alive_count = 3,
        .reconnect_timeout_ms = 3000, 
    };
    ws_client = esp_websocket_client_init(&cfg);
    if (ws_client == NULL) {
        ESP_LOGE(TAG, "WebSocket client init fail");
        return;
    }

    esp_websocket_register_events(ws_client,
        WEBSOCKET_EVENT_ANY,
        websocket_event_handler,
        ws_client);

    esp_err_t ret = esp_websocket_client_start(ws_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WebSocket client start fail: %s", esp_err_to_name(ret));
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

        /* 本地监听:播放与发送给 ASR 的同一份数据(扬声器回声进麦会啸叫)。
         * 默认关闭——喇叭要留给 TTS 播报,两者混在一起会互相打断。*/
        if (MYHAL_MIC_GetMonitor()) {
            MYHAL_MIC_Play(pcm, got);
        }

        /* TTS 正在播报:喇叭的声音会被本机麦克风拾到,原样送到阿里云就会被
         * 识别成一条新指令,再触发一次播报 —— 自己和自己对话的死循环。
         * 固件里没有 AEC,只能整帧丢弃;等 tts_play_task 把缓冲放空再恢复。*/
        if (tts_speaking) {
            continue;
        }

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
