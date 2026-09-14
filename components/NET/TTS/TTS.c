#include "TTS.h"
#include "MYHAL_MIC.h"
#include "SystemState.h"
#include "esp_websocket_client.h"
#include "esp_crt_bundle.h"
#include "esp_random.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/event_groups.h"

static const char *TAG = "TTS";

/* 发音人/语速等:见阿里云 NLS StartSynthesis 参数说明。
 * sample_rate 固定 16000,必须和 MYHAL_MIC_Init 里的 I2S 采样率一致。*/
#define TTS_VOICE        "xiaoyun"
#define TTS_SAMPLE_RATE  16000
#define TTS_VOLUME       80
#define TTS_SPEECH_RATE  0      /* -500~500,0 = 正常语速 */

/* 播放缓冲:约 0.75 秒 PCM(16000 × 2 字节 × 0.75)。音频按实时速率到达,
 * 缓冲只需吸收网络抖动,不需要很大;太大反而白占堆。*/
#define TTS_STREAM_BYTES (TTS_SAMPLE_RATE * 2 * 3 / 4)

/* 单次播报的总超时:建连 + 合成 + 收流。超过就放弃这条,避免任务卡死。*/
#define TTS_SESSION_TIMEOUT_MS 15000
/* 建连超时 */
#define TTS_CONNECT_TIMEOUT_MS 8000

#define TTS_CONNECTED_BIT BIT0
#define TTS_DONE_BIT      BIT1

static StreamBufferHandle_t tts_stream = NULL;
static EventGroupHandle_t   tts_events = NULL;

/* 每次会话的上下文。tts_task 串行执行,同一时刻只有一个会话。*/
static esp_websocket_client_handle_t tts_client = NULL;
static char   tts_task_id[37];
static char   tts_text[512];
static char   tts_rx[2048];
static int    tts_rx_len = 0;
/* 上一条非续帧的 op_code。RFC6455 允许一条消息拆成多帧,续帧(0x00)要
 * 继承前一帧的类型,不能按 0x00 直接丢掉。*/
static uint8_t tts_last_op = 0;
static volatile uint32_t tts_dropped = 0;

/* task_id / message_id:32 位十六进制,不能含 '-' 或 '_'(阿里云要求) */
static void tts_uuid(char *out)
{
    sprintf(out, "%08lx%08lx%08lx%08lx",
            (unsigned long)esp_random(), (unsigned long)esp_random(),
            (unsigned long)esp_random(), (unsigned long)esp_random());
}

/* 解析服务端文本事件,只看 header.name */
static void tts_handle_json(const char *json)
{
    cJSON *root = cJSON_Parse(json);
    if (root == NULL) {
        ESP_LOGW(TAG, "json parse fail: %.160s", json);
        return;
    }
    cJSON *hdr  = cJSON_GetObjectItem(root, "header");
    cJSON *name = hdr ? cJSON_GetObjectItem(hdr, "name") : NULL;
    const char *n = (name && name->valuestring) ? name->valuestring : "";

    if (strcmp(n, "SynthesisCompleted") == 0) {
        ESP_LOGI(TAG, "synthesis completed");
        if (tts_events) xEventGroupSetBits(tts_events, TTS_DONE_BIT);
    } else if (strcmp(n, "TaskFailed") == 0) {
        cJSON *payload = cJSON_GetObjectItem(root, "payload");
        cJSON *msg = payload ? cJSON_GetObjectItem(payload, "message") : NULL;
        ESP_LOGE(TAG, "TaskFailed: %s",
                 (msg && msg->valuestring) ? msg->valuestring : "(no message)");
        if (tts_events) xEventGroupSetBits(tts_events, TTS_DONE_BIT);
    } else if (strcmp(n, "SynthesisStarted") == 0) {
        ESP_LOGI(TAG, "synthesis started");
    }
    /* SentenceBegin / SentenceSynthesis / SentenceEnd 属于字幕事件,忽略 */

    cJSON_Delete(root);
}

static void tts_ws_handler(void *arg, esp_event_base_t base,
                           int32_t event_id, void *event_data)
{
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "connected");
        if (tts_events) xEventGroupSetBits(tts_events, TTS_CONNECTED_BIT);
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "disconnected");
        break;

    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "websocket error");
        /* 出错也要放行等待方,否则 tts_task 会一直等到超时 */
        if (tts_events) {
            xEventGroupSetBits(tts_events, TTS_CONNECTED_BIT | TTS_DONE_BIT);
        }
        break;

    case WEBSOCKET_EVENT_DATA: {
        esp_websocket_event_data_t *d = (esp_websocket_event_data_t *)event_data;

        /* op_code 是"帧"的操作码,不是"消息"的。两层拆分要分清:
         *  ① RFC6455 消息级:一条消息可拆成 FIN=0 的首帧 + 若干 0x00 续帧;
         *  ② esp_websocket_client 事件级:每帧只读一次 opcode/fin
         *     (esp_websocket_client.c:1090 在拆分循环外),再按 buffer_size
         *     把该帧载荷切成多个事件派发,所以同一帧的每个分片事件带的是
         *     同一个 op_code 和同一个 fin。
         * 因此绝对不能用 fin 判断"这条消息收完了":帧超过 4KB 时每个分片都是
         * fin=1,会在第一个分片就解析出截断的 JSON。要判断"已收到该帧全部载荷"
         * 只能用 payload_offset + data_len >= payload_len。*/
        if (d->op_code == 0x01 || d->op_code == 0x02) {
            tts_last_op = d->op_code;
        } else if (d->op_code != 0x00) {
            break;                        /* ping/pong/close 等不处理 */
        }

        /* 二进制帧 = PCM 音频(续帧继承该类型)。服务端把一段音频切成多帧
         * 下发,直接顺次写进流缓冲,播放端按实时速率取走,拼接顺序天然正确。*/
        if (tts_last_op == 0x02) {
            if (d->data_len > 0 && tts_stream) {
                size_t sent = xStreamBufferSend(tts_stream, d->data_ptr,
                                                (size_t)d->data_len, 0);
                if (sent < (size_t)d->data_len) {
                    tts_dropped += (uint32_t)(d->data_len - sent);
                    ESP_LOGW(TAG, "play buffer full, dropped %u bytes total",
                             (unsigned)tts_dropped);
                }
            }
            break;
        }

        /* 文本帧:累积到整帧收完再整体解析 */
        if (d->payload_offset == 0) tts_rx_len = 0;
        if (d->data_len > 0) {
            int room = (int)sizeof(tts_rx) - 1 - tts_rx_len;
            if (room > 0) {
                int n = d->data_len < room ? d->data_len : room;
                memcpy(tts_rx + tts_rx_len, d->data_ptr, n);
                tts_rx_len += n;
            }
        }
        if (d->payload_offset + d->data_len >= d->payload_len) {
            tts_rx[tts_rx_len] = '\0';
            tts_handle_json(tts_rx);
        }
        break;
    }

    default:
        break;
    }
}

/* 一条 StartSynthesis 指令。经典 SpeechSynthesizer 命名空间把待合成文本
 * 直接放在 payload.text 里(流式版 FlowingSpeechSynthesizer 才用 RunSynthesis
 * 分次送文本)。*/
static void tts_send_start(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *header = cJSON_AddObjectToObject(root, "header");
    cJSON_AddStringToObject(header, "message_id", tts_task_id);
    cJSON_AddStringToObject(header, "task_id", tts_task_id);
    cJSON_AddStringToObject(header, "namespace", "SpeechSynthesizer");
    cJSON_AddStringToObject(header, "name", "StartSynthesis");
    cJSON_AddStringToObject(header, "appkey", CONFIG_ASR_APPKEY);

    cJSON *payload = cJSON_AddObjectToObject(root, "payload");
    cJSON_AddStringToObject(payload, "text", tts_text);
    cJSON_AddStringToObject(payload, "format", "pcm");
    cJSON_AddNumberToObject(payload, "sample_rate", TTS_SAMPLE_RATE);
    cJSON_AddStringToObject(payload, "voice", TTS_VOICE);
    cJSON_AddNumberToObject(payload, "volume", TTS_VOLUME);
    cJSON_AddNumberToObject(payload, "speech_rate", TTS_SPEECH_RATE);

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (json == NULL) {
        ESP_LOGE(TAG, "build StartSynthesis fail");
        return;
    }
    int rc = esp_websocket_client_send_text(tts_client, json, strlen(json),
                                            pdMS_TO_TICKS(2000));
    if (rc < 0) ESP_LOGE(TAG, "send StartSynthesis fail");
    free(json);
}

/* 合成一次并播放。阻塞直到播完/失败/超时。*/
static void tts_speak(const char *text)
{
    if (CONFIG_ASR_TOKEN[0] == '\0' || CONFIG_ASR_APPKEY[0] == '\0') {
        ESP_LOGE(TAG, "ASR token/appkey 未配置,无法合成语音");
        return;
    }

    strncpy(tts_text, text, sizeof(tts_text) - 1);
    tts_text[sizeof(tts_text) - 1] = '\0';
    tts_uuid(tts_task_id);

    /* 上一条的尾巴可能还在放。服务端推流快于 16kHz 实时播放,缓冲里最多会剩
     * TTS_STREAM_BYTES(约 0.75 秒),直接 reset 会把上一条的话尾吃掉。
     * 先等它排空,最多等 1 秒(不能无限等,否则这条永远轮不上);
     * 注意 xStreamBufferReset 在接收方正阻塞时返回 pdFAIL 且不清数据,
     * 所以这里靠轮询字节数而不是靠 reset 的返回值来判断。*/
    if (tts_stream) {
        for (int i = 0; i < 50 && xStreamBufferBytesAvailable(tts_stream) > 0; i++) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        xStreamBufferReset(tts_stream);
    }
    tts_rx_len = 0;
    tts_last_op = 0;   /* 新连接,帧类型状态归零 */
    tts_dropped = 0;

    char uri[256];
    snprintf(uri, sizeof(uri),
             "wss://nls-gateway.cn-shanghai.aliyuncs.com/ws/v1?token=%s",
             CONFIG_ASR_TOKEN);

    esp_websocket_client_config_t cfg = {
        .uri = uri,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .buffer_size = 4096,
        .network_timeout_ms = 5000,
        /* 一次播报一个连接,断了不复用也不自动重连,由下一次播报重新建连 */
        .disable_auto_reconnect = true,
    };

    tts_client = esp_websocket_client_init(&cfg);
    if (tts_client == NULL) {
        ESP_LOGE(TAG, "client init fail");
        return;
    }
    esp_websocket_register_events(tts_client, WEBSOCKET_EVENT_ANY,
                                  tts_ws_handler, tts_client);

    xEventGroupClearBits(tts_events, TTS_CONNECTED_BIT | TTS_DONE_BIT);

    if (esp_websocket_client_start(tts_client) != ESP_OK) {
        ESP_LOGE(TAG, "client start fail");
        goto cleanup;
    }

    EventBits_t bits = xEventGroupWaitBits(tts_events, TTS_CONNECTED_BIT,
                                           pdFALSE, pdTRUE,
                                           pdMS_TO_TICKS(TTS_CONNECT_TIMEOUT_MS));
    if ((bits & TTS_CONNECTED_BIT) == 0) {
        ESP_LOGE(TAG, "connect timeout");
        goto cleanup;
    }

    tts_send_start();

    bits = xEventGroupWaitBits(tts_events, TTS_DONE_BIT,
                               pdFALSE, pdTRUE,
                               pdMS_TO_TICKS(TTS_SESSION_TIMEOUT_MS));
    if ((bits & TTS_DONE_BIT) == 0) {
        ESP_LOGW(TAG, "synthesis timeout, giving up");
    }

cleanup:
    /* 先停再销毁:destroy 内部会停,但显式停一次能让事件回调先收敛 */
    esp_websocket_client_stop(tts_client);
    esp_websocket_client_destroy(tts_client);
    tts_client = NULL;
}

/* 播放任务:从流缓冲取 PCM 写喇叭。单独成任务是为了不让 WebSocket 回调
 * 阻塞在 i2s_channel_write 上(回调卡住会导致协议超时断连)。*/
static void tts_play_task(void *arg)
{
    static int16_t pcm[MIC_SAMPLES];   /* static:4KB 不进任务栈 */

    for (;;) {
        size_t got = xStreamBufferReceive(tts_stream, pcm, sizeof(pcm),
                                          pdMS_TO_TICKS(200));
        if (got >= sizeof(int16_t)) {
            tts_speaking = true;   /* 必须在写 I2S 之前置位 */
            MYHAL_MIC_Play(pcm, (int)(got / sizeof(int16_t)));
        } else {
            /* 缓冲空 = 这句放完了。清标志只能放在这一侧,不能放在 tts_speak()
             * 返回时 —— 服务端推流比实时播放快,SynthesisCompleted 到达时
             * 常常还有大半句堆在缓冲里没放完。这样也不会把标志永久卡在 true:
             * 没有音频时它根本不会被置位。*/
            tts_speaking = false;
        }
    }
}

void TTS_Init(void)
{
    tts_stream = xStreamBufferCreate(TTS_STREAM_BYTES, 1);
    if (tts_stream == NULL) {
        ESP_LOGE(TAG, "stream buffer create fail");
        return;
    }
    tts_events = xEventGroupCreate();
    if (tts_events == NULL) {
        ESP_LOGE(TAG, "event group create fail");
        return;
    }
    if (xTaskCreatePinnedToCore(tts_play_task, "tts_play", 4096, NULL, 6, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "play task create fail");
    }
}

void tts_task(void *arg)
{
    ai_voice_t msg;

    if (tts_stream == NULL || tts_events == NULL) {
        ESP_LOGE(TAG, "TTS 未初始化,播报任务退出");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        if (xQueueReceive(ai_voice_queue, &msg, portMAX_DELAY) != pdPASS) {
            continue;
        }
        if (msg.buf[0] == '\0') continue;

        ESP_LOGI(TAG, "speak: %s", msg.buf);
        tts_speak(msg.buf);
    }
}
