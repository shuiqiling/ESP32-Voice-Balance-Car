#include "MYHAL_MIC.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"
#include "driver/i2s_std.h"
#include "BSP_MIC.h"
#include "esp_log.h"

#define SAMPLE_RATE        16000
#define PCM_SAMPLES        MIC_SAMPLES              // 100ms @16kHz
#define TAG "I2S"

/* 音频流缓冲：约 1 秒数据（100ms × 10 块 = 32000 字节）。
 * 解耦采集与网络发送，吸收发送抖动，避免 I2S overrun。*/
#define MIC_STREAM_BYTES   (MIC_SAMPLES * sizeof(int16_t) * 10)

static i2s_chan_handle_t rx_handle = NULL;
static i2s_chan_handle_t tx_handle = NULL;
static StreamBufferHandle_t mic_stream = NULL;
static uint32_t play_fail_count = 0;

/* Play 会被 asr_send_task(本地监听)和 tts_play_task(TTS 播报)两条路径调用,
 * 而它内部用的是静态 out32 缓冲,必须串行化,否则两块音频互相覆盖。 */
static SemaphoreHandle_t tx_mutex = NULL;
/* 本地监听默认关闭:它是调试手段,开着会和 TTS 播报抢喇叭、并把麦克风
 * 拾到的声音再放出去形成啸叫。需要时用 MYHAL_MIC_SetMonitor(true) 打开。*/
static volatile bool mic_monitor_enabled = false;

/* 采集任务：持续读 I2S → 24bit→16bit → 压入流缓冲。
 * 高优先级、整帧非阻塞写入：下游阻塞时丢整帧，绝不卡住采集导致 overrun。*/
static void mic_task(void *arg)
{
    /* ESP32-S3 STD mono：驱动/硬件为"打包单声道"(rx_mono + chan_mask=左slot)，
     * DMA 每帧只有 1 个 32bit 样本，无 [L,R] 交错——不要再做奇偶抽取!
     * (旧版 ESP32 驱动才是交错布局。) 用 static 避免大数组撑爆任务栈。*/
    static int32_t pcm32[PCM_SAMPLES];   // 100ms 打包单声道样本
    static int16_t pcm16[PCM_SAMPLES];
    const size_t frame_bytes = sizeof(pcm16);
    uint32_t drop_cnt = 0;

    for (;;) {
        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(rx_handle, pcm32, sizeof(pcm32),
                                          &bytes_read, pdMS_TO_TICKS(1000));
        if (ret != ESP_OK || bytes_read < sizeof(pcm32)) {
            ESP_LOGW(TAG, "I2S read fail/short: ret=%d bytes=%d", ret, bytes_read);
            continue;
        }

        /* 打包单声道:每样本直接取高 16bit。
         * 若麦克风 L/R 接了 VDD(右声道),驱动只收左 slot,采出来是全 0 静音,
         * 需把 L/R 改接 GND(硬件接法,不是软件能改的)。*/
        for (int i = 0; i < PCM_SAMPLES; i++) {
            pcm16[i] = pcm32[i] >> 16;       // 32bit slot(24bit有效) → 16bit
        }

        /* 整帧写入或整帧丢弃，避免半帧写入破坏字节对齐 */
        if (xStreamBufferSpacesAvailable(mic_stream) >= frame_bytes) {
            xStreamBufferSend(mic_stream, pcm16, frame_bytes, 0);
        } else {
            if ((++drop_cnt & 0x3F) == 0) {
                ESP_LOGW(TAG, "stream full, dropped %lu frames",
                         (unsigned long)drop_cnt);
            }
        }
    }
}

void MYHAL_MIC_Init(void){
    /* 全双工：同时创建 TX（扬声器）+ RX（麦克风）*/
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_NUM_0,
        I2S_ROLE_MASTER);

    ESP_ERROR_CHECK(i2s_new_channel(
        &chan_cfg,
        &tx_handle,      // TX 给扬声器
        &rx_handle));    // RX 给麦克风

    /* RX 标准模式配置 */
    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_MONO
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = BSP_MICS[BSP_RIGHT_MIC].BSP_MIC_CLK,
            .ws   = BSP_MICS[BSP_RIGHT_MIC].BSP_MIC_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = BSP_MICS[BSP_RIGHT_MIC].BSP_MIC_SD,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &rx_std_cfg));

    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_MONO
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = BSP_MICS[BSP_RIGHT_MIC].BSP_MIC_CLK,
            .ws   = BSP_MICS[BSP_RIGHT_MIC].BSP_MIC_WS,
            .dout = 3,                         // 扬声器数据脚 GPIO3
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &tx_std_cfg));

    /* 全双工通道共享 BCLK/WS，两个方向都配置完成后再启动时钟。 */
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    /* 音频流缓冲 + 采集任务（高优先级，保证 I2S 不 overrun）*/
    tx_mutex = xSemaphoreCreateMutex();
    if (tx_mutex == NULL) {
        ESP_LOGE(TAG, "tx mutex create failed");
        return;
    }
    mic_stream = xStreamBufferCreate(MIC_STREAM_BYTES, 1);
    if (mic_stream == NULL) {
        ESP_LOGE(TAG, "stream buffer create failed");
        return;
    }
    if (xTaskCreatePinnedToCore(mic_task, "mic_task", 4096, NULL, 10, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "mic_task create failed");
        return;
    }

    ESP_LOGI(TAG, "I2S Full-Duplex Init OK (RX:GPIO%d, TX:GPIO3) %dms/frame",
             BSP_MICS[BSP_RIGHT_MIC].BSP_MIC_SD, PCM_SAMPLES * 1000 / SAMPLE_RATE);
}

/* 消费者接口：从音频流缓冲取数据，返回实际取得的样本数。*/
int MYHAL_MIC_Take(int16_t *out, int max_samples)
{
    if (mic_stream == NULL || out == NULL || max_samples <= 0) {
        return 0;
    }
    size_t want = (size_t)max_samples * sizeof(int16_t);
    size_t got = xStreamBufferReceive(mic_stream, out, want, pdMS_TO_TICKS(200));
    return (int)(got / sizeof(int16_t));
}

/* 播放接口：16bit 单声道 → 32bit slot → 写扬声器(TX)。
 * TX mono 时硬件自动把同一 32bit 样本复制到左右两 slot(tx_chan_equal),
 * 直接写打包单声道流即可,无需奇偶展开。扬声器回声进麦会啸叫,注意隔离。
 * 线程安全:内部 tx_mutex 串行化,可被监听与 TTS 两条路径并发调用。*/
void MYHAL_MIC_Play(const int16_t *data, int samples)
{
    if (tx_handle == NULL || data == NULL || samples <= 0) {
        return;
    }
    static int32_t out32[MIC_SAMPLES];   /* tx_mutex 保护下的单例缓冲 */
    if (samples > MIC_SAMPLES) {
        samples = MIC_SAMPLES;
    }

    if (tx_mutex) xSemaphoreTake(tx_mutex, portMAX_DELAY);

    for (int i = 0; i < samples; i++) {
        /* 乘法避免对负的有符号整数左移（C 语言未定义行为）。 */
        out32[i] = (int32_t)data[i] * 65536;
    }
    const size_t wanted = (size_t)samples * sizeof(int32_t);
    size_t written = 0;
    esp_err_t ret = i2s_channel_write(tx_handle, out32, wanted,
                                      &written, pdMS_TO_TICKS(200));
    if ((ret != ESP_OK || written != wanted) && ((++play_fail_count & 0x3F) == 1)) {
        ESP_LOGW(TAG, "I2S write fail/short: ret=%d bytes=%u/%u",
                 ret, (unsigned)written, (unsigned)wanted);
    }

    if (tx_mutex) xSemaphoreGive(tx_mutex);
}

/* 本地监听开关:默认关。开着时 asr_send_task 会把送 ASR 的音频同步回放,
 * 用于确认麦克风通路是否正常;接上 TTS 播报后应保持关闭。*/
void MYHAL_MIC_SetMonitor(bool enabled)
{
    mic_monitor_enabled = enabled;
    ESP_LOGI(TAG, "local monitor %s", enabled ? "ON" : "OFF");
}

bool MYHAL_MIC_GetMonitor(void)
{
    return mic_monitor_enabled;
}
