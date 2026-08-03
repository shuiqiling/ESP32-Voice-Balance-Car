#include "MYHAL_MIC.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
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

/* 采集任务：持续读 I2S → 24bit→16bit → 压入流缓冲。
 * 高优先级、整帧非阻塞写入：下游阻塞时丢整帧，绝不卡住采集导致 overrun。*/
static void mic_task(void *arg)
{
    /* ESP32-S3 STD mono：DMA 仍按 stereo 交错传输(total_slot=2)，
     * buffer 布局为 [L,R,L,R,...]。读 PCM_SAMPLES 个立体声帧后取左声道
     * 降为单声道。用 static 避免大数组撑爆任务栈（单例任务，安全）。*/
    static int32_t pcm32[PCM_SAMPLES * 2];   // 立体声交错，2 倍 mono 样本
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

        /* 取左声道(偶数索引)。若麦克风 L/R 接右声道导致静音，
         * 改为 pcm32[2*i + 1] 取右声道。*/
        for (int i = 0; i < PCM_SAMPLES; i++) {
            pcm16[i] = pcm32[2 * i] >> 16;    // 32bit slot(24bit有效) → 16bit
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
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

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
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    /* 音频流缓冲 + 采集任务（高优先级，保证 I2S 不 overrun）*/
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
