#ifndef TTS_H
#define TTS_H

/**
 * 阿里云 NLS 语音合成(SpeechSynthesizer 命名空间, WebSocket)。
 *
 * 数据流:
 *   ai_voice_queue ──► tts_task ──► WSS 合成 ──► tts_stream ──► tts_play_task ──► 喇叭
 *                     (文本)                    (PCM 二进制帧)     (MYHAL_MIC_Play)
 *
 * 与 ASR 的分工:ASR 是长连接常驻(要一直听),TTS 是每次播报单独建连
 * (播报不频繁,换来的是状态机简单、不会有卡死的会话锁)。代价是每次播报
 * 多一次 TLS 握手,约 1~2 秒才出声。
 */

/* 创建音频流缓冲 + 播放任务。由 SER_AIWORK_Init() 调用,不阻塞。*/
void TTS_Init(void);

/* 语音播报任务:消费 ai_voice_queue,每次取一条文本合成并播放。
 * 由 SER_AIWORK_CreatPin() 创建。*/
void tts_task(void *arg);

#endif
