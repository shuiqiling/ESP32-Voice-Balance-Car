# 语音控制自平衡小车

基于 ESP32-S3 的语音控制双轮自平衡小车：

**阿里云 NLS 语音识别 → DeepSeek 大模型理解意图 → 生成动作与语音回复 → 阿里云 NLS 语音合成播报**，

运动控制采用位置环 → 速度环 → 角度环三环 PID 级联 + 互补滤波姿态解算。

## 功能特性

- **语音识别**：阿里云智能语音交互 ASR（SpeechTranscriber 命名空间，WebSocket 长连接）
- **语义理解**：DeepSeek 大模型（HTTPS），一次回复可含动作序列 `actions[]`、语音回复 `voice`、分层记忆 `memory`/`digest`
- **语音播报**：阿里云 NLS TTS（SpeechSynthesizer 命名空间，每次播报独立建连，16kHz PCM）
- **运动控制**：5ms 控制环，位置/速度/角度三环级联 PID，MPU6050 + 互补滤波（α=0.98）
- **在线调试**：串口实时调 PID 参数、VOFA FireWater 协议遥测
- **回环防护**：TTS 播报期间自动暂停麦克风上传（扬声器与麦克风同板，无硬件回声消除）

## 硬件连接

| 模块 | 接口 | 引脚 |
|---|---|---|
| 主控 | ESP32-S3（16MB Flash） | — |
| 姿态传感器 | MPU6050 | I2C：SDA=9, SCL=8, 地址 0x68 |
| 右电机 | 直流电机 + H 桥（MCPWM） | PWM=12, IN1=11, IN2=10 |
| 左电机 | 直流电机 + H 桥（MCPWM） | PWM=15, IN1=13, IN2=14 |
| 右编码器 | 增量式（PCNT） | A=4, B=5 |
| 左编码器 | 增量式（PCNT） | A=16, B=17 |
| 麦克风 | I2S 数字麦克风 | WS=6, CLK=18, SD=7 |
| 扬声器 | I2S（GPIO3） | DOUT=3 |
| 调试串口 | UART0 | TX=43, RX=44, 115200 |

## 目录结构

```
components/
├── BSP/      板级支持（引脚定义）
├── Config/   参数与枚举定义
├── Core/     PID、互补滤波、调试串口、跨任务共享状态
├── MYHAL/    驱动抽象（I2C、电机、编码器、MPU6050、I2S 麦克风/扬声器）
├── NET/      WIFI、AI（DeepSeek）、ASR、TTS
└── SER/      业务层（平衡车控制、AI 工作流）
```

## 构建与烧录

- ESP-IDF v5.5.4（xtensa-esp32s3-elf-gcc）
- 16MB Flash，自定义分区表 [partitions-16MiB.csv](partitions-16MiB.csv)
- FreeRTOS 1000Hz tick（控制环 5ms 周期依赖，`CONFIG_FREERTOS_HZ=1000`）

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## 配置凭据

`idf.py menuconfig` → **"AI 语音控制配置"**：

| 配置项 | 说明 |
|---|---|
| DEEPSEEK_API_KEY | DeepSeek API 密钥 |
| DEEPSEEK_MODEL | 模型名（默认 `deepseek-v4-flash`） |
| WIFI_SSID / WIFI_PASSWORD | 连接 WiFi |
| ASR_TOKEN / ASR_APPKEY | 阿里云 NLS 项目 Token / AppKey |

> ⚠️ 凭据只写入本机 `sdkconfig`（已被 `.gitignore` 忽略，不会进入版本库）。**切勿**使用 `git add -f sdkconfig` 之类的命令强制提交。`sdkconfig.defaults` 中所有默认值均为空，克隆后不填凭据也能正常编译，相关功能在运行时给出日志提示。

## 串口调试命令

调试串口（115200）支持在线调参，例如 `akp 25`：

| 命令 | 作用 |
|---|---|
| `akp / aki / akd <值>` | 角度环 P / I / D |
| `skp / ski / skd <值>` | 速度环 P / I / D |
| `pkp / pki / pkd <值>` | 位置环 P / I / D |
| `pos <右> <左>` | 目标位置，两个参数都必须给，如 `pos 100 -100` |

## VOFA 遥测

调试串口以 40Hz 输出 FireWater 帧：`<右轮速度, 右参考速度, 左轮速度, 左参考速度>`（浮点逗号分隔），在 [VOFA+](https://www.vofa.plus/) 中选 FireWater 协议即可实时查看。

## 说明

- 扬声器与麦克风同板、固件无 AEC：TTS 播报期间自动停止上传麦克风数据，避免车听到自己的播报再次触发指令
- PID 增益按采样点定义（无 dt 归一化），控制周期由 1000Hz tick 保证为 5ms
- `pos` 命令参数单位为 mm，但位置环反馈是编码器计数，实际行程需按轮径换算
