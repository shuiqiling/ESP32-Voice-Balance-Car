#include "AI.h"
#include "esp_http_client.h"
#include "cJSON.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "SystemState.h"
#include "esp_crt_bundle.h"
#include "debug_uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

QueueHandle_t ai_voice_queue = NULL;
QueueHandle_t ai_cmd_queue = NULL;

static const char *TAG = "AI";

/* 固定系统提示词:每次请求都带在 system role 里,设定 AI 的角色和回复风格 */
static const char *AI_SYSTEM_PROMPT =
    "你是一个平衡车的车灵。用户会通过语音下达指令,请理解真实意图,"
    "必要时根据谐音或上下文推断。"
    "你的回复必须且只能是一个 JSON 对象,不允许输出任何解释、Markdown 或代码块。格式固定:\n"
    "{\"voice\":\"给用户听的简短中文回复\",\"actions\":[{\"cmd\":\"动作码\",\"cmd_value\":\"动作附带数值\"}],\"memory\":\"对话记录与总结,总字数三百以内\"}\n"
    "动作码只能从:forward/backward/left/right/stop/none。"
    "没有控制动作时 actions 放一个 cmd 为 none 的对象。"
    "当一句话包含多个动作时,按顺序生成多个 actions 元素。voice 只生成一次,memory 只维护一份。"
    "永远返回 JSON 对象(即使只有一个动作)。"
    "voice 字段不包含双引号,引用用书名号《》或单引号。"
    "不允许输出 JSON 之外的任何字符。"
    "用户的语音识别可能不准,你有时需要根据谐音猜测用户意图。"
    "如果对话历史前有★提炼标记,说明某维已满,请把标记里列出的5条记录总结为一条摘要,放在JSON的digest字段里,同时仍要正常回复voice/actions/memory。"
    "如果没有★提炼标记,可以省略digest字段。";

#define MEM_DIM      5           /* 5 个维度 */
#define MEM_SLOTS    5           /* 每维 5 格 */
#define MEM_LEN      96          /* 每格 96 字节(约32个中文字) */
static char  mem_grid[MEM_DIM][MEM_SLOTS][MEM_LEN];
static int   mem_cnt[MEM_DIM];   /* 每维当前已用格数 */
static int   digest_dim = -1;    /* 本轮需提炼的维(-1=无需提炼) */

/* 写入格子:UTF-8 安全截断,不切中文 */
static void mem_put(char *dst, const char *src)
{
    size_t slen = strlen(src);
    if (slen < MEM_LEN) { memcpy(dst, src, slen); dst[slen] = '\0'; return; }
    int cut = MEM_LEN - 1;
    /* 回退跳过 UTF-8 后续字节(10xxxxxx),保证不截在多字节字符中间 */
    while (cut > 0 && ((unsigned char)src[cut] & 0xC0) == 0x80) cut--;
    memcpy(dst, src, cut);
    dst[cut] = '\0';
}

/* 追加一格到第 d 维。带边界保护:满了就整体上移丢掉最旧一条,再写末格。
 * 原来直接 mem_grid[0][mem_cnt[0]++] 没有上界——digest 只是"请"AI 返回,
 * AI 不返回(或走了 fallback 分支)时 mem_cnt[0] 会一路涨,写穿整个网格。*/
static void mem_push(int d, const char *src)
{
    if (d < 0 || d >= MEM_DIM || src == NULL) return;
    if (mem_cnt[d] >= MEM_SLOTS) {
        ESP_LOGW(TAG, "mem dim%d full (no digest), drop oldest", d);
        for (int s = 0; s < MEM_SLOTS - 1; s++)
            memcpy(mem_grid[d][s], mem_grid[d][s + 1], MEM_LEN);
        mem_cnt[d] = MEM_SLOTS - 1;
    }
    mem_put(mem_grid[d][mem_cnt[d]], src);
    mem_cnt[d]++;
}


static char response_buffer[8192];
static int resp_index = 0;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id)
    {
        case HTTP_EVENT_ON_DATA:
            if (resp_index + evt->data_len < (int)sizeof(response_buffer)) {
                memcpy(response_buffer + resp_index, evt->data, evt->data_len);
                resp_index += evt->data_len;
            } else {
                ESP_LOGW(TAG, "response too long, truncated at %d", (int)sizeof(response_buffer));
            }
            break;

        case HTTP_EVENT_ON_FINISH:
            response_buffer[resp_index] = '\0';
            ESP_LOGD(TAG, "response:%.300s", response_buffer);   /* 降级为 DEBUG,避免刷屏 */
            AITRANSCJSON_PermitLock = PermitLock_Free;
            break;

        default:
            break;
    }
    return ESP_OK;
}

static void AI_request(char *Message)
{
    /* 清游标的同时必须清内容:HTTP 失败时 esp_http_client 不会派发
     * HTTP_EVENT_ON_FINISH,response_buffer 里仍是上一轮的响应体。若此处
     * 只把 resp_index 归零,下一轮 perform 失败后 parse_ai_response 会把
     * 上一轮的响应再解析一遍,把同一条动作重新塞进 ai_cmd_queue —— 用户
     * 没说话,车却自己动。清空后最坏情况只是解析空串失败。*/
    resp_index = 0;
    response_buffer[0] = '\0';

    esp_http_client_config_t config = {
        .url = "https://api.deepseek.com/v1/chat/completions",
        .event_handler = http_event_handler,
        /* 已经挂了 crt_bundle 做链校验,再关 common name 校验只会白白削弱 TLS,
         * 让中间人用任意受信证书冒充 api.deepseek.com。保持默认(校验开启)。*/
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 20000,        /* 给 deepseek 足够时间生成 memory */
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if(client == NULL){
        ESP_LOGE(TAG, "http_init fail, heap=%lu", (unsigned long)esp_get_free_heap_size());
        return;
    }

    esp_http_client_set_method(client, HTTP_METHOD_POST);

    esp_http_client_set_header(client,
        "Content-Type",
        "application/json");

    esp_http_client_set_header(client,
        "Authorization",
        "Bearer " CONFIG_DEEPSEEK_API_KEY);

    /* 用 cJSON 构造请求体,自动转义,prompt/message 可含引号/换行等任意字符 */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "model", CONFIG_DEEPSEEK_MODEL);
    cJSON *messages = cJSON_AddArrayToObject(root, "messages");

    cJSON *sys = cJSON_CreateObject();
    cJSON_AddStringToObject(sys, "role", "system");
    cJSON_AddStringToObject(sys, "content", AI_SYSTEM_PROMPT);
    cJSON_AddItemToArray(messages, sys);

    cJSON *user = cJSON_CreateObject();

    static char user_content[4096];   /* 25格全满+格式≈2.5KB,static不占栈 */
    int pos = 0;

    /* 提炼标记:某维满5格时,让AI在回复里附带digest。
     * 最后一维没有上级可承接,不参与 digest(它的溢出由 mem_push 丢最旧兜底)。*/
    digest_dim = -1;
    for (int d = 0; d < MEM_DIM - 1; d++) {
        if (mem_cnt[d] == MEM_SLOTS) { digest_dim = d; break; }
    }
    if (digest_dim >= 0) {
        pos += snprintf(user_content, sizeof(user_content),
            "★请提炼第%d维的5条记录为一条摘要,放在JSON的digest字段:\n",
            digest_dim);
        for (int s = 0; s < MEM_SLOTS; s++)
            pos += snprintf(user_content + pos, sizeof(user_content) - pos,
                " [%d] %s\n", s, mem_grid[digest_dim][s]);
        pos += snprintf(user_content + pos, sizeof(user_content) - pos,
            "---\n");
    }

    /* 正常指令 + 所有维的记忆 */
    pos += snprintf(user_content + pos, sizeof(user_content) - pos,
                    "指令: %s\n", Message);
    for (int d = 0; d < MEM_DIM; d++) {
        if (mem_cnt[d] == 0) continue;
        pos += snprintf(user_content + pos, sizeof(user_content) - pos,
                        "第%d维(%d/5):\n", d, mem_cnt[d]);
        for (int s = 0; s < mem_cnt[d]; s++)
            pos += snprintf(user_content + pos, sizeof(user_content) - pos,
                            "  [%d] %s\n", s, mem_grid[d][s]);
    }
    cJSON_AddStringToObject(user, "role", "user");
    cJSON_AddStringToObject(user, "content", user_content);
    cJSON_AddItemToArray(messages, user);

    char *post_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (post_data) {
        esp_http_client_set_post_field(client, post_data, strlen(post_data));
        /* set_post_field 只存指针不拷贝,post_data 必须在 perform 期间有效,
         * 所以 free 放到 cleanup 之后 */
    } else {
        ESP_LOGE(TAG, "json build fail");
    }

    esp_err_t err = esp_http_client_perform(client);
    dbg_printf("AI: perform ret=%d\n", err);

    if(err == ESP_OK)
    {
        ESP_LOGI(TAG, "HTTP OK");
    }
    else
    {
        ESP_LOGE(TAG, "HTTP FAIL");
    }

    esp_http_client_cleanup(client);
    free(post_data);   /* perform 已用完,现在释放 */
}

static void parse_ai_response(void)
{
    if(AITRANSCJSON_PermitLock != PermitLock_Free){
        ESP_LOGW(TAG, "parse skip: HTTP 未完成(PermitLock 未 Free)");
        return;
    }
    /* 立刻上锁,不能等到函数末尾:下面有多个提前 return(解析失败/无 choices/
     * 无 message/fallback 提取),只在末尾上锁会让这些路径把锁留在 Free,
     * 于是下一轮即使 HTTP 失败也会放行解析。*/
    AITRANSCJSON_PermitLock = PermitLock_Lock;

    cJSON *root = cJSON_Parse(response_buffer);
    if(root == NULL){
        ESP_LOGW(TAG, "parse fail, response: %s", response_buffer);
        return;
    }

    cJSON *choices = cJSON_GetObjectItem(root, "choices");
    if(choices == NULL){ ESP_LOGW(TAG, "no choices, response: %s", response_buffer); cJSON_Delete(root); return; }

    cJSON *first   = cJSON_GetArrayItem(choices, 0);
    if(first == NULL){ cJSON_Delete(root); ESP_LOGW(TAG, "no first choice"); return; }

    cJSON *message = cJSON_GetObjectItem(first, "message");
    if(message == NULL){ cJSON_Delete(root); ESP_LOGW(TAG, "no message"); return; }

    cJSON *content = cJSON_GetObjectItem(message, "content");
    if(content)
    {
        cJSON *rootin = cJSON_Parse(content->valuestring);
        if(rootin == NULL){
            ESP_LOGW(TAG, "parse fail, response: %s", content->valuestring);
            /* fallback:AI 偶尔输出未转义双引号导致 JSON 坏,用字符串匹配提取 cmd 保证控制不断 */
            ai_cmd_t cmd_msg;
            memset(&cmd_msg, 0, sizeof(cmd_msg));
            const char *p = strstr(content->valuestring, "\"cmd\":\"");
            if(p){
                p += 7;
                int i = 0;
                while(*p && *p != '"' && i < (int)sizeof(cmd_msg.cmd)-1) cmd_msg.cmd[i++] = *p++;
                cmd_msg.cmd[i] = '\0';
            }
            const char *v = strstr(content->valuestring, "\"cmd_value\":\"");
            if(v){
                v += 13;
                char valbuf[16] = {0};
                int j = 0;
                while(*v && *v != '"' && j < (int)sizeof(valbuf)-1) valbuf[j++] = *v++;
                cmd_msg.cmd_value = (int16_t)strtol(valbuf, NULL, 10);
            }
            if(cmd_msg.cmd[0]){
                xQueueSend(ai_cmd_queue, &cmd_msg, 0);
                ESP_LOGI(TAG, "fallback cmd=%s val=%d", cmd_msg.cmd, cmd_msg.cmd_value);
            }
            cJSON_Delete(root);   /* fallback 路径也要释放 root */
            return;
        }

        /* voice + actions[] + memory 协议 */
        cJSON *voice   = cJSON_GetObjectItem(rootin, "voice");
        cJSON *actions = cJSON_GetObjectItem(rootin, "actions");
        cJSON *memory  = cJSON_GetObjectItem(rootin, "memory");

        /* voice → 语音队列(tts_task 消费,走阿里云 NLS 合成后由喇叭播出) */
        if (voice && voice->valuestring && voice->valuestring[0]) {
            ai_voice_t voice_msg;
            memset(&voice_msg, 0, sizeof(voice_msg));
            strncpy(voice_msg.buf, voice->valuestring, sizeof(voice_msg.buf) - 1);
            if (xQueueSend(ai_voice_queue, &voice_msg, 0) != pdPASS) {
                ESP_LOGW(TAG, "ai_voice_queue full, voice dropped: %s", voice_msg.buf);
            } else {
                printf("voice: %s\n", voice_msg.buf);
            }
        }

        /* actions[] → 逐条塞控制队列 */
        if (actions && cJSON_IsArray(actions)) {
            int n = cJSON_GetArraySize(actions);
            for (int i = 0; i < n; i++) {
                cJSON *act = cJSON_GetArrayItem(actions, i);
                cJSON *acmd = cJSON_GetObjectItem(act, "cmd");
                cJSON *aval = cJSON_GetObjectItem(act, "cmd_value");
                if (acmd && acmd->valuestring) {
                    /* stop 置急停标志(粘性:后续动作不再清除,直到控制任务消费) */
                    if (strcmp(acmd->valuestring, "stop") == 0) ser_emg_stop = true;

                    ai_cmd_t cmd_msg;
                    strncpy(cmd_msg.cmd, acmd->valuestring, sizeof(cmd_msg.cmd) - 1);
                    cmd_msg.cmd[sizeof(cmd_msg.cmd) - 1] = '\0';
                    if (aval && cJSON_IsString(aval))
                        cmd_msg.cmd_value = (int16_t)strtol(aval->valuestring, NULL, 10);
                    else if (aval && cJSON_IsNumber(aval))
                        cmd_msg.cmd_value = (int16_t)aval->valuedouble;
                    else
                        cmd_msg.cmd_value = 0;
                    if (xQueueSend(ai_cmd_queue, &cmd_msg, 0) != pdPASS)
                        ESP_LOGW(TAG, "ai_cmd_queue full, action[%d] dropped: %s", i, cmd_msg.cmd);
                    else
                        printf("action[%d]: %s val=%d\n", i, cmd_msg.cmd, cmd_msg.cmd_value);
                }
            }
        } else {
            ESP_LOGW(TAG, "no actions, response: %s", content->valuestring);
        }

        /* memory → 历史记忆 */
        /* digest:AI 提炼满维 → 清空该维,摘要进上级 */
        cJSON *digest = cJSON_GetObjectItem(rootin, "digest");
        if (digest && digest->valuestring && digest_dim >= 0 && digest_dim + 1 < MEM_DIM) {
            int d_up = digest_dim + 1;
            memset(mem_grid[digest_dim], 0, sizeof(mem_grid[digest_dim]));
            mem_cnt[digest_dim] = 0;
            mem_push(d_up, digest->valuestring);
            printf("digest: dim%d→dim%d[%d]\n", digest_dim, d_up, mem_cnt[d_up] - 1);
        }
        digest_dim = -1;

        /* memory:当前轮存入第0维下一格 */
        if (memory && memory->valuestring) {
            mem_push(0, memory->valuestring);
            printf("memory[0][%d]: %s\n", mem_cnt[0] - 1, mem_grid[0][mem_cnt[0] - 1]);
        }

        cJSON_Delete(rootin);   /* 必须释放,否则每次请求泄漏一棵 JSON 树 */
    }
    else
    {
        ESP_LOGW(TAG, "no content");
    }

    cJSON_Delete(root);   /* 锁已在函数开头置为 Lock,此处不再重复 */
}

void ai_task(void *arg) {
    while (1) {
        static char buf[256];

        /* 未拿到 IP(开机初期或掉线中)就挂起:WIFI.c 在 GOT_IP 时置 Free、
         * 在 DISCONNECTED 时置回 Lock,所以这里逐轮检查即可覆盖重连。*/
        if (WIFIGOTIP_PermitLock != PermitLock_Free) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        asr_result_t new_cmd;

        if (xQueueReceive(asr_queue, &new_cmd, 0) != pdPASS) {
            vTaskDelay(10);
            continue;
        }
        if (new_cmd.cmd[0] == '\0') {   /* 空识别结果(静音/噪声),忽略不发 AI */
            continue;
        }
        memset(buf, 0, sizeof(buf));
        strncpy(buf, new_cmd.cmd, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        printf("Received input: %s\n", buf);
        AI_request(buf);
        parse_ai_response();
    }
}

void AI_Init(void) {
    ai_voice_queue = xQueueCreate(4, sizeof(ai_voice_t));   // 深度4 = 缓存4条指令
    ai_cmd_queue = xQueueCreate(8, sizeof(ai_cmd_t));   // 深度8 = 缓存8条动作(AI 可一次返回多个动作)
    if (ai_voice_queue == NULL || ai_cmd_queue == NULL) {
        ESP_LOGE(TAG, "queue create fail");
    }
    if (CONFIG_DEEPSEEK_API_KEY[0] == '\0') {
        ESP_LOGE(TAG, "DeepSeek API key 未配置!请在 menuconfig 的 'AI 语音控制配置' 中填写");
    }
    /* 注意:ai_task 栈必须给到 12288——HTTPS+TLS 握手 + cJSON 解析开销大,4K 会溢出。
     * 见 SER_AIWORK_CreatPin() 里 ai_task 的栈参数。*/
}
