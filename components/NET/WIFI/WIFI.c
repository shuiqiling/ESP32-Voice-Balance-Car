#include "WIFI.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "SystemState.h"


static const char *TAG = "WIFI";

/* 断线重连:指数退避(1,2,4,8,16,32s),之后固定 60s,避免重连风暴 */
#define WIFI_RETRY_MAX_BACKOFF_S 60
#define WIFI_RETRY_INIT_BACKOFF_S 1

static int retry_cnt = 0;
static esp_timer_handle_t retry_timer = NULL;

static void wifi_retry_connect(void *arg)
{
    esp_wifi_connect();
}

static void WIFI_EventHandler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    if(event_base == WIFI_EVENT)
    {
        switch(event_id)
        {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG,"WiFi Start");
                esp_wifi_connect();
                break;

            case WIFI_EVENT_STA_DISCONNECTED: {
                int delay_s = WIFI_RETRY_INIT_BACKOFF_S << retry_cnt;
                if (delay_s > WIFI_RETRY_MAX_BACKOFF_S) delay_s = WIFI_RETRY_MAX_BACKOFF_S;
                retry_cnt++;
                ESP_LOGW(TAG, "Disconnected, retry in %ds", delay_s);

                if (retry_timer == NULL) {
                    esp_timer_create_args_t args = {
                        .callback = wifi_retry_connect,
                        .name = "wifi_retry"
                    };
                    esp_timer_create(&args, &retry_timer);
                }
                esp_timer_start_once(retry_timer, (uint64_t)delay_s * 1000 * 1000);
                break;
            }

            default:
                break;
        }
    }

    if(event_base == IP_EVENT)
    {
        if(event_id == IP_EVENT_STA_GOT_IP)
        {
            ip_event_got_ip_t *event =
                (ip_event_got_ip_t *)event_data;

            retry_cnt = 0;   /* 连上后重置退避计数 */
            if (retry_timer) esp_timer_stop(retry_timer);

            ESP_LOGI(TAG,
                "IP:" IPSTR,
                IP2STR(&event->ip_info.ip));
            WIFIGOTIP_PermitLock = PermitLock_Free;
        }
    }
}

void WIFI_Init(void){
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init fail: %s", esp_err_to_name(ret));
        return;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &WIFI_EventHandler,
        NULL,
        NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &WIFI_EventHandler,
        NULL,
        NULL));

    if (CONFIG_WIFI_SSID[0] == '\0') {
        ESP_LOGE(TAG, "WiFi SSID 未配置!请在 menuconfig 的 'AI 语音控制配置' 中填写");
        return;
    }

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_wifi_start());
}
