#include "MYHAL_ENCODER.h"
#include "BSP_ENCODER.h"
#include "driver/pulse_cnt.h"
#include <stdio.h>

#define MYHAL_ENCODER_GLITCH_NS    1000
#define MYHAL_ENCODER_COUNT_LIMIT  30000

static pcnt_unit_handle_t ENCODER_Unit_HANDLE[BSP_ENCODER_NUM];
static pcnt_channel_handle_t ENCODER_Chan_HANDLE[BSP_ENCODER_NUM];
static int MYHAL_ENCODER_CURRENTCOUNT[BSP_ENCODER_NUM];
static int MYHAL_ENCODER_LASTCOUNT[BSP_ENCODER_NUM];
static int MYHAL_ENCODER_LASTVALID[BSP_ENCODER_NUM];   /* 最近一次成功读到的计数 */

void MYHAL_ENCODER_Init(BSP_ENCODER_ID_m MYHAL_ENCODER_ID){
    esp_err_t ret;

    const pcnt_unit_config_t ENCODER_Unit_config = {
        .high_limit = MYHAL_ENCODER_COUNT_LIMIT,
        .low_limit = -MYHAL_ENCODER_COUNT_LIMIT,
        .intr_priority = 0,
        .flags.accum_count = 1
    };
    const pcnt_chan_config_t ENCODER_Chan_config = {
        .edge_gpio_num = BSP_ENCODERS[MYHAL_ENCODER_ID].BSP_ENCODER_A,
        .level_gpio_num = BSP_ENCODERS[MYHAL_ENCODER_ID].BSP_ENCODER_B
    };
    const pcnt_glitch_filter_config_t ENCODER_Fil_config = {
        .max_glitch_ns = MYHAL_ENCODER_GLITCH_NS
    };

    ret = pcnt_new_unit(&ENCODER_Unit_config, &ENCODER_Unit_HANDLE[MYHAL_ENCODER_ID]);
    if(ret != ESP_OK){
        printf("ENCODER NEW UNIT FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = pcnt_new_channel(ENCODER_Unit_HANDLE[MYHAL_ENCODER_ID], &ENCODER_Chan_config, &ENCODER_Chan_HANDLE[MYHAL_ENCODER_ID]);
    if(ret != ESP_OK){
        printf("ENCODER NEW CHANNEL FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = pcnt_channel_set_edge_action(ENCODER_Chan_HANDLE[MYHAL_ENCODER_ID], PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD);
    if(ret != ESP_OK){
        printf("ENCODER SET EDGE FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = pcnt_channel_set_level_action(ENCODER_Chan_HANDLE[MYHAL_ENCODER_ID], PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if(ret != ESP_OK){
        printf("ENCODER SET LEVEL FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = pcnt_unit_set_glitch_filter(ENCODER_Unit_HANDLE[MYHAL_ENCODER_ID], &ENCODER_Fil_config);
    if(ret != ESP_OK){
        printf("ENCODER SET FILTER FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = pcnt_unit_enable(ENCODER_Unit_HANDLE[MYHAL_ENCODER_ID]);
    if(ret != ESP_OK){
        printf("ENCODER ENABLE FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = pcnt_unit_clear_count(ENCODER_Unit_HANDLE[MYHAL_ENCODER_ID]);
    if(ret != ESP_OK){
        printf("ENCODER CLEAR FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = pcnt_unit_start(ENCODER_Unit_HANDLE[MYHAL_ENCODER_ID]);
    if(ret != ESP_OK){
        printf("ENCODER START FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
}

int MYHAL_ENCODER_GetCounter(BSP_ENCODER_ID_m MYHAL_ENCODER_ID){
    /* 两条防线。原来这里是 `int value;` 加一句忽略返回值的调用:
     * ① 初始化失败时句柄是 NULL,pcnt_unit_get_count 会直接返回且不写 *value,
     *    于是那个未初始化的栈变量被控制环当成真实位置用 —— 车会一次满速纠正;
     * ② 读失败同理。两种情况下都沿用上次有效值(不动作)比返回 0 安全:
     *    位置本来在 5000,突然读成 0 会被当成 5000 的偏差去追。*/
    if (ENCODER_Unit_HANDLE[MYHAL_ENCODER_ID] == NULL) {
        return MYHAL_ENCODER_LASTVALID[MYHAL_ENCODER_ID];
    }
    int value = 0;
    if (pcnt_unit_get_count(ENCODER_Unit_HANDLE[MYHAL_ENCODER_ID], &value) != ESP_OK) {
        return MYHAL_ENCODER_LASTVALID[MYHAL_ENCODER_ID];
    }
    MYHAL_ENCODER_LASTVALID[MYHAL_ENCODER_ID] = value;
    return value;
}

int MYHAL_ENCODER_GetSpeedCounter(BSP_ENCODER_ID_m MYHAL_ENCODER_ID){
    if (ENCODER_Unit_HANDLE[MYHAL_ENCODER_ID] == NULL) {
        return 0;
    }
    int MYHAL_ENCODER_TEMPCOUNT;
    if (pcnt_unit_get_count(ENCODER_Unit_HANDLE[MYHAL_ENCODER_ID], &MYHAL_ENCODER_CURRENTCOUNT[MYHAL_ENCODER_ID]) != ESP_OK) {
        return 0;   /* 读失败:本轮增量算 0,不要用残值凑一个假速度 */
    }
    MYHAL_ENCODER_TEMPCOUNT = MYHAL_ENCODER_CURRENTCOUNT[MYHAL_ENCODER_ID] - MYHAL_ENCODER_LASTCOUNT[MYHAL_ENCODER_ID];
    if (MYHAL_ENCODER_TEMPCOUNT > MYHAL_ENCODER_COUNT_LIMIT/2)
        MYHAL_ENCODER_TEMPCOUNT -= MYHAL_ENCODER_COUNT_LIMIT;
    if (MYHAL_ENCODER_TEMPCOUNT < -MYHAL_ENCODER_COUNT_LIMIT/2)
        MYHAL_ENCODER_TEMPCOUNT += MYHAL_ENCODER_COUNT_LIMIT;
    MYHAL_ENCODER_LASTCOUNT[MYHAL_ENCODER_ID] = MYHAL_ENCODER_CURRENTCOUNT[MYHAL_ENCODER_ID];
    return MYHAL_ENCODER_TEMPCOUNT;
}
