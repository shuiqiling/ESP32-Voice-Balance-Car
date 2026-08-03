#include "MYHAL_MOTOR.h"
#include "BSP_MOTOR.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include <stdio.h>

#define HAL_MOTOR_PERIOD 500
#define HAL_MOTOR_RESOLUTION 1000000
#define HAL_MOTOR_DUTY 0

static mcpwm_timer_handle_t MYHAL_MOTOR_TIMER_HANDLE;
static mcpwm_oper_handle_t MYHAL_MOTOR_Operator_HANDLE[BSP_MOTOR_NUM];
static mcpwm_cmpr_handle_t MYHAL_MOTOR_Comparator_HANDLE[BSP_MOTOR_NUM];
static mcpwm_gen_handle_t MYHAL_MOTOR_Generator_HANDLE[BSP_MOTOR_NUM];

void MYHAL_MOTOR_Init(BSP_MOTOR_m MYHAL_MOTOR_ID){
    esp_err_t ret;

    gpio_config_t MYHAL_GPIO_IN1Config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BSP_MOTORS[MYHAL_MOTOR_ID].BSP_MOTOR_IN1,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config_t MYHAL_GPIO_IN2Config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BSP_MOTORS[MYHAL_MOTOR_ID].BSP_MOTOR_IN2,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    ret = gpio_config(&MYHAL_GPIO_IN1Config);
    if(ret != ESP_OK){
        printf("MOTOR GPIO IN1 FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = gpio_config(&MYHAL_GPIO_IN2Config);
    if(ret != ESP_OK){
        printf("MOTOR GPIO IN2 FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    gpio_set_level(BSP_MOTORS[MYHAL_MOTOR_ID].BSP_MOTOR_IN1, 0);
    gpio_set_level(BSP_MOTORS[MYHAL_MOTOR_ID].BSP_MOTOR_IN2, 0);

    const mcpwm_timer_config_t MYHAL_MOTOR_Timer_config = {
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .group_id = 0,
        .intr_priority = 0,
        .period_ticks = HAL_MOTOR_PERIOD,
        .resolution_hz = HAL_MOTOR_RESOLUTION
    };
    const mcpwm_operator_config_t MYHAL_MOTOR_Operator_config = {
        .group_id = 0,
        .intr_priority = 0
    };
    const mcpwm_comparator_config_t MYHAL_MOTOR_Comparator_config = {
        .flags.update_cmp_on_tez = 1
    };
    const mcpwm_generator_config_t MYHAL_MOTOR_Generator_config = {
        .gen_gpio_num = BSP_MOTORS[MYHAL_MOTOR_ID].BSP_MOTOR_PWM
    };

    if (MYHAL_MOTOR_TIMER_HANDLE == NULL){
        ret = mcpwm_new_timer(&MYHAL_MOTOR_Timer_config, &MYHAL_MOTOR_TIMER_HANDLE);
        if(ret != ESP_OK){
            printf("MOTOR NEW TIMER FAIL: %s\n", esp_err_to_name(ret));
            return;
        }
        ret = mcpwm_timer_enable(MYHAL_MOTOR_TIMER_HANDLE);
        if(ret != ESP_OK){
            printf("MOTOR TIMER ENABLE FAIL: %s\n", esp_err_to_name(ret));
            return;
        }
        ret = mcpwm_timer_start_stop(MYHAL_MOTOR_TIMER_HANDLE, MCPWM_TIMER_START_NO_STOP);
        if(ret != ESP_OK){
            printf("MOTOR TIMER START FAIL: %s\n", esp_err_to_name(ret));
            return;
        }
    }
    ret = mcpwm_new_operator(&MYHAL_MOTOR_Operator_config, &MYHAL_MOTOR_Operator_HANDLE[MYHAL_MOTOR_ID]);
    if(ret != ESP_OK){
        printf("MOTOR NEW OPERATOR FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = mcpwm_operator_connect_timer(MYHAL_MOTOR_Operator_HANDLE[MYHAL_MOTOR_ID], MYHAL_MOTOR_TIMER_HANDLE);
    if(ret != ESP_OK){
        printf("MOTOR CONNECT TIMER FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = mcpwm_new_comparator(MYHAL_MOTOR_Operator_HANDLE[MYHAL_MOTOR_ID], &MYHAL_MOTOR_Comparator_config, &MYHAL_MOTOR_Comparator_HANDLE[MYHAL_MOTOR_ID]);
    if(ret != ESP_OK){
        printf("MOTOR NEW COMPARATOR FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = mcpwm_new_generator(MYHAL_MOTOR_Operator_HANDLE[MYHAL_MOTOR_ID], &MYHAL_MOTOR_Generator_config, &MYHAL_MOTOR_Generator_HANDLE[MYHAL_MOTOR_ID]);
    if(ret != ESP_OK){
        printf("MOTOR NEW GENERATOR FAIL: %s\n", esp_err_to_name(ret));
        return;
    }

    mcpwm_gen_timer_event_action_t MYHAL_MOTOR_Generator_event_action = {
        .action = MCPWM_GEN_ACTION_HIGH,
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .event = MCPWM_TIMER_EVENT_EMPTY
    };
    mcpwm_gen_compare_event_action_t MYHAL_MOTOR_compare_event_action = {
        .action = MCPWM_GEN_ACTION_LOW,
        .comparator = MYHAL_MOTOR_Comparator_HANDLE[MYHAL_MOTOR_ID],
        .direction = MCPWM_TIMER_DIRECTION_UP
    };

    ret = mcpwm_comparator_set_compare_value(MYHAL_MOTOR_Comparator_HANDLE[MYHAL_MOTOR_ID], HAL_MOTOR_DUTY * HAL_MOTOR_PERIOD / 100);
    if(ret != ESP_OK){
        printf("MOTOR SET COMPARE FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = mcpwm_generator_set_action_on_timer_event(MYHAL_MOTOR_Generator_HANDLE[MYHAL_MOTOR_ID], MYHAL_MOTOR_Generator_event_action);
    if(ret != ESP_OK){
        printf("MOTOR SET TIMER ACTION FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
    ret = mcpwm_generator_set_action_on_compare_event(MYHAL_MOTOR_Generator_HANDLE[MYHAL_MOTOR_ID], MYHAL_MOTOR_compare_event_action);
    if(ret != ESP_OK){
        printf("MOTOR SET COMPARE ACTION FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
}

void MYHAL_MOTOR_FORWARD(BSP_MOTOR_m MYHAL_MOTOR_ID, float Speed){
    /* 先关反向桥再开正向桥,避免换向瞬间 IN1/IN2 同时为高(瞬时刹车大电流) */
    gpio_set_level(BSP_MOTORS[MYHAL_MOTOR_ID].BSP_MOTOR_IN2, 0);
    gpio_set_level(BSP_MOTORS[MYHAL_MOTOR_ID].BSP_MOTOR_IN1, 1);
    esp_err_t ret;
    uint32_t ticks = (uint32_t)(Speed * HAL_MOTOR_PERIOD / 100.0f);
    ret = mcpwm_comparator_set_compare_value(MYHAL_MOTOR_Comparator_HANDLE[MYHAL_MOTOR_ID], ticks);
    if(ret != ESP_OK){
        printf("MOTOR FORWARD SET SPEED FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
}

void MYHAL_MOTOR_MOVEBACK(BSP_MOTOR_m MYHAL_MOTOR_ID, float Speed){
    /* 先关正向桥再开反向桥(同上) */
    gpio_set_level(BSP_MOTORS[MYHAL_MOTOR_ID].BSP_MOTOR_IN1, 0);
    gpio_set_level(BSP_MOTORS[MYHAL_MOTOR_ID].BSP_MOTOR_IN2, 1);
    esp_err_t ret;
    uint32_t ticks = (uint32_t)(Speed * HAL_MOTOR_PERIOD / 100.0f);
    ret = mcpwm_comparator_set_compare_value(MYHAL_MOTOR_Comparator_HANDLE[MYHAL_MOTOR_ID], ticks);
    if(ret != ESP_OK){
        printf("MOTOR MOVEBACK SET SPEED FAIL: %s\n", esp_err_to_name(ret));
        return;
    }
}

void MYHAL_MOTOR_STOP(BSP_MOTOR_m MYHAL_MOTOR_ID){
    gpio_set_level(BSP_MOTORS[MYHAL_MOTOR_ID].BSP_MOTOR_IN1, 0);
    gpio_set_level(BSP_MOTORS[MYHAL_MOTOR_ID].BSP_MOTOR_IN2, 0);
}

void MYHAL_MOTOR_MOVE(BSP_MOTOR_m MYHAL_MOTOR_ID, float Speed){
    if(Speed >= 0.0f){
        MYHAL_MOTOR_FORWARD(MYHAL_MOTOR_ID, Speed);
    } 
    else if(Speed < 0.0f){
        MYHAL_MOTOR_MOVEBACK(MYHAL_MOTOR_ID, -Speed);
    }
    else{
        MYHAL_MOTOR_STOP(MYHAL_MOTOR_ID);
    }

}
