#include "board.h"
#include "hpm_pwm_drv.h"
#include <stdint.h>

#define DEAD_TIME_NS(t) ((uint32_t)((t) / 1e9f * clock_get_frequency(clock_mot0) * 2)) // 计算t ns的死区时间值

/**
 * @brief 复位PWM计数器
 *
 */
static inline void reset_pwm_counter(void)
{
    pwm_enable_reload_at_synci(BOARD_BLDCPWM);
}

int pwm_init(uint32_t pwm_freq, uint32_t adc_trigger_cmp, uint32_t dead_time_ns)
{
    uint8_t cmp_index = BOARD_BLDCPWM_CMP_INDEX_0;
    pwm_cmp_config_t cmp_config[5] = {};
    pwm_pair_config_t pwm_pair_config = {};
    pwm_output_channel_t pwm_output_ch_cfg = {};
    uint32_t pwm_reload = clock_get_frequency(clock_mot0) / pwm_freq;

    init_pwm_pins(BOARD_BLDCPWM);

    pwm_deinit(BOARD_BLDCPWM);
    pwm_stop_counter(BOARD_BLDCPWM);
    reset_pwm_counter();

    pwm_set_reload(BOARD_BLDCPWM, 0, pwm_reload);
    pwm_set_start_count(BOARD_BLDCPWM, 0, 0);

    /* 配置PWM置位比较寄存器 */
    cmp_config[0].mode = pwm_cmp_mode_output_compare;
    cmp_config[0].cmp = pwm_reload + 1;
    cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

    /* 配置PWM复位比较寄存器 */
    cmp_config[1].mode = pwm_cmp_mode_output_compare;
    cmp_config[1].cmp = pwm_reload + 1;
    cmp_config[1].update_trigger = pwm_shadow_register_update_on_hw_event;


    pwm_get_default_pwm_pair_config(BOARD_BLDCPWM, &pwm_pair_config);

    pwm_pair_config.pwm[0].enable_output = true;
    pwm_pair_config.pwm[0].dead_zone_in_half_cycle = DEAD_TIME_NS(dead_time_ns);
    pwm_pair_config.pwm[0].invert_output = false;
    pwm_pair_config.pwm[0].fault_mode = pwm_fault_mode_force_output_0;            // 设置故障保护电平
    pwm_pair_config.pwm[0].fault_recovery_trigger = pwm_fault_recovery_on_reload; // 故障清除后下一个PWM周期自动恢复

    pwm_pair_config.pwm[1].enable_output = true;
    pwm_pair_config.pwm[1].dead_zone_in_half_cycle = DEAD_TIME_NS(dead_time_ns);
    pwm_pair_config.pwm[1].invert_output = true;
    pwm_pair_config.pwm[1].fault_mode = pwm_fault_mode_force_output_1;            // 设置故障保护电平
    pwm_pair_config.pwm[1].fault_recovery_trigger = pwm_fault_recovery_on_reload; // 故障清除后下一个PWM周期自动恢复

    /*
     * config pwm
     */
    if (status_success != pwm_setup_waveform_in_pair(BOARD_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN, &pwm_pair_config,
                                                     cmp_index, &cmp_config[0], 2))
    {
        printf("failed to setup waveform U\n");
        return 1;
    }
    if (status_success != pwm_setup_waveform_in_pair(BOARD_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN, &pwm_pair_config,
                                                     cmp_index + 2, &cmp_config[0], 2))
    {
        printf("failed to setup waveform V\n");
        return 1;
    }
    if (status_success != pwm_setup_waveform_in_pair(BOARD_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN, &pwm_pair_config,
                                                     cmp_index + 4, &cmp_config[0], 2))
    {
        printf("failed to setup waveform W\n");
        return 1;
    }

    /* 配置边缘采样时刻比较寄存器 */
    cmp_config[2].enable_ex_cmp = false;
    cmp_config[2].mode = pwm_cmp_mode_output_compare;
    cmp_config[2].cmp = 5;
    cmp_config[2].update_trigger = pwm_shadow_register_update_on_hw_event;
    pwm_config_cmp(BOARD_BLDCPWM, 8, &cmp_config[2]);

    pwm_output_ch_cfg.cmp_start_index = 8; /* start channel 8 */
    pwm_output_ch_cfg.cmp_end_index = 8;   /* end channel 8 */
    pwm_output_ch_cfg.invert_output = false;
    pwm_config_output_channel(BOARD_BLDCPWM, 8, &pwm_output_ch_cfg);

    /* 配置中间采样时刻比较寄存器 */
    cmp_config[3].enable_ex_cmp = false;
    cmp_config[3].mode = pwm_cmp_mode_output_compare;
    cmp_config[3].cmp = pwm_reload / 2;
    cmp_config[3].update_trigger = pwm_shadow_register_update_on_hw_event;
    pwm_config_cmp(BOARD_BLDCPWM, 9, &cmp_config[3]);

    pwm_output_ch_cfg.cmp_start_index = 9; /* start channel 9 */
    pwm_output_ch_cfg.cmp_end_index = 9;   /* end channel 9 */
    pwm_output_ch_cfg.invert_output = false;
    pwm_config_output_channel(BOARD_BLDCPWM, 9, &pwm_output_ch_cfg);

    /* 配置寄存器加载时刻比较寄存器 */
    cmp_config[4].mode = pwm_cmp_mode_output_compare;
    cmp_config[4].cmp = pwm_reload;
    cmp_config[4].update_trigger = pwm_shadow_register_update_on_modify;
    pwm_load_cmp_shadow_on_match(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_TRIG_CMP, &cmp_config[4]);

    pwm_start_counter(BOARD_BLDCPWM);
    pwm_issue_shadow_register_lock_event(BOARD_BLDCPWM);

    /* 开启debug故障保护模式 */
    pwm_fault_source_config_t fault_config = {};
    fault_config.source_mask = PWM_GCR_DEBUGFAULT_SET(1);
    pwm_config_fault_source(BOARD_BLDCPWM, &fault_config);

    return 0;
}

void enable_all_pwm_output(void)
{
    /*force pwm*/
    pwm_disable_sw_force(BOARD_BLDCPWM);
}

void disable_all_pwm_output(void)
{
    /*force pwm*/
    pwm_config_force_cmd_timing(BOARD_BLDCPWM, pwm_force_immediately);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_UH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_UL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_VH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_VL_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_WH_PWM_OUTPIN);
    pwm_enable_pwm_sw_force_output(BOARD_BLDCPWM, BOARD_BLDC_WL_PWM_OUTPIN);
    pwm_set_force_output(BOARD_BLDCPWM, PWM_FORCE_OUTPUT(BOARD_BLDC_UH_PWM_OUTPIN, pwm_output_0) |
                                            PWM_FORCE_OUTPUT(BOARD_BLDC_UL_PWM_OUTPIN, pwm_output_0) |
                                            PWM_FORCE_OUTPUT(BOARD_BLDC_VH_PWM_OUTPIN, pwm_output_0) |
                                            PWM_FORCE_OUTPUT(BOARD_BLDC_VL_PWM_OUTPIN, pwm_output_0) |
                                            PWM_FORCE_OUTPUT(BOARD_BLDC_WH_PWM_OUTPIN, pwm_output_0) |
                                            PWM_FORCE_OUTPUT(BOARD_BLDC_WL_PWM_OUTPIN, pwm_output_0));
    pwm_enable_sw_force(BOARD_BLDCPWM);
}
