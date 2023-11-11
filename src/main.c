#include "adc_init.h"
#include "board.h"
#include "hpm_adc.h"
#include "hpm_adc16_regs.h"
#include "hpm_bldc_define.h"
#include "hpm_clock_drv.h"
#include "hpm_foc.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_iomux.h"
#include "hpm_pwm_drv.h"
#include "hpm_sei_drv.h"
#include "hpm_soc.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#include "mt6701_ssi_init.h"
#include "pwm_init.h"
#include "stdbool.h"
#include "trgm.h"
#include <stdint.h>
#include <stdio.h>

#define LED_FLASH_PERIOD_IN_MS 300
#define USE_AUTO_SIMPLETIME 1

float hall_pos = 0;

BLDC_CONTROL_PWM_PARA pwm_para;
BLDC_CONTROL_FOC_PARA foc_para;
float ang = 0;
float SPEED = 0.5;
float V = 0.5;
int16_t ele_ang_offset = 0;

DMA_ATTR CurrentADC_t current_adc = {.adc_u = {.adc_base.adc16 = BOARD_BLDC_ADC_U_BASE, .module = adc_module_adc16},
                                     .adc_w = {.adc_base.adc16 = BOARD_BLDC_ADC_W_BASE, .module = adc_module_adc16}};

float adc_IU, adc_IV, adc_IW;
volatile int adc_calibration_status = 0;
int adc_calibration_u = 0;
int adc_calibration_w = 0;
uint32_t adc_calibration_num = 0;

int16_t adc_raw_u;
int16_t adc_raw_w;

int32_t motor_clock_hz;
#define PWM_FREQUENCY (50000)                       /*PWM 频率  单位HZ*/
#define PWM_RELOAD (motor_clock_hz / PWM_FREQUENCY) /*20K hz  = 200 000 000/PWM_RELOAD */

/**
 * @brief 设置功率部分电源使能引脚PB15为输出模式
 *
 * @return int
 */
int init_power_enable_gpio()
{
    HPM_IOC->PAD[IOC_PAD_PB15].PAD_CTL = IOC_PAD_PAD_CTL_SPD_SET(0) | IOC_PAD_PAD_CTL_OD_SET(0);
    HPM_IOC->PAD[IOC_PAD_PB15].FUNC_CTL = IOC_PB15_FUNC_CTL_GPIO_B_15;
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOB, 15, gpiom_soc_gpio0);
    gpio_set_pin_output(HPM_GPIO0, GPIO_OE_GPIOB, 15);
    return 0;
}

int power_enable(bool en)
{
    gpio_write_pin(HPM_GPIO0, GPIO_OE_GPIOB, 15, en);
    return 0;
}

void hpm_mcl_bldc_foc_pwmset(BLDC_CONTROL_PWMOUT_PARA *par)
{
    uint32_t pwm_reload_half;
    uint32_t pwm_u_half, pwm_v_half, pwm_w_half;

    pwm_reload_half = par->i_pwm_reload >> 1;
    switch (par->i_motor_id)
    {
    case BLDC_MOTOR0_INDEX:
        pwm_u_half = par->pwm_u >> 1;
        pwm_v_half = par->pwm_v >> 1;
        pwm_w_half = par->pwm_w >> 1;
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_0, PWM_CMP_CMP_SET((pwm_reload_half + pwm_u_half)));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_1, PWM_CMP_CMP_SET((pwm_reload_half - pwm_u_half)));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_2, PWM_CMP_CMP_SET((pwm_reload_half + pwm_v_half)));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_3, PWM_CMP_CMP_SET((pwm_reload_half - pwm_v_half)));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_4, PWM_CMP_CMP_SET((pwm_reload_half + pwm_w_half)));
        pwm_cmp_force_value(BOARD_BLDCPWM, BOARD_BLDCPWM_CMP_INDEX_5, PWM_CMP_CMP_SET((pwm_reload_half - pwm_w_half)));
#if USE_AUTO_SIMPLETIME
        if (adc_calibration_status != 2)
            break;

        if (pwm_reload_half > par->pwm_u)
        {
            trgm_output_update_source(HPM_TRGM0, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A, HPM_TRGM0_INPUT_SRC_PWM0_CH8REF);
            hpm_adc_disable_interrupts(&current_adc.adc_u, adc16_event_trig_complete);
            hpm_adc_enable_interrupts(&current_adc.adc_w, adc16_event_trig_complete);
        }
        else
        {
            trgm_output_update_source(HPM_TRGM0, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A, HPM_TRGM0_INPUT_SRC_PWM0_CH9REF);
            hpm_adc_enable_interrupts(&current_adc.adc_u, adc16_event_trig_complete);
            hpm_adc_disable_interrupts(&current_adc.adc_w, adc16_event_trig_complete);
        }

        if (pwm_reload_half > par->pwm_w)
        {
            trgm_output_update_source(HPM_TRGM0, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0B, HPM_TRGM0_INPUT_SRC_PWM0_CH8REF);
        }
        else
        {
            trgm_output_update_source(HPM_TRGM0, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0B, HPM_TRGM0_INPUT_SRC_PWM0_CH9REF);
        }
#endif
        break;

    default:
        break;
    }
}

void adc_isr_callback(ADC16_Type *adc, uint32_t status)
{
    if (adc_calibration_status == 0)
        return;

    if (adc_calibration_status == 1)
    {
        adc_calibration_u += current_adc.adc_u_buff[0] & 0xffff;
        adc_calibration_w += current_adc.adc_w_buff[4] & 0xffff;
        adc_calibration_num++;
        if (adc_calibration_num == 1024)
        {
            adc_calibration_u /= 1024;
            adc_calibration_w /= 1024;
            adc_calibration_status = 2;
        }
        return;
    }

    adc_raw_u = (current_adc.adc_u_buff[0] & 0xffff) - adc_calibration_u;
    adc_raw_w = (current_adc.adc_w_buff[4] & 0xffff) - adc_calibration_w;

    adc_IU = (adc_raw_u * 3.3f / 65536.0f) / 0.04f;
    adc_IW = (adc_raw_w * 3.3f / 65536.0f) / 0.04f;
    adc_IV = 0 - adc_IU - adc_IW;

    foc_para.samplcurpar.cal_u = adc_raw_u;
    foc_para.samplcurpar.cal_v = 0 - adc_raw_u - adc_raw_w;
    foc_para.samplcurpar.cal_w = adc_raw_w;

    // ((void(*)(BLDC_CONTROL_FOC_PARA *))foc_para.func_dqsvpwm)(&foc_para);

    board_led_write(adc == HPM_ADC0);

    // motor0.adc_trig_event_callback();
}

int16_t mt6701_ang = 0;
uint8_t mt6701_status = 0;

void mt6701_isr_callback(uint32_t isr_flag)
{
    uint32_t sample_latch_tm;
    uint32_t update_latch_tm;
    uint32_t delta;

    // 获取采样时间
    sample_latch_tm = sei_get_latch_time(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_0);
    // 获取更新时间
    update_latch_tm = sei_get_latch_time(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_1);
    // 计算传输时间
    delta = (update_latch_tm > sample_latch_tm) ? (update_latch_tm - sample_latch_tm)
                                                : (update_latch_tm - sample_latch_tm + 0xFFFFFFFFu);
    mt6701_ang = sei_get_data_value(BOARD_SEI, SEI_DAT_2);
    mt6701_status = sei_get_data_value(BOARD_SEI, SEI_DAT_3);

    foc_para.electric_angle = (((mt6701_ang - ele_ang_offset) % (16384 / 7)) / (16384.0f / 7) * 360);

    if (adc_calibration_status != 2)
        return;
    hpm_mcl_bldc_foc_ctrl_dq_to_pwm(&foc_para);
    hpm_mcl_bldc_foc_pwmset(&foc_para.pwmpar.pwmout);
}

void foc_current_cal(BLDC_CONTROL_CURRENT_PARA *par)
{
    // par->cal_u = adc_IW;
    // par->cal_v = adc_IV;
    // par->cal_w = adc_IU;

    // par->cal_u = adc_IW;
    // par->cal_v = adc_IU;
    // par->cal_w = adc_IV;

    // par->cal_u = adc_IU;
    // par->cal_v = adc_IW;
    // par->cal_w = adc_IV;

    // par->cal_u = adc_IU;
    // par->cal_v = adc_IV;
    // par->cal_w = adc_IW;

    // par->cal_u = adc_IV;
    // par->cal_v = adc_IU;
    // par->cal_w = adc_IW;

    // par->cal_u = adc_IV;
    // par->cal_v = adc_IW;
    // par->cal_w = adc_IU;
}

void pi_contrl_stub_0(BLDC_CONTRL_PID_PARA *par)
{
    par->outval = 0;
}

void pi_contrl_stub_max(BLDC_CONTRL_PID_PARA *par)
{
    par->outval = par->i_max;
}

void foc_pi_contrl(BLDC_CONTRL_PID_PARA *par)
{
    HPM_MOTOR_MATH_TYPE result = 0;

    HPM_MOTOR_MATH_TYPE curerr = 0;
    // HPM_MOTOR_MATH_TYPE portion_asp = 0;
    // HPM_MOTOR_MATH_TYPE portion_asi = 0;

    curerr = par->target - par->cur;
    par->mem += curerr;
    if (par->mem > 50000)
        par->mem = 50000;

    if (par->mem < -50000)
        par->mem = -50000;

    result = par->i_kp * curerr + par->i_ki * par->mem;

    if (result < (-par->i_max)) {
        result = -par->i_max;
    } else if (result > par->i_max) {
        result = par->i_max;
    }

    par->outval = result;
}

int main(void)
{
    motor_clock_hz = clock_get_frequency(clock_mot0);

    board_init();
    board_init_led_pins();
    init_power_enable_gpio();

    sei_trigger_input_config_t mt6701_trigger_config = {
        .trig_in0_enable = true,
        .trig_in0_select = 0,
    };
    mt6701_ssi_init(9000000, &mt6701_trigger_config, mt6701_isr_callback);
    intc_m_enable_irq_with_priority(BOARD_MTSEI_IRQn, 1);

    /* 连接PWMCH8、SEI_TRIG_IN0 */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM0_CH8REF, HPM_TRGM0_OUTPUT_SRC_SEI_TRIG_IN0, trgm_output_same_as_input, false);

    /* PWM ADC初始化 */
    pwm_init(50000, 5, 100);
    current_adc_init(&current_adc, 21, adc_isr_callback);

    /* 连接PWMCH8、ADCX_PTRGI0A */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM0_CH8REF, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A, trgm_output_same_as_input, false);
    /* 连接PWMCH9、ADCX_PTRGI0B */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM0_CH8REF, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0B, trgm_output_same_as_input, false);
    power_enable(true);
    printf("power on\n");
    clock_cpu_delay_ms(100);
    enable_all_pwm_output();
    printf("enable pwm\n");

    foc_para.currentqpipar.i_kp = 0.4;
    foc_para.currentqpipar.i_ki = 0.03;
    foc_para.currentqpipar.target = 2000;
    foc_para.currentqpipar.i_max = PWM_RELOAD * 0.6;
    foc_para.currentqpipar.func_pid = foc_pi_contrl;

    foc_para.currentdpipar.i_kp = 0.4;
    foc_para.currentdpipar.i_ki = 0.03;
    foc_para.currentdpipar.i_max = PWM_RELOAD * 0.6;
    foc_para.currentdpipar.func_pid = foc_pi_contrl;

    foc_para.pwmpar.func_spwm = hpm_mcl_bldc_foc_svpwm;
    foc_para.pwmpar.i_pwm_reload_max = PWM_RELOAD * 0.9;
    foc_para.pwmpar.pwmout.i_pwm_reload = PWM_RELOAD;
    foc_para.pwmpar.pwmout.i_motor_id = BLDC_MOTOR0_INDEX;

    foc_para.samplcurpar.func_sampl = foc_current_cal;
    foc_para.func_dqsvpwm = hpm_mcl_bldc_foc_ctrl_dq_to_pwm;

    foc_para.pwmpar.target_beta = 0;
    foc_para.pwmpar.target_alpha = PWM_RELOAD * 0.5;
    printf("foc para init done, test ele_ang_offset ...\n");
    hpm_mcl_bldc_foc_svpwm(&foc_para.pwmpar);
    hpm_mcl_bldc_foc_pwmset(&foc_para.pwmpar.pwmout);
    clock_cpu_delay_ms(500);

    ele_ang_offset = mt6701_ang - 16384;
    foc_para.electric_angle = 0;

    foc_para.pwmpar.target_beta = 0;
    hpm_mcl_bldc_foc_svpwm(&foc_para.pwmpar);
    hpm_mcl_bldc_foc_pwmset(&foc_para.pwmpar.pwmout);
    printf("test ele_ang_offset done %d\n", ele_ang_offset);

    disable_all_pwm_output();
    printf("disable pwm\n");
    foc_para.pwmpar.target_alpha = 0;
    foc_para.pwmpar.target_beta = 0;

    intc_m_enable_irq_with_priority(IRQn_ADC0, 1);
#if USE_AUTO_SIMPLETIME
    intc_m_enable_irq_with_priority(IRQn_ADC1, 1);
#endif

    hpm_adc_enable_interrupts(&current_adc.adc_u, adc16_event_trig_complete);
    // hpm_adc_enable_interrupts(&current_adc.adc_w, adc16_event_trig_complete);
    printf("calibration adc\n");

    adc_calibration_status = 1;
    while (adc_calibration_status == 1)
        clock_cpu_delay_ms(1);
    printf("calibration adc done u:%.3f, w:%.3f\n", adc_calibration_u * 3.3f / 65536.0f,
           adc_calibration_w * 3.3f / 65536.0f);
    enable_all_pwm_output();
    printf("enable pwm\n");

    int16_t ele_ang_offset_bak = ele_ang_offset;
    while (1)
    {
        int c = getchar();
        switch (c)
        {
        case 'q':
            foc_para.currentdpipar.i_kp += 0.01;
            break;
        case 'w':
            foc_para.currentdpipar.i_kp -= 0.01;
            break;
        case 'a':
            foc_para.currentdpipar.i_ki += 0.01;
            break;
        case 's':
            foc_para.currentdpipar.i_ki -= 0.01;
            break;
        case 'z':
            foc_para.currentdpipar.target += 10;
            break;
        case 'x':
            foc_para.currentdpipar.target -= 10;
            break;
        case 'e':
            foc_para.currentqpipar.i_kp += 0.01;
            break;
        case 'r':
            foc_para.currentqpipar.i_kp -= 0.01;
            break;
        case 'd':
            foc_para.currentqpipar.i_ki += 0.01;
            break;
        case 'f':
            foc_para.currentqpipar.i_ki -= 0.01;
            break;
        case 'c':
            foc_para.currentqpipar.target += 10;
            break;
        case 'v':
            foc_para.currentqpipar.target -= 10;
            break;
        }
        printf("Id => p: %.2f, i: %.2f, target: %.2f\n", foc_para.currentdpipar.i_kp, foc_para.currentdpipar.i_ki,
               foc_para.currentdpipar.target);
        printf("Iq => p: %.2f, i: %.2f, target: %.2f\n", foc_para.currentqpipar.i_kp, foc_para.currentqpipar.i_ki,
               foc_para.currentqpipar.target);
    }

    return 0;
}
