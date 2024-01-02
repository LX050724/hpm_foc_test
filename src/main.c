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
#include "hpm_motor_math.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_pwm_drv.h"
#include "hpm_sei_drv.h"
#include "hpm_soc.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#include "mt6701_ssi_init.h"
#include "pwm_init.h"
#include "stdbool.h"
#include "trgm.h"
#include "usb_config.h"
#include "usb_dc.h"
#include "can_init.h"
#include <hpm_math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern volatile uint8_t dtr_enable;
extern volatile uint8_t rts_enable;
extern void cdc_acm_init(void);
extern void cdc_acm_data_send_with_dtr_test(void);
extern volatile bool ep_tx_busy_flag;

void foc_pi_contrl(BLDC_CONTRL_PID_PARA *par);

#define LED_FLASH_PERIOD_IN_MS 300
#define USE_AUTO_SIMPLETIME 1

typedef struct
{
    float data[15];
    uint8_t tail[4];
} just_float_data;

#define BUF_NUM (2048 * 2 / sizeof(just_float_data))

BLDC_CONTROL_FOC_PARA foc_para;
int16_t ele_ang_offset = 0;

volatile int write_ptr;
DMA_ATTR just_float_data vofa_data[BUF_NUM];

DMA_ATTR CurrentADC_t current_adc;

float adc_IU; //!<@brief U相电流
float adc_IV; //!<@brief V相电流
float adc_IW; //!<@brief W相电流
float VBUS;   //!<@brief 母线电压
float CUR;    //!<@brief 母线电流

volatile int adc_calibration_status = 0;
int adc_calibration_u = 0;
int adc_calibration_w = 0;
int adc_calibration_i = 0;
uint32_t adc_calibration_num = 0;

int16_t adc_raw_u;
int16_t adc_raw_w;
int16_t adc_raw_i;
int16_t adc_raw_vbus;

int16_t mt6701_ang = 0;
int16_t mt6701_lst_ang = 0;
uint8_t mt6701_status = 0;

BLDC_CONTRL_PID_PARA speed_pid;
float speed, speed_last;
float speed_filter = 1 / 0.02;
int intr_count;

int32_t motor_clock_hz;
#define PWM_FREQUENCY (50000) /*PWM 频率  单位HZ*/
#define SPEED_PID_FREQUENCY (5000)
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
            // hpm_adc_disable_interrupts(&current_adc.adc_u, adc16_event_trig_complete);
            // hpm_adc_enable_interrupts(&current_adc.adc_w, adc16_event_trig_complete);
        }
        else
        {
            trgm_output_update_source(HPM_TRGM0, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A, HPM_TRGM0_INPUT_SRC_PWM0_CH9REF);
            // hpm_adc_enable_interrupts(&current_adc.adc_u, adc16_event_trig_complete);
            // hpm_adc_disable_interrupts(&current_adc.adc_w, adc16_event_trig_complete);
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
        adc_calibration_u += (current_adc.adc_u_buff[0] >> 4) & 0xfff;
        adc_calibration_w += (current_adc.adc_w_buff[4] >> 4) & 0xfff;
        adc_calibration_i += (current_adc.adc_w_buff[5] >> 4) & 0xfff;
        adc_calibration_num++;
        if (adc_calibration_num == 1024)
        {
            adc_calibration_u /= 1024;
            adc_calibration_w /= 1024;
            adc_calibration_i /= 1024;
            adc_calibration_status = 2;
        }
        return;
    }
}

void mt6701_isr_callback(uint32_t isr_flag)
{
    gpio_write_pin(HPM_GPIO0, GPIO_OE_GPIOB, 5, 1);
    // uint32_t sample_latch_tm;
    // uint32_t update_latch_tm;
    // uint32_t delta;

    // 获取采样时间
    // sample_latch_tm = sei_get_latch_time(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_0);
    // 获取更新时间
    // update_latch_tm = sei_get_latch_time(BOARD_SEI, BOARD_SEI_CTRL, SEI_LATCH_1);
    // 计算传输时间
    // delta = (update_latch_tm > sample_latch_tm) ? (update_latch_tm - sample_latch_tm)
    //                                             : (update_latch_tm - sample_latch_tm + 0xFFFFFFFFu);
    mt6701_ang = sei_get_data_value(BOARD_SEI, SEI_DAT_2);
    mt6701_status = sei_get_data_value(BOARD_SEI, SEI_DAT_3);
    int16_t mt6701_diff = mt6701_ang - mt6701_lst_ang;
    if (mt6701_diff > 8192)
        mt6701_diff -= 16384;
    if (mt6701_diff < -8192)
        mt6701_diff += 16384;
    mt6701_lst_ang = mt6701_ang;
    speed += ((mt6701_diff / 16384.0f * PWM_FREQUENCY * 60) - speed_last) / speed_filter;
    if (speed > 1000)
        speed = 1000;
    if (speed < -1000)
        speed = -1000;
    speed_last = speed;

    foc_para.electric_angle = (((mt6701_ang - ele_ang_offset) % (16384 / 7)) / (16384.0f / 7) * 360);

    if (adc_calibration_status != 2)
        return;

    adc_raw_u = ((current_adc.adc_u_buff[0] >> 4) & 0xfff) - adc_calibration_u;
    adc_raw_vbus = ((current_adc.adc_u_buff[1] >> 4) & 0xfff);
    adc_raw_w = ((current_adc.adc_w_buff[4] >> 4) & 0xfff) - adc_calibration_w;
    adc_raw_i = ((current_adc.adc_w_buff[5] >> 4) & 0xfff) - adc_calibration_i;

    adc_IU = (adc_raw_u * 3.3f / 4096.0f) / 0.1f;
    adc_IW = (adc_raw_w * 3.3f / 4096.0f) / 0.1f;
    adc_IV = 0 - adc_IU - adc_IW;
    CUR = (adc_raw_i * 3.3f / 4096.0f) / 0.1f;
    VBUS = (adc_raw_vbus * 3.3f / 4096.0f) * 10.0f;

    // 电压保护，复位芯片恢复
    if (VBUS < 15 || VBUS > 30)
    {
        disable_all_pwm_output();
        power_enable(false);
    }

    foc_para.samplcurpar.cal_u = adc_raw_u;
    foc_para.samplcurpar.cal_v = 0 - adc_raw_u - adc_raw_w;
    foc_para.samplcurpar.cal_w = adc_raw_w;

    // float alpha, beta;

    // hpm_dsp_clarke_f32(adc_IU, adc_IV, &alpha, &beta);
    // float deg = ((mt6701_ang - ele_ang_offset) / (16384.0f / 7) * (2 * M_PI));
    // float sin_angle = hpm_dsp_sin_f32(deg);
    // float cos_angle = hpm_dsp_cos_f32(deg);

    // float iq, id;
    // hpm_dsp_park_f32(alpha, beta, &id, &iq, sin_angle, cos_angle);

    // float ualpha, ubeta;
    // hpm_dsp_inv_park_f32((foc_para.currentdpipar.outval * 3.3f / 4096.0f) / 0.1f,
    //                      (foc_para.currentqpipar.outval * 3.3f / 4096.0f) / 0.1f, &ualpha, &ubeta, sin_angle,
    //                      cos_angle);

    hpm_mcl_bldc_foc_ctrl_dq_to_pwm(&foc_para);
    hpm_mcl_bldc_foc_pwmset(&foc_para.pwmpar.pwmout);

    /* 使能DTR才发送数据，方便vofa静止查看波形 */
    if (dtr_enable)
    {
        vofa_data[write_ptr].data[0] = foc_para.currentqpipar.cur;
        vofa_data[write_ptr].data[1] = foc_para.currentqpipar.target;
        vofa_data[write_ptr].data[2] = foc_para.currentqpipar.outval;

        vofa_data[write_ptr].data[3] = foc_para.currentdpipar.cur;
        vofa_data[write_ptr].data[4] = foc_para.currentdpipar.target;
        vofa_data[write_ptr].data[5] = foc_para.currentdpipar.outval;

        vofa_data[write_ptr].data[6] = adc_IU;
        vofa_data[write_ptr].data[7] = adc_IV;
        vofa_data[write_ptr].data[8] = adc_IW;

        vofa_data[write_ptr].data[9] = foc_para.electric_angle;

        vofa_data[write_ptr].data[10] = VBUS;
        vofa_data[write_ptr].data[11] = CUR;

        vofa_data[write_ptr].data[12] = speed_pid.cur;
        // vofa_data[write_ptr].data[13] = _x;
        //  vofa_data[write_ptr].data[14] = pll_speed * PWM_FREQUENCY * 60 / 2 / M_PI;

        // vofa_data[write_ptr].data[12] = foc_para.pwmpar.pwmout.pwm_u;
        // vofa_data[write_ptr].data[14] = foc_para.pwmpar.pwmout.pwm_w;

        // vofa_data[write_ptr].data[14] = (float)hpm_csr_get_core_cycle() / hpm_core_clock;

        if (write_ptr == 0)
        {
            if (!ep_tx_busy_flag)
            {
                ep_tx_busy_flag = true;
                usbd_ep_start_write(0x81, &vofa_data[BUF_NUM / 2], sizeof(just_float_data) * BUF_NUM / 2);
            }
            else
            {
                // 丢包时翻转LED
                board_led_toggle();
            }
        }

        if (write_ptr == BUF_NUM / 2)
        {
            if (!ep_tx_busy_flag)
            {
                ep_tx_busy_flag = true;
                usbd_ep_start_write(0x81, &vofa_data[0], sizeof(just_float_data) * BUF_NUM / 2);
            }
            else
            {
                board_led_toggle();
            }
        }
    }

    if (++intr_count >= (PWM_FREQUENCY / SPEED_PID_FREQUENCY))
    {
        speed_pid.cur = speed;
        speed_pid.func_pid(&speed_pid);
        foc_para.currentqpipar.target = speed_pid.outval;
        intr_count = 0;
    }

    write_ptr = (write_ptr + 1) % BUF_NUM;
    gpio_write_pin(HPM_GPIO0, GPIO_OE_GPIOB, 5, 0);
}

void foc_current_cal(BLDC_CONTROL_CURRENT_PARA *par)
{
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
    HPM_MOTOR_MATH_TYPE int_max = (par->i_max / par->i_ki);
    // HPM_MOTOR_MATH_TYPE portion_asp = 0;
    // HPM_MOTOR_MATH_TYPE portion_asi = 0;

    curerr = par->target - par->cur;
    par->mem += curerr;

    if (par->mem > int_max)
        par->mem = int_max;

    if (par->mem < -int_max)
        par->mem = -int_max;

    result = par->i_kp * curerr + par->i_ki * par->mem;

    if (result < (-par->i_max))
    {
        result = -par->i_max;
    }
    else if (result > par->i_max)
    {
        result = par->i_max;
    }

    par->outval = result;
}

int main(void)
{

    board_init();
    board_init_usb_pins();
    board_init_canfd();

    intc_set_irq_priority(CONFIG_HPM_USBD_IRQn, 2);
    cdc_acm_init();
    motor_clock_hz = clock_get_frequency(clock_mot0);
    printf("PWM_RELOAD=%d\n", PWM_RELOAD);

    HPM_IOC->PAD[IOC_PAD_PB05].PAD_CTL = IOC_PAD_PAD_CTL_SPD_SET(3);
    HPM_IOC->PAD[IOC_PAD_PB05].FUNC_CTL = IOC_PB05_FUNC_CTL_GPIO_B_05;
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOB, 5, gpiom_soc_gpio0);
    gpio_set_pin_output(HPM_GPIO0, GPIO_OE_GPIOB, 5);

    for (int i = 0; i < BUF_NUM; i++)
    {
        vofa_data[i].tail[0] = 0x00;
        vofa_data[i].tail[1] = 0x00;
        vofa_data[i].tail[2] = 0x80;
        vofa_data[i].tail[3] = 0x7f;
    }

    board_init_led_pins();
    init_power_enable_gpio();

    sei_trigger_input_config_t mt6701_trigger_config = {
        .trig_in0_enable = true,
        .trig_in0_select = 0,
    };
    mt6701_ssi_init(13000000, &mt6701_trigger_config, mt6701_isr_callback);
    intc_m_enable_irq_with_priority(BOARD_MTSEI_IRQn, 1);

    /* 连接PWMCH8、SEI_TRIG_IN0 */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM0_CH8REF, HPM_TRGM0_OUTPUT_SRC_SEI_TRIG_IN0, trgm_output_same_as_input, false);

    /* PWM ADC初始化 */
    pwm_init(PWM_FREQUENCY, 5, 100);

    current_adc.adc_u.adc_base.adc16 = BOARD_BLDC_ADC_U_BASE;
    current_adc.adc_u.module = adc_module_adc16;
    current_adc.adc_w.adc_base.adc16 = BOARD_BLDC_ADC_W_BASE;
    current_adc.adc_w.module = adc_module_adc16;

    current_adc_init(&current_adc, 25, adc_isr_callback);

    /* 连接PWMCH8、ADCX_PTRGI0A */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM0_CH8REF, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0A, trgm_output_same_as_input, false);
    /* 连接PWMCH9、ADCX_PTRGI0B */
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM0_CH8REF, HPM_TRGM0_OUTPUT_SRC_ADCX_PTRGI0B, trgm_output_same_as_input, false);

    /* 触发信号引出，测量ADC采样时刻或作为示波器观察波形的触发源 */
    HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PA25_FUNC_CTL_TRGM0_P_01;
    HPM_IOC->PAD[IOC_PAD_PA25].PAD_CTL = IOC_PAD_PAD_CTL_SPD_SET(3);
    trgm_connect(HPM_TRGM0_INPUT_SRC_PWM0_CH8REF, HPM_TRGM0_OUTPUT_SRC_MOT_GPIO1, trgm_output_same_as_input, true);
    trgm_enable_io_output(HPM_TRGM0, 0x06);

    power_enable(true);
    printf("power on\n");
    clock_cpu_delay_ms(100);
    enable_all_pwm_output();
    printf("enable pwm\n");

    speed_pid.i_kp = 3;
    speed_pid.i_ki = 0.01;
    speed_pid.target = 60;
    speed_pid.i_max = PWM_RELOAD * 0.9;
    speed_pid.func_pid = foc_pi_contrl;

    foc_para.currentqpipar.i_kp = 30;
    foc_para.currentqpipar.i_ki = 0.03;
    foc_para.currentqpipar.target = 100;
    foc_para.currentqpipar.i_max = PWM_RELOAD * 0.9;
    foc_para.currentqpipar.func_pid = foc_pi_contrl;

    foc_para.currentdpipar.i_kp = 0;
    foc_para.currentdpipar.i_ki = 0.7;
    foc_para.currentdpipar.i_max = PWM_RELOAD * 0.5;
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
    // ele_ang_offset = -4037;
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

    /* 关闭ADC中断，完全使用SSI中断，节省CPU */
    hpm_adc_disable_interrupts(&current_adc.adc_u, adc16_event_trig_complete);
    hpm_adc_disable_interrupts(&current_adc.adc_w, adc16_event_trig_complete);

    // adc_calibration_i = adc_calibration_u;
    printf("calibration adc done u:%.3f, w:%.3f, i:%.3f\n", adc_calibration_u * 3.3f / 4096.0f,
           adc_calibration_w * 3.3f / 4096.0f, adc_calibration_i * 3.3f / 4096.0f);
    enable_all_pwm_output();
    printf("enable pwm\n");

    while (1)
    {
    }

    return 0;
}

typedef struct
{
    const char *name;
    void (*fun)(float);
    float *tar_val;
} CmdCallback_t;

CmdCallback_t cmd_list[] = {
    {"exp_iq", NULL, &foc_para.currentqpipar.target},
    {"id_p", NULL, &foc_para.currentdpipar.i_kp},
    {"id_i", NULL, &foc_para.currentdpipar.i_ki},
    {"iq_p", NULL, &foc_para.currentqpipar.i_kp},
    {"iq_i", NULL, &foc_para.currentqpipar.i_ki},
    {"speed_p", NULL, &speed_pid.i_kp},
    {"speed_i", NULL, &speed_pid.i_ki},
    {"exp_speed", NULL, &speed_pid.target},
    {"exp_id", NULL, &foc_para.currentdpipar.target},
    {"speed_filter", NULL, &speed_filter},
};

void usbd_read_callback(char *data, uint32_t len)
{
    char name[64] = {};
    float value = 0;
    char *start = data;
    for (int i = 0; i < len - 1; i++)
    {
        if (data[i] == ':')
        {
            strncpy(name, start, data + i - start);
            value = strtof(&data[i + 1], &start);
            if (start[0] == '\n')
                start++;
            for (int index = 0; index < sizeof(cmd_list) / sizeof(CmdCallback_t); index++)
            {
                if (strcmp(cmd_list[index].name, name) == 0)
                {
                    // printf("set %s %f\n", cmd_list[index].name, value);
                    if (cmd_list[index].fun == NULL)
                    {
                        *(cmd_list[index].tar_val) = value;
                    }
                    else
                    {
                        cmd_list[index].fun(value);
                    }
                    break;
                }
            }
        }
    }
}