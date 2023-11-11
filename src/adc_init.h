#pragma once
#include "hpm_adc.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DMA_ATTR ATTR_PLACE_AT_NONCACHEABLE ATTR_ALIGN(ADC_SOC_DMA_ADDR_ALIGNMENT)

typedef struct ATTR_ALIGN(ADC_SOC_DMA_ADDR_ALIGNMENT)
{
    adc_type adc_u;
    adc_type adc_w;
    volatile uint32_t adc_u_buff[48];
    volatile uint32_t adc_w_buff[48];
} CurrentADC_t;

void current_adc_init(CurrentADC_t *self, uint32_t sample_cycle, void (*isr_callback)(ADC16_Type *, uint32_t));

#ifdef __cplusplus
}
#endif