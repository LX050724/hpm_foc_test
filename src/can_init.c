#include "hpm_clock_drv.h"
#include "hpm_common.h"
#include "hpm_mcan_drv.h"
#include "hpm_soc.h"
#include "pinmux.h"
#include <stdint.h>
#include <stdio.h>

void board_init_canfd()
{
    init_can_pins(HPM_MCAN3);

    clock_add_to_group(clock_can3, 0);
    clock_set_source_divider(clock_can3, clk_src_pll0_clk0, 10);
    uint32_t freq = clock_get_frequency(clock_can3);

    mcan_config_t can_config;
    mcan_filter_elem_t can_filters[16];
    mcan_get_default_config(HPM_MCAN3, &can_config);
    can_filters[0].filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER;
    can_filters[0].filter_config = MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO0_IF_MATCH;
    can_filters[0].can_id_type = MCAN_CAN_ID_TYPE_STANDARD;
    can_filters[0].sync_message = 1;
    can_filters[0].filter_id = 0x0;
    can_filters[0].filter_mask = 0x0;

    can_config.baudrate = 1000000;
    can_config.mode = mcan_mode_normal;
    can_config.all_filters_config.std_id_filter_list.filter_elem_list = &can_filters[0];
    can_config.all_filters_config.std_id_filter_list.mcan_filter_elem_count = 1;
    can_config.all_filters_config.global_filter_config.accept_non_matching_std_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT;
    can_config.all_filters_config.global_filter_config.accept_non_matching_ext_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT;
    can_config.all_filters_config.global_filter_config.reject_remote_std_frame = true;
    can_config.all_filters_config.global_filter_config.reject_remote_ext_frame = true;
    can_config.txbuf_trans_interrupt_mask = ~0UL;
    can_config.interrupt_mask = MCAN_EVENT_RECEIVE;
    hpm_stat_t status = mcan_init(HPM_MCAN3, &can_config, freq);
    printf("mcan_init status = %d\n", status);

    mcan_enable_interrupts(HPM_MCAN3, MCAN_EVENT_RECEIVE);
    mcan_enable_txbuf_transmission_interrupt(HPM_MCAN3, ~0UL);

    intc_m_enable_irq_with_priority(IRQn_CAN3, 1);
}

SDK_DECLARE_EXT_ISR_M(IRQn_CAN3, board_canfd_isr);
void board_canfd_isr()
{
    mcan_rx_message_t s_can_rx_buf;
    uint32_t flags = mcan_get_interrupt_flags(HPM_MCAN3);

    if ((flags & MCAN_INT_RXFIFI0_NEW_MSG) != 0)
    {
        mcan_read_rxfifo(HPM_MCAN3, 0, &s_can_rx_buf);
        printf("MCAN_INT_RXFIFI0_NEW_MSG %#x\n", s_can_rx_buf.std_id);
    }

    if ((flags & MCAN_INT_RXFIFO1_NEW_MSG) != 0U)
    {
        mcan_read_rxfifo(HPM_MCAN3, 0, &s_can_rx_buf);
        printf("MCAN_INT_RXFIFO1_NEW_MSG %#x\n", s_can_rx_buf.std_id);
    }

    if ((flags & MCAN_INT_MSG_STORE_TO_RXBUF) != 0U)
    {
    }

    mcan_clear_interrupt_flags(HPM_MCAN3, flags);
}

int board_can_transmit_std_txfifo_nonblocking(uint16_t std_id, void *data, uint8_t dlc)
{
    mcan_tx_frame_t can_frame = {};
    uint32_t fifo_index = 0;
    can_frame.dlc = dlc;
    can_frame.std_id = std_id;
    can_frame.event_fifo_control = 1;
    can_frame.message_marker_l = 0;
    memcpy(can_frame.data_8, data, dlc);

    int ret = mcan_transmit_via_txfifo_nonblocking(HPM_MCAN3, &can_frame, &fifo_index);
    if (ret != status_success)
        return -ret;
    return fifo_index;
}

int board_can_transmit_ext_txfifo_nonblocking(uint32_t ext_id, void *data, uint8_t dlc)
{
    mcan_tx_frame_t can_frame = {};
    uint32_t fifo_index = 0;
    can_frame.dlc = 8;
    can_frame.ext_id = ext_id;
    can_frame.use_ext_id = true;
    can_frame.event_fifo_control = 1;
    can_frame.message_marker_l = 0;
    memcpy(can_frame.data_8, data, dlc);

    int ret = mcan_transmit_via_txfifo_nonblocking(HPM_MCAN3, &can_frame, &fifo_index);
    if (ret != status_success)
        return -ret;
    return fifo_index;
}
