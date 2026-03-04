#include <ads704x_pio.h>
#include <ads704x_pio.pio.h>

#include <stdio.h>
#include <stdint.h>

#include <pico/stdlib.h>
#include <hardware/dma.h>
#include <hardware/pio.h>


void ads704x_init(ads704x_cfg_t *cfg, PIO pio, uint8_t cs_pin, uint8_t sck_pin, uint8_t miso_pin, uint8_t channel) {
    cfg->spi.sck = sck_pin;
    cfg->spi.miso = miso_pin;
    cfg->spi.cs = cs_pin;
    cfg->channel = channel;

    cfg->dma.data_chan = dma_claim_unused_channel(true);
    // cfg->dma.ctrl_chan = dma_claim_unused_channel(true);

    cfg->dma.timer = dma_claim_unused_timer(true);
    cfg->sample_rate = 200*1000; // Default sample rate
    dma_timer_set_fraction(cfg->dma.timer, 1, clock_get_hz(clk_sys) / (float)cfg->sample_rate);

    cfg->pio.inst = pio;
    if (cfg->channel)
        cfg->pio.offset = pio_add_program(cfg->pio.inst, &ads704x_1_program);
    else
        cfg->pio.offset = pio_add_program(cfg->pio.inst, &ads704x_0_program);
    if (cfg->pio.offset < 0)
        printf("Failed to add program\n");
    assert(cfg->pio.offset != -1);
    cfg->pio.sm = pio_claim_unused_sm(cfg->pio.inst, true);
    // Determine which program to run based on data_bits.
    // uint data_bits = 12; // Fix data_bits at 12 for now to only support ADS7049.
    // cfg->pio.sm_cfg =
    //     (data_bits == 8) ?  ads7029_program_get_default_config(cfg->pio.offset) :
    //     (data_bits == 10) ? ads7039_program_get_default_config(cfg->pio.offset) :
    //                         ads7049_program_get_default_config(cfg->pio.offset);
    if (cfg->channel)
        cfg->pio.sm_cfg = ads704x_1_program_get_default_config(cfg->pio.offset);
    else 
        cfg->pio.sm_cfg = ads704x_0_program_get_default_config(cfg->pio.offset);


    // Configure pio program (defined in auto-generated header from .pio file)
    setup_pio_ads704x(cfg->pio.inst, cfg->pio.sm, &(cfg->pio.sm_cfg), cfg->pio.offset, cfg->spi.cs, cfg->spi.sck, cfg->spi.miso);
}


// void ads704x_reset(ads704x_cfg_t *cfg)
// {
//     // Kill stream-to-memory and interrupt triggering if configured.
//     if (cfg->dma.data_chan != -1)
//         dma_channel_cleanup(cfg->dma.data_chan);
//     if (cfg->dma.ctrl_chan != -1)
//         dma_channel_cleanup(cfg->dma.ctrl_chan);
//     cfg->dma.data_chan = -1;
//     cfg->dma.ctrl_chan = -1;
// }

void _setup_dma_stream_to_memory(ads704x_cfg_t *cfg, volatile uint16_t* buff, size_t sample_count);

void ads704x_setup_dma_stream_to_memory(ads704x_cfg_t *cfg, volatile uint16_t* starting_address, size_t sample_count) {
    // _setup_dma_stream_to_memory(cfg, starting_address, sample_count, false, 0, (irq_handler_t)NULL);
    _setup_dma_stream_to_memory(cfg, starting_address, sample_count);
}

// void ads704x_setup_dma_stream_to_memory_with_interrupt(ads704x_cfg_t *cfg, volatile uint16_t* starting_address, size_t sample_count, int dma_irq_source, irq_handler_t handler_func) {
//     _setup_dma_stream_to_memory(cfg, starting_address, sample_count, true, dma_irq_source, handler_func);
// }

// void _setup_dma_stream_to_memory(ads704x_cfg_t *cfg, volatile uint16_t* buff, size_t sample_count, irq_handler_t frame_complete_handler) {
void _setup_dma_stream_to_memory(ads704x_cfg_t *cfg, volatile uint16_t* buff, size_t sample_count) {

    // FIXME: check that dma_irq_source is either DMA_IRQ_0 or DMA_IRQ_1.
    // Setup inspired by logic analyzer example:
    // https://github.com/raspberrypi/pico-examples/blob/master/pio/logic_analyser/logic_analyser.c#L65

    cfg->buf = buff;

    // Clear input shift counter, and FIFO, to remove any leftover ISR content.
    pio_sm_clear_fifos(cfg->pio.inst, cfg->pio.sm);
    pio_sm_restart(cfg->pio.inst, cfg->pio.sm);

    // cfg->dma.frame_complete_handler = frame_complete_handler;

    // Get two open DMA channels.
    // data_chan samples the adc, paced by DREQ_ADC and chained to ctrl_chan.
    // ctrl_chan reconfigures & retriggers data_chan when data_chan finishes.
    // data_chan may also trigger an interrupt if configured to do so.
    // cfg->dma.data_chan = dma_claim_unused_channel(true);
    // cfg->dma.ctrl_chan = dma_claim_unused_channel(true);
    //printf("Data channel: %d\r\n", data_chan);
    //printf("Ctrl channel: %d\r\n", ctrl_chan);
    cfg->dma.data_conf = dma_channel_get_default_config(cfg->dma.data_chan);
    // cfg->dma.ctrl_conf = dma_channel_get_default_config(cfg->dma.ctrl_chan);

    // Setup Sample Channel.
    channel_config_set_transfer_data_size(&cfg->dma.data_conf, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg->dma.data_conf, false);
    channel_config_set_write_increment(&cfg->dma.data_conf, true);
    // channel_config_set_irq_quiet(&cfg->dma.data_conf, !trigger_interrupt);
    // Pace data according to pio providing data.
    channel_config_set_dreq(&cfg->dma.data_conf, pio_get_dreq(cfg->pio.inst, cfg->pio.sm, false));
    // Pace data according to pio providing data.
    // channel_config_set_dreq(&cfg->dma.data_conf, dma_get_timer_dreq(cfg->dma.timer));
    // channel_config_set_chain_to(&cfg->dma.data_conf, cfg->dma.ctrl_chan);
    channel_config_set_enable(&cfg->dma.data_conf, true);
    // Apply data_chan configuration.
    dma_channel_configure(
        cfg->dma.data_chan,      // Channel to be configured
        &cfg->dma.data_conf,
        cfg->buf,        // write (dst) address will be loaded by ctrl_chan.
        &cfg->pio.inst->rxf[cfg->pio.sm],  // read (source) address. Does not change.
        sample_count,   // Number of word transfers i.e: count_of(adc_samples_dest).
        false           // Don't Start immediately.
    );

    dma_channel_start(cfg->dma.data_chan);

    // Enable interrupt request for the particular IRQ number if
    // trigger_interrupt is true.
    // dma_channel_set_irq0_enabled(cfg->dma.data_chan, trigger_interrupt);
    // Connnect handler function and enable interrupt.
    // if (trigger_interrupt && (handler_func != nullptr))
    // {
    //     irq_set_exclusive_handler(cfg->dma.dma_irq_source, handler_func);
    //     irq_set_enabled(cfg->dma.dma_irq_source, true);
    // }
    //printf("Configured DMA sample channel.\r\n");

    // Setup Reconfiguration Channel
    // This channel will write the starting address to the write address
    // "trigger" register, which will restart the DMA Sample Channel.
    // cfg->buf = buff;
    // //printf("data_ptr_[0] = 0x%x\r\n", buff);
    // channel_config_set_transfer_data_size(&cfg->dma.ctrl_conf, DMA_SIZE_32);
    // channel_config_set_read_increment(&cfg->dma.ctrl_conf, false); // read a single uint32.
    // channel_config_set_write_increment(&cfg->dma.ctrl_conf, false);
    // channel_config_set_irq_quiet(&cfg->dma.ctrl_conf, true);
    // channel_config_set_dreq(&cfg->dma.ctrl_conf, DREQ_FORCE); // Go as fast as possible.
    // channel_config_set_enable(&cfg->dma.ctrl_conf, true);
    // // Apply reconfig channel configuration.
    // dma_channel_configure(
    //     cfg->dma.ctrl_chan,  // Channel to be configured
    //     &cfg->dma.ctrl_conf,
    //     &dma_hw->ch[cfg->dma.data_chan].al2_write_addr_trig, // dst address.
    //     cfg->buf,   // Read (src) address is a single array with the starting address.
    //     1,          // Number of word transfers.
    //     false       // Don't Start immediately.
    // );
    // dma_channel_start(cfg->dma.ctrl_chan);
    // //printf("Configured DMA control channel.\r\n");
}

uint32_t ads704x_get_dma_channel(ads704x_cfg_t *cfg) {
    return cfg->dma.data_chan;
}

void ads704x_start(ads704x_cfg_t* cfg) {
    // sm_config_set_sideset_pins(cfg->pio.sm_cfg, cfg->spi.sck); // SCK pin controlled with SIDESET cmds.
    // pio_sm_init(cfg->pio.inst, cfg->pio.sm, cfg->pio.offset, cfg->pio.sm_cfg);
    // pio_sm_set_sideset_pins(cfg->pio.inst, cfg->pio.sm, cfg->spi.sck); // SCK pin controlled with SIDESET cmds.

    pio_gpio_init(cfg->pio.inst, cfg->spi.sck); // Reclaim control of SCK pin
    pio_gpio_init(cfg->pio.inst, cfg->spi.miso); // Reclaim control of SCK pin
    // gpio_set_function(cfg->spi.sck, GPIO_FUNC_PIO0 + (cfg->pio.inst == pio1));
    // gpio_set_function(cfg->spi.miso, GPIO_FUNC_PIO0 + (cfg->pio.inst == pio1));
    // gpio_set_function(cfg->spi.sck, PIO_FUNCSEL_NUM(pio, pin)cfg->pio.inst == pio1));
    // gpio_set_function(cfg->spi.miso, GPIO_FUNC_PIO0 + (cfg->pio.inst == pio1));
    // 

    // launch the PIO program.
    pio_sm_set_enabled(cfg->pio.inst, cfg->pio.sm, true);

}

void ads704x_stop(ads704x_cfg_t* cfg) {
    // pio_sm_set_sideset_pins(cfg->pio.inst, cfg->pio.sm, 8); // Release control of SCK pin. (usr_int_1)

    pio_sm_set_enabled(cfg->pio.inst, cfg->pio.sm, false);
    // pio_sm_clear_fifos(cfg->pio.inst, cfg->pio.sm);
    // pio_sm_restart(cfg->pio.inst, cfg->pio.sm);
    // pio_sm_clkdiv_restart(cfg->pio.inst, cfg->pio.sm);
    // pio_sm_exec(cfg->pio.inst, cfg->pio.sm, pio_encode_jmp(cfg->pio.offset));
    // pio_sm_set_enabled(cfg->pio.inst, cfg->pio.sm, true);

    // pio_sm_set_sideset_pins(cfg->pio.sm_cfg, cfg->spi.sck); // SCK pin controlled with SIDESET cmds.
    // pio_sm_set_config(cfg->pio.inst, cfg->pio.sm, &cfg->pio.sm_cfg);
}
