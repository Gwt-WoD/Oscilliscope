
#include <stdio.h>
#include <stdlib.h> // For malloc and free
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"


#include "hardware/pio.h"
#include "cs_out.pio.h"

int main() {
    stdio_init_all();
    sleep_ms(100);

    // =============================================================

    // PIO pio = pio0;
    // uint sm = 0;
    // uint offset = pio_add_program(pio, &cs0_out_program);

    // pio_gpio_init(pio, 25);
    // pio_sm_set_consecutive_pindirs(pio, sm, 25, 1, true);

    // pio_sm_config c = cs0_out_program_get_default_config(offset);
    // sm_config_set_out_pins(&c, 25, 1);
    // sm_config_set_out_shift(&c, true, false, 1);
    // pio_sm_init(pio, sm, offset, &c);
    // pio_sm_set_enabled(pio, sm, true);

    // =============================================================

    // Minimal DMA to PIO TX FIFO example
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &cs0_out_program);

    pio_gpio_init(pio, 28);
    pio_sm_set_consecutive_pindirs(pio, sm, 28, 1, true);

    pio_sm_config c = cs0_out_program_get_default_config(offset);
    sm_config_set_out_pins(&c, 28, 1);
    sm_config_set_out_shift(&c, true, true, 1); // autopull
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    uint32_t pattern[8] = {1,0,1,0,1,0,1,0};

    int dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dmacfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&dmacfg, DMA_SIZE_32);
    channel_config_set_dreq(&dmacfg, DREQ_PIO0_TX0 + sm);

    dma_channel_configure(
        dma_chan,
        &dmacfg,
        &pio->txf[sm], // dest
        pattern,       // src
        8,             // transfer count
        true           // start immediately
    );


    while (1) {
        // pio_sm_put_blocking(pio, sm, 1);
        // sleep_ms(500);
        // pio_sm_put_blocking(pio, sm, 0);
        // sleep_ms(500);

        dma_channel_configure(
            dma_chan,
            &dmacfg,
            &pio->txf[sm], // dest
            pattern,       // src
            8,             // transfer count
            true           // start immediately
        );
    }
}