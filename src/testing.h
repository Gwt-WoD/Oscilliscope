// =============================================

// LATEST WORKING TEST CODE

//DEBUG: Validation cosines
// for (uint16_t x=0; x < DataArray_len; x++) {
//     // ((uint16_t*)data_buffer0)[i] = (uint16_t)floor((2*1000 * cos(2 * M_PI * i))) + 2.5;
//     // ((uint16_t*)data_buffer1)[i] = (uint16_t)floor((2*1000 * cos(2 * M_PI * i))) + 2.5;
//     // Normalized period to length of data array => freq. * 2pi/len * x
//     uint16_t y = 1000.0f * ((2.5f * cosf(7.5f * (2.0f*M_PI) * x / DataArray_len)) + 2.5f);
//     uint16_t y2 = 1000.0f * ((0.75f * cosf(15.0f * (2.0f*M_PI) * x / DataArray_len)) + 3.0f);
//     DataArray0[x] = y;
//     DataArray1[x] = y2;
// }

// DEBUG: Validation cosines
// IMPORTANT: (Wiggle wiggle wiggle)
// const float phase1 = 2.0f * ((float)get_rand_32() / UINT32_MAX); // jitter is 200.0% of period
// const float phase2 = 0.10f * ((float)get_rand_32() / UINT32_MAX); // jitter is 10.0% of period
// for (uint x=0; x < DataArray_len; x++) {
// for (uint x=0; x < DataArray_len; x+=2) { // Half resolution
//     // Normalized period to length of data array => freq. * 2pi/len * x
//     uint16_t y = 1000.0f * ((2.5f * cosf((7.5f * (2.0f*M_PI) * x / DataArray_len) + (phase1*(4.0f*M_PI)))) + 2.5f);
//     uint16_t y2 = 1000.0f * ((0.75f * cosf((15.0f * (2.0f*M_PI) * x / DataArray_len) + (phase2*(4.0f*M_PI)))) + 3.0f);
//     DataArray0[x] = y;
//     DataArray1[x] = y2;
//     if (x+1 < DataArray_len) {
//         DataArray0[x+1] = y;
//         DataArray1[x+1] = y2;
//     }
// }


// ========================================================





// Create test waveforms in the same format as the ADC
    // generate_12bit_sin((uint16_t*)data_buffer0, (64*1024 / 2), (((1 << 12) - 1) / 2), (((1 << 12) - 1) / 2), 1, ((64*1024 / 2) / 2));
    // generate_12bit_sin((uint16_t*)data_buffer1, (64*1024 / 2), (((1 << 12) - 1) / 4), (((1 << 12) - 1) / 2), 1.5, 0);
    // generate_12bit_sin((uint16_t*)data_buffer0, (32*1024 / 2), (((1 << 12) - 1) / 2), (((1 << 12) - 1) / 2), 10000, ((32*1024 / 2) / 2));
    // generate_12bit_sin((uint16_t*)data_buffer1, (32*1024 / 2), (((1 << 12) - 1) / 4), (((1 << 12) - 1) / 2), 0.00001, 0);
    // for (int j=0; j<16;j++) {
    //     printf("%d", (bool)(((uint16_t*)data_buffer0)[(32*1024 / 2) / 4] & (1 << j)));
    // }
    // printf("\n");

    // Convert test waveforms into a scaled uint in mV
    // This is purely for testing. This functionality should be rolled
    // into the data driver code
    // conv_raw_adc((uint16_t*)data_buffer0, (64*1024 / 2), 0.5f);
    // conv_raw_adc((uint16_t*)data_buffer1, (64*1024 / 2), 0.5f);
    // conv_raw_adc((uint16_t*)data_buffer0, (32*1024 / 2), 0.5f);
    // conv_raw_adc((uint16_t*)data_buffer1, (32*1024 / 2), 0.5f);



// generate_12bit_sin((uint16_t*)data_buffer0, (32*1024 / 2), (((1 << 12) - 1) / 2), (((1 << 12) - 1) / 2), 5, ((32*1024 / 2) / 2));
// generate_12bit_sin((uint16_t*)data_buffer1, (32*1024 / 2), (((1 << 12) - 1) / 4), (((1 << 12) - 1) / 2), 1.5, 0);
// conv_raw_adc((uint16_t*)data_buffer0, (32*1024 / 2), 0.5f);
// conv_raw_adc((uint16_t*)data_buffer1, (32*1024 / 2), 0.5f);

// generate_12bit_sin((uint16_t*)data_buffer0, (32*1024 / 2), (((1 << 16) - 1) / 2), (((1 << 16) - 1) / 2), 5, ((32*1024 / 2) / 2));
// generate_12bit_sin((uint16_t*)data_buffer1, (32*1024 / 2), (((1 << 16) - 1) / 4), (((1 << 16) - 1) / 2), 1.5, 0);
// const int len = (32*1024 / 2);
// for (int x=0; x<len; x++) {
//     // ((uint16_t*)data_buffer0)[i] = (uint16_t)floor((2*1000 * cos(2 * M_PI * i))) + 2.5;
//     // ((uint16_t*)data_buffer1)[i] = (uint16_t)floor((2*1000 * cos(2 * M_PI * i))) + 2.5;

//     // float y = (2.5 * cos(2 * M_PI * x*(0.01))) + 2.5;
//     // float y2 = (0.5 * cos(3 * M_PI * x*(0.01))) + 2.5;
//     uint16_t y = 1000 * ((2.5 * cos(2 * M_PI * x * 0.0001)) + 2.5);
//     uint16_t y2 = 1000 * ((0.75 * cos(15 * M_PI * x * 0.0001)) + 3);
//     DataArray0[x] = y;
//     DataArray1[x] = y2;
// }



void generate_12bit_sin(uint16_t* buff, size_t len, uint16_t y_range, int16_t v_offset, double freq, int16_t phase);
void generate_sin(uint16_t* buff, size_t len, uint16_t y_range, int16_t v_offset, double freq, int16_t phase);
inline void conv_raw_adc(uint16_t* buff, size_t len, float gain);




// Generates a cos with positive amplitude between 0 and y_range
// Freq is in units of samples. If you have a buffer of length 100 and want it filled with one
// period of a sin wave, you would set the freq to 100
// y_range: 2^n = 1<<n-1
void generate_12bit_sin(uint16_t* buff, size_t len, uint16_t y_range, int16_t v_offset, double freq, int16_t phase) {
    // DEBUG: Validation cosines
    for (int x=0; x < len; x++) {
        // cos wave with a period of 1, amplitude of 1, shifted to a range of 0-1
        // Then phase shifted by the user specified amount, and freq. set
        double wave = cos(((2*M_PI) * freq * (x/len)) + phase);
        uint32_t y = round((y_range * (wave + 1) / 2) + v_offset);
        if (y < 0) buff[x] = 0;
        else if (y > (1 << 12) - 1) buff[x] = (1 << 12) - 1;
        else buff[x] = (uint16_t)y;
        // 16 - 12 = 4 => 4 - 2 = 2
        buff[x] = buff[x] << 2;
    }
}

void generate_sin(uint16_t* buff, size_t len, uint16_t y_range, int16_t v_offset, double freq, int16_t phase) {
    // DEBUG: Validation cosines
    for (int x=0; x < len; x++) {
        // cos wave with a period of 1, amplitude of 1, shifted to a range of 0-1
        // Then phase shifted by the user specified amount, and freq. set
        double wave = cos(((2*M_PI) * freq * (x/len)) + phase);
        uint16_t y = (uint16_t)round((y_range * (wave + 1) / 2) + v_offset);
        buff[x] = y;
    }
}

// Convert raw ADC value to 16-bit value in mV
void conv_raw_adc(uint16_t* buff, size_t len, float gain) {
    // Full scale voltage input to ADC / 12-bit full scale range / gain
    const double convertion_factor = (2.8 / 0x0FFF) / gain;
    for (int i=0; i<len; i++) {
        // Align raw 12-bit data to 16-bit container
        buff[i] = (buff[i] >> 2);
        // Convert from raw value to full scale voltage
        buff[i] = round(buff[i] * convertion_factor);
    }
}