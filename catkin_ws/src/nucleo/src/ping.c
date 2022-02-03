#include "ping.h"

#define DOWNSAMPLING_RATIO 15
#define DOWNSAMPLING_LENGTH 160

const arm_cfft_instance_f32 instance = {
  512, twiddleCoef_512, armBitRevIndexTable512, ARMBITREVINDEXTABLE_512_TABLE_LENGTH
};


float32_t fft(uint32_t* buff, uint32_t size, uint32_t target, float32_t fs)
{
    float32_t temp_buff[size];
    for (int i = 0; i < size; i++)
    {
        temp_buff[i] = (float32_t) buff[i];
    }

    arm_cfft_f32(&instance, temp_buff, 0, 0);

    float32_t freq[size / 2];
    arm_cmplx_mag_f32(temp_buff, freq, size / 2);

    uint32_t target_bin = (uint32_t) (target * size / fs);

    float32_t sum = 0;
    for (int i = 0; i < size / 2; i++)
    {
        sum += freq[i];
    }
    return freq[target_bin] / sum;
}


uint32_t get_frequency(uint32_t* buff, uint32_t size, float32_t fs)
{
    uint32_t target_frequencies[] = {25000, 30000, 35000, 40000};
    float32_t temp_buff[size];

    for (int i = 0; i < size; i++)
    {
        temp_buff[i] = (float32_t) buff[i];
    }

    arm_cfft_f32(&instance, temp_buff, 0, 0);

    float32_t freq[size / 2];
    arm_cmplx_mag_f32(temp_buff, freq, size / 2);

    uint32_t max = 0;
    uint32_t frequency = 0;
    uint32_t target_bin = 0;

    for (int i = 0; i < 4; i++) {
        target_bin = (uint32_t) round(target_frequencies[i] * size / fs);
        if (freq[target_bin] > max) {
            frequency = target_frequencies[i];
            max = freq[target_bin];
        }
    }

    return frequency;
}

uint32_t get_total_power(uint32_t* buff, uint32_t size)
{
    uint32_t power = 0;
    for (uint32_t i = 0; i < size; i += 2)
    {
        int16_t current = ((int16_t) buff[i] - DC_OFFSET);
        power += abs(current);
    }

    return power;
}


uint32_t get_power_at_target_frequency(uint32_t* buff, uint32_t size)
{
    int32_t power = 0;
    for (uint32_t i = 0; i < size; i += 2 * DOWNSAMPLING_RATIO)
    {
        int32_t to_add = 0;
        for (uint32_t j = i; j < DOWNSAMPLING_RATIO; j++)
        {
            to_add += ((int16_t) buff[j] - DC_OFFSET);
        }

        int32_t to_subtract = 0;
        for (uint32_t j = i + DOWNSAMPLING_RATIO; j < DOWNSAMPLING_RATIO; j++)
        {
            to_subtract -= ((int16_t) buff[j] - DC_OFFSET);
        }
        power += to_add - to_subtract;
    }

    return power * power;
}


uint8_t has_ping(uint32_t* buff, uint32_t size, uint32_t threshold)
{
    // Get total power.

    // Verify if power at start of signal is greater than the threshold.
    // uint32_t total_start_power = get_total_power(buff, DOWNSAMPLING_LENGTH);
    // uint32_t start_power = get_power_at_target_frequency(
    //     buff,
    //     DOWNSAMPLING_LENGTH) / total_start_power;
    // if (start_power < threshold)
    // {
    //     // No ping.
    //     return 0;
    // }

    // // Verify if power at end of signal is greater than the threshold.
    // uint32_t total_end_power = get_total_power(
    //     buff + size - DOWNSAMPLING_LENGTH,
    //     DOWNSAMPLING_LENGTH);
    // uint32_t end_power = get_power_at_target_frequency(
    //     buff + size - DOWNSAMPLING_LENGTH,
    //     DOWNSAMPLING_LENGTH) / total_end_power;
    // if (end_power < threshold)
    // {
    //     // No ping.
    //     return 0;
    // }

    // // Otherwise, ping.
    // char details[40];
    // sprintf(
    //     details, "Found ping: start: %u, end: %u",
    //     start_power, end_power);
    // log_debug(details);
    // return 1;

    float32_t amp = fft(buff, size, 30000, 972972.97297);

    if (amp > (float32_t) threshold * 1.5 / 1000.0)
    {
        char details[30];
        sprintf(details, "Energy at %f", amp);
        log_debug(details);
        return 1;
    }
    return 0;
}
