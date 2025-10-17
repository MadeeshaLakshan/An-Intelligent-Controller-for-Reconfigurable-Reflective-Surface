/*
 * "Hello World" example.
 *
 * This example prints 'Hello from Nios II' to the STDOUT stream. It runs on
 * the Nios II 'standard', 'full_featured', 'fast', and 'low_cost' example
 * designs. It runs with or without the MicroC/OS-II RTOS and requires a STDOUT
 * device in your system's hardware.
 * The memory footprint of this hosted application is ~69 kbytes by default
 * using the standard reference design.
 *
 * For a reduced footprint version of this template, and an explanation of how
 * to reduce the memory footprint for a given application, see the
 * "small_hello_world" template.
 *
 */
#include "system.h"
#include "io.h"
#include <math.h>

// ==== Configuration ====
#define CHANNELS        9
#define MAX_COUNT       65535  // 16-bit resolution
#define PI              3.14159265358979323846

// ==== System Parameters ====
#define FREQUENCY       2.4e9      // 2.4 GHz
#define SPEED_OF_LIGHT  3.0e8      // m/s
#define WAVELENGTH      (SPEED_OF_LIGHT / FREQUENCY)

// ==== IRS Configuration ====
#define NR              1
#define NC              9
#define UNIT_CELL_SIZE  0.026      // meters

// ==== Positions ====
#define XT              0.9        // Transmitter X
#define YT              0.9        // Transmitter Y
#define XR              0.5        // Receiver X
#define YR              0.9        // Receiver Y

// ==== PWM Parameters ====
#define GAIN            6.08   // 1 + 18/(3.3 + 0.24)
#define MAX_VOLTAGE     3.3        // Full-scale voltage

// ==== Lookup Table for Phase-to-Voltage ====
// From estimate_voltages.m
const float voltages_lut[] = {
    0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 8.1, 8.2, 8.5, 8.7, 9.0, 9.2,
    9.5, 9.7, 9.8, 10.0, 10.2, 10.5, 10.7, 11.0, 11.2, 11.5, 11.7, 12.0, 12.2,
    12.5, 12.7, 13.0, 13.2, 13.5, 13.7, 14.0, 14.2, 14.5, 14.7, 15.0, 15.2,
    15.5, 15.7, 16.0, 16.2, 16.5, 16.7, 17.0, 17.2, 17.5, 17.7, 18.0, 18.2,
    18.5, 18.7
};

const float phases_deg_lut[] = {
    -164.495, -162.145, -161.325, -158.655, -156.975, -154.065, -150.245,
    -142.805, -125.765, -124.565, -120.565, -109.715, -101.205, -79.685,
    -57.695, -3.545, 45.845, 56.615, 84.665, 98.195, 110.655, 118.285,
    128.635, 131.875, 137.355, 138.935, 144.305, 146.465, 148.915, 149.915,
    151.905, 152.625, 154.125, 154.745, 156.195, 156.485, 157.605, 158.015,
    158.845, 159.525, 159.735, 161.055, 161.245, 162.065, 162.395, 162.655,
    163.345, 163.515, 164.025, 164.275, 164.105, 164.325, 164.375, 164.495
};

#define LUT_SIZE 54

// ==== Function Prototypes ====
void compute_phase_shifts(float *phase_shifts);
float estimate_voltage(float phase_deg);
float linear_interpolate(float x, const float *x_data, const float *y_data, int n);
unsigned short voltage_to_pwm(float voltage);
void configure_pwm_duty_cycles(unsigned short *duty_cycles);
unsigned short get_pwm_channel(unsigned int channel);
void print_debug_info(float *phase_shifts, float *voltages, unsigned short *duty_cycles);

// ==== 1. Distance & Phase Shift Calculation ====
void compute_phase_shifts(float *phase_shifts) {
    int i;
    float x_irs, y_irs;
    float dt_irs, dirs_r, total_phase;

    for (i = 0; i < NC; i++) {
        // IRS element position
        x_irs = i * UNIT_CELL_SIZE;
        y_irs = 0.0;

        // Distance from transmitter to IRS element
        dt_irs = sqrtf((x_irs - XT) * (x_irs - XT) + (y_irs - YT) * (y_irs - YT));

        // Distance from IRS element to receiver
        dirs_r = sqrtf((XR - x_irs) * (XR - x_irs) + (YR - y_irs) * (YR - y_irs));

        // Total phase
        total_phase = 2.0 * PI * (dt_irs + dirs_r) / WAVELENGTH;

        // Wrap to [-pi, pi]
        phase_shifts[i] = fmodf(total_phase + PI, 2.0 * PI) - PI;
    }
}

// ==== 2. Linear Interpolation Helper ====
float linear_interpolate(float x, const float *x_data, const float *y_data, int n) {
    int i;

    // Handle out of bounds - clamp to min/max
    if (x <= x_data[0]) return y_data[0];
    if (x >= x_data[n-1]) return y_data[n-1];

    // Find interval
    for (i = 0; i < n - 1; i++) {
        if (x >= x_data[i] && x <= x_data[i+1]) {
            // Linear interpolation
            float t = (x - x_data[i]) / (x_data[i+1] - x_data[i]);
            return y_data[i] + t * (y_data[i+1] - y_data[i]);
        }
    }

    return y_data[n-1]; // fallback
}

// ==== 3. Voltage Estimation ====
float estimate_voltage(float phase_deg) {
    float voltage = linear_interpolate(phase_deg, phases_deg_lut, voltages_lut, LUT_SIZE);

    // Clamp to valid range
    if (voltage < 0.0) voltage = 0.0;
    if (voltage > 18.7) voltage = 18.7;

    return voltage;
}

// ==== 4. Voltage to PWM Conversion ====
unsigned short voltage_to_pwm(float voltage) {
    float scaled_voltage, normalized_voltage;
    unsigned int pwm_count;

    // Apply gain correction
    scaled_voltage = voltage / GAIN;

    // Normalize to [0, 1]
    normalized_voltage = scaled_voltage / MAX_VOLTAGE;

    // Clamp to valid range
    if (normalized_voltage < 0.0) normalized_voltage = 0.0;
    if (normalized_voltage > 1.0) normalized_voltage = 1.0;

    // Convert to 16-bit PWM count
    pwm_count = (unsigned int)(normalized_voltage * MAX_COUNT);

    return (unsigned short)pwm_count;
}

void configure_pwm_duty_cycles(unsigned short *duty_cycles) {
    int i;

    // Write duty cycle values to PWM IP registers
    for (i = 0; i < CHANNELS; i++) {
        IOWR(PWM_IP_0_BASE, i, duty_cycles[i]);
    }
}


// ==== 6. Debug Print Function (integer-scaled, no %f) ====
void print_debug_info(float *phase_shifts, float *voltages, unsigned short *duty_cycles) {
    int i;
    printf("\n============================================================\n");
    printf("        IRS Phase Shift & PWM Control Debug Info\n");
    printf("============================================================\n\n");

    // ---- System Parameters ----
    {
        float freq_ghz = FREQUENCY / 1e9;
        printf("  Frequency       : %d.%02d GHz\n",
               (int)freq_ghz, (int)((freq_ghz - (int)freq_ghz) * 100));
    }
    {
        printf("  Wavelength      : %d.%06d m\n",
               (int)WAVELENGTH, (int)((WAVELENGTH - (int)WAVELENGTH) * 1000000));
    }
    printf("  Unit Cell Size  : %d.%03d m\n",
           (int)UNIT_CELL_SIZE, (int)((UNIT_CELL_SIZE - (int)UNIT_CELL_SIZE) * 1000));
    printf("  Gain            : %d.%04d\n",
           (int)GAIN, (int)((GAIN - (int)GAIN) * 10000));
    printf("  Max Voltage     : %d.%01d V\n\n",
           (int)MAX_VOLTAGE, (int)((MAX_VOLTAGE - (int)MAX_VOLTAGE) * 10));

    printf("Positions:\n");
    printf("  Transmitter     : (%d.%02d, %d.%02d) m\n",
           (int)XT, (int)((XT - (int)XT) * 100),
           (int)YT, (int)((YT - (int)YT) * 100));
    printf("  Receiver        : (%d.%02d, %d.%02d) m\n",
           (int)XR, (int)((XR - (int)XR) * 100),
           (int)YR, (int)((YR - (int)YR) * 100));
    printf("  IRS Elements    : %d (along X-axis)\n\n", NC);

    // ---- IRS Element Table ----
    printf("============================================================\n");
    printf(" IRS Element Calculations\n");
    printf("============================================================\n");
    printf("Elem | X Pos  | Phase(rad) | Phase(deg) | Voltage | Duty(%%)\n");
    printf("-----|--------|------------|------------|---------|--------\n");

    for (i = 0; i < NC; i++) {
        float x_pos = i * UNIT_CELL_SIZE;
        float phase_deg = phase_shifts[i] * 180.0f / PI;

        // Duty cycle percentage calculation
        float duty_percent = ((float)duty_cycles[i] * 100.0f) / (float)MAX_COUNT;
        int duty_int = (int)duty_percent;
        int duty_frac = (int)((duty_percent - duty_int) * 100);

        printf(" %2d  | %d.%04d | %+d.%04d | %+d.%02d | %d.%02d  | %2d.%02d%%\n",
               i + 1,
               (int)x_pos, (int)((x_pos - (int)x_pos) * 10000),
               (int)phase_shifts[i], (int)((phase_shifts[i] - (int)phase_shifts[i]) * 10000),
               (int)phase_deg, (int)((phase_deg - (int)phase_deg) * 100),
               (int)voltages[i], (int)((voltages[i] - (int)voltages[i]) * 100),
               duty_int, duty_frac);
    }

    printf("============================================================\n\n");
    printf("Starting PWM signal generation...\n");
    printf("============================================================\n\n");
}
unsigned short get_pwm_channel(unsigned int channel) {
    if (channel < CHANNELS) {
        return (unsigned short)IORD(PWM_IP_0_BASE, channel);
    }
    return 0;
}

// ==== Main Function ====
int main() {
    float phase_shifts[NC];
    float voltages[NC];
    unsigned short duty_cycles[NC];
    int i;

    // Step 1: Compute phase shifts for all IRS elements
    compute_phase_shifts(phase_shifts);

    // Step 2 & 3: Convert phase shifts to voltages
    for (i = 0; i < NC; i++) {
        // Convert radians to degrees
        float phase_deg = phase_shifts[i] * 180.0 / PI;
        voltages[i] = estimate_voltage(phase_deg);
    }

    // Step 4: Convert voltages to PWM duty cycles
    for (i = 0; i < NC; i++) {
        duty_cycles[i] = voltage_to_pwm(voltages[i]);
    }

    // Step 5: Generate PWM signals continuously
    configure_pwm_duty_cycles(duty_cycles);

    print_debug_info(phase_shifts, voltages,duty_cycles);
    int j;
    for (j = 0; j < CHANNELS; j++) {
        unsigned short value = get_pwm_channel(i);
        printf("Channel %d: 0x%04X (%d)\n", j, value, value);
    }

    return 0;
}

