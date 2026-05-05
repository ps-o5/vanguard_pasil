#include <nuttx/config.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/i2c/i2c_master.h> 

#include "pasil_ekf.h" 
#include "vl53l0x_api.h"

#define LOOP_PERIOD_NS 2000000 /* 500Hz */
#define SEC_TO_NS      1000000000
#define DT_SEC         0.002f

/* Scale Factors: Adjust these based on your specific IMU resolution (e.g., MPU6050) */
#define GYRO_TO_RAD    (M_PI / 180.0f) /* Assuming raw was deg/s. Change if raw ADC */
#define ACCEL_TO_G     1.0f            /* Assuming raw was Gs. Change if raw ADC */

/* --- CASCADED PID GAINS --- */
#define KP_ANG_PR      4.0f   /* Pitch/Roll Angle P-Gain */
#define KP_ANG_YAW     2.0f   /* Yaw Angle P-Gain */

#define KP_RATE_PR     15.0f  
#define KI_RATE_PR     1.0f
#define KD_RATE_PR     0.5f

#define KP_RATE_YAW    10.0f 
#define KI_RATE_YAW    1.0f
#define KD_RATE_YAW    0.0f

#define PID_I_LIMIT    300.0f

/* --- PWM & FAILSAFE CONFIG --- */
#define PWM_NEUTRAL    4915
#define PWM_MIN        3276
#define PWM_MAX        6553
#define FAILSAFE_TIMEOUT_CYCLES 100 /* 200ms at 500Hz */

#define QMC5883P_ADDR  0x2C
#define BMP280_ADDR    0x76

#define CLAMP_PWM(val) ((uint16_t)((val) > PWM_MAX ? PWM_MAX : ((val) < PWM_MIN ? PWM_MIN : (val))))

/* =====================================================================
 * TIMING PROFILER
 * ===================================================================== */
#define ENABLE_LOOP_PROFILING 1  /* Set to 0 to completely compile out */

#if ENABLE_LOOP_PROFILING
    #include "stm32_gpio.h"
    #define GPIO_PROFILER_PA8 (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                               GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN8)
    #define PROFILER_INIT()  stm32_configgpio(GPIO_PROFILER_PA8)
    #define PROFILER_START() stm32_gpiowrite(GPIO_PROFILER_PA8, true)
    #define PROFILER_STOP()  stm32_gpiowrite(GPIO_PROFILER_PA8, false)
#else
    #define PROFILER_INIT()
    #define PROFILER_START()
    #define PROFILER_STOP()
#endif

typedef struct __attribute__((packed)) {
    uint8_t  magic;       
    uint8_t  state;       
    int16_t  throttle;    
    int16_t  pitch;       
    int16_t  roll;        
    int16_t  yaw;         
    uint16_t checksum;    
} rc_payload_t;

/* =====================================================================
 * GLOBALS & SHARED THREAD STATE
 * ===================================================================== */
int pasil_fd_i2c = -1;

/* BMP280 Calibration State */
static uint16_t dig_T1, dig_P1;
static int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t  t_fine;

/* VL53L0X Laser State */
static VL53L0X_Dev_t tof_dev_struct;
static VL53L0X_DEV tof_dev = &tof_dev_struct;

/* Shared state between 500Hz Fast Loop and 50Hz Background Loop */
volatile int16_t  shared_mag_x = 0, shared_mag_y = 0, shared_mag_z = 0;
volatile bool     shared_mag_updated = false;
volatile float    shared_true_pressure_pa = 0.0f;
volatile uint16_t shared_laser_dist_mm = 0;

typedef struct {
    bool is_armed;
    bool is_failsafe;
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
    float pressure;
    uint16_t laser;
    int m1, m2, m3, m4;
} telemetry_snapshot_t;

volatile telemetry_snapshot_t current_telem;

/* =====================================================================
 * HELPER FUNCTIONS
 * ===================================================================== */
static void timespec_add_ns(struct timespec *ts, long ns) {
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= SEC_TO_NS) { ts->tv_sec++; ts->tv_nsec -= SEC_TO_NS; }
}

static void i2c_write_reg(int fd, uint8_t addr, uint8_t reg, uint8_t value) {
    struct i2c_msg_s msg; struct i2c_transfer_s xfer;
    uint8_t tx_data[2] = {reg, value};
    msg.frequency = 400000; msg.addr = addr; msg.flags = 0;    
    msg.buffer = tx_data; msg.length = 2; xfer.msgv = &msg; xfer.msgc = 1;
    ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&xfer);
}

static void bmp280_extract_cal(int fd) {
    struct i2c_msg_s msg[2]; 
    struct i2c_transfer_s xfer;
    uint8_t reg = 0x88; 
    uint8_t cal[24];
    
    msg[0].frequency = 400000; msg[0].addr = BMP280_ADDR; msg[0].flags = 0; msg[0].buffer = &reg; msg[0].length = 1;
    msg[1].frequency = 400000; msg[1].addr = BMP280_ADDR; msg[1].flags = I2C_M_READ; msg[1].buffer = cal; msg[1].length = 24;
    xfer.msgv = msg; 
    xfer.msgc = 2;
    
    if(ioctl(fd, I2CIOC_TRANSFER, (unsigned long)&xfer) >= 0) {
        dig_T1 = (cal[1] << 8) | cal[0];  dig_T2 = (cal[3] << 8) | cal[2];  dig_T3 = (cal[5] << 8) | cal[4];
        dig_P1 = (cal[7] << 8) | cal[6];  dig_P2 = (cal[9] << 8) | cal[8];  dig_P3 = (cal[11] << 8) | cal[10];
        dig_P4 = (cal[13] << 8) | cal[12]; dig_P5 = (cal[15] << 8) | cal[14]; dig_P6 = (cal[17] << 8) | cal[16];
        dig_P7 = (cal[19] << 8) | cal[18]; dig_P8 = (cal[21] << 8) | cal[20]; dig_P9 = (cal[23] << 8) | cal[22];
    }
}

static float bmp280_calculate_pressure(int32_t adc_T, int32_t adc_P) {
    int32_t var1, var2; int64_t p_var1, p_var2, p;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    p_var1 = ((int64_t)t_fine) - 128000;
    p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) + ((p_var1 * (int64_t)dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dig_P1) >> 33;
    if (p_var1 == 0) return 0.0f; 
    p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (float)p / 256.0f;
}

/* =====================================================================
 * BACKGROUND SENSOR & TELEMETRY THREAD (50 Hz)
 * ===================================================================== */
static void *pasil_background_thread(void *arg) {
    struct timespec wakeup_time;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    
    /* 50 Hz loop period (20,000,000 ns) */
    const long SLOW_LOOP_PERIOD_NS = 20000000; 
    uint32_t slow_cycle_count = 0;

    while (1) {
        slow_cycle_count++;
        timespec_add_ns(&wakeup_time, SLOW_LOOP_PERIOD_NS);
        
        /* Fixed sleep bug: loops back if awoken early by signals */
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL) == EINTR);

        /* 1. 50Hz: MAGNETOMETER READ */
        if (pasil_fd_i2c >= 0) {
            struct i2c_msg_s msg[2]; struct i2c_transfer_s xfer; 
            uint8_t reg_data = 0x01; uint8_t rx_data[6] = {0};
            msg[0].frequency = 400000; msg[0].addr = QMC5883P_ADDR; msg[0].flags = 0; msg[0].buffer = &reg_data; msg[0].length = 1;
            msg[1].frequency = 400000; msg[1].addr = QMC5883P_ADDR; msg[1].flags = I2C_M_READ; msg[1].buffer = rx_data; msg[1].length = 6;
            xfer.msgv = msg; xfer.msgc = 2;
            if (ioctl(pasil_fd_i2c, I2CIOC_TRANSFER, (unsigned long)&xfer) >= 0) {
                shared_mag_x = (int16_t)(rx_data[0] | (rx_data[1] << 8));
                shared_mag_y = (int16_t)(rx_data[2] | (rx_data[3] << 8));
                shared_mag_z = (int16_t)(rx_data[4] | (rx_data[5] << 8));
                shared_mag_updated = true;
            }
        }

        /* 2. 10Hz: BAROMETER TRIGGER (Offset 0) */
        if (pasil_fd_i2c >= 0 && (slow_cycle_count % 5 == 0)) {
            i2c_write_reg(pasil_fd_i2c, BMP280_ADDR, 0xF4, 0x55);
        }

        /* 3. 10Hz: BAROMETER READ (Offset 2 - 40ms later) */
        if (pasil_fd_i2c >= 0 && (slow_cycle_count % 5 == 2)) {
            struct i2c_msg_s msg[2]; struct i2c_transfer_s xfer;
            uint8_t reg_press = 0xF7; uint8_t rx_data[6] = {0};
            msg[0].frequency = 400000; msg[0].addr = BMP280_ADDR; msg[0].flags = 0; msg[0].buffer = &reg_press; msg[0].length = 1;
            msg[1].frequency = 400000; msg[1].addr = BMP280_ADDR; msg[1].flags = I2C_M_READ; msg[1].buffer = rx_data; msg[1].length = 6;
            xfer.msgv = msg; xfer.msgc = 2;
            if (ioctl(pasil_fd_i2c, I2CIOC_TRANSFER, (unsigned long)&xfer) >= 0) {
                int32_t adc_P = (rx_data[0] << 12) | (rx_data[1] << 4) | (rx_data[2] >> 4);
                int32_t adc_T = (rx_data[3] << 12) | (rx_data[4] << 4) | (rx_data[5] >> 4);
                shared_true_pressure_pa = bmp280_calculate_pressure(adc_T, adc_P);
            }
        }

        /* 4. 10Hz: LASER TELEMETRY READ */
        if (pasil_fd_i2c >= 0 && (slow_cycle_count % 5 == 0)) {
            uint8_t dataReady = 0;
            VL53L0X_GetMeasurementDataReady(tof_dev, &dataReady);
            if(dataReady == 1) {
                VL53L0X_RangingMeasurementData_t r_data;
                VL53L0X_GetRangingMeasurementData(tof_dev, &r_data);
                shared_laser_dist_mm = r_data.RangeMilliMeter;
                VL53L0X_ClearInterruptMask(tof_dev, 0); 
            }
            
            /* 5. TELEMETRY PRINT */
            telemetry_snapshot_t snap = current_telem;
            printf("STATE:%s | ATT:[P:%5.1f R:%5.1f Y:%5.1f] | BARO:%8.1f | L:%4d | M1:%4d M2:%4d M3:%4d M4:%4d\n", 
                   snap.is_armed ? (snap.is_failsafe ? "FAILSAFE" : "ARMED") : "STANDBY",
                   snap.pitch_deg, snap.roll_deg, snap.yaw_deg,
                   snap.pressure, snap.laser,
                   snap.m1, snap.m2, snap.m3, snap.m4);
        }
    }
    return NULL;
}

/* =====================================================================
 * MAIN FAST CONTROL TASK (500 Hz)
 * ===================================================================== */
int pasil_imu_task(int argc, char *argv[]) {
    struct timespec wakeup_time;
    int fd_imu, fd_pwm0, fd_pwm1, fd_rf;
    struct pwm_info_s pwm0_info;
    struct pwm_info_s pwm1_info;
    uint32_t cycle_count = 0;
    
    /* Inner Loop PID States */
    float int_rate_p = 0.0f, last_err_rate_p = 0.0f;
    float int_rate_r = 0.0f, last_err_rate_r = 0.0f;
    float int_rate_y = 0.0f, last_err_rate_y = 0.0f;

    /* Local Fast-Loop Copies */
    uint16_t laser_dist_mm = 0;
    float true_pressure_pa = 0.0f;
    float current_gyro_x = 0.0f, current_gyro_y = 0.0f, current_gyro_z = 0.0f;

    /* Flight States */
    bool is_armed = false;
    bool is_failsafe = false;
    uint32_t arming_counter = 0;
    int32_t failsafe_hover_throttle = PWM_MIN;

    tof_dev->I2cDevAddr = 0x29; 

    printf("[PASIL-MASTER] Arming ESKF Production Logic...\n");
    eskf_init();

    fd_imu = open("/dev/imu0", O_RDONLY);
    fd_pwm0 = open("/dev/pwm0", O_RDWR);
    fd_pwm1 = open("/dev/pwm1", O_RDWR);
    fd_rf = open("/dev/nrf24l01", O_RDONLY | O_NONBLOCK);
    pasil_fd_i2c = open("/dev/i2c1", O_RDWR);

    pwm0_info.frequency = 50; 
    pwm0_info.channels[0].channel = 2; pwm0_info.channels[0].duty = PWM_MIN; 
    pwm0_info.channels[1].channel = 3; pwm0_info.channels[1].duty = PWM_MIN; 
    pwm0_info.channels[2].channel = 4; pwm0_info.channels[2].duty = PWM_MIN; 
    
    pwm1_info.frequency = 50; 
    pwm1_info.channels[0].channel = 2; pwm1_info.channels[0].duty = PWM_MIN; 

    ioctl(fd_pwm0, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm0_info));
    ioctl(fd_pwm0, PWMIOC_START, 0);
    ioctl(fd_pwm1, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm1_info));
    ioctl(fd_pwm1, PWMIOC_START, 0);

    /* ---------------------------------------------------------
     * HARDWARE SENSOR BOOT SEQUENCE
     * --------------------------------------------------------- */
    if (pasil_fd_i2c >= 0) {
        printf("[PASIL-MASTER] Waking Barometer & Magnetometer...\n");
        bmp280_extract_cal(pasil_fd_i2c); 
        i2c_write_reg(pasil_fd_i2c, BMP280_ADDR, 0xF4, 0x55);
        
        i2c_write_reg(pasil_fd_i2c, QMC5883P_ADDR, 0x0B, 0x01); usleep(50000); 
        i2c_write_reg(pasil_fd_i2c, QMC5883P_ADDR, 0x20, 0x40);
        i2c_write_reg(pasil_fd_i2c, QMC5883P_ADDR, 0x21, 0x01);
        i2c_write_reg(pasil_fd_i2c, QMC5883P_ADDR, 0x0A, 0x1D); usleep(50000);

        printf("[PASIL-MASTER] Uploading ST Firmware to Laser...\n");
        VL53L0X_DataInit(tof_dev);
        VL53L0X_StaticInit(tof_dev);
        uint32_t refSpadCount; uint8_t isApertureSpads;
        VL53L0X_PerformRefSpadManagement(tof_dev, &refSpadCount, &isApertureSpads);
        VL53L0X_PerformRefCalibration(tof_dev, NULL, NULL);
        VL53L0X_SetDeviceMode(tof_dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
        VL53L0X_StartMeasurement(tof_dev);
    }

    /* Spawn Background Thread for Slow Operations */
    pthread_t bg_thread;
    pthread_create(&bg_thread, NULL, pasil_background_thread, NULL);

    PROFILER_INIT();    //500Hz Analysis Init
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    for(int i = 0; i < 250000; i++) {
        cycle_count++;
        timespec_add_ns(&wakeup_time, LOOP_PERIOD_NS);
        
        /* Fixed sleep bug: handle EINTR properly */
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL) == EINTR);
        
        PROFILER_START();   //Profiler Starts at PA8
        
        /* -----------------------------------------
         * 1. ESKF SENSOR FUSION INGESTION
         * ----------------------------------------- */
        int16_t raw_mem[16] = {0}; 
        if (read(fd_imu, raw_mem, sizeof(raw_mem)) >= 12) {
            float gx = (float)raw_mem[0] * GYRO_TO_RAD;
            float gy = (float)raw_mem[1] * GYRO_TO_RAD;
            float gz = (float)raw_mem[2] * GYRO_TO_RAD;
            float ax = (float)raw_mem[3] * ACCEL_TO_G;
            float ay = (float)raw_mem[4] * ACCEL_TO_G;
            float az = (float)raw_mem[5] * ACCEL_TO_G;
            
            current_gyro_x = gx; current_gyro_y = gy; current_gyro_z = gz;

            eskf_predict(gx, gy, gz, DT_SEC); 
            eskf_update_accel(ax, ay, az);
        }

        /* Pull Non-Blocking Data from Background Thread */
        if (shared_mag_updated) {
            eskf_update_mag((float)shared_mag_x, (float)shared_mag_y, (float)shared_mag_z);
            shared_mag_updated = false;
        }
        true_pressure_pa = shared_true_pressure_pa;
        laser_dist_mm = shared_laser_dist_mm;

        /* -----------------------------------------
         * 2. RF TELEMETRY & FAILSAFE STATE MACHINE
         * ----------------------------------------- */
        static rc_payload_t rx_data;
        static uint32_t last_packet_time = 0;
        
        if (fd_rf >= 0 && read(fd_rf, &rx_data, sizeof(rc_payload_t)) == sizeof(rc_payload_t)) {
            if (rx_data.magic == 0x5A) last_packet_time = cycle_count;
        }

        if ((cycle_count - last_packet_time) > FAILSAFE_TIMEOUT_CYCLES) {
            if (is_armed && !is_failsafe) {
                is_failsafe = true;
                failsafe_hover_throttle = PWM_MIN + ((rx_data.throttle * (PWM_MAX - PWM_MIN)) / 1000);
                if (failsafe_hover_throttle > PWM_MAX - 500) failsafe_hover_throttle = PWM_NEUTRAL; 
            }
        } else {
            is_failsafe = false;
        }

        if (!is_failsafe && rx_data.throttle < 50) {
            if (!is_armed && rx_data.yaw > 900) {      
                arming_counter++;
                if (arming_counter > 1000) { is_armed = true; arming_counter = 0; }
            } else if (is_armed && rx_data.yaw < -900) { 
                arming_counter++;
                if (arming_counter > 1000) { is_armed = false; arming_counter = 0; }
            } else {
                arming_counter = 0;
            }
        } else {
            arming_counter = 0;
        }

        /* -----------------------------------------
         * 3. CASCADED PID CONTROLLER
         * ----------------------------------------- */
        euler_angles_t current_attitude = eskf_get_euler();
        eskf_state_t eskf_state = eskf_get_state();
        
        int32_t throttle_base = PWM_MIN; 
        float target_pitch_rad = 0.0f, target_roll_rad = 0.0f, target_yaw_rate_rads = 0.0f;

        if (is_failsafe && is_armed) {
            target_pitch_rad = 0.0f; target_roll_rad = 0.0f; target_yaw_rate_rads = 0.0f; 
            if (cycle_count % 50 == 0) { 
                if (laser_dist_mm > 100) failsafe_hover_throttle -= 5;
                else is_armed = false; 
            }
            throttle_base = failsafe_hover_throttle;
        } 
        else if (is_armed) {
            throttle_base = PWM_MIN + ((rx_data.throttle * (PWM_MAX - PWM_MIN)) / 1000);
            target_pitch_rad = (float)rx_data.pitch * (30.0f / 1000.0f) * (M_PI / 180.0f); 
            target_roll_rad  = (float)rx_data.roll  * (30.0f / 1000.0f) * (M_PI / 180.0f); 
            target_yaw_rate_rads = (float)rx_data.yaw   * (45.0f / 1000.0f) * (M_PI / 180.0f); 
        }

        float target_rate_p = KP_ANG_PR * (target_pitch_rad - current_attitude.pitch_rad);
        float target_rate_r = KP_ANG_PR * (target_roll_rad - current_attitude.roll_rad);
        float target_rate_y = target_yaw_rate_rads; 

        float true_rate_p = current_gyro_y - eskf_state.bg_y;
        float true_rate_r = current_gyro_x - eskf_state.bg_x;
        float true_rate_y = current_gyro_z - eskf_state.bg_z;

        if (!is_armed || throttle_base <= (PWM_MIN + 50)) {
            int_rate_p = 0.0f; int_rate_r = 0.0f; int_rate_y = 0.0f; 
        }

        float err_rate_p = target_rate_p - true_rate_p; 
        if (is_armed) int_rate_p += err_rate_p * DT_SEC; 
        if (int_rate_p > PID_I_LIMIT) int_rate_p = PID_I_LIMIT; else if (int_rate_p < -PID_I_LIMIT) int_rate_p = -PID_I_LIMIT;
        int32_t p_out = (int32_t)((KP_RATE_PR * err_rate_p) + (KI_RATE_PR * int_rate_p) + (KD_RATE_PR * ((err_rate_p - last_err_rate_p) / DT_SEC)));
        last_err_rate_p = err_rate_p;

        float err_rate_r = target_rate_r - true_rate_r; 
        if (is_armed) int_rate_r += err_rate_r * DT_SEC; 
        if (int_rate_r > PID_I_LIMIT) int_rate_r = PID_I_LIMIT; else if (int_rate_r < -PID_I_LIMIT) int_rate_r = -PID_I_LIMIT;
        int32_t r_out = (int32_t)((KP_RATE_PR * err_rate_r) + (KI_RATE_PR * int_rate_r) + (KD_RATE_PR * ((err_rate_r - last_err_rate_r) / DT_SEC)));
        last_err_rate_r = err_rate_r;

        float err_rate_y = target_rate_y - true_rate_y; 
        if (is_armed) int_rate_y += err_rate_y * DT_SEC; 
        if (int_rate_y > PID_I_LIMIT) int_rate_y = PID_I_LIMIT; else if (int_rate_y < -PID_I_LIMIT) int_rate_y = -PID_I_LIMIT;
        int32_t y_out = (int32_t)((KP_RATE_YAW * err_rate_y) + (KI_RATE_YAW * int_rate_y) + (KD_RATE_YAW * ((err_rate_y - last_err_rate_y) / DT_SEC)));
        last_err_rate_y = err_rate_y;

        /* -----------------------------------------
         * 4. QUAD-X MIXER & ESC OUTPUT
         * ----------------------------------------- */
        if (is_armed) {
            pwm0_info.channels[0].duty = CLAMP_PWM(throttle_base - p_out - r_out + y_out); 
            pwm0_info.channels[1].duty = CLAMP_PWM(throttle_base - p_out + r_out - y_out); 
            pwm0_info.channels[2].duty = CLAMP_PWM(throttle_base + p_out - r_out - y_out); 
            pwm1_info.channels[0].duty = CLAMP_PWM(throttle_base + p_out + r_out + y_out); 
        } else {
            pwm0_info.channels[0].duty = PWM_MIN; pwm0_info.channels[1].duty = PWM_MIN; 
            pwm0_info.channels[2].duty = PWM_MIN; pwm1_info.channels[0].duty = PWM_MIN; 
        }

        ioctl(fd_pwm0, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm0_info));
        ioctl(fd_pwm1, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm1_info));

        /* Update Telemetry Snapshot */
        if (cycle_count % 50 == 0) { 
            current_telem.is_armed = is_armed;
            current_telem.is_failsafe = is_failsafe;
            current_telem.pitch_deg = current_attitude.pitch_rad * (180.0f/M_PI);
            current_telem.roll_deg = current_attitude.roll_rad * (180.0f/M_PI);
            current_telem.yaw_deg = current_attitude.yaw_rad * (180.0f/M_PI);
            current_telem.pressure = true_pressure_pa;
            current_telem.laser = laser_dist_mm;
            current_telem.m1 = (int)pwm0_info.channels[0].duty;
            current_telem.m2 = (int)pwm0_info.channels[1].duty;
            current_telem.m3 = (int)pwm0_info.channels[2].duty;
            current_telem.m4 = (int)pwm1_info.channels[0].duty;
        }

        PROFILER_STOP();    //Profiler Stop
    }

    /* System Disarm */
    pwm0_info.channels[0].duty = PWM_MIN; pwm0_info.channels[1].duty = PWM_MIN; pwm0_info.channels[2].duty = PWM_MIN;
    pwm1_info.channels[0].duty = PWM_MIN;
    ioctl(fd_pwm0, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm0_info));
    ioctl(fd_pwm0, PWMIOC_STOP, 0);
    ioctl(fd_pwm1, PWMIOC_SETCHARACTERISTICS, (unsigned long)((uintptr_t)&pwm1_info));
    ioctl(fd_pwm1, PWMIOC_STOP, 0);
    
    if (fd_pwm0 >= 0) close(fd_pwm0);
    if (fd_pwm1 >= 0) close(fd_pwm1);
    if (pasil_fd_i2c >= 0) close(pasil_fd_i2c);
    if (fd_rf >= 0) close(fd_rf);
    if (fd_imu >= 0) close(fd_imu);
    
    /* Cancel background thread on exit */
    pthread_cancel(bg_thread);
    pthread_join(bg_thread, NULL);
    
    return OK;
}
