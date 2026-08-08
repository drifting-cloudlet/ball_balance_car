#include "gyroscope_driver.h"
#include "uart_driver.h"
#include <math.h>

// ??????????
EulerAngles_t euler_angles = {0};
GyroCalibration_t gyro_calibration = {0};

/* Bias-corrected yaw rate, deg/s. Rate only: an integrated heading needs the
 * bias to be exact or it ramps away, while the same bias in a rate is a constant
 * offset that the stationary boot calibration removes. */
static volatile float gyro_yaw_rate_dps = 0.0f;

// ?????????????
static const float CALIBRATION_TIME_MS = 2000.0f;  // ��????2??
static const float STATIC_THRESHOLD = 5.0f;       // ????????? (??/s)
static const float DRIFT_THRESHOLD = 10.0f;       // ???????? (??/s)

/**
 * @brief ???????????????
 */
void Gyroscope_Driver_Init(void)
{
    // ?????????????
    euler_angles.roll = 0.0f;
    euler_angles.pitch = 0.0f;
    euler_angles.yaw = 0.0f;
    
    // ?????????????��?????
    euler_angles.q0 = 1.0f;
    euler_angles.q1 = 0.0f;
    euler_angles.q2 = 0.0f;
    euler_angles.q3 = 0.0f;
    
    // ????????
    euler_angles.gx_bias = 0.0f;
    euler_angles.gy_bias = 0.0f;
    euler_angles.gz_bias = 0.0f;
    
    euler_angles.calibrated = 0;
    euler_angles.last_update_time = HAL_GetTick();
    
    // ?????��?????
    gyro_calibration.calibration_count = 0;
    gyro_calibration.gyro_sum_x = 0.0f;
    gyro_calibration.gyro_sum_y = 0.0f;
    gyro_calibration.gyro_sum_z = 0.0f;
    gyro_calibration.is_calibrating = 0;
}

/**
 * @brief ?????????��?
 */
void Gyroscope_Calibrate_Start(void)
{
    gyro_calibration.calibration_count = 0;
    gyro_calibration.gyro_sum_x = 0.0f;
    gyro_calibration.gyro_sum_y = 0.0f;
    gyro_calibration.gyro_sum_z = 0.0f;
    gyro_calibration.is_calibrating = 1;
    gyro_calibration.calibration_start_time = HAL_GetTick();
}

/**
 * @brief ??????????��?
 * @param gyro ??????????
 */
void Gyroscope_Calibrate_Update(Vector3f *gyro)
{
    if (!gyro_calibration.is_calibrating) return;
    
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - gyro_calibration.calibration_start_time;
    
    // ???��????
    if (elapsed_time >= CALIBRATION_TIME_MS) {
        if (gyro_calibration.calibration_count > 0) {
            // ??????????
            euler_angles.gx_bias = gyro_calibration.gyro_sum_x / gyro_calibration.calibration_count;
            euler_angles.gy_bias = gyro_calibration.gyro_sum_y / gyro_calibration.calibration_count;
            euler_angles.gz_bias = gyro_calibration.gyro_sum_z / gyro_calibration.calibration_count;
            
            euler_angles.calibrated = 1;
                       
            // ��??????????????????
            Gyroscope_Reset_Attitude();
        }
        
        gyro_calibration.is_calibrating = 0;
        return;
    }
    
    // ???????????????????
    if (fabs(gyro->x) < STATIC_THRESHOLD && 
        fabs(gyro->y) < STATIC_THRESHOLD && 
        fabs(gyro->z) < STATIC_THRESHOLD) {
        
        gyro_calibration.gyro_sum_x += gyro->x;
        gyro_calibration.gyro_sum_y += gyro->y;
        gyro_calibration.gyro_sum_z += gyro->z;
        gyro_calibration.calibration_count++;
        
        // ???��?????
        if (gyro_calibration.calibration_count % 50 == 0) {
            float progress = (float)elapsed_time / CALIBRATION_TIME_MS * 100.0f;
        }
    }
}

/**
 * @brief ??????��????
 * @param gyro ??????????
 * @param accel ??????????
 * @return 1-??��, 0-??��
 */
static uint8_t data_validity_check(Vector3f *gyro, Vector3f *accel)
{
    // ????????????????
    if (!IS_FINITE(gyro->x) || !IS_FINITE(gyro->y) || !IS_FINITE(gyro->z) ||
        !IS_FINITE(accel->x) || !IS_FINITE(accel->y) || !IS_FINITE(accel->z)) {
        return 0;
    }
    
    // ????????????????????????�� (??2000??/s)
    if (fabs(gyro->x) > 2000.0f || fabs(gyro->y) > 2000.0f || fabs(gyro->z) > 2000.0f) {
        return 0;
    }
    
    // ???????????????????????�� (??16g)
    if (fabs(accel->x) > 16.0f || fabs(accel->y) > 16.0f || fabs(accel->z) > 16.0f) {
        return 0;
    }
    
    return 1;
}

/**
 * @brief ??????????
 * @param q0,q1,q2,q3 ????????????
 */
static void quaternion_normalize(float *q0, float *q1, float *q2, float *q3)
{
    float norm = sqrtf((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
    
    if (norm > 0.0f) {
        float inv_norm = 1.0f / norm;
        *q0 *= inv_norm;
        *q1 *= inv_norm;
        *q2 *= inv_norm;
        *q3 *= inv_norm;
    }
}

/**
 * @brief ??????????????
 * @param q0,q1,q2,q3 ?????????
 * @param roll,pitch,yaw ???????????
 */
static void quaternion_to_euler(float q0, float q1, float q2, float q3, 
                                float *roll, float *pitch, float *yaw)
{
    // Roll (x?????)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    
    // Pitch (y?????)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1.0f) {
        *pitch = copysignf(90.0f, sinp);  // ???90???|sin(pitch)| >= 1
    } else {
        *pitch = asinf(sinp) * RAD_TO_DEG;
    }
    
    // Yaw (z?????)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

/**
 * @brief ?????????????
 * @param gyro ?????????? (??/s)
 * @param accel ?????????? (g)
 * @param dt ????? (s)
 */
static void complementary_filter_update(Vector3f *gyro, Vector3f *accel, float dt)
{
    // ??????????
    float gx = (gyro->x - euler_angles.gx_bias) * DEG_TO_RAD;
    float gy = (gyro->y - euler_angles.gy_bias) * DEG_TO_RAD;
    float gz = (gyro->z - euler_angles.gz_bias) * DEG_TO_RAD;

    /* Publish the bias-corrected turn rate from the one place that already
     * computes it. Keeping it here rather than in the app layer stops the driver
     * from having to reach upward for a value it owns. */
    gyro_yaw_rate_dps = gyro->z - euler_angles.gz_bias;


    // ??????????????
    float accel_norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    if (accel_norm == 0.0f) return;
    
    float ax = accel->x / accel_norm;
    float ay = accel->y / accel_norm;
    float az = accel->z / accel_norm;
    
    // ????????
    float q0 = euler_angles.q0;
    float q1 = euler_angles.q1;
    float q2 = euler_angles.q2;
    float q3 = euler_angles.q3;
    
    // ?????�ħ�???????��????????????
    if (!euler_angles.calibrated) {
        // ??��??????????????????????????????????????
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        if (norm > 0.1f) {
            // ??????Roll??Pitch
            float init_roll = atan2f(ay, az);
            float init_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
            
            // ???????????Yaw???0??
            float cr = cosf(init_roll * 0.5f);
            float sr = sinf(init_roll * 0.5f);
            float cp = cosf(init_pitch * 0.5f);
            float sp = sinf(init_pitch * 0.5f);
            
            euler_angles.q0 = cr * cp;
            euler_angles.q1 = sr * cp;
            euler_angles.q2 = cr * sp;
            euler_angles.q3 = 0.0f;  // Yaw = 0
            
            // ?????
            quaternion_normalize(&euler_angles.q0, &euler_angles.q1, &euler_angles.q2, &euler_angles.q3);
        }
        return;
    }
    
    // ?????????????????
    float q0_gyro = q0 + 0.5f * dt * (-q1 * gx - q2 * gy - q3 * gz);
    float q1_gyro = q1 + 0.5f * dt * ( q0 * gx - q3 * gy + q2 * gz);
    float q2_gyro = q2 + 0.5f * dt * ( q3 * gx + q0 * gy - q1 * gz);
    float q3_gyro = q3 + 0.5f * dt * (-q2 * gx + q1 * gy + q0 * gz);
    
    // ????????????? (????????)
    float gx_est = 2.0f * (q1 * q3 - q0 * q2);
    float gy_est = 2.0f * (q0 * q1 + q2 * q3);
    float gz_est = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    // ??????????? (???)
    float ex = ay * gz_est - az * gy_est;
    float ey = az * gx_est - ax * gz_est;
    float ez = ax * gy_est - ay * gx_est;
    
    // ???????????????
    float kp = 0.5f;  // ????????
    float ki = 0.01f; // ???????��???????????????
    
    // ????????��???????
    static float ex_int = 0.0f, ey_int = 0.0f, ez_int = 0.0f;
    
    // ?????????????��?????
    ex_int += ex * dt;
    ey_int += ey * dt;
    ez_int += ez * dt;
    
    // ?????????
    float int_limit = 1.0f;
    if (ex_int > int_limit) ex_int = int_limit;
    if (ex_int < -int_limit) ex_int = -int_limit;
    if (ey_int > int_limit) ey_int = int_limit;
    if (ey_int < -int_limit) ey_int = -int_limit;
    if (ez_int > int_limit) ez_int = int_limit;
    if (ez_int < -int_limit) ez_int = -int_limit;
    
    // PI??????��??
    float gx_corrected = gx + kp * ex + ki * ex_int;
    float gy_corrected = gy + kp * ey + ki * ey_int;
    float gz_corrected = gz + kp * ez + ki * ez_int;
    
    // ???��???????????????????????
    euler_angles.q0 = q0 + 0.5f * dt * (-q1 * gx_corrected - q2 * gy_corrected - q3 * gz_corrected);
    euler_angles.q1 = q1 + 0.5f * dt * ( q0 * gx_corrected - q3 * gy_corrected + q2 * gz_corrected);
    euler_angles.q2 = q2 + 0.5f * dt * ( q3 * gx_corrected + q0 * gy_corrected - q1 * gz_corrected);
    euler_angles.q3 = q3 + 0.5f * dt * (-q2 * gx_corrected + q1 * gy_corrected + q0 * gz_corrected);
    
    // ??????????
    quaternion_normalize(&euler_angles.q0, &euler_angles.q1, &euler_angles.q2, &euler_angles.q3);
    
    // ?????????
    quaternion_to_euler(euler_angles.q0, euler_angles.q1, euler_angles.q2, euler_angles.q3,
                       &euler_angles.roll, &euler_angles.pitch, &euler_angles.yaw);
}

/**
 * @brief ????????? (?????????)
 * @param gyro ?????????? (??/s)
 * @param accel ?????????? (g)
 * @param dt ????? (s)
 */
void Gyroscope_Update_Euler(Vector3f *gyro, Vector3f *accel, float dt)
{
    if (!gyro || !accel) return;
    
    // ??????��????
    if (!data_validity_check(gyro, accel)) {
        return;
    }
    
    // ???????��???????��?????
    if (gyro_calibration.is_calibrating) {
        Gyroscope_Calibrate_Update(gyro);
        return;
    }
    
    // ?????????????��? (?????????)
    if (euler_angles.calibrated) {
        float gyro_magnitude = sqrtf(gyro->x * gyro->x + gyro->y * gyro->y + gyro->z * gyro->z);
        if (gyro_magnitude > DRIFT_THRESHOLD) {
            static uint32_t last_drift_check = 0;
            uint32_t current_time = HAL_GetTick();
            
            // ?5????????????
            if (current_time - last_drift_check > 300000) {
                last_drift_check = current_time;
            }
        }
    }
    
    // ???????
    complementary_filter_update(gyro, accel, dt);
    
    euler_angles.last_update_time = HAL_GetTick();
}

/**
 * @brief ????????
 * @param roll,pitch,yaw ???????????
 */
void Gyroscope_Get_Euler_Angles(float *roll, float *pitch, float *yaw)
{
    if (roll) *roll = euler_angles.roll;
    if (pitch) *pitch = euler_angles.pitch;
    if (yaw) *yaw = euler_angles.yaw;
}

/**
 * @brief ???????
 */
void Gyroscope_Reset_Attitude(void)
{
    euler_angles.q0 = 1.0f;
    euler_angles.q1 = 0.0f;
    euler_angles.q2 = 0.0f;
    euler_angles.q3 = 0.0f;
    
    euler_angles.roll = 0.0f;
    euler_angles.pitch = 0.0f;
    euler_angles.yaw = 0.0f;
}

/**
 * @brief ????????????��??????????
 * @param accel ??????????
 */
void Gyroscope_Initialize_Attitude(Vector3f *accel)
{
    if (!accel) return;
    
    // ??????????????
    float norm = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
    if (norm < 0.1f) {
        return;
    }
    
    float ax = accel->x / norm;
    float ay = accel->y / norm;
    float az = accel->z / norm;
    
    // ??????Roll??Pitch???????????????????
    float init_roll = atan2f(ay, az);
    float init_pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    
    // ???????????Yaw???0??
    float cr = cosf(init_roll * 0.5f);
    float sr = sinf(init_roll * 0.5f);
    float cp = cosf(init_pitch * 0.5f);
    float sp = sinf(init_pitch * 0.5f);
    
    euler_angles.q0 = cr * cp;
    euler_angles.q1 = sr * cp;
    euler_angles.q2 = cr * sp;
    euler_angles.q3 = 0.0f;  // Yaw = 0

    // ��һ����Ԫ��
    quaternion_normalize(&euler_angles.q0, &euler_angles.q1, &euler_angles.q2, &euler_angles.q3);

    // ת��Ϊŷ����
    quaternion_to_euler(euler_angles.q0, euler_angles.q1, euler_angles.q2, euler_angles.q3,
                       &euler_angles.roll, &euler_angles.pitch, &euler_angles.yaw);

}

/**
 * @brief ????????????
 * @return ????????? (??/s)
 */
float Gyroscope_Get_Yaw_Rate(void)
{
    /* Was returning gz_bias, the calibration offset, not the rate. Any caller
     * got a near-constant number that looked plausible and meant nothing. */
    return gyro_yaw_rate_dps;
}

/**
 * @brief ???????????????????????
 * @param gyro ??????????
 * @param accel ??????????
 */
void Gyroscope_Print_Raw_Data(Vector3f *gyro, Vector3f *accel)
{
    if (!gyro || !accel) return;
    
    // ?????????????????????
    float gyro_mag = sqrtf(gyro->x * gyro->x + gyro->y * gyro->y + gyro->z * gyro->z);
    float accel_mag = sqrtf(accel->x * accel->x + accel->y * accel->y + accel->z * accel->z);
}

// ??t????????????????��???
float g_last_yaw = 0.0f;
int g_revolution_count = 0;
bool g_is_yaw_initialized = false;

/**
 * @brief ???????[-180, 180]??��???yaw??????????????????
 * 
 * @param current_yaw ??????????????yaw? (-180 to 180)??
 * @return float ??????yaw???? (???? 370, -450 ??)??
 */
float convert_to_continuous_yaw(float current_yaw) 
{
    // ???????????????????????????????180??????270??300???????
    const float WRAP_AROUND_THRESHOLD = 300.0f;

    // ??��???????��????
    if (!g_is_yaw_initialized) {
        g_last_yaw = current_yaw;
        g_is_yaw_initialized = true;
        g_revolution_count = 0;
    }

    // ????????��????????
    float diff = current_yaw - g_last_yaw;

    // ???????????????
    if (diff > WRAP_AROUND_THRESHOLD) {
        // ???????????????? (????, ?? 170?? ?? -175??), ??????????, ??????????
        // ??? diff ??? -360 (???? -175 - 170 = -345)
        // ???????????????-180????+180?????????????????????
        g_revolution_count--;
    } else if (diff < -WRAP_AROUND_THRESHOLD) {
        // ??????????????? (????, ?? -170?? ?? 175??), ??????????, ?????��?��
        // ??? diff ??? 360 (???? 175 - (-170) = 345)
        // ???????????????+180????-180?????????????????????
        g_revolution_count++;
    }

    // ??????��?yaw?????���???
    g_last_yaw = current_yaw;

    // ??????????yaw?
    float continuous_yaw = current_yaw + (float)g_revolution_count * 360.0f;

    return continuous_yaw;
}
