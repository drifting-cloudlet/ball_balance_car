#ifndef __GYROSCOPE_DRIVER_H__
#define __GYROSCOPE_DRIVER_H__

#include "main.h"
#include "icm20608.h"
#include "stdbool.h"


// ��̬������غ궨��
#define IS_FINITE(x) (!((x) != (x)) && (x) > -1e30f && (x) < 1e30f)
#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.017453292f

// ��̬����ṹ��
typedef struct {
    float roll;     // ����� (��)
    float pitch;    // ������ (��) 
    float yaw;      // ƫ���� (��)
    float q0, q1, q2, q3;  // ��Ԫ��
    float gx_bias, gy_bias, gz_bias;  // ��������ƫ
    uint8_t calibrated;  // У׼״̬��־
    uint32_t last_update_time;  // �ϴθ���ʱ��
} EulerAngles_t;

// У׼�����ṹ��
typedef struct {
    uint32_t calibration_count;
    float gyro_sum_x, gyro_sum_y, gyro_sum_z;
    uint8_t is_calibrating;
    uint32_t calibration_start_time;
} GyroCalibration_t;

// �ⲿ��������
extern EulerAngles_t euler_angles;
extern GyroCalibration_t gyro_calibration;

// ��������
void Gyroscope_Driver_Init(void);
void Gyroscope_Update_Euler(Vector3f *gyro, Vector3f *accel, float dt);
void Gyroscope_Calibrate_Start(void);
void Gyroscope_Calibrate_Update(Vector3f *gyro);
void Gyroscope_Get_Euler_Angles(float *roll, float *pitch, float *yaw);
void Gyroscope_Reset_Attitude(void);
void Gyroscope_Initialize_Attitude(Vector3f *accel);
float Gyroscope_Get_Yaw_Rate(void);
void Gyroscope_Print_Status(void);
void Gyroscope_Print_Raw_Data(Vector3f *gyro, Vector3f *accel);

float convert_to_continuous_yaw(float current_yaw);

#endif

