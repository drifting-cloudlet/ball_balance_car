#ifndef __ENCODER_APP_H__
#define __ENCODER_APP_H__

#include "MyDefine.h"

void Encoder_Init(void);
void Encoder_Task(void);
void Encoder_Debug_Print(void);
void Encoder_Reset_Total_Count(void);
float Encoder_Get_Left_Distance_Cm(void);
float Encoder_Get_Right_Distance_Cm(void);
float Encoder_Get_Average_Distance_Cm(void);

extern Encoder left_encoder;
extern Encoder right_encoder;

#endif
