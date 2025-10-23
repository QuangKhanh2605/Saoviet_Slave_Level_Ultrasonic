#ifndef USER_APP_SENSOR_H__
#define USER_APP_SENSOR_H__

#define USING_APP_SENSOR

#include "user_util.h"
#include "event_driven.h"

#define NUMBER_SAMPLING_SS      10

#define LEVEL_MIN               50
#define LEVEL_MAX               600

#define CURR_OUT_MIN            4
#define CURR_OUT_MAX            20

#define DAC_MIN                 0
#define DAC_MAX                 4095

typedef enum
{
    _EVENT_SENSOR_ENTRY,
    _EVENT_SENSOR_HANDLE,
    _EVENT_SENSOR_DAC,
    
    _EVENT_DETECT_CONNECT,
    
    _EVENT_TEMP_ALARM,
    _EVENT_REFRESH_IWDG,
    
    _EVENT_SENSOR_END,
}eKindEventSENSOR;

typedef enum
{
    _KIND_CALIB_OFFSET,
    _KIND_CALIB_POINT_1,
    _KIND_CALIB_POINT_2,
}eKindCalibLevel;

typedef struct
{
    float LevelValueReal_f;
    float LevelValueFilter_f;
    float CurrOutValue_f;
    
    float Calib_Offset;
    
    float CalibPoint1_x_f;
    float CalibPoint1_y_f;
    
    float CalibPoint2_x_f;
    float CalibPoint2_y_f;
}struct_SensorLevel;

typedef struct
{
    uint8_t State;
    float Alarm_Lower;
    float Alarm_Upper;
}struct_TempAlarm;

typedef struct
{
    uint8_t stateDAC_u8;
    uint16_t DAC_Min_u16;
    uint16_t DAC_Max_u16;
    uint16_t DAC_ATcmd_u16;
}struct_CalibDAC;

extern sEvent_struct        sEventAppSensor[];
extern struct_TempAlarm     sTempAlarm;
extern struct_SensorLevel   sSensorLevel;
/*====================Function Handle====================*/

uint8_t    AppSensor_Task(void);
void       Init_AppSensor(void);

void       Save_CalibTemperature(uint8_t eKind, float var);
void       Init_CalibTemperature(void);

void       Save_TempAlarm(uint8_t State, float AlarmLower, float AlarmUpper);
void       Init_TempAlarm(void);

void       Save_CalibDAC(uint16_t DAC_Min, uint16_t DAC_Max);
void       Init_CalibDAC(void);

float      Filter_pH(float var);
float      ConvertTemperature_Calib(float var);

void       quickSort_Sampling(int32_t array_stt[],int32_t array_sampling[], uint8_t left, uint8_t right);
float      quickSort_Sampling_Value(int32_t Value);

#endif
