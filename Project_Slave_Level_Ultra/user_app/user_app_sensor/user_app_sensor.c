#include "user_app_sensor.h"
#include "user_define.h"
#include "dac.h"
#include "user_convert_variable.h"
#include "user_app_modbus_rtu.h"
#include "iwdg.h"
#include "user_connect_sensor.h"

/*=========================Fucntion Static=========================*/
static uint8_t fevent_sensor_entry(uint8_t event);
static uint8_t fevent_sensor_handle(uint8_t event);
static uint8_t fevent_sensor_dac(uint8_t event);

static uint8_t fevent_detect_connect(uint8_t event);
static uint8_t fevent_temp_alarm(uint8_t event);
static uint8_t fevent_refresh_iwdg(uint8_t event);
/*==============================Struct=============================*/
sEvent_struct               sEventAppSensor[]=
{
  {_EVENT_SENSOR_ENTRY,              1, 5, 2,                fevent_sensor_entry},            //Doi slave khoi dong moi truyen opera
  {_EVENT_SENSOR_HANDLE,             0, 5, 10,               fevent_sensor_handle}, 
  {_EVENT_SENSOR_DAC,                1, 5, 500,              fevent_sensor_dac},
  
  {_EVENT_DETECT_CONNECT,            1, 5, 15000,            fevent_detect_connect},
  {_EVENT_TEMP_ALARM,                1, 5, 2000,             fevent_temp_alarm},
  {_EVENT_REFRESH_IWDG,              1, 5, 250,              fevent_refresh_iwdg},
};
int32_t aSampling_STT[NUMBER_SAMPLING_SS] = {0};
int32_t aSampling_VALUE[NUMBER_SAMPLING_SS] = {0};

struct_SensorLevel  sSensorLevel = {0};
struct_TempAlarm    sTempAlarm = {0};
struct_CalibDAC     sCalibDAC = {0};

extern sData sUart232;
/*========================Function Handle========================*/
static uint8_t fevent_sensor_entry(uint8_t event)
{
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    return 1;
}
uint32_t test_gettick = 0;
uint32_t test_gettick_2 = 0;

static uint8_t fevent_sensor_handle(uint8_t event)
{
    test_gettick = HAL_GetTick() - test_gettick_2;
    test_gettick_2 = HAL_GetTick();
    uint32_t TempU32 = 0;
    if(sUart232.Data_a8[0] == 'R' && sUart232.Data_a8[4] == 0x0D)
    {
//        uint8_t length = 0;
//        for(uint8_t i = 1; i < sUart232.Length_u16; i++)
//        {
//            if( sUart232.Data_a8[i] < '0' || sUart232.Data_a8[i]>'9') break;
//            else length++;
//        }
//        TempU32 = Convert_String_To_Dec(sUart232.Data_a8 , length);
        TempU32 = (sUart232.Data_a8[1] - 48)*100 + (sUart232.Data_a8[2] - 48)*10 + (sUart232.Data_a8[3] - 48)*1;
    }
    
//    sSensorLevel.LevelValueReal_f = (float)(TempU32);
    
    sSensorLevel.LevelValueReal_f = quickSort_Sampling_Value((int32_t)TempU32);
    
//    sSensorLevel.LevelValueFilter_f = Filter_pH(sSensorLevel.LevelValueReal_f);
    sSensorLevel.LevelValueFilter_f = ConvertTemperature_Calib(sSensorLevel.LevelValueReal_f);
    
    if(sSensorLevel.LevelValueReal_f == 0)
    {
        fevent_enable(sEventAppSensor, _EVENT_DETECT_CONNECT);
    }
    Reset_Buff(&sUart232);
    fevent_enable(sEventAppSensor, _EVENT_DETECT_CONNECT);

    return 1; 
}
        
static uint8_t fevent_sensor_dac(uint8_t event)
{
    uint32_t Data = 0;
    float stamp = 0;
    float stamp_2 = 0;

    if(sCalibDAC.stateDAC_u8 == 0)
    {
        if(sSensorLevel.LevelValueFilter_f < 20)
        {
            Data = 0;
        }
        else
        {
            stamp = (sSensorLevel.LevelValueFilter_f-LEVEL_MIN)/(LEVEL_MAX-LEVEL_MIN);
            stamp = stamp * (sCalibDAC.DAC_Max_u16 - sCalibDAC.DAC_Min_u16) + sCalibDAC.DAC_Min_u16;
            Data = (uint32_t)(stamp);
        }
    }
    else
    {
        if(sCalibDAC.DAC_ATcmd_u16 < sCalibDAC.DAC_Min_u16)
            Data = sCalibDAC.DAC_Min_u16;
        else if (sCalibDAC.DAC_ATcmd_u16 > sCalibDAC.DAC_Max_u16)
            Data = sCalibDAC.DAC_Max_u16;
        else
            Data = sCalibDAC.DAC_ATcmd_u16;
    }
    
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, Data);
    
    stamp_2 = (float)(Data - sCalibDAC.DAC_Min_u16)/(sCalibDAC.DAC_Max_u16 - sCalibDAC.DAC_Min_u16);
    sSensorLevel.CurrOutValue_f = (stamp_2*(CURR_OUT_MAX - CURR_OUT_MIN)) + CURR_OUT_MIN;
    
    fevent_enable(sEventAppSensor, event);
    return 1; 
}

static uint8_t fevent_detect_connect(uint8_t event)
{
    sSensorLevel.LevelValueFilter_f = 0;
    fevent_enable(sEventAppSensor, event);
    return 1;
}

static uint8_t fevent_temp_alarm(uint8_t event)
{
//    if(sTempAlarm.State == 1)
//    {
//        if((sSensorTemp.TempObject_f > sTempAlarm.Alarm_Upper) || 
//           (sSensorTemp.TempObject_f < sTempAlarm.Alarm_Lower))
//        {
//            ALARM_ON;
//        }
//        else
//            ALARM_OFF;
//    }
//    else 
//        ALARM_OFF;
    
    fevent_enable(sEventAppSensor, event);
    return 1;
}

static uint8_t fevent_refresh_iwdg(uint8_t event)
{
    HAL_IWDG_Refresh(&hiwdg);
    fevent_enable(sEventAppSensor, event);
    return 1;
}

/*=====================quickSort Sampling======================*/
void quickSort_Sampling(int32_t array_stt[],int32_t array_sampling[], uint8_t left, uint8_t right)
{
/*---------------------- Sap xep noi bot --------------------*/
  
  for(uint8_t i = 0; i < NUMBER_SAMPLING_SS; i++)
  {
    for(uint8_t j = 0; j < NUMBER_SAMPLING_SS - 1; j++)
    {
        if(array_sampling[j] > array_sampling[j + 1])
        {
			int temp = 0;
            temp = array_sampling[j];
			array_sampling[j] = array_sampling[j+1];
			array_sampling[j+1] = temp;
            
            temp = array_stt[j];
			array_stt[j] = array_stt[j+1];
			array_stt[j+1] = temp;
        }
    }
  }
}

float quickSort_Sampling_Value(int32_t Value)
{
    static uint8_t Handle_Once = 0;
    float Result = 0;
    
//    if(Value == 0)
//    {
//        Handle_Once = 0;
//    }
//    else
//    {
        if(Handle_Once == 0)
        {
            Handle_Once = 1;
            for(uint8_t i = 0; i< NUMBER_SAMPLING_SS; i++)
            {
              aSampling_STT[i] = i;
              
              if(aSampling_VALUE[i] == 0)
                aSampling_VALUE[i] = Value;
            }
            
            quickSort_Sampling(aSampling_STT, aSampling_VALUE, 0, NUMBER_SAMPLING_SS - 1);
            Result = aSampling_VALUE[NUMBER_SAMPLING_SS/2];
        }
        else
        {
            for(uint8_t i = 0; i < NUMBER_SAMPLING_SS; i++)
            {
                if(aSampling_STT[i] == NUMBER_SAMPLING_SS - 1)
                {
                    aSampling_STT[i] = 0;
                    aSampling_VALUE[i] = Value;
                }
                else
                    aSampling_STT[i] = aSampling_STT[i] + 1;
            }

            quickSort_Sampling(aSampling_STT, aSampling_VALUE, 0, NUMBER_SAMPLING_SS - 1);
            Result = aSampling_VALUE[NUMBER_SAMPLING_SS/2];
        }
//    }
    return Result;
}
                         
/*=====================Filter Temperature====================*/
float Filter_pH(float var)
{
    //Kalman Filter
    static float x_est = 0.0;   // Uoc luong ban dau
    static float P = 1.0;       // Hiep phuong sai ban dau
    
    static float x_est_last = 0;
  
    float Q = 0.00000001;  // Nhieu mo hinh
    float R = 0.00001;   // Nhieu cam bien

    float x_pred, P_pred, K;
    
    float varFloat = 0;
//    int32_t varInt32 = 0;
    
    if(var != 0)
    {
//        varFloat = Handle_int32_To_Float_Scale(var, scale);
        varFloat = var;
        
        //Thay doi nhanh du lieu
        if(x_est_last - varFloat > 10 || varFloat - x_est_last > 10)
        {
           Q *=1000; 
        }
        
        // Buoc du doan
        x_pred = x_est;
        P_pred = P + Q;

        // Tinh he so kalman
        K = P_pred / (P_pred + R);

        // Cap nhat gia tri
        x_est = x_pred + K * (varFloat - x_pred);
        P = (1 - K) * P_pred;
        
//        varInt32 = Hanlde_Float_To_Int32_Scale_Round(x_est, scale);
    }
    else
    {
        P = 1;
        x_est = 0;
    }
    x_est_last = x_est;
//    return varInt32;
    return x_est;
}

float ConvertTemperature_Calib(float var)
{
    float result = 0;
    // Tinh he so a và b cua phuong trinh y = a*pH + b
    float a = 0;
    float b = 0;
    
    if((sSensorLevel.CalibPoint1_x_f != sSensorLevel.CalibPoint2_x_f) && (sSensorLevel.CalibPoint1_y_f != sSensorLevel.CalibPoint2_y_f))
    {
        a = (sSensorLevel.CalibPoint2_y_f  - sSensorLevel.CalibPoint1_y_f) / (sSensorLevel.CalibPoint2_x_f - sSensorLevel.CalibPoint1_x_f);
        b = sSensorLevel.CalibPoint1_y_f - a * sSensorLevel.CalibPoint1_x_f;
        result = a * var + b;
    }
    else
    {
        result = var;
    }
    
    result = result + sSensorLevel.Calib_Offset;
    
    if(result < LEVEL_MIN)
      result = LEVEL_MIN;
    
    if(result > LEVEL_MAX)
      result = LEVEL_MAX;
    
    return result;
}
/*===================Save and Init Calib====================*/
void Save_CalibTemperature(uint8_t eKind, float var)
{
#ifdef USING_APP_SENSOR
    uint8_t aData[50] = {0};
    uint8_t length = 0;
  
    uint32_t hexUint_Compensation   = 0;
    uint32_t hexUint_CalibPoint1_x  = 0;
    uint32_t hexUint_CalibPoint1_y  = 0;
    uint32_t hexUint_CalibPoint2_x  = 0;
    uint32_t hexUint_CalibPoint2_y  = 0;
    
    switch(eKind)
    {
        case _KIND_CALIB_OFFSET:
          sSensorLevel.Calib_Offset = var;
          break;
          
        case _KIND_CALIB_POINT_1:
          sSensorLevel.CalibPoint1_x_f = sSensorLevel.LevelValueReal_f;
          sSensorLevel.CalibPoint1_y_f = var;
          break;
          
        case _KIND_CALIB_POINT_2:
          sSensorLevel.CalibPoint2_x_f = sSensorLevel.LevelValueReal_f;
          sSensorLevel.CalibPoint2_y_f = var;
          break;
        
        default:
          break;
    }
    
    hexUint_Compensation   = Handle_Float_To_hexUint32(sSensorLevel.Calib_Offset);
    hexUint_CalibPoint1_x  = Handle_Float_To_hexUint32(sSensorLevel.CalibPoint1_x_f);
    hexUint_CalibPoint1_y  = Handle_Float_To_hexUint32(sSensorLevel.CalibPoint1_y_f);
    hexUint_CalibPoint2_x  = Handle_Float_To_hexUint32(sSensorLevel.CalibPoint2_x_f);
    hexUint_CalibPoint2_y  = Handle_Float_To_hexUint32(sSensorLevel.CalibPoint2_y_f);
    
    aData[length++] = hexUint_Compensation >> 24;
    aData[length++] = hexUint_Compensation >> 16;
    aData[length++] = hexUint_Compensation >> 8;
    aData[length++] = hexUint_Compensation ;
    
    aData[length++] = hexUint_CalibPoint1_x >> 24;
    aData[length++] = hexUint_CalibPoint1_x >> 16;
    aData[length++] = hexUint_CalibPoint1_x >> 8;
    aData[length++] = hexUint_CalibPoint1_x ;
    
    aData[length++] = hexUint_CalibPoint1_y >> 24;
    aData[length++] = hexUint_CalibPoint1_y >> 16;
    aData[length++] = hexUint_CalibPoint1_y >> 8;
    aData[length++] = hexUint_CalibPoint1_y ;
    
    aData[length++] = hexUint_CalibPoint2_x >> 24;
    aData[length++] = hexUint_CalibPoint2_x >> 16;
    aData[length++] = hexUint_CalibPoint2_x >> 8;
    aData[length++] = hexUint_CalibPoint2_x ;
    
    aData[length++] = hexUint_CalibPoint2_y >> 24;
    aData[length++] = hexUint_CalibPoint2_y >> 16;
    aData[length++] = hexUint_CalibPoint2_y >> 8;
    aData[length++] = hexUint_CalibPoint2_y ;

    Save_Array(ADDR_CALIB_TEMPERATURE, aData, length);
#endif   
}

void Init_CalibTemperature(void)
{
#ifdef USING_APP_SENSOR
    uint32_t hexUint_Compensation   = 0;
    uint32_t hexUint_CalibPoint1_x  = 0;
    uint32_t hexUint_CalibPoint1_y  = 0;
    uint32_t hexUint_CalibPoint2_x  = 0;
    uint32_t hexUint_CalibPoint2_y  = 0;
  
    if(*(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE) != FLASH_BYTE_EMPTY)
    {
        hexUint_Compensation  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+2) << 24;
        hexUint_Compensation  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+3)<< 16;
        hexUint_Compensation  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+4)<< 8;
        hexUint_Compensation  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+5);
        
        hexUint_CalibPoint1_x  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+6) << 24;
        hexUint_CalibPoint1_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+7)<< 16;
        hexUint_CalibPoint1_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+8)<< 8;
        hexUint_CalibPoint1_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+9);
        
        hexUint_CalibPoint1_y  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+10) << 24;
        hexUint_CalibPoint1_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+11)<< 16;
        hexUint_CalibPoint1_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+12)<< 8;
        hexUint_CalibPoint1_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+13);
        
        hexUint_CalibPoint2_x  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+14) << 24;
        hexUint_CalibPoint2_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+15)<< 16;
        hexUint_CalibPoint2_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+16)<< 8;
        hexUint_CalibPoint2_x  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+17);
        
        hexUint_CalibPoint2_y  = *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+18) << 24;
        hexUint_CalibPoint2_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+19)<< 16;
        hexUint_CalibPoint2_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+20)<< 8;
        hexUint_CalibPoint2_y  |= *(__IO uint8_t*)(ADDR_CALIB_TEMPERATURE+21);
        
        Convert_uint32Hex_To_Float(hexUint_Compensation,  &sSensorLevel.Calib_Offset );
        Convert_uint32Hex_To_Float(hexUint_CalibPoint1_x, &sSensorLevel.CalibPoint1_x_f);
        Convert_uint32Hex_To_Float(hexUint_CalibPoint1_y, &sSensorLevel.CalibPoint1_y_f);
        Convert_uint32Hex_To_Float(hexUint_CalibPoint2_x, &sSensorLevel.CalibPoint2_x_f);
        Convert_uint32Hex_To_Float(hexUint_CalibPoint2_y, &sSensorLevel.CalibPoint2_y_f);
    }
    else
    {
        sSensorLevel.Calib_Offset = 0;
        sSensorLevel.CalibPoint1_x_f = LEVEL_MIN;
        sSensorLevel.CalibPoint1_y_f = LEVEL_MIN;
        sSensorLevel.CalibPoint2_x_f = LEVEL_MAX;
        sSensorLevel.CalibPoint2_y_f = LEVEL_MAX;
    }
#endif   
}

void Save_TempAlarm(uint8_t State, float AlarmLower, float AlarmUpper)
{
#ifdef USING_APP_SENSOR
    uint8_t aData[50] = {0};
    uint8_t length = 0;
  
    uint32_t hexUint_AlarmUpper  = 0;
    uint32_t hexUint_AlarmLower  = 0;
    
    sTempAlarm.State = State;
    sTempAlarm.Alarm_Upper = AlarmUpper;
    sTempAlarm.Alarm_Lower = AlarmLower;
    
    hexUint_AlarmUpper  = Handle_Float_To_hexUint32(sTempAlarm.Alarm_Upper);
    hexUint_AlarmLower  = Handle_Float_To_hexUint32(sTempAlarm.Alarm_Lower);
    
    aData[length++] = sTempAlarm.State;
    
    aData[length++] = hexUint_AlarmUpper >> 24;
    aData[length++] = hexUint_AlarmUpper >> 16;
    aData[length++] = hexUint_AlarmUpper >> 8;
    aData[length++] = hexUint_AlarmUpper ;
    
    aData[length++] = hexUint_AlarmLower >> 24;
    aData[length++] = hexUint_AlarmLower >> 16;
    aData[length++] = hexUint_AlarmLower >> 8;
    aData[length++] = hexUint_AlarmLower ;

    Save_Array(ADDR_TEMPERATURE_ALARM, aData, length);
#endif   
}

void Init_TempAlarm(void)
{
#ifdef USING_APP_SENSOR
  
    uint32_t hexUint_AlarmUpper  = 0;
    uint32_t hexUint_AlarmLower  = 0;
  
    if(*(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM) != FLASH_BYTE_EMPTY)
    {
        sTempAlarm.State  = *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+2);
      
        hexUint_AlarmUpper  = *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+3) << 24;
        hexUint_AlarmUpper  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+4)<< 16;
        hexUint_AlarmUpper  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+5)<< 8;
        hexUint_AlarmUpper  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+6);
        
        hexUint_AlarmLower  = *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+7) << 24;
        hexUint_AlarmLower  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+8)<< 16;
        hexUint_AlarmLower  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+9)<< 8;
        hexUint_AlarmLower  |= *(__IO uint8_t*)(ADDR_TEMPERATURE_ALARM+10);
        
        Convert_uint32Hex_To_Float(hexUint_AlarmUpper, &sTempAlarm.Alarm_Upper);
        Convert_uint32Hex_To_Float(hexUint_AlarmLower, &sTempAlarm.Alarm_Lower);
    }
    else
    {
        sTempAlarm.State = 0;
        sTempAlarm.Alarm_Upper = LEVEL_MIN;
        sTempAlarm.Alarm_Lower = LEVEL_MAX;
    }
#endif   
}

void Save_CalibDAC(uint16_t DAC_Min, uint16_t DAC_Max)
{
#ifdef USING_APP_SENSOR
    uint8_t aData[50] = {0};
    uint8_t length = 0;
  
    if(DAC_Min < DAC_Max)
    {
        sCalibDAC.DAC_Min_u16 = DAC_Min;
        sCalibDAC.DAC_Max_u16 = DAC_Max;
        
        aData[length++] = sCalibDAC.DAC_Min_u16 >> 8;
        aData[length++] = sCalibDAC.DAC_Min_u16 ;
        
        aData[length++] = sCalibDAC.DAC_Max_u16 >> 8;
        aData[length++] = sCalibDAC.DAC_Max_u16 ;

        Save_Array(ADDR_CALIB_DAC, aData, length);
    }
#endif   
}

void Init_CalibDAC(void)
{
#ifdef USING_APP_SENSOR
  
    if(*(__IO uint8_t*)(ADDR_CALIB_DAC) != FLASH_BYTE_EMPTY)
    {
        sCalibDAC.DAC_Min_u16  |= *(__IO uint8_t*)(ADDR_CALIB_DAC+2)<< 8;
        sCalibDAC.DAC_Min_u16  |= *(__IO uint8_t*)(ADDR_CALIB_DAC+3);
        
        sCalibDAC.DAC_Max_u16  |= *(__IO uint8_t*)(ADDR_CALIB_DAC+4)<< 8;
        sCalibDAC.DAC_Max_u16  |= *(__IO uint8_t*)(ADDR_CALIB_DAC+5);
    }
    else
    {
        sCalibDAC.DAC_Min_u16 = DAC_MIN;
        sCalibDAC.DAC_Max_u16 = DAC_MAX;
    }
#endif   
}
/*==================Handle Define AT command=================*/
#ifdef USING_AT_CONFIG
void AT_CMD_Reset_Slave(sData *str, uint16_t Pos)
{
    uint8_t aTemp[60] = {0};   
    uint16_t length = 0;
    Save_InforSlaveModbusRTU(ID_DEFAULT, BAUDRATE_DEFAULT);
      
    Insert_String_To_String(aTemp, &length, (uint8_t*)"Reset OK!\r\n",0 , 11);
//	Modem_Respond(PortConfig, aTemp, length, 0);
    HAL_UART_Transmit(&uart_debug, aTemp,length, 1000);
}

void AT_CMD_Restore_Slave(sData *str, uint16_t Pos)
{
    uint8_t aTemp[60] = {0};   
    uint16_t length = 0;
    OnchipFlashPageErase(ADDR_CALIB_TEMPERATURE);
    sSensorLevel.Calib_Offset = 0;
    sSensorLevel.CalibPoint1_x_f = LEVEL_MIN;
    sSensorLevel.CalibPoint1_y_f = LEVEL_MIN;
    sSensorLevel.CalibPoint2_x_f = LEVEL_MAX;
    sSensorLevel.CalibPoint2_y_f = LEVEL_MAX;
    
    OnchipFlashPageErase(ADDR_TEMPERATURE_ALARM);
    sTempAlarm.State = 0;
    sTempAlarm.Alarm_Upper = LEVEL_MAX;
    sTempAlarm.Alarm_Lower = LEVEL_MIN;
    
    
    Insert_String_To_String(aTemp, &length, (uint8_t*)"Restore OK!\r\n",0 , 13);
//	Modem_Respond(PortConfig, aTemp, length, 0);
    HAL_UART_Transmit(&uart_debug, aTemp,length, 1000);
}

void AT_CMD_Get_ID_Slave (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "ID Slave: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 12}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sSlave_ModbusRTU.ID), 0x00);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)"\r\n",0 , 2);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}

void AT_CMD_Set_ID_Slave (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <= 255 )
        {
            Save_InforSlaveModbusRTU(TempU32, sSlave_ModbusRTU.Baudrate);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 1000);
        }
        else
        {
//            Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
    }
}

void AT_CMD_Get_BR_Slave (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "BR Slave: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 12}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sSlave_ModbusRTU.Baudrate), 0x00);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)"\r\n",0 , 2);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}

void AT_CMD_Set_BR_Slave (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <= 11 )
        {
            Save_InforSlaveModbusRTU(sSlave_ModbusRTU.ID, TempU32);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 1000);
        }
        else
        {
//            Modem_Respond(PortConfig, (uint8_t*)"ERROR", 5, 0);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
    }
}

void AT_CMD_Get_State_Clb_DAC (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "State DAC: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 11}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sCalibDAC.stateDAC_u8), 0x00);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)"\r\n",0 , 2);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}

void AT_CMD_Set_State_Clb_DAC (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <= 1 )
        {
            sCalibDAC.stateDAC_u8 = TempU32;
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 1000);
        }
        else
        {
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
    }
}

void AT_CMD_Get_Value_Clb_DAC (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "Value DAC: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 11}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sCalibDAC.DAC_ATcmd_u16), 0x00);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)"\r\n",0 , 2);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}

void AT_CMD_Set_Value_Clb_DAC (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <= 4095 )
        {
            sCalibDAC.DAC_ATcmd_u16 = TempU32;
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 1000);
        }
        else
        {
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
    }
}

void AT_CMD_Get_Value_DAC_4mA (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "DAC 4mA: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 9}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sCalibDAC.DAC_Min_u16), 0x00);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)"\r\n",0 , 2);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}

void AT_CMD_Set_Value_DAC_4mA (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <= 4095 )
        {
            Save_CalibDAC((uint16_t)(TempU32), sCalibDAC.DAC_Max_u16);
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 1000);
        }
        else
        {
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
    }
}

void AT_CMD_Get_Value_DAC_20mA (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "DAC 20mA: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 10}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sCalibDAC.DAC_Max_u16), 0x00);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)"\r\n",0 , 2);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}

void AT_CMD_Set_Value_DAC_20mA (sData *str_Receiv, uint16_t Pos)
{
    uint32_t TempU32 = 0;
    if( str_Receiv->Data_a8[0] >= '0' && str_Receiv->Data_a8[0] <= '9')
    {
        uint8_t length = 0;
        for(uint8_t i = 0; i < str_Receiv->Length_u16; i++)
        {
            if( str_Receiv->Data_a8[i] < '0' || str_Receiv->Data_a8[i]>'9') break;
            else length++;
        }
        TempU32 = Convert_String_To_Dec(str_Receiv->Data_a8 , length);
        if(TempU32 <= 4095 )
        {
            Save_CalibDAC(sCalibDAC.DAC_Min_u16, (uint16_t)(TempU32));
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"OK", 2, 1000);
        }
        else
        {
            HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
        }
    }
    else
    {
        HAL_UART_Transmit(&uart_debug, (uint8_t*)"ERROR", 5, 1000);
    }
}

void AT_CMD_Get_Current_Out (sData *str, uint16_t Pos)
{
    uint8_t aTemp[50] = "Current: ";   //13 ki tu dau tien
    sData StrResp = {&aTemp[0], 9}; 

    Convert_Point_Int_To_String_Scale (aTemp, &StrResp.Length_u16, (int)(sSensorLevel.CurrOutValue_f*100), 0xFE);
    Insert_String_To_String(aTemp, &StrResp.Length_u16, (uint8_t*)" mA\r\n",0 , 5);

    HAL_UART_Transmit(&uart_debug, StrResp.Data_a8, StrResp.Length_u16, 1000);
}
#endif

/*==================Handle Task and Init app=================*/
void       Init_AppSensor(void)
{
    Init_CalibTemperature();
    Init_TempAlarm();
    Init_CalibDAC();
#ifdef USING_AT_CONFIG
    /* regis cb serial */
    CheckList_AT_CONFIG[_RESET_SLAVE].CallBack = AT_CMD_Reset_Slave;
    CheckList_AT_CONFIG[_RESTORE_SLAVE].CallBack = AT_CMD_Restore_Slave;
    
    CheckList_AT_CONFIG[_GET_ID_SLAVE].CallBack = AT_CMD_Get_ID_Slave;
    CheckList_AT_CONFIG[_SET_ID_SLAVE].CallBack = AT_CMD_Set_ID_Slave;
    CheckList_AT_CONFIG[_GET_BR_SLAVE].CallBack = AT_CMD_Get_BR_Slave;
    CheckList_AT_CONFIG[_SET_BR_SLAVE].CallBack = AT_CMD_Set_BR_Slave;
    
    CheckList_AT_CONFIG[_GET_STATE_CLB_DAC].CallBack = AT_CMD_Get_State_Clb_DAC;
    CheckList_AT_CONFIG[_SET_STATE_CLB_DAC].CallBack = AT_CMD_Set_State_Clb_DAC;
    
    CheckList_AT_CONFIG[_GET_VALUE_CLB_DAC].CallBack = AT_CMD_Get_Value_Clb_DAC;
    CheckList_AT_CONFIG[_SET_VALUE_CLB_DAC].CallBack = AT_CMD_Set_Value_Clb_DAC;
    
    CheckList_AT_CONFIG[_GET_VALUE_DAC_4MA].CallBack = AT_CMD_Get_Value_DAC_4mA;
    CheckList_AT_CONFIG[_SET_VALUE_DAC_4MA].CallBack = AT_CMD_Set_Value_DAC_4mA;
    
    CheckList_AT_CONFIG[_GET_VALUE_DAC_20MA].CallBack = AT_CMD_Get_Value_DAC_20mA;
    CheckList_AT_CONFIG[_SET_VALUE_DAC_20MA].CallBack = AT_CMD_Set_Value_DAC_20mA;
    
    CheckList_AT_CONFIG[_GET_CURRENT_OUT].CallBack = AT_CMD_Get_Current_Out;
#endif
    RS232_Stop_RX_Mode();
    RS232_Init_RX_Mode();
}

uint8_t        AppSensor_Task(void)
{
    uint8_t i = 0;
    uint8_t Result =  false;
    
    for(i = 0; i < _EVENT_SENSOR_END; i++)
    {
        if(sEventAppSensor[i].e_status == 1)
        {
            Result = true; 
            
            if((sEventAppSensor[i].e_systick == 0) ||
               ((HAL_GetTick() - sEventAppSensor[i].e_systick) >= sEventAppSensor[i].e_period))
            {
                sEventAppSensor[i].e_status = 0; //Disable event
                sEventAppSensor[i].e_systick= HAL_GetTick();
                sEventAppSensor[i].e_function_handler(i);
            }
        }
    }
    
    return Result;
}
