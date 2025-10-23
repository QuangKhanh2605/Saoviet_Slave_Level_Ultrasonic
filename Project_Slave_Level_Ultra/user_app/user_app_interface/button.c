

#include "button.h"
#include "lcd_display.h"

sButtonStruct sButton;


/*============ Function =============*/

void BUTTON_Process(void)
{
    switch(sButton.Value)
    {
        case _ENTER:  
            BUTTON_Enter_Process();
            break;
        case _UP:  
            BUTTON_Up_Process();
            break;
        case _DOWN:               
            BUTTON_Down_Process();
            break;
        case _ESC:
            BUTTON_ESC_Process();
            break;
        default:
            break;
    }
}



uint8_t BUTTON_scan(void)
{
    uint8_t r_value = 0;
    static uint8_t last_value = 0;
    static uint32_t hold = 0;
    
	if (sButton.Status == 0)
	{
		r_value = (!BUTTON_ENTER<<0)|(!BUTTON_UP<<1)|(!BUTTON_DOWN<<2)|(!BUTTON_ESC<<3);
        
		switch(r_value)
		{
			case _ENTER:
            case _UP:
            case _DOWN:
            case _ESC:
                sButton.Count++;
                if (sButton.Count >= BUT_TIME_DETECT) {
                    sButton.Value = r_value;
                    sButton.Status = 1;
                    
                    if ( r_value == last_value ) {
                        hold++;
                        if ( hold >= BUT_COUNT_FAST ) {
                            sButton.Count = BUT_TIME_DETECT;
                            hold = BUT_COUNT_FAST;
                        } else if (hold >= BUT_COUNT_HOLD) {
                            sButton.Count = BUT_TIME_DETECT - 2;
                        } else {
                            sButton.Count = 0;
                        }
                    } else {
                        sButton.Count = 0;  
                        hold = 0;
                    } 
                                
                    last_value = r_value;
                }
  				break;
			default:
                last_value = 0;
				sButton.Count = BUT_TIME_DETECT/2;
                hold = 0;
				break;
		}
	}

    return sButton.Value;
}

static char aTEST[10] = {"1234"};
//static sPressureLinearInter sPressConfig = {0};
//static Struct_Pulse sPulseConfig = {0};
//static int16_t PressOldCalib_i16 = 0;
//static char sConfirm[2][10] = {"No", "Yes"};

float       Old_Value_f = 0;
uint32_t    Old_Value_u32 = 0;
int16_t     Old_Value_i16 = 0;
float       Button_Stamp1_f = 0;
float       Button_Stamp2_f = 0;
      
void BUTTON_Enter_Process (void)
{
//    static uint8_t tempu8 = 0;
//    static uint8_t TempScale = 1;
    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCREEN_1:
            sButton.Old_value = 0;
            UTIL_MEM_set(aTEST, 0, sizeof(aTEST));
            aTEST[0] = '0'; aTEST[1] = '0', aTEST[2] = '0', aTEST[3] = '0';
          
            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_PASS, 0,
                               __PASS_WORD_2, __PASS_WORD_1, __PASS_WORD_2,
                               aTEST, (sButton.Old_value << 4) + 0x02);
          break;
          
        case _LCD_SCR_PASS:
            sButton.Old_value++;
            if (sButton.Old_value > 3)
            { 
                if (Display_Check_Password((uint8_t *) aTEST) == true) {
                    //copy screen now to screenback
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                       __SCR_SET_MODBUS, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                       NULL, 0xF1);
                } else {
                    sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;  //thoat
                }
            } else {
                Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, (sButton.Old_value << 4) + 0x02);
            }
            break;
            
        case _LCD_SCR_SETTING:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SCR_SET_MODBUS:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_MODBUS, 0,
                                       __SET_MODBUS_ID, __SET_MODBUS_ID, __SET_MODBUS_BR,
                                       NULL, 0xF1);
                    break;
                    
                case __SCR_SET_CALIB:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_TAB_1, 0,
                                       __SCR_CALIB_TAB_1_POINT_1, __SCR_CALIB_TAB_1_POINT_1, __SCR_CALIB_TAB_1_POINT_2,
                                       NULL, 0xF1);
                    break;
                    
                case __SCR_SET_INFOR:
                    UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                    Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_INFORMATION, 0,
                                       __SCR_INFOR_FW_VERSION_2, __SCR_INFOR_FW_VERSION_2, __SCR_INFOR_FW_VERSION_2,
                                       NULL, 0x00);
                    break;
            }
            break;
            
        case _LCD_SCR_SET_MODBUS:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SET_MODBUS_ID:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_MODBUS, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_MODBUS_ID, __SET_MODBUS_ID, __SET_MODBUS_BR,
                                               &sButton.Old_value, 0xF2);
                             sButton.Old_value = sParaDisplay.ID_u8;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SET_MODBUS_BR:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_MODBUS, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SET_MODBUS_BR, __SET_MODBUS_BR, __SET_MODBUS_BR,
                                               &sButton.Old_value, 0xF2);
                             sButton.Old_value = sParaDisplay.Baudrate_u32;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
                            break;
                        
                        default:
                            break;
                    }
                    break;
                   
            }
            break;
            
        case _LCD_SCR_SET_CALIB_TAB_1:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SCR_CALIB_TAB_1_POINT_1:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_TAB_1, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SCR_CALIB_TAB_1_POINT_1, __SCR_CALIB_TAB_1_POINT_1, __SCR_CALIB_TAB_1_POINT_2,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.Calib_Point1;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SCR_CALIB_TAB_1_POINT_2:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_TAB_1, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SCR_CALIB_TAB_1_POINT_2, __SCR_CALIB_TAB_1_POINT_1, __SCR_CALIB_TAB_1_POINT_2,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.Calib_Point2;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                default:
                  break;
            }
            break;
            
        case _LCD_SCR_SET_CALIB_TAB_2:
            switch (sLCD.sScreenNow.Para_u8)
            {
                case __SCR_CALIB_TAB_2_OFFSET:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_TAB_2, (sLCD.sScreenNow.SubIndex_u8+1),
                                               __SCR_CALIB_TAB_2_OFFSET, __SCR_CALIB_TAB_2_OFFSET, __SCR_CALIB_TAB_2_RESTORE,
                                               &sButton.Old_value, 0xF2);
                            sButton.Old_value = sParaDisplay.Calib_Offset;
                            break;
                            
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                               __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                               NULL, 0xF0);
                            sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
                            break;
                        
                        default:
                            break;
                    }
                    break;
                    
                case __SCR_CALIB_TAB_2_RESTORE:
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_CHECK_SETTING, 0,
                                           __CHECK_STATE_SETTING, __CHECK_STATE_SETTING, __CHECK_STATE_SETTING,
                                           NULL, 0xF0);
                        sParaDisplay.State_Setting = _STATE_SETTING_ENTER;
                    break;
                    
                default:
                  break;
            }
            break;
            
            
        case _LCD_SCR_CHECK_SETTING:
          switch(sLCD.sScreenBack.Index_u8)
          {
              case _LCD_SCR_SET_MODBUS:
                sParaDisplay.State_Setting = _STATE_SETTING_DONE;
                switch (sLCD.sScreenBack.Para_u8)
                {
                    case __SET_MODBUS_ID:
                      Save_InforSlaveModbusRTU((uint8_t)sButton.Old_value, sSlave_ModbusRTU.Baudrate);
                      break;
                      
                    case __SET_MODBUS_BR:
                      for(uint8_t i = 0; i < 11; i++)
                      {
                        if(aBaudrate_value[i] == sButton.Old_value)
                        {
                            sSlave_ModbusRTU.Baudrate = i;
                            break;
                        }
                      }
                      Save_InforSlaveModbusRTU(sSlave_ModbusRTU.ID, sSlave_ModbusRTU.Baudrate);
                      break;
                      
                    default:
                      break;
                }
                break;
                
              case _LCD_SCR_SET_CALIB_TAB_1:
                sParaDisplay.State_Setting = _STATE_SETTING_DONE;
                switch (sLCD.sScreenBack.Para_u8)
                {
                    case __SCR_CALIB_TAB_1_POINT_1:
                      Save_CalibTemperature(_KIND_CALIB_POINT_1, (float)sButton.Old_value);
                      break;
                      
                    case __SCR_CALIB_TAB_1_POINT_2:
                      Save_CalibTemperature(_KIND_CALIB_POINT_2, (float)sButton.Old_value);
                      break;
                      
                    default:
                      break;
                }
                break;
                
              case _LCD_SCR_SET_CALIB_TAB_2:
                sParaDisplay.State_Setting = _STATE_SETTING_DONE;
                switch (sLCD.sScreenBack.Para_u8)
                {
                    case __SCR_CALIB_TAB_2_OFFSET:
                      Save_CalibTemperature(_KIND_CALIB_OFFSET, (float)sButton.Old_value);
                      break;
                      
                    case __SCR_CALIB_TAB_2_RESTORE:
                        sSensorLevel.Calib_Offset = 0;
                        sSensorLevel.CalibPoint1_x_f = LEVEL_MIN;
                        sSensorLevel.CalibPoint1_y_f = LEVEL_MIN;
                        sSensorLevel.CalibPoint2_x_f = LEVEL_MAX;
                        sSensorLevel.CalibPoint2_y_f = LEVEL_MAX;

                        OnchipFlashPageErase(ADDR_TEMPERATURE_ALARM);
                        OnchipFlashPageErase(ADDR_CALIB_TEMPERATURE);
                        sTempAlarm.State = 0;
                        sTempAlarm.Alarm_Upper = LEVEL_MAX;
                        sTempAlarm.Alarm_Lower = LEVEL_MIN;
                      break;
                      
                    default:
                      break;
                }
                break;
                
              default:
                break;
          }
          break;
          
        default:
          break;
    }
}

void BUTTON_Up_Process (void)
{
    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCREEN_1:
          break;
          
        case _LCD_SCR_PASS:
            aTEST[sButton.Old_value] ++;
            if (aTEST[sButton.Old_value] > '9')
                aTEST[sButton.Old_value] = '0';
            break;
            
        case _LCD_SCR_SETTING:
            if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                sLCD.sScreenNow.Para_u8--;
            }
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            break;
            
        case _LCD_SCR_SET_MODBUS:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                        sLCD.sScreenNow.Para_u8--;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SET_MODBUS_ID:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value < 256)
                                    sButton.Old_value++;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_MODBUS_BR:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                for(uint8_t i = 0; i < 11; i++)
                                {
                                    if(aBaudrate_value[i] == sButton.Old_value)
                                    {
                                        if(i<10)
                                        {
                                            sButton.Old_value = aBaudrate_value[i+1];
                                            break;
                                        }
                                    }
                                }
                                break;
                            
                            default:
                                break;
                        }
                        break;
                }
            }
            break;
          
        case _LCD_SCR_SET_CALIB_TAB_1:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                        sLCD.sScreenNow.Para_u8--;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SCR_CALIB_TAB_1_POINT_1:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value < LEVEL_MAX)
                                    sButton.Old_value++;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SCR_CALIB_TAB_1_POINT_2:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value < LEVEL_MAX)
                                    sButton.Old_value++;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    default:
                      break;
                }
            }
            break;
            
        case _LCD_SCR_SET_CALIB_TAB_2:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 > sLCD.sScreenNow.ParaMin_u8 ) {
                        sLCD.sScreenNow.Para_u8--;
                        Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
                    } else {
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_TAB_1, 0,
                                               __SCR_CALIB_TAB_1_POINT_2, __SCR_CALIB_TAB_1_POINT_1, __SCR_CALIB_TAB_1_POINT_2,
                                               NULL, 0xF1);
                    }
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SCR_CALIB_TAB_2_OFFSET:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                sButton.Old_value++;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SCR_CALIB_TAB_2_RESTORE:
                        break;

                    default:
                      break;
                }
            }
            break;
            
        default:
          break;
    }
}


void BUTTON_Down_Process (void)
{
    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCREEN_1:
          break;
          
        case _LCD_SCR_PASS:
            aTEST[sButton.Old_value] --;
            if (aTEST[sButton.Old_value] < '0') {
                aTEST[sButton.Old_value] = '9';
            }
            break;
            
        case _LCD_SCR_SETTING:
            if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                sLCD.sScreenNow.Para_u8++;
            }
            Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
            break;
            
        case _LCD_SCR_SET_MODBUS:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                        sLCD.sScreenNow.Para_u8++;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SET_MODBUS_ID:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value > 0)
                                    sButton.Old_value--;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SET_MODBUS_BR:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                for(uint8_t i = 10; i>0; i--)
                                {
                                    if(aBaudrate_value[i] == sButton.Old_value)
                                    {
                                        if(i>0)
                                        {
                                            sButton.Old_value = aBaudrate_value[i-1];
                                            break;
                                        }
                                    }
                                }
                                break;
                            
                            default:
                                break;
                        }
                        break;
                }
            }
            break;
          
        case _LCD_SCR_SET_CALIB_TAB_1:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 == sLCD.sScreenNow.ParaMax_u8) {
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_TAB_2, 0,
                                            __SCR_CALIB_TAB_2_OFFSET, __SCR_CALIB_TAB_2_OFFSET, __SCR_CALIB_TAB_2_RESTORE,
                                            NULL, 0xF1);
                    } else {
                        sLCD.sScreenNow.Para_u8++;
                        Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
                    }
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SCR_CALIB_TAB_1_POINT_1:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value > LEVEL_MIN)
                                    sButton.Old_value--;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SCR_CALIB_TAB_1_POINT_2:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                if(sButton.Old_value > LEVEL_MIN)
                                    sButton.Old_value--;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    default:
                      break;
                }
            }
            break;
            
        case _LCD_SCR_SET_CALIB_TAB_2:
              if(sLCD.sScreenNow.SubIndex_u8 == 0)
              {
                    if (sLCD.sScreenNow.Para_u8 < sLCD.sScreenNow.ParaMax_u8) {
                        sLCD.sScreenNow.Para_u8++;
                    }
                    Display_Set_Screen_Flag(&sLCD.sScreenNow, NULL, 0xF1);
              }
              else
              {
                switch (sLCD.sScreenNow.Para_u8)
                {
                    case __SCR_CALIB_TAB_2_OFFSET:
                        switch(sLCD.sScreenNow.SubIndex_u8)
                        {
                            case 0:
                                break;
                                
                            case 1:
                                    sButton.Old_value--;
                                break;
                            
                            default:
                                break;
                        }
                        break;
                        
                    case __SCR_CALIB_TAB_2_RESTORE:
                        break;

                    default:
                      break;
                }
            }
            break;
            
        default:
          break;
    }
}

void BUTTON_ESC_Process (void)
{
    switch (sLCD.sScreenNow.Index_u8)
    {
        case _LCD_SCREEN_1:
          break;
          
        case _LCD_SCR_PASS:
            sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
            break;
            
        case _LCD_SCR_SETTING:
            sLCD.sScreenNow.Index_u8 = _LCD_SCREEN_1;
            break;
          
        case _LCD_SCR_SET_MODBUS:   
          switch(sLCD.sScreenNow.Para_u8)
          {
                case __SET_MODBUS_ID:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                                __SCR_SET_MODBUS, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                                NULL, 0xF1);
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            break;             
                          
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_MODBUS, (sLCD.sScreenNow.SubIndex_u8-1),
                                               __SET_MODBUS_ID, __SET_MODBUS_ID, __SET_MODBUS_BR,
                                               &sParaDisplay.ID_u8, 0xF1);
                            break;
                            
                        default:
                            break;
                    }
                    break;
                
                case __SET_MODBUS_BR:
                    switch(sLCD.sScreenNow.SubIndex_u8)
                    {
                        case 0:
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                                __SCR_SET_MODBUS, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                                NULL, 0xF1);
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            break;         
                          
                        case 1:
                            UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                            Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_MODBUS, (sLCD.sScreenNow.SubIndex_u8-1),
                                               __SET_MODBUS_BR, __SET_MODBUS_ID, __SET_MODBUS_BR,
                                               &sParaDisplay.Baudrate_u32, 0xF1);
                            break;
                            
                        default:
                            break;
                    }
                    break;
                
              default:
                break;
          }
          break;
            
        case _LCD_SCR_SET_CALIB_TAB_1:
          switch(sLCD.sScreenNow.Para_u8)
          {
            case __SCR_CALIB_TAB_1_POINT_1:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                            __SCR_SET_CALIB, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_TAB_1, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SCR_CALIB_TAB_1_POINT_1, __SCR_CALIB_TAB_1_POINT_1, __SCR_CALIB_TAB_1_POINT_2,
                                           &sParaDisplay.Calib_Point1, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SCR_CALIB_TAB_1_POINT_2:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                            __SCR_SET_CALIB, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_TAB_1, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SCR_CALIB_TAB_1_POINT_2, __SCR_CALIB_TAB_1_POINT_1, __SCR_CALIB_TAB_1_POINT_2,
                                           &sParaDisplay.Calib_Point2, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
              default:
                break;
          }
          break;
          
        case _LCD_SCR_SET_CALIB_TAB_2:
          switch(sLCD.sScreenNow.Para_u8)
          {
            case __SCR_CALIB_TAB_2_OFFSET:
                switch(sLCD.sScreenNow.SubIndex_u8)
                {
                    case 0:
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                            __SCR_SET_CALIB, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                            NULL, 0xF1);
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        break;
                      
                    case 1:
                        UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                        Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SET_CALIB_TAB_2, (sLCD.sScreenNow.SubIndex_u8-1),
                                           __SCR_CALIB_TAB_2_OFFSET, __SCR_CALIB_TAB_2_OFFSET, __SCR_CALIB_TAB_2_RESTORE,
                                           &sParaDisplay.Calib_Offset, 0xF1);
                        break;
                        
                    default:
                        break;
                }
                break;
                
            case __SCR_CALIB_TAB_2_RESTORE:
                Display_Set_Screen(&sLCD.sScreenNow, _LCD_SCR_SETTING, 0,
                                    __SCR_SET_CALIB, __SCR_SET_MODBUS, __SCR_SET_INFOR,
                                    NULL, 0xF1);
                UTIL_MEM_cpy(&sLCD.sScreenBack, &sLCD.sScreenNow, sizeof(sScreenInformation));
                break;
                
              default:
                break;
          }
          break;
          
        case _LCD_SCR_SET_INFORMATION:
            UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            break;
          
        case _LCD_SCR_CHECK_SETTING:
          if(sParaDisplay.State_Setting != _STATE_SETTING_WAIT)
          {
            UTIL_MEM_cpy(&sLCD.sScreenNow, &sLCD.sScreenBack, sizeof(sScreenInformation));
            sParaDisplay.State_Setting = _STATE_SETTING_FREE;
          }
          break;
          
        default:
          break;
    }
}



