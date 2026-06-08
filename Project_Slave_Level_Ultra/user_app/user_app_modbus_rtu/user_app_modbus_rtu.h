#ifndef __USER_APP_MODBUS_RTU_
#define __USER_APP_MODBUS_RTU_

#include "user_define.h"
#define USING_CHECK_MODBUS_RTU

#define ID_DEFAULT          0x1A
#define BAUDRATE_DEFAULT    3       //9600

typedef enum
{
    _E_REGISTER_BEGIN,
    _E_REGISTER_ID,
    _E_REGISTER_BAUDRATE,
//    _E_REGISTER_DISTANCE,
    _E_REGISTER_UNIT,
    _E_REGISTER_DECIMAL_POINT,
    _E_REGISTER_VALUE,
    _E_REGISTER_ZERO_POINT,
    _E_REGISTER_RESTORE,
    
    _E_REGISTER_COMPENSATION,
    _E_REGISTER_CALIBPOINT_1,
    _E_REGISTER_CALIBPOINT_2,
    _E_REGISTER_CALIBPOINT_3,
    _E_REGISTER_CALIBPOINT_4,
    
    _E_REGISTER_MODE_CONFIG,
    _E_REGISTER_MODE_LEVEL,
    
    _E_REGISTER_4_20_MODE,
    _E_REGISTER_4_20_MIN,
    _E_REGISTER_4_20_MAX,
    _E_REGISTER_4_20_DAC,
    
    _E_REGISTER_END,
}eRegister_ModbusRTU;

typedef uint8_t (*_func_callback_handle)(sData *str_Receiv, uint16_t Pos);

typedef struct {
	int 			    idStep;
    int                 idRegister;
    int                 Length;
	_func_callback_handle	CallBack_Read;
    _func_callback_handle	CallBack_Write;
}struct_CheckList_Reg_Modbus_RTU;

typedef struct {
    uint8_t Mode_u8;
    uint8_t ID;
    uint8_t Baudrate;
}struct_Slave_ModbusRTU;

extern struct_CheckList_Reg_Modbus_RTU sCheckList_Reg_Modbus_RTU[];
extern sData   sLogData_ModbusRTU;
extern sData   sFrameData_ModbusRTU;
extern struct_Slave_ModbusRTU sSlave_ModbusRTU;
extern uint32_t aBaudrate_value[11];
/*====================Function Handle====================*/
void Save_InforSlaveModbusRTU(uint8_t Mode, uint8_t ID, uint8_t Baudrate);
void Init_InforSlaveModbusRTU(void);

void Reset_sData(sData *str);
uint8_t Modem_Check_RTU(sData *StrUartRecei);
void    Send_RS458_Sensor(uint8_t *aData, uint16_t Length_u16);

#endif
