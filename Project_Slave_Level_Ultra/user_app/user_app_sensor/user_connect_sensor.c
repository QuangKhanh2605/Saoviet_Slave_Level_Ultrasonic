#include "user_connect_sensor.h"
#include "user_define.h"

static uint8_t aUART_232_DATA [512];
sData   sUart232 = {(uint8_t *) &aUART_232_DATA[0], 0};

static uint16_t OldPosDMA232 = 0;
static uint8_t aUART_RS232_DMA_RX [100];
static uint8_t Uart232RxByte = 0;

/*--- variable recv or pending*/
uint8_t Rs232Status_u8 = false;

/*=========================== Function ===========================*/
void RS232_Init_Data (void)
{
    Reset_Buff(&sUart232);
    Rs232Status_u8  = PENDING;
}

void RS232_Init_Uart (void)
{
    MX_UART_RS232_Init();
}

void RS232_Init_RX_Mode(void)
{
#if (UART_RS232_MODE == UART_RS232_IT)
    //    __HAL_UART_ENABLE_IT(&uart_rs485, UART_IT_RXNE);
    HAL_UART_Receive_IT(&uart_rs232, &Uart232RxByte, 1);
#else
    HAL_UARTEx_ReceiveToIdle_DMA(&uart_rs232, aUART_RS232_DMA_RX, sizeof(aUART_RS232_DMA_RX));
    __HAL_DMA_DISABLE_IT(&uart_dma_rs232, DMA_IT_HT); 
    OldPosDMA232 = 0;
#endif
    
    HAL_NVIC_SetPriority(USART_RS232_IRQn, USART_RS232_Priority, 0);
    HAL_NVIC_EnableIRQ(USART_RS232_IRQn);
}

void RS232_Stop_RX_Mode (void)
{
#if (UART_RS232_MODE == UART_RS232_IT)
//    __HAL_UART_DISABLE_IT(&uart_rs232, UART_IT_RXNE);   
#else
    HAL_UART_DMAStop(&uart_rs232);
#endif
}

void RS232_Rx_Callback (uint16_t Size)
{
#if (UART_RS232_MODE == UART_RS232_IT)
    *(sUart232.Data_a8 + sUart232.Length_u16++) = Uart232RxByte;

    if (sUart232.Length_u16 >= (sizeof(aUART_232_DATA) - 1))
        sUart232.Length_u16 = 0;

    HAL_UART_Receive_IT(&uart_rs232, &Uart232RxByte, 1);

    Rs232Status_u8 = TRUE;
    
    fevent_enable(sEventAppSensor, _EVENT_SENSOR_HANDLE);
#else
    uint16_t NbByteRec = 0, i = 0;
    uint16_t OldPosData = OldPosDMA232;
       
    if (uart_dma_rs232.Init.Mode == DMA_NORMAL)   //sua o day
    {
        if ( (sUart232.Length_u16 + Size) >= sizeof(aUART_232_DATA) )
                sUart232.Length_u16 = 0;
        
        for (i = 0; i < Size; i++)
            *(sUart232.Data_a8 + sUart232.Length_u16 + i) = aUART_RS232_DMA_RX[i]; 

        sUart232.Length_u16 += Size ;
    } else
    {
        //Chia 2 truong hop
        if (Size > OldPosData)
        {
            NbByteRec = Size - OldPosData;
            
            //Kiem tra length cua buff nhan
            if ( (sUart232.Length_u16 + NbByteRec) >= sizeof(aUART_232_DATA) )
                sUart232.Length_u16 = 0;
        
            for (i = 0; i < NbByteRec; i++)
                *(sUart232.Data_a8 + sUart232.Length_u16 + i) = aUART_RS232_DMA_RX[OldPosData + i]; 
            
            sUart232.Length_u16 += NbByteRec;
        } else
        {
            NbByteRec = sizeof(aUART_RS232_DMA_RX) - OldPosData;

            //Kiem tra length cua buff nhan
            if ( (sUart232.Length_u16 + Size + NbByteRec) >= sizeof(aUART_232_DATA) )
                sUart232.Length_u16 = 0;
            
            for (i = 0; i < NbByteRec; i++)
                *(sUart232.Data_a8 + sUart232.Length_u16 + i) = aUART_RS232_DMA_RX[OldPosData + i]; 
            
            for (i = 0; i < Size; i++)
                *(sUart232.Data_a8 + sUart232.Length_u16 + i) = aUART_RS232_DMA_RX[i]; 
            
            sUart232.Length_u16 += (Size + NbByteRec) ;
        }
        
        //Cap nhat lai Old Position
        OldPosDMA232 = Size;
    }
          
    if (uart_dma_rs232.Init.Mode == DMA_NORMAL)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&uart_rs232, aUART_RS232_DMA_RX, sizeof(aUART_RS232_DMA_RX));
        __HAL_DMA_DISABLE_IT(&uart_dma_rs232, DMA_IT_HT);
    }
        
    Rs232Status_u8 = TRUE;
#endif
}


