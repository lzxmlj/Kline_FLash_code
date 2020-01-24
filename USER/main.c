#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "sys.h"
#include "stm32f4xx_flash.h"
#define HUGE

//u32 testsram[250000] __attribute__((at(0X68000000)));//²âÊÔÓÃÊý×é
CanTxMsg TxMsg2={0x700,0,CAN_ID_STD,CAN_RTR_DATA,8,{0xAC,99,80,50,40,0,0,0}};
CanTxMsg TxMsg1={0x600,0,CAN_ID_STD,CAN_RTR_DATA,8,{0xAC,99,80,50,40,0,0,0}};
u32 delay_time;
u8 flash_is_ok;
u8 start_load_data;
int test_temp;
u8 check_sum;
u32 monitor_time;
extern u32 time_count_ms;
extern u8 complete_count,complete_count1;
extern  u16 count_canmessage1, count_canmessage2;
extern u8 loop_enable, VblistA_full;
unsigned char test_char;
u16 text_count, text_count_pre;
extern u32 time_count, test_count;
u32 time_count_pre, time_count_diff, time_count_curr, get_count;
u8 backup_type, irror_happen,switch_pre, need_update_rtc, send_data_check, run_reset, Vbfirst_time;
extern u8 CAN_need_update, test_buff[8], CAN_need_update_data[8], VbWaitforsend;
u8 Vbcan_pending, rtc_update, Roll_CAN_node;
extern u8 receive_data[20], receive_count;
extern u32 can_receive_data_utility;
extern u32 can_receive_data_cal;
extern u32 can_receive_data_rom;
u8 VcRAM_Utilty[0x800];
u8 VcRAM_Cal[0x6000];
u8 VcRAM_Rom[0x35000] __attribute__((at(0X68000008)));
u8 start_message[5] = {0x81, 0x11, 0xF1, 0x81, 0x04};
u8 seed_message[7] = {0x80, 0x11, 0xF1, 0x02, 0x27, 0x01, 0xAC };
u8 security_message[9] = {0x80, 0x11, 0xF1, 0x04, 0x27, 0x02, 0x84, 0x74, 0xA7 };
//u8 ready_to_flash[7] ={0x80, 0x11, 0xF1, 0x02, 0x10, 0x85, 0x19 };
u8 ready_to_flash[7] ={0x83, 0x11, 0xF1, 0x10, 0x85, 0x3,0x1D };

//u8 ready_to_flash1[7] ={0x80, 0x11, 0xF1, 0x02, 0x83, 0x00, 0x07};
u8 ready_to_flash2[13] ={0x80, 0x11, 0xF1, 0x08, 0x34, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0xA6 };
u8 access1[7] = {0x80,0x11,0xF1,0x02, 0x83,0x00, 0x07};
u8 access2[12] = {0x80,0x11,0xF1,0x07, 0x83,0x03, 0x00,0x00,0xFF,0x00,0xFF,0x0E};
u8 test_present[9] = {0x80, 0x11, 0xF1, 0x04, 0x38, 0xE0, 0x00, 0x00, 0x9E};
u8 Loop_message[5] = {0x80, 0x11, 0xF1, 0x84, 0x36}; 
u8 end_message[6] = {0x80, 0x11, 0xF1, 0x01, 0x37, 0xBA};
u8 end_message1[7] = {0x80, 0x11, 0xF1, 0x02, 0x31, 0x01, 0xB6};
u8 end_message2[7] = {0x80, 0x11, 0xF1, 0x02, 0x11, 0x00, 0x95};
u8 START_PROG_ENGINE[7] = {0x80,0x11,0xF1,0x02,0x31,0xFD,0xB2};
u8 START_PROG_ENGINE_CAL[7] = {0x80,0x11,0xF1,0x02,0x31,0x07,0xBC};
u8 START_PROGRAM_MODE[8] = {0x80,0x11,0xF1,0x03,0x10,0x85,0x03,0x1D};
u8 ACCESS_TIMING_PARAMETERS1[7] = {0x80,0x11,0xF1,0x02,0x83,0x00,0x07};
u8 ECU_Reset[7] = {0x80,0x11,0xF1,0x02,0x11,0x01,0x96};
u8 test_message[8] = {1,2,3,4,5,6,7,8};
u8 check_over,cal_is_over;
u32 count_flash_data;
u32 br1;
u16 txid_byte, rxid_byte, Adress_support;
u16 flash_block = 0;
u8 initial_complete = 0;
u32 baudrate, temp1;
u8 check_fail;
u32 file_result=0;
u8 output_arry[30000];
u32 data_address = 0xE00000;
u8 Vbmessage_send = 0;
u32 count;
u8 sizeofvalue, Add_ext_element, count_space;
u32 address_value;
u8 Flash_CAL = 0;
void EXIT_Config(void);
void TIM3_MS(void);
void TIM4_uS(void);
void MYRCC_DeInit(void);
void send_message_SCI1(const u8* message , u32 start_address, u16 length_Size);
void clearCANbuffer();
extern void CAN1_Config();
extern void MYRCC_DeInit();
extern void CAN_SendData(CAN_TypeDef* CANx,CanTxMsg* CanData);
static void RCC_Configuration(void);
static u8 CTR, MTA_Number, Add_ext, DAQ_number, object_ODT, element_ODT, last_ODT_number, event_channel_no;
static u32 DTO_ID, pended_message;
static u16 transmit_rate;
extern u32 can_receive_data;
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void test_reset();
void send_message_SCI(u8 *message, u16 length_Size);
void send_message_SCI1_long(const u8* message , u32 start_address, u16 length_Size);
void send_message_SCI_long(u8* message , u16 length_Size);
void send_message_SCI1_long_witchecksum(const u8* message , u32 start_address, u16 length_Size);
extern u8 loop_enable, loop_count;
extern void CAN2_Config(void);
extern void CAN1_Config(void);
void ECU_Reset_process();
void test_code();
extern u32 TIM_GetCounter(TIM_TypeDef* TIMx);
u8 Rx_CAL_A[4];
u8 Rx_CAL_B[4];
u8 Rx_ADC_float1[112];
u8 Rx_ADC_float2[4];
void RTE_INITIAL(void);
void TIM3_INITIAL(void);
void Clr_canmessage(void);
u16 STMFLASH_ReadHalfWord(u32 faddr);

void Usart_Send(u8 ch);
static u16 CalcKey(u16 Seed);
extern u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);
u8 test_cpnst;
int main(void)
 {

   u32 i,j,k,l,m;
   u8 temp_address[3];
  FSMC_SRAM_Init();
   Stm32_Clock_Init(336,8,2,7);
   CAN1_Config();
   RTE_INITIAL();   
   //uart_init(84,10400);   //´®¿Ú³õÊ¼»¯Îª115200
   
   TIM3_INITIAL();
    Roll_CAN_node = 2;

   text_count = 0;
   text_count_pre = 0;
   backup_type = 0;


   while(1)
   {
    if(loop_enable == 1)
     {

      loop_enable = 0;
      test_code();
     //CAN1_Send_Msg(0x111,test_message);
      //data_address = 0xC0B000;
       if((CAN_need_update_data[0] == 1) && (CAN_need_update_data[1] == 1)
         && (CAN_need_update_data[2] == 1) && (CAN_need_update_data[3] == 1)
         && (CAN_need_update_data[4] == 1) && (CAN_need_update_data[5] == 1)
         && (CAN_need_update_data[6] == 1) && (CAN_need_update_data[7] == 1))
      {
         start_load_data = 1;

         can_receive_data = 0;
         check_over = 0;
         Clr_canmessage();
         Flash_CAL = 1;
         count = 0;
         initial_complete = 0;
         Vbmessage_send = 0;
         flash_block = 0;
         data_address = 0xE00000;
         receive_count = 0;
         cal_is_over = 1;
         can_receive_data_utility = 0;
         can_receive_data_cal = 0;
         can_receive_data_rom = 0;
         Vbmessage_send = 0;
          flash_is_ok = 0;
         RCC->AHB1ENR|=1<<0;//Ê¹ÄÜPORTFÊ±ÖÓ 
         GPIO_Set(GPIOA,PIN9|PIN10,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
            GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_SET);
                     for( i = 0; i < 8; i++)
               test_message[i] = 1;
 

         CAN1_Send_Msg(0x111,test_message);
      }
      else
      {
         if((CAN_need_update_data[0] == 2) && (CAN_need_update_data[1] == 2)
         && (CAN_need_update_data[2] == 2) && (CAN_need_update_data[3] == 2)
         && (CAN_need_update_data[4] == 2) && (CAN_need_update_data[5] == 2)
         && (CAN_need_update_data[6] == 2) && (CAN_need_update_data[7] == 2))
      {
         start_load_data = 1;
         can_receive_data_utility = 0;
         can_receive_data_cal = 0;
         can_receive_data_rom = 0;
         can_receive_data = 0;
         check_over = 0;
         Clr_canmessage();
         Flash_CAL = 2;
         count = 0;
         initial_complete = 0;
         Vbmessage_send =0;
         flash_block = 0;
         data_address = 0xE00000;
         cal_is_over = 1;
         receive_count = 0;
         Vbmessage_send=0;
         RCC->AHB1ENR|=1<<0;//Ê¹ÄÜPORTFÊ±ÖÓ 
         GPIO_Set(GPIOA,PIN9|PIN10,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
            GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_SET);
         flash_is_ok = 0;
         for( i = 0; i < 8; i++)
         test_message[i] = 2;
         CAN1_Send_Msg(0x222,test_message);
      }
      }
      if((can_receive_data_utility == 0x800) && (check_over == 0) && (start_load_data == 1))
       {

           check_fail = 1;


        check_over = 1;
        start_load_data = 0;
              can_receive_data = 0;
         for( i = 0; i < 8; i++)
         test_message[i] = 0x55;     

         CAN1_Send_Msg(0x112,test_message);
      }
           else if((can_receive_data_cal == 0x6000) && (check_over == 1))
         {
            check_fail = 0;

           check_fail = 1;


        check_over = 2;
        start_load_data = 0;
         can_receive_data = 0;
         for( i = 0; i < 8; i++)
               test_message[i] = 0xAA;     
        //if(check_fail == 0)
         CAN1_Send_Msg(0x113,test_message);
      }
      else if((can_receive_data_rom == 0x35000) && (check_over == 2))
      {

           check_fail = 1;

        check_over = 3;
        start_load_data = 0;
         can_receive_data = 0;

         for( i = 0; i < 8; i++)
            test_message[i] = 0x22;     
        //if(check_fail == 0)
         CAN1_Send_Msg(0x114,test_message);
      }        

      if((Flash_CAL != 0) && (CAN_need_update_data[0] == 3) && (CAN_need_update_data[1] == 3)
         && (CAN_need_update_data[2] == 3) && (CAN_need_update_data[3] == 3)
         && (CAN_need_update_data[4] == 3) && (CAN_need_update_data[5] == 3)
         && (CAN_need_update_data[6] == 3) && (CAN_need_update_data[7] == 3))
      {

      if(( count != 70) && ( initial_complete == 0))
      {
         count++;

         if((count == 20) && ( initial_complete == 0))
         {

            GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_RESET);
         }
         if((count == 45)&& (initial_complete == 0))
         {
      
            GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_SET);
                  uart_init(84,10400);
         } 
      }
      if(( initial_complete != 0))
      {
         if (count != 0xFFFF)
            count++;
      }
      if(monitor_time != 0)
         monitor_time--;
      
      if((initial_complete == 0)&&(Vbmessage_send == 0)&&(count >= 70))
      {
         send_message_SCI(start_message, sizeof(start_message)/sizeof(start_message[0]));
         if(Flash_CAL == 2)
            monitor_time = 319 * 1000;
         else
            monitor_time = 39 * 1000;
      }
      else if((initial_complete == 1)&&(Vbmessage_send == 0)&&(count >= 1))
      {
         send_message_SCI(seed_message, sizeof(seed_message)/sizeof(seed_message[0]));
      }
      else if((initial_complete == 2)&&(Vbmessage_send == 0)&&(count >= 1))
      {
         send_message_SCI(security_message, sizeof(security_message)/sizeof(security_message[0]));      
      }
      else if((initial_complete == 3)&&(Vbmessage_send == 0)&&(count >= 1))
      {
         send_message_SCI(ready_to_flash, sizeof(ready_to_flash)/sizeof(ready_to_flash[0]));     
      }
      else if((initial_complete == 4)&&(Vbmessage_send == 0)&&(count >= 1000))
      {
         
         uart_init(84,38400);
         send_message_SCI(ready_to_flash2, sizeof(ready_to_flash2)/sizeof(ready_to_flash2[0]));      

      }
      else if((initial_complete == 5)&&(Vbmessage_send == 0)&&(count >= 10))
      {
         check_sum = 0;
         if(flash_block <= 15)
         {
      
            flash_block = flash_block + 1;
            temp_address[0] = (data_address & 0xFF0000) / 0x10000;
            temp_address[1] = (data_address & 0xFF00 )/ 0x100;
            temp_address[2] = (data_address & 0xFF);      
            check_sum = temp_address[0] + temp_address[1] + temp_address[2] + 0x3C;   
            send_message_SCI_long(Loop_message, sizeof(Loop_message)/sizeof(Loop_message[0]));
            send_message_SCI_long(temp_address, sizeof(temp_address)/sizeof(temp_address[0]));  
            send_message_SCI1_long(VcRAM_Utilty, (data_address & 0x0FFFFF), 0x80);
            Usart_Send(check_sum);
            data_address = data_address + 0x80;
            Vbmessage_send = 1;
         }
         else
         {
            flash_block = 0;
            data_address = 0xC05000;
            initial_complete = 6;
         }
      }
      else if((initial_complete == 6)&&(Vbmessage_send == 0)&&(count >= 1))
      {
         send_message_SCI(test_present, sizeof(test_present)/sizeof(test_present[0]));
      }
      else if((initial_complete == 7)&&(Vbmessage_send == 0)&&(count >= 11000))
      {
         //send_message_SCI(access1, sizeof(access1)/sizeof(access1[0]));
         //uart_init(84,38400);
         initial_complete = 9;
         count = 0;
         Vbmessage_send = 0;
         VbWaitforsend = 0;
         receive_count =0;
         cal_is_over = 0;
      }
      else if((initial_complete == 8)&&(Vbmessage_send == 0)&&(count >= 1))
      {
         send_message_SCI(access2, sizeof(access2)/sizeof(access2[0]));     
      }
      else if((initial_complete == 9)&&(Vbmessage_send == 0)&&(count >= 10))
      {

         check_sum = 0;
         if(cal_is_over == 0)
         {
            if((flash_block <= 191) )
            {
      
               flash_block = flash_block + 1;
               temp_address[0] = (data_address & 0xFF0000) / 0x10000;
               temp_address[1] = (data_address & 0xFF00 )/ 0x100;
               temp_address[2] = (data_address & 0xFF);    
               check_sum = temp_address[0] + temp_address[1] + temp_address[2] + 0x3C;   
               send_message_SCI_long(Loop_message, sizeof(Loop_message)/sizeof(Loop_message[0]));
               send_message_SCI_long(temp_address, sizeof(temp_address)/sizeof(temp_address[0]));   
               send_message_SCI1_long(VcRAM_Cal, ((data_address - 0xC05000 )& 0x0FFFFF), 0x80);
               Usart_Send(check_sum);
               data_address = data_address + 0x80;
               Vbmessage_send = 1;
            }
            else
            {
               flash_block = 0;
               data_address = 0xC0B000;
               cal_is_over = 1;
            }
         }
         if((cal_is_over == 1) && (Flash_CAL == 2))
         {
            if(flash_block < 1696)
            {
      
               flash_block = flash_block + 1;
               temp_address[0] = (data_address & 0xFF0000) / 0x10000;
               temp_address[1] = (data_address & 0xFF00 )/ 0x100;
               temp_address[2] = (data_address & 0xFF);    
               check_sum = temp_address[0] + temp_address[1] + temp_address[2] + 0x3C;   
               send_message_SCI_long(Loop_message, sizeof(Loop_message)/sizeof(Loop_message[0]));
               send_message_SCI_long(temp_address, sizeof(temp_address)/sizeof(temp_address[0]));   
               send_message_SCI1_long(VcRAM_Rom, ((data_address - 0xC0B000 )& 0x0FFFFF), 0x80);
               Usart_Send(check_sum);
               data_address = data_address + 0x80;
               Vbmessage_send = 1;
            }
            else
            {
               flash_block = 0;
               cal_is_over = 1;
               initial_complete = 10;
            }
         }
         else if((cal_is_over == 1)&& (Flash_CAL == 1))
         {
              flash_block = 0;
               cal_is_over = 1;
               initial_complete = 10;
         }
      
     }
      else if((initial_complete == 10)&&(Vbmessage_send == 0)&&(count >= 1))
      {
       send_message_SCI(end_message, sizeof(end_message)/sizeof(end_message[0]));    
       monitor_time = 1000;
       flash_is_ok = 1;
      }
      else if((initial_complete == 11)&&(Vbmessage_send == 0)&&(count >= 1))
      {
       send_message_SCI(end_message1, sizeof(end_message1)/sizeof(end_message1[0]));     
         monitor_time = 1000;
         flash_is_ok = 1;
      }
      else if((initial_complete == 12)&&(Vbmessage_send == 0)&&(count >= 1))
      {
       send_message_SCI(end_message2, sizeof(end_message2)/sizeof(end_message2[0]));
         flash_is_ok = 1;
         monitor_time = 1000;
      }
      if((monitor_time == 0) && (initial_complete != 13) &&(initial_complete != 0xFF) && (Vbmessage_send == 1)
         && (flash_is_ok == 0))
      {
         for(i = 0; i < 8; i++)
         {
            test_message[i] = 0xEE;
         }
         CAN1_Send_Msg(0x210,test_message);
         initial_complete = 13;
      }
      else if((monitor_time == 0) && (initial_complete >= 12) && (flash_is_ok == 1))
      {
         for(i = 0; i < 8; i++)
         {
            test_message[i] = 0xFF;
         }
         CAN1_Send_Msg(0x210,test_message);
         flash_is_ok = 22;
         initial_complete = 0xFF;
      }
   }
      }
   }
}
void send_message_SCI1_long_witchecksum(const u8* message , u32 start_address, u16 length_Size)
{
   u32 i;
u8 output;
u8 check_sum_temp = 0;
for( i = start_address; i < (start_address+length_Size-1);i++)
{
      output = message[i];
   Usart_Send(output);
   check_sum_temp += output;
} 
Usart_Send(check_sum_temp);
}

void send_message_SCI1_long(const u8* message , u32 start_address, u16 length_Size)
{
   u32 i;
  u8 output;
  for( i = start_address; i < (start_address+length_Size);i++)
  {
        output = message[i];
     Usart_Send(output);
     check_sum += output;
  } 

}
void send_message_SCI_long(u8* message , u16 length_Size)
{
   int i;

for( i = 0; i < length_Size;i++)
{
      
   Usart_Send(message[i]);
} 

}
void send_message_SCI1(const u8* message , u32 start_address, u16 length_Size)
{
   u32 i;
u8 output;
for( i = start_address; i < length_Size;i++)
{
      output = message[i];
   Usart_Send(output);
   check_sum += output;
} 
Vbmessage_send = 1;
}
void send_message_SCI(u8* message , u16 length_Size)
{
   int i;

for( i = 0; i < length_Size;i++)
{
      
   Usart_Send(message[i]);
} 
Vbmessage_send = 1;
}
void Usart_Send(u8 ch)
{
   while((USART1->SR&0X40)==0);//�ȴ����ͽ���
   USART1->DR= ch;
}
void RTE_INITIAL(void)
{

   GPIO_InitTypeDef GPIO_InitStructure; 
   NVIC_InitTypeDef NVIC_InitStructure;
   TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
   TIM_ICInitTypeDef TIM_ICInitStructure;
   
         RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE);
      TIM_DeInit(TIM2);
   TIM_TimeBaseStructure.TIM_Period = 2000 - 1 ;
   TIM_TimeBaseStructure.TIM_Prescaler = 42 - 1;//1439
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
   TIM_ClearFlag(TIM2, TIM_FLAG_Update);
   TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
   TIM_Cmd(TIM2,ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

}
void TIM3_INITIAL(void)
{

   GPIO_InitTypeDef GPIO_InitStructure; 
   NVIC_InitTypeDef NVIC_InitStructure;
   TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
   TIM_ICInitTypeDef TIM_ICInitStructure;
   
         RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE);
      TIM_DeInit(TIM3);
   TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
   TIM_TimeBaseStructure.TIM_Prescaler = 1049;//1439
   TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
   TIM_ClearFlag(TIM3, TIM_FLAG_Update);
   TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);
   TIM_Cmd(TIM3,ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}
void Clr_canmessage(void)
{
   u8 i;
   for( i = 0; i < 8; i++)
      CAN_need_update_data[i] = 0;
}
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
   return *(vu16*)faddr; 
}
u8 send_message_wait_receive;
u8 test_one_time = 0;
u16 test_loop;
u32 result_key;
extern u16 Seed_27;
u8 initital_complete = 0;
extern u8 receive_count;
u8 ECU_reset_request = 0;
void test_code()
{
     test_loop++;
  if((test_one_time == 1) &&(initital_complete == 0))
  {
    test_one_time = 1;
    initial_complete = 0;
    Vbmessage_send = 0;
    initital_complete = 1;
          GPIO_Set(GPIOA,PIN9|PIN10,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
            GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_SET);
	}
  if(test_one_time == 1)
  {
    if(( initial_complete == 0))
    {
       count++;

       if((count == 20) && ( initial_complete == 0))
       {

          GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_RESET);
       }
       if((count == 45)&& (initial_complete == 0))
       {
    
          GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_SET);
                uart_init(84,10400);
       } 
    }
    if((initial_complete == 0)&&(Vbmessage_send == 0)&&(count == 70))
    {
       receive_count = 0;
       send_message_SCI(start_message, sizeof(start_message)/sizeof(start_message[0]));
       send_message_wait_receive = 1;
       count = 71;

    }
    if ((initial_complete == 0)&&(count == 250))
    {

      receive_count = 0;
      send_message_SCI(seed_message, sizeof(seed_message)/sizeof(seed_message[0]));
      send_message_wait_receive = 2;
    }
    if ((initial_complete == 0)&&(count == 450))
    {
       receive_count = 0;

      send_message_wait_receive = 3;
      security_message[6] = CalcKey(Seed_27) / 0x100;
      security_message[7] = CalcKey(Seed_27) & 0xFF;
      send_message_SCI1_long_witchecksum(security_message, 0, sizeof(security_message)/sizeof(security_message[0]));
    }
    if ((initial_complete == 0)&&(count == 550))
    {

      send_message_wait_receive = 4;
      send_message_SCI1_long_witchecksum(START_PROG_ENGINE, 0, sizeof(START_PROG_ENGINE)/sizeof(START_PROG_ENGINE[0]));

    }   
    if ((initial_complete == 0)&&(count == 4550))
    {

      send_message_wait_receive = 5;
      send_message_SCI1_long_witchecksum(START_PROGRAM_MODE, 0, sizeof(START_PROGRAM_MODE)/sizeof(START_PROGRAM_MODE[0]));

    }
    if ((initial_complete == 0)&&(count == 4650))
    {
       uart_init(84,38400);

      send_message_wait_receive = 6;
      send_message_SCI1_long_witchecksum(START_PROG_ENGINE_CAL, 0, sizeof(START_PROG_ENGINE_CAL)/sizeof(START_PROG_ENGINE_CAL[0]));
    }
    if ((initial_complete == 0)&&(count == 5650))
    {

       receive_count = 0;
      test_one_time = 0;
      send_message_wait_receive = 7;
      send_message_SCI1_long_witchecksum(ACCESS_TIMING_PARAMETERS1, 0, sizeof(ACCESS_TIMING_PARAMETERS1)/sizeof(ACCESS_TIMING_PARAMETERS1[0]));

      count = 0;
      initital_complete = 0;
    }
  }
  ECU_Reset_process();
}
#define TOPBIT 0x8000 
#define POLYNOM_1 0x4632 
#define POLYNOM_2 0x8752 
#define BITMASK 0x0080 
#define INITIAL_REMINDER 0xFFFE 
#define MSG_LEN 2
void ECU_Reset_process()
{
  if(ECU_reset_request == 1)
  {
    count++;

    if((count == 20) && ( initial_complete == 0))
    {

      GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_RESET);
    }
    if((count == 45)&& (initial_complete == 0))
    {

      GPIO_WriteBit(GPIOA,GPIO_Pin_9,Bit_SET);
      uart_init(84,38400);
    } 
    if((count == 70) && ( initial_complete == 0))
    {

      send_message_SCI1_long_witchecksum(ECU_Reset, 0, sizeof(ECU_Reset)/sizeof(ECU_Reset[0]));
      ECU_reset_request = 0;
     count = 0;
    }  

  }
}
static u16 CalcKey(u16 Seed) 
{ 
  u8 bSeed[2]; 
  u16 remainder; 
  u8 n, i; 
  bSeed[0] = (u8)(Seed >> 8); 
  bSeed[1] = (u8)Seed; 
  remainder = INITIAL_REMINDER; 
  for (n = 0; n < MSG_LEN; n++) 
  { 
    remainder ^= ((bSeed[n]) << 8); 
    for (i = 0; i < 8; i++) 
    { 
      if (remainder & TOPBIT) 
      { 
        if(remainder & BITMASK) 
        {remainder = (remainder << 1) ^ POLYNOM_1;} 
        else 
        {
          remainder = (remainder << 1) ^ POLYNOM_2;
        } 
      } 
      else 
      {
        remainder = (remainder << 1);
      } 
    } 
  } 
  return remainder;
}

