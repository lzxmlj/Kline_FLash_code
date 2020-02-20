/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
 
u8 VblistA_full;
u16 count_canmessage;

u16 count_canmessage1, count_canmessage2;
u32 time_count, time_count_ms, time_count_us,time_count_sec;
u32 time_count1, test_count, Keep_count;
u8 time_err;
u8 test_buff[8], CAN_need_update_data[8], temp_data[8];
u8 needupdate, CAN_need_update = 1;
u32 down_edge_time, up_edge_time, down_edge_count, up_edge_count, temp_id;
u8 loop_enable,time3_count;
u32 can_receive_data_utility;
u32 can_receive_data_cal;
u32 can_receive_data_rom;
u32 can_receive_data;
extern u8 check_over,start_load_data;
extern u8 VcRAM_Utilty[0x800];
extern u8 VcRAM_Cal[0x6000];
extern u8 VcRAM_Rom[0x35000];
/** @addtogroup Template_Project
  * @{
  */
u8 UpdateforASCII_H(u8 number);
u8 UpdateforASCII_L(u8 number);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern u32 ntime;
void SysTick_Handler(void)
{
	ntime--;
}

void CAN1_TX_IRQHandler(void)
{
}
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
	u32 i;
    RxMessage.StdId=0x00;
    RxMessage.ExtId=0x00;
	  RxMessage.RTR=CAN_RTR_DATA;  // ����֡orԶ��
    RxMessage.IDE=CAN_ID_STD;	 // ��׼or��չ
    RxMessage.DLC=0;
    RxMessage.FMI=0;
    RxMessage.Data[0]=0x00;
    RxMessage.Data[1]=0x00;

	for(i =0; i < 8; i++)
	{
		RxMessage.Data[i]=0x00;
	}
	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);	// CAN�������� 
	temp_id = RxMessage.StdId;
  if(RxMessage.StdId == 0x599)
  {
    for(i =0; i < 8; i++)
    {
        VcRAM_Cal[can_receive_data_cal] = RxMessage.Data[i];
        can_receive_data_cal++;
    }
  }
  else if(RxMessage.StdId == 0x600)
  {
    for(i =0; i < 8; i++)
    {
      VcRAM_Utilty[can_receive_data_utility] = RxMessage.Data[i];
      can_receive_data_utility++;
    }
  }
  else if(RxMessage.StdId == 0x598)
  {
    for(i =0; i < 8; i++)
    {
      VcRAM_Rom[can_receive_data_rom] = RxMessage.Data[i];
      can_receive_data_rom++;
    }
  }
  else
  {
    for(i =0; i < 8; i++)
    {
      CAN_need_update_data[i] = RxMessage.Data[i];
    }
  }

}

/***********************
 CAN2 FIFO1
***********************/
void CAN2_RX1_IRQHandler(void)
{
    CanRxMsg RxMessage;
   u8 i, j;
   u32 time_count_temp;
   RxMessage.StdId=0x00;
    RxMessage.ExtId=0x00;
	RxMessage.RTR=CAN_RTR_DATA;  // ����֡orԶ��
    RxMessage.IDE=CAN_ID_STD;	 // ��׼or��չ
    RxMessage.DLC=0;
    RxMessage.FMI=0;
    RxMessage.Data[0]=0x00;
    RxMessage.Data[1]=0x00;

    CAN_Receive(CAN2,CAN_FIFO1, &RxMessage);	 // CAN��������	
    
    
}
u8 UpdateforASCII_H(u8 number)
{
   if(((number>> 4) >= 10) && ((number >> 4) <= 15) )
      return  (number >> 4) + 55;
   else
      return (number >> 4) + 48;
}
u8 UpdateforASCII_L(u8 number)
{
   if(((number & 0xF) >= 10) && ((number & 0xF) <= 15) )
      return  (number & 0xF) + 55;
   else
      return (number & 0xF) + 48;
}

/***********************
 CAN2 FIFO    �ݲ�ʹ��
***********************/								   								  
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
	u32 i;
    RxMessage.StdId=0x00;
    RxMessage.ExtId=0x00;
	  RxMessage.RTR=CAN_RTR_DATA;  // ����֡orԶ��
    RxMessage.IDE=CAN_ID_STD;	 // ��׼or��չ
    RxMessage.DLC=0;
    RxMessage.FMI=0;
    RxMessage.Data[0]=0x00;
    RxMessage.Data[1]=0x00;

	for(i =0; i < 8; i++)
	{
		RxMessage.Data[i]=0x00;
	}
	CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);	// CAN�������� 
	temp_id = RxMessage.StdId;


}						 								



/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;

    RxMessage.StdId=0x00;
    RxMessage.ExtId=0x00;
	RxMessage.RTR=CAN_RTR_DATA;  // ����֡orԶ��
    RxMessage.IDE=CAN_ID_STD;	 // ��׼or��չ
    RxMessage.DLC=0;
    RxMessage.FMI=0;
    RxMessage.Data[0]=0x00;
    RxMessage.Data[1]=0x00;

    CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);	 // CAN��������		
    //CAN_Receive(CAN2,CAN_FIFO1, &RxMessage);	 // CAN��������		
	//Comm_Send_CANmsg_str(1,&RxMessage);
}
void TIM2_IRQHandler(void)
{
   u16 i;
   if(TIM_GetITStatus(TIM2,TIM_IT_CC1) != RESET)
   {
     TIM_ClearITPendingBit(TIM2,TIM_IT_CC1);
     loop_enable = 1;
     TIM_ClearFlag(TIM2, TIM_FLAG_Update);
   }
}
void TIM3_IRQHandler(void)
{
   u16 i;
   if(TIM_GetITStatus(TIM3,TIM_IT_CC1) != RESET)
   {
     TIM_ClearITPendingBit(TIM3,TIM_IT_CC1);
     time3_count = time3_count + 1;
     TIM_ClearFlag(TIM3, TIM_FLAG_Update);
   }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
