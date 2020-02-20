#include "sys.h"
#include "usart.h"	  
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
extern void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);    
u8 receive_data[256];
u16 receive_count;
u8 VbWaitforsend = 0;
extern u8 start_message[5];
extern u8 seed_message[7];
extern u8 security_message[9];
extern u8 ready_to_flash[7];
extern u8 ready_to_flash1[7];
extern u8 test_present[9];
extern u8 initial_complete,Vbmessage_send;
extern uint16_t count;
extern u8 ready_to_flash2[13];
extern u8 access1[7];
extern u8 access2[12];
extern u8 end_message[6];
extern u8 end_message1[7];
extern u8 end_message2[7];
extern u16 flash_block;
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
   int handle; 
   /* Whatever you require here. If the only file you are using is */ 
   /* standard output using printf() for debugging, no file handling */ 
   /* is required. */ 
}; 
/* FILE is typedef�� d in stdio.h. */ 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
   x = x; 
} 
#if 1
//�ض���fputc����
//printf�������ָ��fputc����fputc���������
//����ʹ�ô���1(USART1)���printf��Ϣ
int fputc(int ch, FILE *f)
{      
   //while((USART2->SR&0X40)==0);//�ȴ���һ�δ������ݷ������  
   //USART2->DR = (u8) ch;         //дDR,����1����������
   //return ch;
}
#endif
#endif 
//end
//////////////////////////////////////////////////////////////////
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���      
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��   ������ɱ�־
//bit14��   ���յ�0x0d
//bit13~0��   ���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���     
u16 Seed_27;
extern u8 send_message_wait_receive;
void USART1_IRQHandler(void)
{
   u8 res;   
#if SYSTEM_SUPPORT_OS       //���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
   OSIntEnter();    
#endif
   if(USART1->SR&(1<<5))   //���յ�����
   {    
      res=USART1->DR; 
      USART_ClearFlag(USART1,USART_FLAG_RXNE);
      receive_data[receive_count] = res;
    if(receive_count != 999)
      receive_count = receive_count + 1;
    else
      receive_count = 0;
    
    if(send_message_wait_receive == 1)
    {
       if(receive_count == ((sizeof(start_message)/sizeof(start_message[0]))))
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }
    if(send_message_wait_receive == 2)
    {
       if(receive_count == (sizeof(seed_message)/sizeof(seed_message[0]))+8)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
          Seed_27 = receive_data[(sizeof(seed_message)/sizeof(seed_message[0]))+5] * 0x100 + receive_data[(sizeof(seed_message)/sizeof(seed_message[0]))+6];
       }
    }    
    if(send_message_wait_receive == 3)
    {
       if(receive_count == (sizeof(security_message)/sizeof(security_message[0]))+7)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }
    if(send_message_wait_receive == 4)
    {
       if(receive_count == 13)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }
    if(send_message_wait_receive == 5)
    {
       if(receive_count == 15)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }
    if(send_message_wait_receive == 6)
    {
       if(receive_count == 22)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }
    if(send_message_wait_receive == 7)
    {
       if(receive_count == 18)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }
    if(send_message_wait_receive == 8)
    {
       if(receive_count == 18)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }
    if(send_message_wait_receive == 9)
    {
       if(receive_count == 20)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }   
    if(send_message_wait_receive == 10)
    {
       if(receive_count == 12)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }
     if(send_message_wait_receive == 12)
    {
       if(receive_count == 11)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }   
     if(send_message_wait_receive == 13)
    {
       if(receive_count == 22)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }
     if(send_message_wait_receive == 14)
    {
       if(receive_count == 13)
       {
           receive_count = 0;
          send_message_wait_receive = 0;
       }
    }


      if(VbWaitforsend == 0)
      {
        if((initial_complete == 0) && (Vbmessage_send == 1))
        {
          if(receive_count == (sizeof(start_message)/sizeof(start_message[0])))
          {
            receive_count = 0;
            initial_complete++;
            VbWaitforsend = 1;
            
          }
        }
        else if((initial_complete == 1)&& (Vbmessage_send == 1))
        {
          if(receive_count == (sizeof(seed_message)/sizeof(seed_message[0])))
          {
            receive_count = 0;
            initial_complete++;
            VbWaitforsend = 1;
            
          }
        }
        else if((initial_complete == 2)&& (Vbmessage_send == 1))
        {
          if(receive_count == (sizeof(security_message)/sizeof(security_message[0])))
          {
            receive_count = 0;
            initial_complete++;
            VbWaitforsend = 1;
          }
        }
        else if((initial_complete == 3)&& (Vbmessage_send == 1))
        {
          if(receive_count == (sizeof(ready_to_flash)/sizeof(ready_to_flash[0])))
          {
            receive_count = 0;
            initial_complete++;
            VbWaitforsend = 1;
          }
        }
        else if((initial_complete == 4)&& (Vbmessage_send == 1))
        {
          if(receive_count == (sizeof(ready_to_flash2)/sizeof(ready_to_flash2[0])))
          {
            receive_count = 0;
            initial_complete++;
            VbWaitforsend = 1;
          }
        }   
        else if(((initial_complete == 5) || (initial_complete == 9) )&& (Vbmessage_send == 1))
        {
          if(receive_count == (0x89))
          {
            receive_count = 0;
            //initial_complete++;
            VbWaitforsend = 1;
          }
        } 
        else if((initial_complete == 6)&& (Vbmessage_send == 1))
        {
          if(receive_count == (sizeof(test_present)/sizeof(test_present[0])))
          {
            receive_count = 0;
            initial_complete++;
            VbWaitforsend = 1;
          }
        }
        else if((initial_complete == 7)&& (Vbmessage_send == 1))
        {
          if(receive_count == (sizeof(access1)/sizeof(access1[0])))
          {
            receive_count = 0;
            initial_complete++;
            VbWaitforsend = 1;
          }
        }
        else if((initial_complete == 8)&& (Vbmessage_send == 1))
        {
          if(receive_count == (sizeof(access2)/sizeof(access2[0])))
          {
            receive_count = 0;
            initial_complete++;
            VbWaitforsend = 1;
          }
        } 
        else if((initial_complete == 10)&& (Vbmessage_send == 1))
        {
          if(receive_count == (sizeof(end_message)/sizeof(end_message[0])))
          {
            receive_count = 0;
            initial_complete++;
            VbWaitforsend = 1;
          }
        } 
        else if((initial_complete == 11)&& (Vbmessage_send == 1))
        {
          if(receive_count == (sizeof(end_message1)/sizeof(end_message1[0])))
          {
            receive_count = 0;
            initial_complete++;
            VbWaitforsend = 1;
          }
        }
        else if((initial_complete == 12)&& (Vbmessage_send == 1))
        {
          if(receive_count == (sizeof(end_message2)/sizeof(end_message2[0])))
          {
            receive_count = 0;
            initial_complete++;
            VbWaitforsend = 1;
          }
        }
      }
      else
      {
        if(initial_complete == 1)
        {
          if(receive_count == 7)
          {
            receive_count = 0;
            VbWaitforsend = 0;
            Vbmessage_send = 0;
            count = 0;
          }
        }
        else if(initial_complete == 2)
        {
          if(receive_count == 8)
          {
            receive_count = 0;
            VbWaitforsend = 0;
            Vbmessage_send = 0;
            count = 0;
          }
        }
        else if((initial_complete == 3) || (initial_complete == 5) || (initial_complete == 6)|| (initial_complete == 12)|| (initial_complete == 13) )
        {
          if(receive_count == 7)
          {
            receive_count = 0;
            VbWaitforsend = 0;
            Vbmessage_send = 0;
            count = 0;
          }
        }
        else if(initial_complete == 7)
        {
          if(receive_count == 9)
          {
            receive_count = 0;
            VbWaitforsend = 0;
            Vbmessage_send = 0;
            count = 0;
          }          
        }
        else if(initial_complete == 4)
        {
          if(receive_count == 6)
          {
            receive_count = 0;
            VbWaitforsend = 0;
            Vbmessage_send = 0;
            count = 0;
          }          
        }
        else if(initial_complete == 8)
        {
          if(receive_count == 11)
          {
            receive_count = 0;
            VbWaitforsend = 0;
            Vbmessage_send = 0;
            count = 0;
          }          
        } 
        else if(initial_complete == 9)
        {
          if((receive_count == 6) && (flash_block == 0))
          {
            receive_count = 0;
            VbWaitforsend = 0;
            Vbmessage_send = 0;
            count = 0;
          }
          else if(receive_count == 7)
          {
            receive_count = 0;
            VbWaitforsend = 0;
            Vbmessage_send = 0;
            count = 0;
          }
        }     
        else if(initial_complete == 10)
        {
          if(receive_count == 6)
          {
            receive_count = 0;
            VbWaitforsend = 0;
            Vbmessage_send = 0;
            count = 0;
          }          
        }
        else if((initial_complete == 11) )
        {
          if(receive_count == 6)
          {
            receive_count = 0;
            VbWaitforsend = 0;
            Vbmessage_send = 0;
            count = 0;
          }          
        }
      }
    }
#if SYSTEM_SUPPORT_OS    //���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
   OSIntExit();                                    
#endif
} 
#endif                               										 
//��ʼ��IO ����1
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������ 
void uart_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV@OVER8=0
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������@OVER8=0 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<0;   	//ʹ��PORTA��ʱ��  
	RCC->APB2ENR|=1<<4;  	//ʹ�ܴ���1ʱ�� 
	GPIO_Set(GPIOA,PIN9|PIN10,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PA9,PA10,���ù���,�������
 	GPIO_AF_Set(GPIOA,9,7);	//PA9,AF7
	GPIO_AF_Set(GPIOA,10,7);//PA10,AF7  	   
	//����������
 	USART1->BRR=mantissa; 	//����������	 
	USART1->CR1&=~(1<<15); 	//����OVER8=0 
	USART1->CR1|=1<<3;  	//���ڷ���ʹ�� 
#if EN_USART1_RX		  	//���ʹ���˽���
	//ʹ�ܽ����ж� 
	USART1->CR1|=1<<2;  	//���ڽ���ʹ��
	USART1->CR1|=1<<5;    	//���ջ������ǿ��ж�ʹ��	    	
	MY_NVIC_Init(3,3,USART1_IRQn,2);//��2��������ȼ� 
#endif
	USART1->CR1|=1<<13;  	//����ʹ��
}














