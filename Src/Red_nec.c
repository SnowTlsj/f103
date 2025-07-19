#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "string.h"
#include <stdlib.h> 
#include <math.h>
#include "stdio.h"
#include "oled.h"

#define  RX_DBG_EN   0

#define  RX_SEQ_NUM  33

#if RX_DBG_EN
#define RX_DBG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define RX_DBG(format, ...) ;
#endif

static uint8_t  tim_udt_cnt      = 0;
static uint8_t  cap_pol          = 0; 
static uint8_t  cap_pulse_cnt    = 0;
static uint8_t  sta_idle         = 1;
static uint8_t  cap_frame        = 0;

static uint16_t rx_frame[RX_SEQ_NUM*2] = {0}; 

struct {
    uint16_t  src_data[RX_SEQ_NUM*2];
    uint16_t  repet_cnt;
    union{
        uint32_t rev;
        struct
        {
            uint32_t key_val_n:8;
            uint32_t key_val  :8;
            uint32_t addr_n   :8;
            uint32_t addr     :8;
        }_rev;
    }data;
}rx;

uint8_t appro(int num1,int num2)
{
    return (abs(num1-num2) < 300);
}

void rx_rcv_init(void)
{
    cap_frame     = 0;                                       //δ����������
    sta_idle      = 0;                                       //�ǿ���״̬
    tim_udt_cnt   = 0;                                       //��ʱ�������0
    cap_pulse_cnt = 0;                                       //���񵽵ļ�����0
    
    memset(rx_frame,0x00,sizeof(rx_frame));
}

/* ����жϻص����� */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{   
	if(TIM1 == htim->Instance)
	{
        if(sta_idle)                                          //����״̬�������κδ���
        {
            return;
        }
        
        tim_udt_cnt++;                                         //���һ��
        if(tim_udt_cnt == 3)                                   //���3��
        {
            tim_udt_cnt = 0;                                   //�����������
            sta_idle    = 1;                                   //����Ϊ����״̬
            cap_frame   = 1;                                   //��ǲ����µ�����
        }
	} 
}


/* ��ƽ�����жϻص� */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    static uint16_t tmp_cnt_l,tmp_cnt_h;
	if(TIM1 == htim->Instance)
	{
        switch(cap_pol)                                                                                         //���ݼ��Ա�־λ�жϲ����ǵ͵�ƽ���Ǹߵ�ƽ
        {   
            /* �����½��� */
            case 0:
                tmp_cnt_l = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);                                    //��¼��ǰʱ��
                TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1);                                               //��λ��������
                TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);                //�ı伫��
                
                cap_pol = 1;                                                                                    //���Ա�־λ��Ϊ������
                
                if(sta_idle)                                                                                    //�����ǰΪ����״̬�����в��񵽵�ʱ��Ϊ��һ���½���
                {
                    rx_rcv_init();
                    break;                                                                                      //����
                }
                rx_frame[cap_pulse_cnt] = tim_udt_cnt * 10000 + tmp_cnt_l - tmp_cnt_h;                          //���ϴβ���ļ�ʱ�����¼ֵ
                tim_udt_cnt = 0;                                                                                //���������0
                RX_DBG("(%2d)%4d us:H\r\n",cap_pulse_cnt,rx_frame[cap_pulse_cnt]);                              //DBG����ӡ���񵽵ĵ�ƽ����ʱ��
                cap_pulse_cnt++;                                                                                //����++
                break;
            
            /* ���������� */
            case 1:
                tmp_cnt_h = HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
                TIM_RESET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1);               
                TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
                
                cap_pol = 0;   
                if(sta_idle)
                {
                    rx_rcv_init();
                    break;
                }
                rx_frame[cap_pulse_cnt] = tim_udt_cnt * 10000 + tmp_cnt_h - tmp_cnt_l;
                tim_udt_cnt = 0;
                RX_DBG("(%2d)%4d us:L\r\n",cap_pulse_cnt,rx_frame[cap_pulse_cnt]);
                cap_pulse_cnt++;
                break;
            
            default:
                break;
        }
    }
}


/*************************************  API Layer **************************************************/
void hx1838_cap_start(void)
{
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
}

uint8_t hx1838_data_decode(void)
{
    memcpy(rx.src_data,rx_frame,RX_SEQ_NUM*4);
    memset(rx_frame,0x00,RX_SEQ_NUM*4);   
    RX_DBG("========= rx.src[] =================\r\n");
    for(uint8_t i = 0;i<=(RX_SEQ_NUM*2);i++)
    {
        RX_DBG("[%d]%d\r\n",i,rx.src_data[i]);
    }
    RX_DBG("========= rx.rec =================\r\n");
    if(appro(rx.src_data[0],9000) && appro(rx.src_data[1],4500))                 //#1. ���ǰ����
    {
        uint8_t tmp_idx = 0;
        rx.repet_cnt  = 0;                                                       //�����ظ�������0
        for(uint8_t i = 2;i<(RX_SEQ_NUM*2);i++)                                  //#2. �������
        {
            if(!appro(rx.src_data[i],560))
            {
                RX_DBG("%d,err:%d != 560\r\n",i,rx.src_data[i]);
                return 0;
            }
            i++;
            if(appro(rx.src_data[i],1680))
            {
                rx.data.rev |= (0x80000000 >> tmp_idx);                          //�� tmp_idx Ϊ��1
                tmp_idx++;
            }
            else if(appro(rx.src_data[i],560))
            {
                rx.data.rev &= ~(0x80000000 >> tmp_idx);                         //�� tmp_idx λ��0
                tmp_idx++;
            }
            else
            {
                RX_DBG("%d,err:%d != 560||1680\r\n",i,rx.src_data[i+1]);
                return 0;
            }
        }
    }
    else if(appro(rx.src_data[0],9000) && appro(rx.src_data[1],2250) && appro(rx.src_data[2],560))
    {
        rx.repet_cnt++;
        return 2;
    }
    else
    {
        RX_DBG("ǰ���������\r\n");
        return 0;
    }
    return 1;
}

void hx1838_proc(uint8_t res)
{
    if(res == 0)
    {    
        return;
    }
    
    if(res == 2)
    {   
        return;
    }      
    switch(rx.data._rev.key_val)
    {
        case 162:
				   OLED_ShowNum(48,6,1,3,16);//��ʾ1
            break;
        
        case 98:
           OLED_ShowNum(48,6,2,3,16);//��ʾ2
            break;
        
        case 226:
            OLED_ShowNum(48,6,3,3,16);//��ʾ3
            break;
        
        case 34:
            OLED_ShowNum(48,6,4,3,16);//��ʾ4
            break;
        
        case 2:
            OLED_ShowNum(48,6,5,3,16);//��ʾ5
            break;
        
        case 194:
            OLED_ShowNum(48,6,6,3,16);//��ʾ6
            break;
        
        case 224:
           OLED_ShowNum(48,6,7,3,16);//��ʾ7
            break;
        
        case 168:
            OLED_ShowNum(48,6,8,3,16);//��ʾ8
            break;
        
        case 144:
            OLED_ShowNum(48,6,9,3,16);//��ʾ9
            break;
        
        case 152:
            OLED_ShowNum(48,6,0,3,16);//��ʾ0
            break;
        
        case 104:
					  OLED_ShowChar(48,6,'*');//��ʾASCII�ַ�	  
            break;
        
        case 176:
						OLED_ShowChar(48,6,'#');//��ʾASCII�ַ�	 
            break;
                
        case 24:
        
						OLED_ShowChar(48,6,'^');//��ʾASCII�ַ�	 
            break;
                
        case 16:
				    OLED_ShowChar(48,6,'<');//��ʾASCII�ַ�	
            break;
        
        case 74:
    
				 OLED_ShowChar(48,6,'v');//��ʾASCII�ַ�	
			
            break;
        
        case 90:
				OLED_ShowChar(48,6,'>');//��ʾASCII�ַ�	
            break;
        
        case 56:
        OLED_ShowString(48,6,"OK"); 
            break;
        
        default:
            
            break;
        
    }
}

void HX1838_demo(void)
{
    hx1838_cap_start();//��ʱ��1ͨ��1�����벶��
    while(1)
    {
        if(cap_frame)//��ǲ����µ�����
        {   
            hx1838_proc(hx1838_data_decode());//��������
            cap_frame = 0;
						
		
        }
    }
    
}

