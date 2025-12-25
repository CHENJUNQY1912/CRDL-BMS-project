#include "HighVoltageDealWith.h"
#include "Fpara_Aspcfg.h"
#include "SampleDataDealWith.h"
#include "Rn8209.h"
#include "SYSMCU_Bspcfg.h"
#include "CortrolMain.h"


uint32   LeakRp,LeakRn;
uint16_t ChannnelVoltage[4];
uint32_t AverageVoltage[5];
uint32   AverageTolVoltage;
uint16   TotalVoltage;//总压值，单位0.1V
uint32   AN1_1,AN1_2,AN0_1,AN0_2,ANC0,ANC1;
uint32   X_Data,Y_Data,UnData,U1Data,UpData,UnpData,U2Data,UppData;

uint16 relay_on_flag=0;

#define Leak_ResConst   2018 //单位K 4M+316K
uint32 Temp_Leak_Data;//临时绝缘值
uint32  Leak_Data=0;//绝缘值
uint8 Leak_Data_Count=0;
uint16 LeakTimeCounter=6000;
extern uint32 SampleTime;
extern uint8 ResetFlag;
uint8 LeakDataBuff[8];
uint32 LeakDelayTime;

//进行总压，高压绝缘值计算，需轮切光继
void HighVoltage_Main(void *pvParameters)
{
    uint16 i,k;
    
    float K_R=1;
    
    uint16 Waitingtime=300;//绝缘检测开关切换后等待时间
    uint16 Last_Leak_Data=300;
    
    Adafruit_ADS1115(0x48);
    RealData.Leak=2000;
   
    while(1)
    {            
        MCU_IOOut_BspSet(14,1);//Ctrl_ISO//测总压
        MCU_IOOut_BspSet(15,1);//Ctr2_ISO
        MCU_IOOut_BspSet(16,1);//Ctr3_ISO
        MCU_IOOut_BspSet(17,1);//Ctr4_ISO
        vTaskDelay(50/portTICK_RATE_MS);
        for(i=0;i<1;i++)
        {
            ChannnelVoltage[i]=(readADC_SingleEnded(i)&0x7fff)*61440/0x7fff;            
        }
        AN0_1=ChannnelVoltage[0];         
//        TotalVoltage=(uint32)ChannnelVoltage[0]*7163/10000; 
        TotalVoltage=(uint32)(ChannnelVoltage[0]*8930.7f/10000); 
        if(TotalVoltage<700)
        {    
            RealData.TotalVoltage=TotalVoltage-82;
        }else
        {
            RealData.TotalVoltage=TotalVoltage;
        }
        
        RealData.TotalVoltage=(uint16)((float)TotalVoltage*Fpara.totalVoltage1_K/1000.0-Fpara.totalVoltage1_B);
        
        if(RealData.TotalVoltage>8000&&RealData.TotalVoltage<12000){          
          K_R=1.4;
        }else if(RealData.TotalVoltage>12000&&RealData.TotalVoltage<16000){          
          K_R=1.8;
        }
/*        if(SampleTime<=5000&& ResetFlag==0x00)
        {
            CalData.AccVol=RealData.TotalVoltage;
        }*/
      
        for(i=2;i<4;i++)
        {
            ChannnelVoltage[i]=(readADC_SingleEnded(i)&0x7fff)*61440/0x7fff;            
        }
//        RealData.hotVoltage = (uint32)ChannnelVoltage[3]*2242/10000;
        RealData.hotVoltage = (uint32)(ChannnelVoltage[3]*4044.2f/10000);
//        RealData.precharVoltage = (uint32)ChannnelVoltage[2]*2242/10000;
        RealData.precharVoltage = (uint32)(ChannnelVoltage[2]*4044.2f/10000);
/*        RealData.precharVoltage = (uint16)TotalVoltage;*/
              
        //组电压小于50V或者绝缘检测不使能或者接收域控下发不使能或者充电枪接上，绝缘不采样默认为2000
        if(RealData.TotalVoltage<500 || Fpara.insulationEn==0x00 || relay_on_flag==1 ||CC2_flag==1)
        {
            RealData.Leak=20000;
            
            Leak_Data=0;
            Leak_Data_Count=0;
            
            continue;
        }
        
        for(k=0;k<1;k++)
        {
            MCU_IOOut_BspSet(14,0);//Ctrl_ISO  //测绝缘
            MCU_IOOut_BspSet(15,1);//Ctr2_ISO  
            MCU_IOOut_BspSet(16,0);//Ctr3_ISO  //接入PE
            MCU_IOOut_BspSet(17,1);//Ctr4_ISO
            vTaskDelay(Waitingtime/portTICK_RATE_MS); //等待1S，改为10ms（DK）
            for(i=0;i<2;i++)
            {
                ChannnelVoltage[i]=(readADC_SingleEnded(i)&0x7fff)*61440/0x7fff;                
            }
            ANC1=ChannnelVoltage[1];   
            ANC0=ChannnelVoltage[0];  
            
            
            UnData=(uint32)((float)ANC1*2500.6/5600);
            if(UnData>1570)UnData=UnData-80;
            if(UnData<160)UnData=UnData-60;
            U1Data=(uint32)((float)ANC0*5001.2/5600);
            UpData=U1Data-UnData;
            if(UpData<UnData)
            {
                MCU_IOOut_BspSet(14,0);//Ctrl_ISO  //测绝缘
                MCU_IOOut_BspSet(15,1);//Ctr2_ISO  
                MCU_IOOut_BspSet(16,0);//Ctr3_ISO  //接入PE
                MCU_IOOut_BspSet(17,1);//Ctr4_ISO
                vTaskDelay(Waitingtime/portTICK_RATE_MS); //等待1S，改为10ms（DK）
                for(i=0;i<2;i++)
                {
                    ChannnelVoltage[i]=(readADC_SingleEnded(i)&0x7fff)*61440/0x7fff;                
                }
                ANC1=ChannnelVoltage[1];   
                ANC0=ChannnelVoltage[0];  
            
                UnData=(uint32)((float)ANC1*2500.6/5600);
                if(UnData>1570)UnData=UnData-80;
                U1Data=(uint32)((float)ANC0*5001.2/5600);
                UpData=U1Data-UnData;
              
                MCU_IOOut_BspSet(14,0);//Ctrl_ISO  //测绝缘
                MCU_IOOut_BspSet(15,0);//Ctr2_ISO  //接入负端R阻值
                MCU_IOOut_BspSet(16,0);//Ctr3_ISO  //接入PE
                MCU_IOOut_BspSet(17,1);//Ctr4_ISO
                vTaskDelay(Waitingtime/portTICK_RATE_MS); //等待1S，改为10ms（DK）           
                for(i=0;i<2;i++)
                {
                    ChannnelVoltage[i]=(readADC_SingleEnded(i)&0x7fff)*61440/0x7fff;                    
                }
                AN1_2=ChannnelVoltage[1];
                AN0_2=ChannnelVoltage[0];                   
                UnpData=(uint32)((float)AN1_2*2500.6/5600);
                if(UnpData>1570)UnpData=UnpData-80;
                U2Data=(uint32)((float)AN0_2*5001.2/5600);
                UppData=U2Data-UnpData;
                if((UnData*UnpData+UpData*UnpData)!=UppData*UnData)
                {
                    LeakRp=(uint32)((float)(UnData*UppData-UpData*UnpData)*2500.6/(UnData*UnpData+UpData*UnpData-UppData*UnData));
                }
                //Temp_Leak_Data=LeakRp; 
                Temp_Leak_Data=(uint32)(LeakRp*K_R);
            }
            else{
                MCU_IOOut_BspSet(14,0);//Ctrl_ISO  //测绝缘
                MCU_IOOut_BspSet(15,1);//Ctr2_ISO  
                MCU_IOOut_BspSet(16,0);//Ctr3_ISO  //接入PE
                MCU_IOOut_BspSet(17,1);//Ctr4_ISO
                vTaskDelay(Waitingtime/portTICK_RATE_MS); //等待1S，改为10ms（DK）
                for(i=0;i<2;i++)
                {
                    ChannnelVoltage[i]=(readADC_SingleEnded(i)&0x7fff)*61440/0x7fff;                
                }
                ANC1=ChannnelVoltage[1];   
                ANC0=ChannnelVoltage[0];  
                
                UnData=(uint32)((float)ANC1*2500.6/5600);
                if(UnData<160)UnData=UnData-60;
                U1Data=(uint32)((float)ANC0*5001.2/5600);
                UpData=U1Data-UnData;
              
                MCU_IOOut_BspSet(14,0);//Ctrl_ISO  //测绝缘
                MCU_IOOut_BspSet(15,1);//Ctr2_ISO
                MCU_IOOut_BspSet(16,0);//Ctr3_ISO  //接入PE
                MCU_IOOut_BspSet(17,0);//Ctr4_ISO  //接入正端R阻值
                vTaskDelay(Waitingtime/portTICK_RATE_MS); //等待1S，改为10ms（DK）                      
                for(i=0;i<2;i++)
                {
                    ChannnelVoltage[i]=(readADC_SingleEnded(i)&0x7fff)*61440/0x7fff;                   
                }           
                AN1_2=ChannnelVoltage[1];
                AN0_2=ChannnelVoltage[0];                                                                    
                UnpData=(uint32)((float)AN1_2*2500.6/5600);
                if(UnpData<160)UnpData=UnpData-60;
                U2Data=(uint32)((float)AN0_2*5001.2/5600); 
                if(U2Data>UnpData){
                    UppData=U2Data-UnpData;
                    if((UnData*UppData+UppData*UpData)!=UpData*UnpData)
                    {                                       
                        LeakRn=(uint32)((float)((UpData*UnpData-UnData*UppData)*2500.6)/(UnData*UppData+UppData*UpData-UpData*UnpData));
                    }
                    Temp_Leak_Data=LeakRn;  
                }
            }                                             
        }
        Leak_Data+=Temp_Leak_Data;
          
        if(Leak_Data_Count++>=2){           
            RealData.Leak=Leak_Data/3; 
            Leak_Data=0;
            Leak_Data_Count=0;
        }
        RealData.Leak=(RealData.Leak>>1)+(Last_Leak_Data>>1);    
        Last_Leak_Data=RealData.Leak;
    }
}
