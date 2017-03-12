#include <project.h>

int main()
{
    int current=0;
    uint8 position=20;
    CyGlobalIntEnable;
    EZI2C_Start();
    EZI2C_EzI2CSetBuffer1(1,0,&position);
    CapSense_Start();
    CapSense_ScanAllWidgets();
    
    while(1)
    {
        if(!CapSense_IsBusy())
        {
            int b0,b1,b2,b3;
            CapSense_ProcessAllWidgets();
            b0 = CapSense_IsWidgetActive(CapSense_BUTTON0_WDGT_ID);
            b1 = CapSense_IsWidgetActive(CapSense_BUTTON1_WDGT_ID);
            b2 = CapSense_IsWidgetActive(CapSense_BUTTON2_WDGT_ID);
            b3 = CapSense_IsWidgetActive(CapSense_BUTTON3_WDGT_ID);
            
            if(b0) {current = 0; position=20;} 
            if(b1) {current = 1; position=40;} 
            if(b2) {current = 2; position=60;} 
            if(b3) {current = 3; position=80;} 
            
            if(MB0_Read()==0) Bootloadable_Load();
           
            CBLED0_Write((current==0)?0:1);
            CBLED1_Write((current==1)?0:1);
            CBLED2_Write((current==2)?0:1);
            CBLED3_Write((current==3)?0:1);
            
            CapSense_ScanAllWidgets();
        }
    }
}
