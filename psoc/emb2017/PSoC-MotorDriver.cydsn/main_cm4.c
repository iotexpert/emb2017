#include "project.h"

int convertPercentToCompare(uint8_t pos)
{
    return 3*(800 + pos * 10);   
}

struct mposition_t {
    uint8_t m1;
    uint8_t m2;
} __attribute__((packed));

int main(void)
{
    struct mposition_t mposition;
    mposition.m1 = 50;
    mposition.m2 = 50;
    
    __enable_irq(); /* Enable global interrupts. */
    
    EZI2C_Start();
    EZI2C_SetBuffer1((uint8_t *)&mposition,1,2);
   
    PWM_1_Start();
    PWM_2_Start();
    
    PWM_1_SetCompare0(convertPercentToCompare(mposition.m1));
    PWM_2_SetCompare0(convertPercentToCompare(mposition.m1));
    for(;;)
    {      
        if(EZI2C_GetActivity() == CY_SCB_EZI2C_STATUS_WRITE1)
        {
            PWM_1_SetCompare0(convertPercentToCompare(mposition.m1));
            PWM_2_SetCompare0(convertPercentToCompare(mposition.m2));
        }
    }
}
