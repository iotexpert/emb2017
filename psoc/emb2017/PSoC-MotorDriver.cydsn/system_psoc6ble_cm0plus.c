/***************************************************************************//**
* \file system_psoc6ble_cm0plus.c
* \version 1.0
*
* The device system-source file.
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "system_psoc6ble.h"
#include "cy_device_headers.h"


/*******************************************************************************
* SystemCoreClockUpdate()
*******************************************************************************/

/** Default HFClk frequency in Hz */
#define CY_CLK_HFCLK0_FREQ_HZ_DEFAULT       ( 8000000UL)

/** Default PeriClk frequency in Hz */
#define CY_CLK_PERICLK_FREQ_HZ_DEFAULT      (4000000UL)

/** Default SlowClk system core frequency in Hz */
#define CY_CLK_SYSTEM_FREQ_HZ_DEFAULT       (4000000UL)

/** IMO frequency in Hz */
#define CY_CLK_IMO_FREQ_HZ                  (8000000UL)


/** Holds the SlowClk system core clock, which is the system clock frequency supplied to the SysTick timer and the
* processor core clock. This variable can be used by debuggers to query the frequency of the debug timer or to configure
* the trace clock speed.
*
* \attention Compilers must be configured to avoid removing this variable in case the application program is not using
* it. Debugging systems require the variable to be physically present in memory so that it can be examined to configure
* the debugger. */
uint32_t SystemCoreClock = CY_CLK_SYSTEM_FREQ_HZ_DEFAULT;

/** Holds the HFClk0 clock frequency. Updated by \ref SystemCoreClockUpdate(). */
uint32_t cy_Hfclk0FreqHz  = CY_CLK_HFCLK0_FREQ_HZ_DEFAULT;

/** Holds the PeriClk clock frequency. Updated by \ref SystemCoreClockUpdate(). */
uint32_t cy_PeriClkFreqHz = CY_CLK_PERICLK_FREQ_HZ_DEFAULT;

/** Holds the Alternate high frequency clock in Hz. Updated by \ref SystemCoreClockUpdate(). */
#if defined (CY_IP_MXBLESS) && (CY_IP_MXBLESS == 1UL)
    uint32_t cy_BleEcoClockFreqHz = CY_CLK_ALTHF_FREQ_HZ;
#endif /* defined (CY_IP_MXBLESS) && (CY_IP_MXBLESS == 1UL) */


/*******************************************************************************
* SystemInit()
*******************************************************************************/
/* WDT lock bits */
#define CY_WDT_LOCK_BIT0                ((uint32_t)0x01u << 30u)
#define CY_WDT_LOCK_BIT1                ((uint32_t)0x01u << 31u)

/* CLK_FLL_CONFIG default values, from MXS40-IP-SRSS */
#define CY_FB_CLK_FLL_CONFIG_VALUE      (0x01000000u)
#define CY_FB_CLK_FLL_CONFIG2_VALUE     (0x00020001u)
#define CY_FB_CLK_FLL_CONFIG3_VALUE     (0x00002800u)
#define CY_FB_CLK_FLL_CONFIG4_VALUE     (0x000000FFu)


/*******************************************************************************
* SystemCoreClockUpdate (void)
*******************************************************************************/
/* Do not use these definitions directly in your application */
#define CY_DELAY_MS_OVERFLOW_THRESHOLD  (0x8000u)
#define CY_DELAY_1K_THRESHOLD           (1000u)
#define CY_DELAY_1K_MINUS_1_THRESHOLD   (CY_DELAY_1K_THRESHOLD - 1u)
#define CY_DELAY_1M_THRESHOLD           (1000000u)
#define CY_DELAY_1M_MINUS_1_THRESHOLD   (CY_DELAY_1M_THRESHOLD - 1u)
uint32_t cy_delayFreqHz   = CY_CLK_SYSTEM_FREQ_HZ_DEFAULT;

uint32_t cy_delayFreqKhz  = (CY_CLK_SYSTEM_FREQ_HZ_DEFAULT + CY_DELAY_1K_MINUS_1_THRESHOLD) /
                            CY_DELAY_1K_THRESHOLD;

uint8_t cy_delayFreqMhz  = (uint8_t)((CY_CLK_SYSTEM_FREQ_HZ_DEFAULT + CY_DELAY_1M_MINUS_1_THRESHOLD) /
                            CY_DELAY_1M_THRESHOLD);

uint32_t cy_delay32kMs    = CY_DELAY_MS_OVERFLOW_THRESHOLD *
                            ((CY_CLK_SYSTEM_FREQ_HZ_DEFAULT + CY_DELAY_1K_MINUS_1_THRESHOLD) / CY_DELAY_1K_THRESHOLD);

#define CY_ROOT_PATH_SRC_IMO            (0UL)
#define CY_ROOT_PATH_SRC_EXT            (1UL)
#define CY_ROOT_PATH_SRC_ECO            (2UL)
#define CY_ROOT_PATH_SRC_ALTHF          (3UL)


/*******************************************************************************
* Function Name: SystemInit
****************************************************************************//**
*
* Initializes the system:
* - Restores FLL registers to the default state.
* - Unlocks and disables WDT.
* - Calls the Cy_SystemInit() function, if compiled from PSoC Creator.
* - Calls \ref SystemCoreClockUpdate().
*
*******************************************************************************/
void SystemInit(void)
{
    /* Restore FLL registers to the default state as they are not restored by the ROM code */
    uint32_t copy = SRSS->CLK_FLL_CONFIG;
    copy &= ~SRSS_CLK_FLL_CONFIG_FLL_ENABLE_Msk;
    SRSS->CLK_FLL_CONFIG = copy;

    copy = SRSS->CLK_ROOT_SELECT[0u];
    copy &= ~SRSS_CLK_ROOT_SELECT_ROOT_DIV_Msk; /* Set ROOT_DIV = 0*/
    SRSS->CLK_ROOT_SELECT[0u] = copy;

    SRSS->CLK_FLL_CONFIG  = CY_FB_CLK_FLL_CONFIG_VALUE;
    SRSS->CLK_FLL_CONFIG2 = CY_FB_CLK_FLL_CONFIG2_VALUE;
    SRSS->CLK_FLL_CONFIG3 = CY_FB_CLK_FLL_CONFIG3_VALUE;
    SRSS->CLK_FLL_CONFIG4 = CY_FB_CLK_FLL_CONFIG4_VALUE;


    /* Unlock and disable WDT */
    SRSS->WDT_CTL = ((SRSS->WDT_CTL & (uint32_t)(~SRSS_WDT_CTL_WDT_LOCK_Msk)) | CY_WDT_LOCK_BIT0);
    SRSS->WDT_CTL = (SRSS->WDT_CTL | CY_WDT_LOCK_BIT1);
    SRSS->WDT_CTL &= (~ (uint32_t) SRSS_WDT_CTL_WDT_EN_Msk);


    Cy_SystemInit();
    SystemCoreClockUpdate();
    Cy_IPC_SystemPipeInit();
}


/*******************************************************************************
* Function Name: Cy_SystemInit
****************************************************************************//**
*
* The function is called during device startup. Once project compiled as part of
* the PSoC Creator project, the Cy_SystemInit() function is generated by the
* PSoC Creator.
*
* The function generated by PSoC Creator performs all of the necessary device
* configuration based on the design settings.  This includes settings from the
* Design Wide Resources (DWR) such as Clocks and Pins as well as any component
* configuration that is necessary.
*
*******************************************************************************/
__WEAK void Cy_SystemInit(void)
{
     /* Empty weak function. The actual implementation to be in the PSoC Creator
      * generated strong function.
     */
}


/*******************************************************************************
* Function Name: SystemCoreClockUpdate
****************************************************************************//**
*
* Gets core clock frequency and updates \ref SystemCoreClock, \ref
* cy_Hfclk0FreqHz, and \ref cy_PeriClkFreqHz.
*
* Updates global variables used by the \ref Cy_SysLib_Delay(), \ref
* Cy_SysLib_DelayUs(), and \ref Cy_SysLib_DelayCycles().
*
*******************************************************************************/
void SystemCoreClockUpdate (void)
{
    uint32_t srcFreqHz;
    uint32_t pathFreqHz;
    uint32_t slowClkDiv;
    uint32_t periClkDiv;
    uint32_t rootPath;
    uint32_t srcClk;

    /* Get root path clock for the high-frequency clock # 0 */
    rootPath = _FLD2VAL(SRSS_CLK_ROOT_SELECT_ROOT_MUX, SRSS->CLK_ROOT_SELECT[0u]);

    /* Get source of the root path clock */
    srcClk = _FLD2VAL(SRSS_CLK_PATH_SELECT_PATH_MUX, SRSS->CLK_PATH_SELECT[rootPath]);

    /* Get frequency of the source */
    switch (srcClk)
    {
    case CY_ROOT_PATH_SRC_IMO:
        srcFreqHz = CY_CLK_IMO_FREQ_HZ;
    break;

    case CY_ROOT_PATH_SRC_EXT:
        srcFreqHz = CY_CLK_EXT_FREQ_HZ;
    break;

    case CY_ROOT_PATH_SRC_ECO:
        srcFreqHz = CY_CLK_ECO_FREQ_HZ;
    break;

#if defined (CY_IP_MXBLESS) && (CY_IP_MXBLESS == 1UL)
    case CY_ROOT_PATH_SRC_ALTHF:
        srcFreqHz = cy_BleEcoClockFreqHz;
    break;
#endif /* defined (CY_IP_MXBLESS) && (CY_IP_MXBLESS == 1UL) */

    default:
        srcFreqHz = CY_CLK_EXT_FREQ_HZ;
    break;
    }

    if (rootPath == 0UL)
    {
        /* FLL */
        bool fllLocked       = ( 0UL != _FLD2VAL(SRSS_CLK_FLL_STATUS_LOCKED, SRSS->CLK_FLL_STATUS));
        bool fllOutputOutput = ( 3UL == _FLD2VAL(SRSS_CLK_FLL_CONFIG3_BYPASS_SEL, SRSS->CLK_FLL_CONFIG3));
        bool fllOutputAuto   = ((0UL == _FLD2VAL(SRSS_CLK_FLL_CONFIG3_BYPASS_SEL, SRSS->CLK_FLL_CONFIG3)) ||
                                (1UL == _FLD2VAL(SRSS_CLK_FLL_CONFIG3_BYPASS_SEL, SRSS->CLK_FLL_CONFIG3)));
        if ((fllOutputAuto && fllLocked) || fllOutputOutput)
        {
            uint32_t fllMult;
            uint32_t refDiv;
            uint32_t outputDiv;

            fllMult = _FLD2VAL(SRSS_CLK_FLL_CONFIG_FLL_MULT, SRSS->CLK_FLL_CONFIG);
            refDiv  = _FLD2VAL(SRSS_CLK_FLL_CONFIG2_FLL_REF_DIV, SRSS->CLK_FLL_CONFIG2);
            outputDiv = _FLD2VAL(SRSS_CLK_FLL_CONFIG_FLL_OUTPUT_DIV, SRSS->CLK_FLL_CONFIG) + 1UL;

            pathFreqHz = ((srcFreqHz / refDiv) * fllMult) / outputDiv;
        }
        else
        {
            pathFreqHz = srcFreqHz;
        }
    }
    else if (rootPath == 1UL)
    {
        /* PLL */
        bool pllLocked       = ( 0UL != _FLD2VAL(SRSS_CLK_PLL_STATUS_LOCKED,     SRSS->CLK_PLL_STATUS[0UL]));
        bool pllOutputOutput = ( 3UL == _FLD2VAL(SRSS_CLK_PLL_CONFIG_BYPASS_SEL, SRSS->CLK_PLL_CONFIG[0UL]));
        bool pllOutputAuto   = ((0UL == _FLD2VAL(SRSS_CLK_PLL_CONFIG_BYPASS_SEL, SRSS->CLK_PLL_CONFIG[0UL])) ||
                                (1UL == _FLD2VAL(SRSS_CLK_PLL_CONFIG_BYPASS_SEL, SRSS->CLK_PLL_CONFIG[0UL])));
        if ((pllOutputAuto && pllLocked) || pllOutputOutput)
        {
            uint32_t feedbackDiv;
            uint32_t referenceDiv;
            uint32_t outputDiv;

            feedbackDiv  = _FLD2VAL(SRSS_CLK_PLL_CONFIG_FEEDBACK_DIV,  SRSS->CLK_PLL_CONFIG[0UL]);
            referenceDiv = _FLD2VAL(SRSS_CLK_PLL_CONFIG_REFERENCE_DIV, SRSS->CLK_PLL_CONFIG[0UL]);
            outputDiv    = _FLD2VAL(SRSS_CLK_PLL_CONFIG_OUTPUT_DIV,    SRSS->CLK_PLL_CONFIG[0UL]);

            pathFreqHz = ((srcFreqHz * feedbackDiv) / referenceDiv) / outputDiv;

        }
        else
        {
            pathFreqHz = srcFreqHz;
        }
    }
    else
    {
        /* Direct */
        pathFreqHz = srcFreqHz;
    }

    /* Get frequency after hf_clk pre-divider */
    pathFreqHz = pathFreqHz >> _FLD2VAL(SRSS_CLK_ROOT_SELECT_ROOT_DIV, SRSS->CLK_ROOT_SELECT[0u]);
    cy_Hfclk0FreqHz = pathFreqHz;

    /* Slow Clock Divider */
    slowClkDiv = 1u + _FLD2VAL(CPUSS_CM0_CLOCK_CTL_SLOW_INT_DIV, CPUSS->CM0_CLOCK_CTL);

    /* Peripheral Clock Divider */
    periClkDiv = 1u + _FLD2VAL(CPUSS_CM0_CLOCK_CTL_PERI_INT_DIV, CPUSS->CM0_CLOCK_CTL);

    pathFreqHz = pathFreqHz / periClkDiv;
    cy_PeriClkFreqHz = pathFreqHz;
    pathFreqHz = pathFreqHz / slowClkDiv;
    SystemCoreClock = pathFreqHz;

    /* Sets clock frequency for Delay API */
    cy_delayFreqHz = SystemCoreClock;
    cy_delayFreqMhz = (uint8_t)((cy_delayFreqHz + CY_DELAY_1M_MINUS_1_THRESHOLD) / CY_DELAY_1M_THRESHOLD);
    cy_delayFreqKhz = (cy_delayFreqHz + CY_DELAY_1K_MINUS_1_THRESHOLD) / CY_DELAY_1K_THRESHOLD;
    cy_delay32kMs   = CY_DELAY_MS_OVERFLOW_THRESHOLD * cy_delayFreqKhz;
}


#if (CY_SYSTEM_CPU_CM0P == 1UL)
/*******************************************************************************
* Function Name: Cy_SysGetCM4Status
****************************************************************************//**
*
* Gets the Cortex-M4 core mode.
*
* \return \ref group_system_config_cm4_status_macro
*
*******************************************************************************/
uint32_t Cy_SysGetCM4Status(void)
{
    uint32_t returnValue;
    uint32_t regValue;

    regValue = CPUSS->CM4_PWR_CTL & CPUSS_CM4_PWR_CTL_Msk;
    switch(regValue)
    {
    case CPUSS_CM4_PWR_CTL_DISABLED:
        returnValue = CY_SYS_CM4_STATUS_DISABLED;
    break;

    case CPUSS_CM4_PWR_CTL_RETAINED:
        returnValue = CY_SYS_CM4_STATUS_RETAINED;
    break;

    case CPUSS_CM4_PWR_CTL_ENABLED:
        returnValue = CY_SYS_CM4_STATUS_ENABLED;
    break;

    case CPUSS_CM4_PWR_CTL_RESET_MODE:
        returnValue = CY_SYS_CM4_STATUS_RESET;
    break;

    default:
        returnValue = CY_SYS_CM4_STATUS_UNKNOWN;
    break;
    }

    return (returnValue);
}


/*******************************************************************************
* Function Name: Cy_SysEnableCM4
****************************************************************************//**
*
* Enables the Cortex-M4 core. The CPU is enabled once if it was in the disabled
* or retained mode. If the CPU is enabled, the vector table base address is
* updated and software reset of the Cortex-M4 core is performed.
*
* \param vectorTableOffset The offset of the vector table base address from
* memory address 0x00000000. The offset should be multiple to 1024 bytes.
*
*******************************************************************************/
void Cy_SysEnableCM4(uint32_t vectorTableOffset)
{
    uint32_t cm4Status;
    uint32_t  interruptState;

    interruptState = Cy_SaveIRQ();
    cm4Status = Cy_SysGetCM4Status();

    switch(cm4Status)
    {
    case CY_SYS_CM4_STATUS_DISABLED:
        CPUSS->CM4_VECTOR_TABLE_BASE = vectorTableOffset;

        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_POWER,   1UL);
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_ISOLATE, 1UL);
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_RESET,   1UL);
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_CLOCK,   1UL);
    break;

    case CY_SYS_CM4_STATUS_RETAINED:
        CPUSS->CM4_VECTOR_TABLE_BASE = vectorTableOffset;

        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_POWER,   1UL);
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_RETAIN,  1UL);
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_ISOLATE, 1UL);
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_CLOCK,   1UL);
    break;

    case CY_SYS_CM4_STATUS_RESET:
        CPUSS->CM4_VECTOR_TABLE_BASE = vectorTableOffset;

        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_RESET,   1UL);
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_CLOCK,   1UL);
    break;

    case CY_SYS_CM4_STATUS_ENABLED:
        CPUSS->CM4_VECTOR_TABLE_BASE = vectorTableOffset;

        /* Move to Reset from Enabled state */
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_CLOCK,   1UL);
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_RESET,   1UL);

        /* Move to Enabled from Reset state */
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_RESET,   1UL);
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_CLOCK,   1UL);
    break;

    default:
        /* Do nothing if Cortex-M4 is already enabled. */
    break;
    }

    Cy_RestoreIRQ(interruptState);
}


/*******************************************************************************
* Function Name: Cy_SysDisableCM4
****************************************************************************//**
*
* Disables the Cortex-M4 core.
*
* \warning Do not call the function while the Cortex-M4 is executing because
* such a call may corrupt/abort a pending bus-transaction by the CPU and cause
* unexpected behavior in the system including a deadlock. Call the function
* while the Cortex-M4 core is in the Sleep or Deep Sleep low-power mode. Use
* the \ref group_syspm Power Management (syspm) API to put the CPU into the
* low-power modes. Use the \ref Cy_SysPm_ReadStatus() to get a status of the
* CPU.
*
*******************************************************************************/
void Cy_SysDisableCM4(void)
{
    uint32_t cm4Status;
    uint32_t  interruptState;

    interruptState = Cy_SaveIRQ();
    cm4Status = Cy_SysGetCM4Status();

    switch(cm4Status)
    {
    case CY_SYS_CM4_STATUS_ENABLED:
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_CLOCK,   1UL);
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_RESET,   1UL);
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_ISOLATE, 1UL);
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_POWER,   1UL);
    break;

    case CY_SYS_CM4_STATUS_RETAINED:
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_RESET,   1UL);
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_RETAIN,  1UL);
    break;

    default:
        /* Do nothing if Cortex-M4 is already disabled. */
    break;
    }

    Cy_RestoreIRQ(interruptState);
}


/*******************************************************************************
* Function Name: Cy_SysRetainCM4
****************************************************************************//**
*
* Retains the Cortex-M4 core.
*
* \warning Do not call the function while the Cortex-M4 is executing because
* such a call may corrupt/abort a pending bus-transaction by the CPU and cause
* unexpected behavior in the system including a deadlock. Call the function
* while the Cortex-M4 core is in the Sleep or Deep Sleep low-power mode. Use
* the \ref group_syspm Power Management (syspm) API to put the CPU into the
* low-power modes. Use the \ref Cy_SysPm_ReadStatus() to get a status of the CPU.
*
*******************************************************************************/
void Cy_SysRetainCM4(void)
{
    uint32_t cm4Status;
    uint32_t  interruptState;

    interruptState = Cy_SaveIRQ();
    cm4Status = Cy_SysGetCM4Status();

    switch(cm4Status)
    {
    case CY_SYS_CM4_STATUS_ENABLED:
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_CLOCK,   1UL);
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_ISOLATE, 1UL);
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_RETAIN,  1UL);
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_POWER,   1UL);
    break;

    case CY_SYS_CM4_STATUS_DISABLED:
        /* Switch from the DISABLED to the RETAINED state is not valid.
         * Do nothing in this case. */
    break;

        default:
            /* Do nothing if Cortex-M4 is already in the RETAINED state. */
        break;
        }

    Cy_RestoreIRQ(interruptState);
}


/*******************************************************************************
* Function Name: Cy_SysResetCM4
****************************************************************************//**
*
* Resets the Cortex-M4 core.
*
* \warning Do not call the function while the Cortex-M4 is executing because
* such a call may corrupt/abort a pending bus-transaction by the CPU and cause
* unexpected behavior in the system including a deadlock. Call the function
* while the Cortex-M4 core is in the Sleep or Deep Sleep low-power mode. Use
* the \ref group_syspm Power Management (syspm) API to put the CPU into the
* low-power modes. Use the \ref Cy_SysPm_ReadStatus() to get a status of the CPU.
*
*******************************************************************************/
void Cy_SysResetCM4(void)
{
    uint32_t cm4Status;
    uint32_t  interruptState;

    interruptState = Cy_SaveIRQ();
    cm4Status = Cy_SysGetCM4Status();

    switch(cm4Status)
    {
    case CY_SYS_CM4_STATUS_ENABLED:
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_CLOCK,   1UL);
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_RESET,   1UL);
    break;

    case CY_SYS_CM4_STATUS_DISABLED:
        CPUSS->CM4_PWR_CTL |=  _VAL2FLD(CPUSS_CM4_PWR_CTL_ISOLATE, 1UL);
        CPUSS->CM4_PWR_CTL &= ~_VAL2FLD(CPUSS_CM4_PWR_CTL_POWER,   1UL);
    break;

        default:
            /* Do nothing if Cortex-M4 is already in the RETAINED state. */
        break;
        }

    Cy_RestoreIRQ(interruptState);
}
#endif /* (CY_SYSTEM_CPU_CM0P == 1UL) */


/* [] END OF FILE */
