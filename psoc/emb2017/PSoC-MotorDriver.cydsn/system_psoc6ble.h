/***************************************************************************//**
* \file system_psoc6ble.h
* \version 1.0
*
* \brief Device system header file.
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#ifndef _SYSTEM_PSOC6BLE_H_
#define _SYSTEM_PSOC6BLE_H_

/**
* \defgroup group_system_config System Configuration Files
* \{
* Provides device startup, system configuration, and linker script files.
* The system startup provides the followings features:
* - See \ref group_system_config_device_initialization for the:
*   * \ref group_system_config_dual_core_device_initialization
*   * \ref group_system_config_single_core_device_initialization
* - See \ref group_system_config_device_memory_definition for the:
*   * \ref group_system_config_device_memory_dual_core_definition
*   * \ref group_system_config_device_memory_single_core_definition
* - \ref group_system_config_heap_stack_config
* - Default interrupt handlers definition
* - Vectors table copy from flash to RAM
* - \ref group_system_config_cm4_functions
*
* \section group_system_config_configuration Configuration Considerations
*
* \subsection group_system_config_device_memory_definition Device Memory Definition
* The physical flash and RAM memory is shared between the CPU cores. The flash and RAM allocation for each
* CPU is defined by the linker scripts.
*
* \subsubsection group_system_config_device_memory_dual_core_definition Dual-Core Devices
* By default, the flash and RAM memory is equally divided between two cores. This proportion can be changed by editing
* the linker configuration files.
*
* <b>ARM GCC</b>\n
* The flash and RAM sections for the CPU are defined in the linker files: 'xxx_yyy.ld', where 'xxx' is the device
* group, and 'yyy' is the target CPU; for example, 'cy8c6x7_cm0plus.ld' and 'cy8c6x7_cm4_dual.ld'.
* Note: if the start of the Cortex-M4 application image is changed, the value of the of the \ref CY_CORTEX_M4_APPL_ADDR
* should also be changed. The \ref CY_CORTEX_M4_APPL_ADDR macro should be used as the parameter for the \ref
* Cy_SysEnableCM4() function call.
*
* The flash and RAM sizes can be changed by editing the LENGTH value in the linker files for both CPUs:
* - 'xxx_cm0plus.ld', where 'xxx' is the device group:
*   * \code rom         (rx)  : ORIGIN = 0x10000000, LENGTH = 0x00080000 \endcode
*   * \code ram_cm0p    (rwx) : ORIGIN = 0x08000000, LENGTH = 0x00024000 \endcode
* - 'xxx_cm4_dual.ld', where 'xxx' is the device group:
*   * \code rom         (rx)  : ORIGIN = 0x10080000, LENGTH = 0x00080000 \endcode
*   * \code ram_cm4     (rwx) : ORIGIN = 0x08024000, LENGTH = 0x00024000 \endcode
* Change the value of the \ref CY_CORTEX_M4_APPL_ADDR macro to the rom ORIGIN's value in the 'xxx_cm4_dual.ld' file,
* where 'xxx' is the device group. Do this by either:
* - Passing the following commands to the compiler:\n
*  * \code -D CY_CORTEX_M4_APPL_ADDR=0x10080000 \endcode
* - Editing the \ref CY_CORTEX_M4_APPL_ADDR value in the 'system_xxx.h', where 'xxx' is device family:\n
*  * \code #define CY_CORTEX_M4_APPL_ADDR (0x10080000u) \endcode
*
* <b>ARM MDK</b>\n
* The flash and RAM sections for the CPU are defined in the linker files: 'xxx_yyy.scat', where 'xxx' is the device
* group, and 'yyy' is the target CPU; for example, 'cy8c6x7_cm0plus.scat' and 'cy8c6x7_cm4_dual.scat'.
* Note: if the start of the Cortex-M4 application image is changed, the value of the of the \ref CY_CORTEX_M4_APPL_ADDR
* should also be changed. The \ref CY_CORTEX_M4_APPL_ADDR macro should be used as the parameter for the \ref
* Cy_SysEnableCM4() function call.
*
* The flash and RAM sizes can be changed by editing the macros value in the linker files for both CPUs:
* - 'xxx_cm0plus.scat', where 'xxx' is the device group:
*   * \code #define FLASH_START 0x10000000 \endcode
*   * \code #define FLASH_SIZE  0x00080000 \endcode
* - 'xxx_cm4_dual.scat', where 'xxx' is the device group:
*   * \code #define FLASH_START 0x10080000 \endcode
*   * \code #define FLASH_SIZE  0x00080000 \endcode
* Change the value of the \ref CY_CORTEX_M4_APPL_ADDR macro to the FLASH_START value in the 'xxx_cm4_dual.scat' file,
* where 'xxx' is the device group. Do this by either:
* - Passing the following commands to the compiler:\n
*  * \code -D CY_CORTEX_M4_APPL_ADDR=0x10080000 \endcode
* - Editing the \ref CY_CORTEX_M4_APPL_ADDR value in the 'system_xxx.h', where 'xxx' is device family:\n
*  * \code #define CY_CORTEX_M4_APPL_ADDR          (0x10080000u) \endcode
*
* <b>IAR</b>\n
* The flash and RAM sections for the CPU are defined in the linker files: 'xxx_yyy.icf', where 'xxx' is the device
* group, and 'yyy' is the target CPU; for example, 'cy8c6x7_cm0plus.icf' and 'cy8c6x7_cm4_dual.icf'.
* Note: if the start of the Cortex-M4 application image is changed, the value of the of the \ref CY_CORTEX_M4_APPL_ADDR
* should also be changed. The \ref CY_CORTEX_M4_APPL_ADDR macro should be used as the parameter for the \ref
* Cy_SysEnableCM4() function call.
*
* The flash and RAM sizes can be changed by editing the macros value in the linker files for both CPUs:
* - 'xxx_cm0plus.icf', where 'xxx' is the device group:
*   * \code define symbol __ICFEDIT_region_IROM_start__ = 0x10000000; \endcode
*   * \code define symbol __ICFEDIT_region_IROM_end__ = 0x10080000; \endcode
*   * \code define symbol __ICFEDIT_region_IRAM_CM0P_start__ = 0x08000000; \endcode
*   * \code define symbol __ICFEDIT_region_IRAM_CM0P_end__ = 0x08024000; \endcode
* - 'xxx_cm4_dual.icf', where 'xxx' is the device group:
*   * \code define symbol __ICFEDIT_region_IROM_start__ = 0x10080000; \endcode
*   * \code define symbol __ICFEDIT_region_IROM_end__ = 0x10100000; \endcode
*   * \code define symbol __ICFEDIT_region_IRAM_CM4_start__ = 0x08024000; \endcode
*   * \code define symbol __ICFEDIT_region_IRAM_CM4_end__ = 0x08048000; \endcode
* Change the value of the \ref CY_CORTEX_M4_APPL_ADDR macro to the __ICFEDIT_region_IROM_start__ value in the
* 'xxx_cm4_dual.icf' file, where 'xxx' is the device group. Do this by either:
* - Passing the following commands to the compiler:\n
*  * \code -D CY_CORTEX_M4_APPL_ADDR=0x10080000 \endcode
* - Editing the \ref CY_CORTEX_M4_APPL_ADDR value in the 'system_xxx.h', where 'xxx' is device family:\n
*  * \code #define CY_CORTEX_M4_APPL_ADDR (0x10080000u) \endcode
*
* \subsubsection group_system_config_device_memory_single_core_definition Single-Core Devices
* For single-core devices, 8 KB of flash and RAM are reserved for the pre-generated Cortex-M0+ application
* that is loaded for the hidden Cortex-M0+.
*
*
* \subsection group_system_config_device_initialization Device Initialization
* After a power-on-reset (POR), the boot process is handled by the boot code from the on-chip ROM that is always
* executed by the Cortex-M0+ core. The boot code passes the control to the Cortex-M0+ startup code located in flash.
*
* \subsubsection group_system_config_dual_core_device_initialization Dual-Core Devices
* The Cortex-M0+ startup code performs the device initialization by a call to \ref SystemInit() and then calls the main()
* function. The Cortex-M4 core is disabled by default. Enable the core using the \ref Cy_SysEnableCM4() function.
* See \ref group_system_config_cm4_functions for more details.
*
* \subsubsection group_system_config_single_core_device_initialization Single-Core Devices
* The Cortex-M0+ core is not user-accessible on these devices. In this case the Cortex-M0+ pre-built application image
* handles setup of the CM0+ core and starts the Cortex-M4 core. The Cortex-M0+ application image performs the following:
* - Enable global interrupts on the Cortex-M0+ core
* - Starts the crypto driver's server with the following options:
*   * Cortex-M0+ crypto notify interrupt number 7 (Deep Sleep wakeup capable).
*   * Cortex-M0+ crypto release interrupt number 27.
*   * Cortex-M0+ crypto error interrupt number 28.
*   See the crypto \ref group_crypto_configuration for the details.
* - Enables the Cortex-M4 core by calling \ref Cy_SysEnableCM4()
* - Requests Deep Sleep mode entry with wakeup on interrupt in the infinite loop
*
* \subsection group_system_config_heap_stack_config Heap and Stack Configuration
* There are two ways to adjust heap and stack configurations:
* -# Editing source code files
* -# Specifying via command line
*
* By default, the stack size is set to 0x00001000 and the heap size is set to 0x00000400.
*
* \subsubsection group_system_config_heap_stack_config_gcc ARM GCC
* - <b>Editing source code files</b>\n
* The heap and stack sizes are defined in the assembler startup files: 'startup_xxx_yyy.S', where 'xxx' is the device
* family, and 'yyy' is the target CPU; for example, startup_psoc6ble_cm0plus.s and startup_psoc6ble_cm4.s.
* Change the heap and stack sizes by modifying the following lines:\n
*   * \code .equ  Stack_Size, 0x00001000 \endcode
*   * \code .equ  Heap_Size,  0x00000400 \endcode
*
* - <b>Specifying via command line</b>\n
* Change the heap and stack sizes passing the following commands to the compiler:\n
*  * \code -D __STACK_SIZE=0x000000400 \endcode
*  * \code -D __HEAP_SIZE=0x000000100 \endcode
*
* \subsubsection group_system_config_heap_stack_config_mdk ARM MDK
* - <b>Editing source code files</b>\n
* The heap and stack sizes are defined in the assembler startup files: 'startup_xxx_yyy.s', where 'xxx' is the device
* family, and 'yyy' is the target CPU; for example, startup_psoc6ble_cm0plus.s and startup_psoc6ble_cm4.s.
* Change the heap and stack sizes by modifying the following lines:\n
*   * \code Stack_Size      EQU     0x00001000 \endcode
*   * \code Heap_Size       EQU     0x00000400 \endcode
*
* - <b>Specifying via command line</b>\n
* Change the heap and stack sizes passing the following commands to the assembler:\n
*  * \code "--predefine=___STACK_SIZE SETA 0x000000400" \endcode
*  * \code "--predefine=__HEAP_SIZE SETA 0x000000100" \endcode
*
* \subsubsection group_system_config_heap_stack_config_iar IAR
* - <b>Editing source code files</b>\n
* The heap and stack sizes are defined in the linker scatter files: 'xxx_yyy.icf', where 'xxx' is the device
* family, and 'yyy' is the target CPU; for example, cy8c6x7_cm0plus.icf and cy8c6x7_cm4_dual.icf.
* Change the heap and stack sizes by modifying the following lines:\n
*   * \code Stack_Size      EQU     0x00001000 \endcode
*   * \code Heap_Size       EQU     0x00000400 \endcode
*
* - <b>Specifying via command line</b>\n
* Change the heap and stack sizes passing the following commands to the linker (including quotation marks):\n
*  * \code --define_symbol __STACK_SIZE=0x000000400 \endcode
*  * \code --define_symbol __HEAP_SIZE=0x000000100 \endcode
*
* \section group_system_config_more_information More Information
* Refer to the <a href="..\..\pdl_user_guide.pdf">PDL User Guide</a> for the
* more details.
*
* \section group_system_config_MISRA MISRA Compliance
*  The drivers violates the following MISRA-C:2004 rules:
*
* <table class="doxtable">
*   <tr>
*       <th>MISRA Rule</th>
*       <th>Rule Class (Required/Advisory)</th>
*       <th>Rule Description</th>
*       <th>Description of Deviation(s)</th>
*   </tr>
*   <tr>
*       <td>8.8</td>
*       <td>Required</td>
*       <td>An external object or function shall be declared in one and only one file.</td>
*       <td>The cm0p_image array is not used within project, so is defined without previous declaration.</td>
*   </tr>
* </table>
*
*
* \section group_system_config_changelog Changelog
*   <table class="doxtable">
*   <tr>
*       <th>Version</th>
*       <th>Changes</th>
*       <th>Reason for Change</th>
*    </tr>
*   <tr>
*       <td>1.0</td>
*       <td>Initial version</td>
*       <td></td>
*   </tr>
* </table>
*
*
* \defgroup group_system_config_macro Macro
* \{
*   \defgroup group_system_config_system_macro            System
*   \defgroup group_system_config_cm4_status_macro        Cortex-M4 Status
*   \defgroup group_system_config_user_settings_macro     User Settings
* \}
* \defgroup group_system_config_functions Functions
* \{
*   \defgroup group_system_config_system_functions        System
*   \defgroup group_system_config_cm4_functions           Cortex-M4 Control
* \}
* \defgroup group_system_config_globals Global Variables
*
* \}
*/

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
* Include files
*******************************************************************************/
#include <stdint.h>


/*******************************************************************************
* Global preprocessor symbols/macros ('define')
*******************************************************************************/
#if ((defined(__GNUC__)        &&  (__ARM_ARCH == 6) && (__ARM_ARCH_6M__ == 1)) || \
     (defined (__ICCARM__)     &&  (__CORE__ == __ARM6M__))  || \
     (defined(__ARMCC_VERSION) &&  (__TARGET_ARCH_THUMB == 3)))
    #define CY_SYSTEM_CPU_CM0P          1UL
#endif

#if defined (CY_PSOC_CREATOR_USED) && (CY_PSOC_CREATOR_USED == 1U)
    #include "cyfitter.h"
#endif /* (CY_PSOC_CREATOR_USED) && (CY_PSOC_CREATOR_USED == 1U) */


/*******************************************************************************
*
*                      START OF USER SETTINGS HERE
*                      ===========================
*
*                 All lines with '<<<' can be set by user.
*
*******************************************************************************/

/**
* \addtogroup group_system_config_user_settings_macro
* \{
*/


#if defined (CYDEV_CLK_EXTCLK__HZ)
    #define CY_CLK_EXT_FREQ_HZ          (CYDEV_CLK_EXTCLK__HZ)
#else
    /***************************************************************************//**
    * External Clock Frequency (in Hz, [value]UL). If compiled within
    * PSoC Creator and the clock is enabled in the DWR, the value from DWR used.
    * Otherwise, edit the value below.
    *        <i>(USER SETTING)</i>
    *******************************************************************************/
    #define CY_CLK_EXT_FREQ_HZ          (24000000UL)    /* <<< 24 MHz */
#endif /* (CYDEV_CLK_EXTCLK__HZ) */


#if defined (CYDEV_CLK_ECO__HZ)
    #define CY_CLK_ECO_FREQ_HZ          (CYDEV_CLK_ECO__HZ)
#else
    /***************************************************************************//**
    * \brief External crystal oscillator frequency (in Hz, [value]UL). If compiled
    * within PSoC Creator and the clock is enabled in the DWR, the value from DWR
    * used.
    *       <i>(USER SETTING)</i>
    *******************************************************************************/
    #define CY_CLK_ECO_FREQ_HZ          (24000000UL)    /* <<< 24 MHz */
#endif /* (CYDEV_CLK_ECO__HZ) */


#if defined (CYDEV_CLK_ALTHF__HZ)
    #define CY_CLK_ALTHF_FREQ_HZ        (CYDEV_CLK_ALTHF__HZ)
#else
    /***************************************************************************//**
    * \brief Alternate high frequency (in Hz, [value]UL). If compiled within
    * PSoC Creator and the clock is enabled in the DWR, the value from DWR used.
    * Otherwise, edit the value below.
    *        <i>(USER SETTING)</i>
    *******************************************************************************/
    #define CY_CLK_ALTHF_FREQ_HZ        (32000000UL)    /* <<< 32 MHz */
#endif /* (CYDEV_CLK_ALTHF__HZ) */


/***************************************************************************//**
* \brief Start address of the Cortex-M4 application ([address]UL)
*        <i>(USER SETTING)</i>
*******************************************************************************/
#define CY_CORTEX_M4_APPL_ADDR          (0x10080000UL)   /* <<< 512 KB reserved for the Cortex-M0+ application */


/*******************************************************************************
*
*                         END OF USER SETTINGS HERE
*                         =========================
*
*******************************************************************************/

/** \} group_system_config_user_settings_macro */


/**
* \addtogroup group_system_config_system_macro
* \{
*/

#if (CY_SYSTEM_CPU_CM0P == 1UL) || defined(CY_DOXYGEN)
    /** The Cortex-M0+ startup driver identifier */
    #define CY_STARTUP_M0P_ID               ((uint32_t)((uint32_t)((0x0Eu) & 0x3FFFu) << 18u))
#endif

#if (CY_SYSTEM_CPU_CM0P != 1UL) || defined(CY_DOXYGEN)
    /** The Cortex-M4 startup driver identifier */
    #define CY_STARTUP_M4_ID        ((uint32_t)((uint32_t)((0x0Fu) & 0x3FFFu) << 18u))
#endif

/** \} group_system_config_system_macro */


/**
* \addtogroup group_system_config_system_functions
* \{
*/
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
/** \} group_system_config_system_functions */


#if (CY_SYSTEM_CPU_CM0P == 1UL) || defined(CY_DOXYGEN)
/**
* \addtogroup group_system_config_cm4_functions
* \{
*/
extern uint32_t Cy_SysGetCM4Status(void);
extern void     Cy_SysEnableCM4(uint32_t vectorTableOffset);
extern void     Cy_SysDisableCM4(void);
extern void     Cy_SysRetainCM4(void);
extern void     Cy_SysResetCM4(void);
/** \} group_system_config_cm4_functions */
#endif

/** \cond */
extern void     Default_Handler (void);
extern uint32_t Cy_SaveIRQ(void);
extern void     Cy_RestoreIRQ(uint32_t saved);

extern void     Cy_SystemInit(void);
extern void     Cy_IPC_SystemLocksInit(void);
extern void     Cy_IPC_SystemPipeInit(void);
extern void     Cy_SystemInitFpuEnable(void);

extern uint32_t cy_delayFreqHz;
extern uint32_t cy_delayFreqKhz;
extern uint8_t  cy_delayFreqMhz;
extern uint32_t cy_delay32kMs;
/** \endcond */


#if (CY_SYSTEM_CPU_CM0P == 1UL) || defined(CY_DOXYGEN)
/**
* \addtogroup group_system_config_cm4_status_macro
* \{
*/
#define CY_SYS_CM4_STATUS_ENABLED   (0u)    /**< The Cortex-M4 core is enabled. */
#define CY_SYS_CM4_STATUS_DISABLED  (1u)    /**< The Cortex-M4 core is disabled. */
#define CY_SYS_CM4_STATUS_RETAINED  (2u)    /**< The Cortex-M4 core is retained. */
#define CY_SYS_CM4_STATUS_UNKNOWN   (3u)    /**< The Cortex-M4 core is in the unknown state. Invalid state. */
#define CY_SYS_CM4_STATUS_RESET     (4u)    /**< The Cortex-M4 core is in the Reset mode. */
/** \} group_system_config_cm4_status_macro */


/** \cond */
    #define CPUSS_CM4_PWR_CTL_Msk            (CPUSS_CM4_PWR_CTL_ISOLATE_Msk | \
                                              CPUSS_CM4_PWR_CTL_RETAIN_Msk  | CPUSS_CM4_PWR_CTL_POWER_Msk | \
                                              CPUSS_CM4_PWR_CTL_RESET_Msk   | CPUSS_CM4_PWR_CTL_CLOCK_Msk)

    #define CPUSS_CM4_PWR_CTL_DISABLED       (CPUSS_CM4_PWR_CTL_RESET_Msk  | CPUSS_CM4_PWR_CTL_ISOLATE_Msk)
    #define CPUSS_CM4_PWR_CTL_RETAINED       (CPUSS_CM4_PWR_CTL_RETAIN_Msk | CPUSS_CM4_PWR_CTL_ISOLATE_Msk)
    #define CPUSS_CM4_PWR_CTL_ENABLED        (CPUSS_CM4_PWR_CTL_CLOCK_Msk  | CPUSS_CM4_PWR_CTL_POWER_Msk)
    #define CPUSS_CM4_PWR_CTL_RESET_MODE     (CPUSS_CM4_PWR_CTL_RESET_Msk  | CPUSS_CM4_PWR_CTL_POWER_Msk)
/** \endcond */
#endif

/** \addtogroup group_system_config_globals
* \{
*/

extern uint32_t SystemCoreClock;
extern uint32_t cy_BleEcoClockFreqHz;
extern uint32_t cy_Hfclk0FreqHz;
extern uint32_t cy_PeriClkFreqHz;

/** \} group_system_config_globals */

#ifdef __cplusplus
}
#endif

#endif /* _SYSTEM_PSOC6BLE_H_ */


/* [] END OF FILE */
