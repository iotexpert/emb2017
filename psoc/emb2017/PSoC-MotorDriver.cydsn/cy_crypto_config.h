/***************************************************************************//**
* \file cy_crypto_config.h
* \version 1.0
*
* \brief
*  This file provides user parameters for the Crypto driver.
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CRYPTO_CONFIG_H)
#define CY_CRYPTO_CONFIG_H

#include "cy_device_headers.h"

/* Defines to configure interrupts used in Crypto driver */

/**
* \addtogroup group_crypto_config_macro
* \{
*/

/** Number of Crypto Notify interrupt mapped to CM0+ */
#define CY_CRYPTO_CM0_NOTIFY_INTR_NR  (NvicMux26_IRQn)
/** Priority of Crypto Notify interrupt, equal to CM0+ and CM4 cores */
#define CY_CRYPTO_NOTIFY_INTR_PR      (2u)

/** Number of Crypto Release interrupt mapped to CM0+ */
#define CY_CRYPTO_CM0_RELEASE_INTR_NR (NvicMux27_IRQn)
/** Priority of Crypto Release interrupt, equal to CM0+ and CM4 cores */
#define CY_CRYPTO_RELEASE_INTR_PR     (2u)

/** Number of Crypto Error interrupt mapped to CM0+ */
#define CY_CRYPTO_CM0_ERROR_INTR_NR   (NvicMux28_IRQn)
/** Priority of Crypto Error interrupt mapped to CM0+ */
#define CY_CRYPTO_ERROR_INTR_PR       (2u)

/** Default Crypto driver configuration */
#if (CY_CPU_CORTEX_M0P)
#define CY_CRYPTO_DEFAULT_CONFIG \
    { \
        CY_CRYPTO_CM0_NOTIFY_INTR_NR,  \
        CY_CRYPTO_CM0_RELEASE_INTR_NR, \
        CY_CRYPTO_CM0_ERROR_INTR_NR,   \
        CY_CRYPTO_NOTIFY_INTR_PR,      \
        CY_CRYPTO_RELEASE_INTR_PR,     \
        CY_CRYPTO_ERROR_INTR_PR,       \
        NULL, \
        NULL, \
        NULL \
    }
#else
#define CY_CRYPTO_DEFAULT_CONFIG \
    { \
        CY_CRYPTO_NOTIFY_INTR_PR,      \
        CY_CRYPTO_RELEASE_INTR_PR,     \
        CY_CRYPTO_ERROR_INTR_PR,       \
        NULL, \
        NULL, \
        NULL \
    }
#endif

/* Defines to Enable/Disable Crypto functionality */

/** Enable/Disable AES CMAC support (0 = no support, 1 = support).
* To use CMAC the AES ECB mode should be enabled also */
#define CY_CRYPTO_USER_CMAC           (1u)
/** Enable/Disable AES ECB cipher support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_AES_ECB        (1u)
/** Enable/Disable AES CBC cipher support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_AES_CBC        (1u)
/** Enable/Disable AES CFB cipher support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_AES_CFB        (1u)
/** Enable/Disable AES CTR cipher support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_AES_CTR        (1u)
/** Enable/Disable PKCS1-v1.5 verification support (0 = no support, 1 = support).
* To use PKCS1-v1.5 at least one of SHA modes should be enabled also */
#define CY_CRYPTO_USER_PKCS1_5        (1u)
/** Enable/Disable HMAC support (0 = no support, 1 = support).
* To use HMAC at least one of SHA modes should be enabled also */
#define CY_CRYPTO_USER_HMAC           (1u)
/** Enable/Disable SHA1 hash support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_SHA1           (1u)
/** Enable/Disable SHA 224 and 256 hash support,  (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_SHA256         (1u)
/** Enable/Disable SHA 384, 512, 512/224 and 512/256 hash support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_SHA512         (1u)
/** Enable/Disable DES and TDES ciphers support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_DES            (1u)
/** Enable/Disable Pseudo random number generation support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_PRNG           (1u)
/** Enable/Disable Cyclic Redundancy Check (CRC) support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_CRC            (1u)
/** Enable/Disable RSA support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_RSA            (1u)
/** Enable/Disable True random number generation support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_TRNG           (1u)
/** Enable/Disable String support (0 = no support, 1 = support) */
#define CY_CRYPTO_USER_STR            (1u)

/** \} group_crypto_config_macro */

#endif /* #if !defined(CY_CRYPTO_CONFIG_H) */


/* [] END OF FILE */
