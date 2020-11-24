/*
 * FreeRTOS PKCS #11 PAL for Numaker_PFM_M487 V1.0.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */


/**
 * @file aws_pkcs11_pal.c
 * @brief FreeRTOS device specific helper functions for
 * PKCS#11 implementation based on mbedTLS.  This
 * file deviates from the FreeRTOS style standard for some function names and
 * data types in order to maintain compliance with the PKCS#11 standard.
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "FreeRTOSIPConfig.h"
#include "task.h"
#include "core_pkcs11.h"
#include "iot_crypto.h"
#include "core_pkcs11_config.h"
#include "stm32F4xx_hal.h"

/* C runtime includes. */
#include <stdio.h>
#include <string.h>

#define pkcs11palFILE_NAME_CLIENT_CERTIFICATE    "FreeRTOS_P11_Certificate.dat"
#define pkcs11palFILE_NAME_KEY                   "FreeRTOS_P11_Key.dat"
#define pkcs11palFILE_CODE_SIGN_PUBLIC_KEY       "FreeRTOS_P11_CodeSignKey.dat"

#define pkcs11OBJECT_CERTIFICATE_MAX_SIZE    2048
#define pkcs11OBJECT_FLASH_CERT_PRESENT      ( 0x1A2B3C4DUL )

enum eObjectHandles
{
    eInvalidHandle = 0, /* From PKCS #11 spec: 0 is never a valid object handle.*/
    eAwsDevicePrivateKey = 1,
    eAwsDevicePublicKey,
    eAwsDeviceCertificate,
    eAwsCodeSigningKey
};

/**
 * @brief Structure for certificates/key storage.
 */

typedef struct
{
    CK_ULONG ulDeviceCertificateMark;
    CK_ULONG ulCertificateSize;
    CK_CHAR cCertificateData[ pkcs11OBJECT_CERTIFICATE_MAX_SIZE ];
} P11CertData_t;

typedef struct
{
    uint32_t DeviceCertificate;
    uint32_t DeviceKey;
    uint32_t CodeSignKey;
} P11KeyConfig_t;


/* Set last 3 sectors for certification storage */
static P11KeyConfig_t P11KeyConfig ={ 0x080A0000, //sec9
                                      0x080C0000, //sec10
                                      0x080E0000}; //sec11

static P11CertData_t P11CertDataSave;
                                
/*-----------------------------------------------------------*/


static CK_RV prvFLASH_update(uint32_t u32StartAddr, uint8_t * pucData, uint32_t ulDataSize)
{
	uint32_t    u32Addr;
	uint32_t    u32data;
	uint8_t    *pDataSrc;
	uint32_t    u32EndAddr = (u32StartAddr + sizeof(P11CertData_t));
	uint32_t    u32Pattern = 0xFFFFFFFF;	
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError = 0;
	uint8_t sector_no;
	
	if (u32StartAddr == 0x080A0000)
	{
		sector_no =	FLASH_SECTOR_9;
	} 
	else if (u32StartAddr == 0x080C0000)
	{
		sector_no =	FLASH_SECTOR_10;
	} 
	else if (u32StartAddr == 0x080E0000)
	{
		sector_no =	FLASH_SECTOR_11;
	}	
	
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_1;
  EraseInitStruct.Sector = sector_no;
  EraseInitStruct.NbSectors = 1;
	HAL_FLASH_Unlock();
	
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK)
	{    
		memcpy( P11CertDataSave.cCertificateData, pucData, ulDataSize );
    P11CertDataSave.ulDeviceCertificateMark = pkcs11OBJECT_FLASH_CERT_PRESENT;
    P11CertDataSave.ulCertificateSize = ulDataSize;
    pDataSrc = (uint8_t *)&P11CertDataSave;
    /* Fill flash range from u32StartAddr to u32EndAddr with data word u32Pattern. */
    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, u32Addr, *pDataSrc);          /* Program flash */
        pDataSrc++;
    }    
    
		HAL_FLASH_Lock(); 		
    return ulDataSize;
	}
	else 
	{
		HAL_FLASH_Lock();
		return 0;
	}    
}


/* Converts a label to its respective filename and handle. */
static void prvLabelToFilenameHandle( uint8_t * pcLabel,
                               char ** pcFileName,
                               CK_OBJECT_HANDLE_PTR pHandle )
{
    if( pcLabel != NULL )
    {
        /* Translate from the PKCS#11 label to local storage file name. */
        if( 0 == memcmp( pcLabel,
                         &pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS,
                         sizeof( pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS ) ) )
        {
            *pcFileName = pkcs11palFILE_NAME_CLIENT_CERTIFICATE;
            *pHandle = eAwsDeviceCertificate;
        }
        else if( 0 == memcmp( pcLabel,
                              &pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS,
                              sizeof( pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS ) ) )
        {
            *pcFileName = pkcs11palFILE_NAME_KEY;
            *pHandle = eAwsDevicePrivateKey;
        }
        else if( 0 == memcmp( pcLabel,
                              &pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS,
                              sizeof( pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS ) ) )
        {
            *pcFileName = pkcs11palFILE_NAME_KEY;
            *pHandle = eAwsDevicePublicKey;
        }
        else if( 0 == memcmp( pcLabel,
                              &pkcs11configLABEL_CODE_VERIFICATION_KEY,
                              sizeof( pkcs11configLABEL_CODE_VERIFICATION_KEY ) ) )
        {
            *pcFileName = pkcs11palFILE_CODE_SIGN_PUBLIC_KEY;
            *pHandle = eAwsCodeSigningKey;
        }
        else
        {
            *pcFileName = NULL;
            *pHandle = eInvalidHandle;
        }
    }
}


/**
 * @brief Writes a file to local storage.
 *
 * Port-specific file write for crytographic information.
 *
 * @param[in] pcFileName    The name of the file to be written to.
 * @param[in] pucData       Data buffer to be written to file
 * @param[in] pulDataSize   Size (in bytes) of file data.
 *
 * @return pdTRUE if data was saved successfully to file,
 * pdFALSE otherwise.
 */
static BaseType_t prvFLASH_SaveFile( char * pcFileName,
                                uint8_t * pucData,
                                uint32_t ulDataSize )
{
    CK_RV xResult = pdFALSE;
    uint32_t certFlashAddr = 0;
    CK_RV xBytesWritten = 0;
    CK_ULONG ulFlashMark = pkcs11OBJECT_FLASH_CERT_PRESENT;
    const P11CertData_t * pCertFlash;
    P11CertData_t * pCertSave = 0;
    
    /* enough room to store the certificate */
    if( ulDataSize > pkcs11OBJECT_CERTIFICATE_MAX_SIZE )
    {
        return xResult;   
    }
    
    /*
     * write client certificate.
     */

    if( strcmp( pcFileName, pkcs11palFILE_NAME_CLIENT_CERTIFICATE ) == 0 )
    {
        certFlashAddr = P11KeyConfig.DeviceCertificate;
    }
    else if( strcmp( pcFileName, pkcs11palFILE_NAME_KEY ) == 0 )
    {
        certFlashAddr = P11KeyConfig.DeviceKey;
    }
    else if( strcmp( pcFileName, pkcs11palFILE_CODE_SIGN_PUBLIC_KEY ) == 0 )
    {
        certFlashAddr = P11KeyConfig.CodeSignKey;
    }
    else
    {
        certFlashAddr = NULL;
    }

    if( certFlashAddr != NULL )
    {
        xBytesWritten = prvFLASH_update( certFlashAddr, pucData, ulDataSize );
        if( xBytesWritten == ulDataSize )
        {
            xResult = pdTRUE;
        }
    }            
    
    return xResult;
}
/*-----------------------------------------------------------*/

/**
 * @brief Reads a file from local storage.
 *
 * Port-specific file access for crytographic information.
 *
 * @param[in] pcFileName    The name of the file to be read.
 * @param[out] ppucData     Pointer to buffer for file data.
 * @param[out] pulDataSize  Size (in bytes) of data located in file.
 *
 * @return pdTRUE if data was retrieved successfully from files,
 * pdFALSE otherwise.
 */
static BaseType_t prvFLASH_ReadFile( char * pcFileName,
                                uint8_t ** ppucData,
                                uint32_t * pulDataSize )
{
    CK_RV xResult = pdFALSE;
    const P11CertData_t * pCertFlash = 0;
    uint32_t certSize = 0;
    uint8_t * pCertData = 0;
    /*
     * Read client certificate.
     */
    if( strcmp( pcFileName, pkcs11palFILE_NAME_CLIENT_CERTIFICATE ) == 0 )
    {
        pCertFlash = (P11CertData_t *)P11KeyConfig.DeviceCertificate;
    }
    else if( strcmp( pcFileName, pkcs11palFILE_NAME_KEY ) == 0 )
    {
        pCertFlash = (P11CertData_t *)P11KeyConfig.DeviceKey;
    }
    else if( strcmp( pcFileName, pkcs11palFILE_CODE_SIGN_PUBLIC_KEY ) == 0 )
    {
        pCertFlash = (P11CertData_t *)P11KeyConfig.CodeSignKey;        
    }

    if( ( pCertFlash !=0 ) && ( pCertFlash->ulDeviceCertificateMark == pkcs11OBJECT_FLASH_CERT_PRESENT ) )
    {
        pCertData = ( uint8_t * ) pCertFlash->cCertificateData;
        certSize = pCertFlash->ulCertificateSize;
        xResult = pdTRUE;
    }
    
    *pulDataSize = certSize;
    *ppucData = pCertData;    
    
    return xResult;
}

CK_RV PKCS11_PAL_Initialize( void )
{
    CRYPTO_Init();
    return CKR_OK;
}

/**
* @brief Writes a file to local storage.
*
* Port-specific file write for crytographic information.
*
* @param[in] pxLabel       Label of the object to be saved.
* @param[in] pucData       Data buffer to be written to file
* @param[in] ulDataSize    Size (in bytes) of data to be saved.
*
* @return The file handle of the object that was stored.
*/
CK_OBJECT_HANDLE PKCS11_PAL_SaveObject( CK_ATTRIBUTE_PTR pxLabel,
                                        CK_BYTE_PTR pucData,
                                        CK_ULONG ulDataSize )
{
    char                *pcFileName = NULL;
    CK_OBJECT_HANDLE    xHandle     = eInvalidHandle;

    /* Converts a label to its respective filename and handle. */

    prvLabelToFilenameHandle( pxLabel->pValue, &pcFileName, &xHandle );

    /* If your project requires additional PKCS#11 objects, add them here. */

    if( pcFileName != NULL )
    {
        if( prvFLASH_SaveFile( pcFileName, pucData, ulDataSize ) == pdFALSE )
        {
            xHandle = eInvalidHandle;
        }
    }

    return xHandle;
}

/**
* @brief Translates a PKCS #11 label into an object handle.
*
* Port-specific object handle retrieval.
*
*
* @param[in] pxLabel         Pointer to the label of the object
*                           who's handle should be found.
* @param[in] usLength       The length of the label, in bytes.
*
* @return The object handle if operation was successful.
* Returns eInvalidHandle if unsuccessful.
*/
CK_OBJECT_HANDLE PKCS11_PAL_FindObject( CK_BYTE_PTR pxLabel,
                                        CK_ULONG usLength )
{
    uint8_t     *pucData = NULL;
    uint32_t    dataSize = 0;  
    /* Avoid compiler warnings about unused variables. */
    ( void ) usLength;
  
    CK_OBJECT_HANDLE xHandle = eInvalidHandle;
    char * pcFileName = NULL;

    /* Converts a label to its respective filename and handle. */
    prvLabelToFilenameHandle( pxLabel,
                              &pcFileName,
                              &xHandle );

    if( pcFileName != NULL )
    {
        /* Check if object exists/has been created before returning. */

        if( pdTRUE != prvFLASH_ReadFile( pcFileName, &pucData, &dataSize) )
        {
            xHandle = eInvalidHandle;
        }
    }

    return xHandle;
}


/**
* @brief Gets the value of an object in storage, by handle.
*
* Port-specific file access for cryptographic information.
*
* This call dynamically allocates the buffer which object value
* data is copied into.  PKCS11_PAL_GetObjectValueCleanup()
* should be called after each use to free the dynamically allocated
* buffer.
*
* @sa PKCS11_PAL_GetObjectValueCleanup
*
* @param[in] pcFileName    The name of the file to be read.
* @param[out] ppucData     Pointer to buffer for file data.
* @param[out] pulDataSize  Size (in bytes) of data located in file.
* @param[out] pIsPrivate   Boolean indicating if value is private (CK_TRUE)
*                          or exportable (CK_FALSE)
*
* @return CKR_OK if operation was successful.  CKR_KEY_HANDLE_INVALID if
* no such object handle was found, CKR_DEVICE_MEMORY if memory for
* buffer could not be allocated, CKR_FUNCTION_FAILED for device driver
* error.
*/
CK_RV PKCS11_PAL_GetObjectValue( CK_OBJECT_HANDLE xHandle,
                                      CK_BYTE_PTR * ppucData,
                                      CK_ULONG_PTR pulDataSize,
                                      CK_BBOOL * pIsPrivate )
{

    CK_RV ulReturn = CKR_OK;
    char        *pcFileName     = NULL;
    uint8_t     *pucData;

    if( xHandle == eAwsDeviceCertificate )
    {
        pcFileName = pkcs11palFILE_NAME_CLIENT_CERTIFICATE;
        *pIsPrivate = CK_FALSE;
    }
    else if( xHandle == eAwsDevicePrivateKey )
    {
        pcFileName = pkcs11palFILE_NAME_KEY;
        *pIsPrivate = CK_TRUE;
    }
    else if( xHandle == eAwsDevicePublicKey )
    {
        /* Public and private key are stored together in same file. */
        pcFileName = pkcs11palFILE_NAME_KEY;
        *pIsPrivate = CK_FALSE;
    }
    else if( xHandle == eAwsCodeSigningKey )
    {
        pcFileName = pkcs11palFILE_CODE_SIGN_PUBLIC_KEY;
        *pIsPrivate = CK_FALSE;
    }
    else
    {
        ulReturn = CKR_KEY_HANDLE_INVALID;
    }

    if( pcFileName != NULL )
    {
        /* Check if object exists/has been created before returning. */

        if( pdTRUE != prvFLASH_ReadFile( pcFileName, ppucData, pulDataSize) )
        {
            xHandle = eInvalidHandle;
        }
    }
    return ulReturn;
}


/**
* @brief Cleanup after PKCS11_GetObjectValue().
*
* @param[in] pucData       The buffer to free.
*                          (*ppucData from PKCS11_PAL_GetObjectValue())
* @param[in] ulDataSize    The length of the buffer to free.
*                          (*pulDataSize from PKCS11_PAL_GetObjectValue())
*/
void PKCS11_PAL_GetObjectValueCleanup( CK_BYTE_PTR pucData,
                                       CK_ULONG ulDataSize )
{
    /* Unused parameters. */
    ( void ) pucData;
    ( void ) ulDataSize;

    /* Since no buffer was allocated on heap, there is no cleanup
     * to be done. */    
}
