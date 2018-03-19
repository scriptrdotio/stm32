/**
  ******************************************************************************
  * @file    lib_NDEF_Text.h
  * @author  MMY Application Team
  * @version V1.0.0
  * @date    20-November-2013
  * @brief   This file help to manage Text NDEF file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MMY-ST Liberty SW License Agreement V2, (the "License");
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

#include "lib_TagType4.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIB_NDEF_TEXT_H
#define __LIB_NDEF_TEXT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lib_NDEF.h"
	 
/** @addtogroup NFC_libraries
  * @{
  */


/** @addtogroup lib_NDEF
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Text structure, to store Phone Number, Message and optional information
  */	 	 
typedef struct 
{
	char Message[400];          /**< text message */
}sTextInfo;

uint16_t TT4_ReadTextToken(sTextInfo *pTextToken);	 	 
uint16_t NDEF_ReadText(struct sRecordInfo *pRecordStruct, sTextInfo *pTextStruct);
uint16_t NDEF_WriteText( sTextInfo *pTextStruct );

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif
	 
#endif /* __LIB_NDEF_TEXT_H */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
