/**
  ******************************************************************************
  * @file    lib_NDEF_Text.c
  * @author  MMY Application Team
  * @version V1.0.0
  * @date    19-March-2014
  * @brief   This file help to manage NDEF file that represent Text.
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

/* Includes ------------------------------------------------------------------*/
#include "lib_NDEF_Text.h"


/** @addtogroup NFC_libraries
 * 	@{
 */


/** @addtogroup lib_NDEF
  * @{
  */
  
  /** @defgroup libText_Private_Functions private functions for NDEF text library
  * @{  
  */ 

/**
 * @brief  This buffer contains the data send/received by TAG
 */
extern uint8_t NDEF_Buffer [NDEF_MAX_SIZE];

    
/**
 * @brief  This structure contains the data of the CC file
 */
extern sCCFileInfo CCFileStruct;    

/**
 * @brief  This structure contains the information encapsuled in the record header
 *				 with few more for SW purpose
 */
extern struct sRecordInfo RecordStruct;

static void NDEF_FillTextStruct( uint8_t* pPayload, uint32_t PayloadSize, sTextInfo *pTextStruct);
static void NDEF_ReadURI_Text ( struct sRecordInfo *pRecordStruct, sTextInfo *pTextStruct );

/**
  * @brief  This fonction fill Text structure with information of NDEF message
	* @param	pPayload : pointer on the payload data of the NDEF message
	* @param	PayloadSize : number of data in the payload
	* @param	pTextStruct : pointer on the structure to fill
  * @retval NONE 
  */
static void NDEF_FillTextStruct( uint8_t* pPayload, uint32_t PayloadSize, sTextInfo *pTextStruct)
{
	uint32_t langCodeSize;
        
/*
        Offset  Content         Explanation                             Syntactical info
        
          0     N/A             IL flag = 0 (no ID field), 
                                SF=1 (Short format)
        
          1     0x01            Length of the record name
        
          2     0x10            The length of the payload data (16 bytes)
        
          3     “T”             The binary encoding of the name, as defined in [1]
                                NDEF record header
        
          4     0x02            Status byte: This is UTF-8, and
                                has a two-byte language code
        
          5     “en”            “en” is the ISO code for “English” Payload
        
          7     text data       UTF-8 string “Hello, world!” The actual body text

*/        
	
	/* First charactere force to NULL in case not matching found */
	*pTextStruct->Message = 0;	       
        langCodeSize = *pPayload;         
        strcpy(pTextStruct->Message, (char const*)pPayload+langCodeSize+1);
}

/**
  * @brief  This fonction read the Text and store data in a structure
	* @param	pRecordStruct : Pointer on the record structure
	* @param	pTextStruct : pointer on the structure to fill
  * @retval NONE 
  */
static void NDEF_ReadURI_Text ( struct sRecordInfo *pRecordStruct, sTextInfo *pTextStruct )
{
	uint8_t* pPayload;
	uint32_t PayloadSize;
        
/*
        Offset  Content         Explanation                             Syntactical info
        
          0     N/A             IL flag = 0 (no ID field), 
                                SF=1 (Short format)
        
          1     0x01            Length of the record name
        
          2     0x10            The length of the payload data (16 bytes)
        
          3     “T”             The binary encoding of the name, as defined in [1]
                                NDEF record header
*/        
	
	PayloadSize = ((uint32_t)(pRecordStruct->PayloadLength3)<<24) | ((uint32_t)(pRecordStruct->PayloadLength2)<<16) |
    ((uint32_t)(pRecordStruct->PayloadLength1)<<8)  | pRecordStruct->PayloadLength0;
	
	/* Read record header */
	pPayload = (uint8_t*)(pRecordStruct->PayloadBufferAdd);
        
        
	
	if( pRecordStruct->NDEF_Type == TEXT_TYPE)
		NDEF_FillTextStruct(pPayload , PayloadSize, pTextStruct);
  
}

/**
  * @}
  */


/** @defgroup libText_Public_Functions public functions for NDEF text library
  * @{  
  */ 

/**
  * @brief  This fonction read NDEF and retrieve Text information if any
	* @param	pRecordStruct : Pointer on the record structure
	* @param	pTextStruct : pointer on the structure to fill 
  * @retval SUCCESS : Text information from NDEF have been retrieve
	* @retval ERROR : Not able to retrieve Text information
  */
uint16_t NDEF_ReadText(struct sRecordInfo *pRecordStruct, sTextInfo *pTextStruct)
{
	uint16_t status = ERROR;
	uint16_t FileId=0;
  
	if( pRecordStruct->NDEF_Type == TEXT_TYPE )
	{	
		NDEF_ReadURI_Text(pRecordStruct, pTextStruct );
		status = SUCCESS;
	}
	
	CloseNDEFSession(FileId);
	
	return status;
}

/**
  * @brief  This fonction write the NDEF file with the Text data given in the structure
	* @param	pTextStruct : pointer on structure that contain the Text information
  * @retval SUCCESS : the function is succesful
	* @retval ERROR : Not able to store NDEF file inside tag.
  */
uint16_t NDEF_WriteText ( sTextInfo *pTextStruct )
{
	/* Text is an URI but can be included in a smart poster to add text to give instruction to user for instance */
	
	/* Text (smart poster) Record Header */
  /************************************/	
  /*	7 |  6 |  5 |  4 |  3 | 2  1  0 */
  /*----------------------------------*/	
  /* MB   ME   CF   SR   IL    TNF    */  /* <---- CF=0, IL=0 and SR=1 TNF=1 NFC Forum Well-known type*/
  /*----------------------------------*/	
  /*					TYPE LENGTH 						*/
  /*----------------------------------*/
  /*				PAYLOAD LENGTH 3 					*/	/* <---- Used only if SR=0 */
  /*----------------------------------*/
  /*			  PAYLOAD LENGTH 2 					*/  /* <---- Used only if SR=0 */
  /*----------------------------------*/
  /*				PAYLOAD LENGTH 1 					*/  /* <---- Used only if SR=0 */
  /*----------------------------------*/	
  /*				PAYLOAD LENGTH 0 					*/  
  /*----------------------------------*/
  /*					ID LENGTH 							*/  /* <---- Not Used  */
  /*----------------------------------*/
  /*							TYPE 								*/
  /*----------------------------------*/
  /*							 ID                 */  /* <---- Not Used  */ 
  /************************************/
  
    return ERROR;
      
}

/**
  * @brief  This fonction read NDEF and retrieve WifiToken information if any
	* @param	pTextToken : Pointer on the record structure
  * @retval SUCCESS : WifiToken information from NDEF have been retrieve
	* @retval ERROR : Not able to retrieve WifiToken information
  */
uint16_t TT4_ReadTextToken(sTextInfo *pTextToken)
{
	uint16_t status = ERROR;
	sCCFileInfo *pCCFile;
	struct  sRecordInfo *pRecordStruct;
	
	pCCFile = &CCFileStruct;
	pRecordStruct = &RecordStruct;
  
	if(OpenNDEFSession(pCCFile->FileID, ASK_FOR_SESSION) == SUCCESS)
	{	
		if(NDEF_IdentifyNDEF( pRecordStruct, NDEF_Buffer) == SUCCESS)
		{
			status = NDEF_ReadText(pRecordStruct, pTextToken);
		}
		CloseNDEFSession(pCCFile->FileID);
	}
	
	return status;
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/


