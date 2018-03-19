   /**
  ******************************************************************************
  * @file    main.c
  * @author  Central LAB, MP
  * @version V2.0.0
  * @date    11-November-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  * 
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"
#include "ctype.h"
#include "string.h"
#include "math.h"
#include "ScriptrIO_Config.h"
#include "stm32f4xx_I2C.h"
#include "stm32f4xx_periph_conf.h"
#include "lib_TagType4.h"
#include "lib_NDEF_Text.h"
#ifndef __IKS01A1
#include "fft.h"
#endif // __IKS01A1

/**
   * @mainpage FP-CLD-WATSON1  firmware package
   * <b>Introduction</b> <br>
   * FP-CLD-WATSON1 provides a software running on STM32 which offers a complete middleware to build applications based on Wi-Fi connectivity (SPW01SA) and to connect STM32 Nucleo boards 
   * with IBM Watson IOT Platform service. This firmware package includes Components Device Drivers, Board Support Package and example application for STMicroelectronics X-NUCLEO-IDW01M1
   * X-NUCLEO-NFC01A1, X-NUCLEO-IKS01A1.

   * <b>IBM Watson IOT Platform example applications:</b><br>
   * This application reads the sensor data values from the accelerometer, magnetometer and Gyroscope, which are transmitted to the IBM IoT Cloud through WiFi. 
   * The URL of the web page where sensors data can be visualized is also written in NFC tag. 
   * The application is configured by default to run in Quickstart mode for data visualization only, but can be quickly modified in order to register and control the 
   * device using IBM Watson IOT Platform service(the latter mode requires an account on IBM Watson). 
  
   * <b>Toolchain Support</b><br>
   * The library has been developed and tested on following toolchains:
   *        - IAR Embedded Workbench for ARM (EWARM) toolchain V7.70 + ST-Link
   *        - Keil Microcontroller Development Kit (MDK-ARM) toolchain V5.10 + ST-LINK
   *        - System Workbench for STM32 (SW4STM32) V1.2 + ST-LINK
*/


/** @addtogroup FP-CLD-WATSON1
  * @{
  */

/* Extern () ------------------------------------------------------------*/
extern UART_HandleTypeDef UartMsgHandle;
extern char print_msg_buff[512];
extern WiFi_Status_t GET_Configuration_Value(char* sVar_name,uint32_t *aValue);
extern int uartReceiveChar(void);

/* Private define ------------------------------------------------------------*/
#define WIFI_SCAN_BUFFER_LIST           32
#define WIFI_SCAN_BUFFER_SIZE           512
#define MQTT_BUFFER_SIZE                512
#define MQTT_PUBLISH_DELAY              700 //4000
#define APPLICATION_DEBUG_MSG           1
// Timeout for setting wifi parameters 
#define APP_TIMEOUT 5000
#define NDEF_WIFI 32
#define BLUEMSYS_FLASH_ADD ((uint32_t)0x08060000)
#define BLUEMSYS_FLASH_SECTOR FLASH_SECTOR_7
#define WIFI_CHECK_SSID_KEY ((uint32_t)0x12345678)
#define UNION_DATA_SIZE ((32+32+6+6)>>2)
#define USART_SPEED_MSG	460800

uNfcTokenInfo UnionNFCToken;
static System_Status_t status = MODULE_SUCCESS;

#if (CONFIG_USE_NFC==USE_NDEF_WIFI_RECORD)    

/**
  * @brief  Wifi token information. This is deprecated. 
  */
typedef union
{
  uint32_t Data[UNION_DATA_SIZE];       /**< contiguous data buffer */
  sWifiTokenInfo TokenInfo;             /**< wifi token */
}uWifiTokenInfo;
static uWifiTokenInfo UnionWifiToken;
#endif

/* MQTT Callback for messages arrived -----------------------------------------*/
void messageArrived(MessageData* md);
void messageArrived_echo(MessageData* md);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void prepare_json_pkt (uint8_t * buffer);
void prepare_json_pkt_fft(uint8_t * buffer);
static System_Status_t  Get_MAC_Add (char *macadd);
static System_Status_t ReCallSSIDPasswordFromMemory(void);
static System_Status_t ResetSSIDPasswordInMemory(void);
static System_Status_t SaveSSIDPasswordToMemory(void);
static System_Status_t ConfigAPSettings(void);
static System_Status_t wifi_get_AP_settings(void);
static System_Status_t InitNucleo(int i_usart_speed);
static System_Status_t InitSensors(void);
static System_Status_t ConfigWiFi(void);
static System_Status_t InitMQTT(void);
static System_Status_t getWiFiParamFromTerminal();
static System_Status_t getScriptrParamsFromTerminal(void);
static void printUserParams();
static void setDefaultConfig();
static void ShowHwConfiguration();
int progFunctionInit(void);
static System_Status_t testUserParamSanity();
static enum QoS getScriptrQoS(void);
static void getScriptrUsername(void);
static void getScriptrPassword(void);
static void getScriptrHostname(void);
static void getScriptrToken(void);
static System_Status_t getWifiSSID(void);
static System_Status_t getWifiSEcurityKey(void);
static void getWifiAuthMode(void);

#if (CONFIG_USE_NFC==USE_NDEF_WIFI_RECORD)
static System_Status_t InitNFC(void);
static System_Status_t ReadWifiTokenFromNFC(void);
static System_Status_t WriteURLtoNFC(char* UrlForNFC);
static System_Status_t ReadUserParamsFromNFC();
#elif (CONFIG_USE_NFC==USE_NDEF_TEXT_RECORD)
static System_Status_t InitNFC(void);
static System_Status_t ReadParametersFromNFC(void);
static System_Status_t WriteURLtoNFC(char* UrlForNFC);
static void ParseStringRecord(char* str);
static void parseToken(char *tt);
static void setParameterValue(char* tokenId, char *value);
static void removeWhiteSpaces(char* input);
static System_Status_t ReadUserParamsFromNFC();
sTextInfo textToken;
#endif 

static void NotifyLEDOn();
static void NotifyLEDOff(void);
static void NotifyLEDBlink(unsigned int msPeriod);

typedef enum
{
  CLOUD,
  FFT
} Prog_Function_t;

/* WiFi. Private variables ---------------------------------------------------------*/  
typedef enum {
  wifi_state_reset = 0,
  wifi_state_ready,
  wifi_state_idle,
  wifi_state_connected,
  wifi_state_connecting,
  wifi_state_disconnected,
  wifi_state_error,
  wifi_state_socket_close,
  wifi_state_reconnect,
  mqtt_socket_create,
  client_conn,
  mqtt_connect,
  mqtt_sub,
  mqtt_pub,
  mqtt_device_control,
  wifi_undefine_state       = 0xFF,  
} wifi_state_t;

wifi_state_t wifi_state;
wifi_config config;
wifi_scan net_scan[WIFI_SCAN_BUFFER_LIST];
uint8_t console_input[1], console_count=0;
wifi_bool set_AP_config = WIFI_FALSE, SSID_found = WIFI_FALSE;
uint8_t json_buffer[MQTT_BUFFER_SIZE];

#if USE_WIFI_DEFAULT_SETTINGS
  /* Default configuration SSID/PWD */ 
  char *ssid = "STM";
  char *seckey = "STMdemoPWD";  
#else
  char *ssid = NULL;
  char *seckey = NULL;
#endif
  
WiFi_Priv_Mode mode = WPA_Personal; 

/* MQTT. Private variables ---------------------------------------------------------*/  
unsigned char MQTT_read_buf[MQTT_BUFFER_SIZE];
unsigned char MQTT_write_buf[MQTT_BUFFER_SIZE];
Network  n;
Client  c;
MQTTMessage  MQTT_msg;
MQTT_vars mqtt_scriptr_setup;
MQTTPacket_connectData options = MQTTPacket_connectData_initializer; 
System_Status_t nfcStatus = MODULE_ERROR;
  
/* Timer for MQTT */
static System_Status_t MQTTtimer_init(void);
TIM_HandleTypeDef        MQTTtimHandle;
TIM_IC_InitTypeDef       sConfig;
   
void TIM4_IRQHandler(void);

/* Sensors. Private variables ---------------------------------------------------------*/  
void *ACCELERO_handle = NULL;
void *GYRO_handle = NULL;
void *MAGNETO_handle = NULL;
void *HUMIDITY_handle = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle = NULL;
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};
 
Prog_Function_t   progFunction;
volatile uint8_t  AXL_DRDY_received = 0;
volatile uint32_t Int_Current_Time1 = 0;
volatile uint32_t Int_Current_Time2 = 0;
uint8_t RxBuffer;
uint8_t sensorInUse;

/**
  * @brief  Main program
  * 
  * @retval None
  */
int main(void) {

  WiFi_Status_t status_spwf = WiFi_MODULE_SUCCESS;
  int8_t ret_code;
  uint32_t i;
    
   /* Init Nucleo Board */
  if (InitNucleo (USART_SPEED_MSG) !=  MODULE_SUCCESS ) {  
    printf("\n\rFailed to Initialize Nucleo board \n\r"); 
    return 0; 
  } else {
    printf("\n\rNucleo board initialized."); 
  }
  
  HAL_Delay(1000);
  printf("\033[2J\033[0;0H******************************************************************************\r\n");  
  printf("\n\r**************************************************************************\r\n");
  printf("***                Scriptr.io SDK for STM32 Nucleo                 ***\r\n");
#ifndef __IKS01A1  
  printf("***                        with FFT extension                          ***\r\n");
  printf("***                                 &                                  ***\r\n");
  printf("***                         FFT demo project                           ***\r\n");
#endif  
  printf("**************************************************************************\n\r");
  printf("Firmware version: %d.%d.%d\n\r", SCRIPTR_IO_DEMO_APP_VERSION_MAJOR, SCRIPTR_IO_DEMO_APP_VERSION_MINOR, SCRIPTR_IO_DEMO_APP_VERSION_MICRO);
#if (CONFIG_USE_NFC==USE_NDEF_WIFI_RECORD) 
  printf("Warning: USE_NDEF_WIFI_RECORD macro is deprecated. It is enabled in this build.\n\r");  
#endif
  fflush(stdout);
  
  /* Initialize NFC expansion board if it is present*/
  if ((nfcStatus=InitNFC()) < 0 ) {  
    printf("\n\rFailed to Initialize NFC board."); 
    if(nfcStatus==MODULE_NFC_NOT_PRESENT) {     
      printf("\n\rUnable to detect NFC HW. Disabling NFC usage."); 
      nfcStatus = MODULE_NFC_NOT_USED;
    } else {
      return nfcStatus;
    }
  } else {
    nfcStatus = MODULE_NFC_IN_USE;    
    printf("\n\rNFC HW initialized.");   
  }
  
  ShowHwConfiguration();
  
  HAL_Delay(1000);
    
  /* Init sensors expansion board */
  if ((status=InitSensors()) < 0 ) {  
    printf("Failed to Initialize Sensors board.\r\n"); 
    return status; 
  } else {
    printf("Sensors board initialized.\r\n");   
    if(BSP_ACCELERO_Get_Instance(ACCELERO_handle, &sensorInUse) == COMPONENT_ERROR) {
      printf("Failed to Initialize Sensors board.\r\n");
      while(1);
    }

    #ifndef __IKS01A1    
    if(sensorInUse == LSM6DSL_X_0){
      printf("LSM6DSL was initialized as accelerometer to use. FFT demo project and extension is disabled.\r\n(Use IIS2DH or LSM303AGR sensor to enable FFT functionalities)\r\n");
    } else if(sensorInUse == IIS2DH_X_0){
      printf("IIS2DH was initialized as accelerometer to use.\r\n");
    } else if(sensorInUse == LSM303AGR_X_0){
      printf("LSM303AGR was initialized as accelerometer to use.\r\n");
    } 
    #endif // __IKS01A1 
   
  }
  
  HAL_Delay(1000);  

  if( !progFunctionInit() ){
    printf("\033[2J\033[0;0HError during program initialization...\r\nPlease restart nucleo board.\r\n");
    while(1);
  }
  
  while (1)
  {
    switch (progFunction)
	{      
          
		/****************************************** CLOUD part of main loop ************************************************/       
		default:                     
		case CLOUD: 
		{
			switch (wifi_state) 
			{ 
			case wifi_state_reconnect:
				  printf("\r\n [E] WiFi connection lost. Wait 10 sec then reconnect with parameters:  \r\n");
				  printf((char *)UnionNFCToken.configParams.NetworkSSID);
				  printf("\r\n");
				  printf((char *)UnionNFCToken.configParams.NetworkKey);
				  printf("\r\n");
				  HAL_Delay(10000);
				  status_spwf = wifi_connect(UnionNFCToken.configParams.NetworkSSID, UnionNFCToken.configParams.NetworkKey, mode); 
				  if(status_spwf!=WiFi_MODULE_SUCCESS){
					 printf("\r\n[E].Error cannot connect with WiFi \r\n");
					 wifi_state = wifi_state_reconnect;
				  } else{
					   printf("\r\n [D] Reconnecting....  \r\n");
					   printf((char *)UnionNFCToken.configParams.NetworkSSID);                
					   wifi_state = wifi_state_idle;
				  }
				  break;       
			  
			 case wifi_state_reset:
				break;
			  
			 case wifi_state_ready: 
			   HAL_Delay(20);
				status_spwf = wifi_network_scan(net_scan, WIFI_SCAN_BUFFER_LIST);
				if(status_spwf==WiFi_MODULE_SUCCESS)
				{             
				  if (strcmp(UnionNFCToken.configParams.AuthenticationType, "NONE") == 0)
					   mode = None;
				  else if (strcmp(UnionNFCToken.configParams.AuthenticationType, "WEP") == 0)
					   mode = WEP;
				  else
					   mode = WPA_Personal;     
		   
				  for ( i=0; i<WIFI_SCAN_BUFFER_LIST; i++ )
					{
						if(( (char *) strstr((const char *)net_scan[i].ssid,(const char *)UnionNFCToken.configParams.NetworkSSID)) !=NULL) {    
						   SSID_found = WIFI_TRUE;
						   memcpy(UnionNFCToken.configParams.AuthenticationType, "WPA2", strlen("WPA2"));                                      
						   status_spwf = wifi_connect(UnionNFCToken.configParams.NetworkSSID, UnionNFCToken.configParams.NetworkKey, mode);
						   
						   if(status_spwf!=WiFi_MODULE_SUCCESS) {
							  printf("\r\n[E].Error cannot connect to WiFi network \r\n");
							  return 0;
						   }else{
							  printf("\r\n [D] Connecting to network with SSID ");
							  printf((char *)UnionNFCToken.configParams.NetworkSSID);  
						   }
						   break;
						 }              
					}
				  
					if(!SSID_found)  
					{                  
						   /* Can happen in crowdy environments */ 
						   printf("\r\n[E]. Error, given SSID not found! Trying to force connection with SSID: \r\n");
						   printf((char *)UnionNFCToken.configParams.NetworkSSID); 
						   status_spwf = wifi_connect(UnionNFCToken.configParams.NetworkSSID, UnionNFCToken.configParams.NetworkKey, mode);
						   if(status_spwf!=WiFi_MODULE_SUCCESS){
							  printf("\r\n[E].Error cannot connect with WiFi \r\n");
							  return 0;
						   } else{
								printf("\r\n [D] Connected to network with SSID  \r\n");
								printf((char *)UnionNFCToken.configParams.NetworkSSID);  
						   } 
				   
					}       
					memset(net_scan, 0x00, sizeof(net_scan));
				}
				else
					printf("\r\n[E]. Error, network AP not found! \r\n");
		  
				wifi_state = wifi_state_idle; 
				break;

			case wifi_state_connected:
				// Low power mode not used
				break;
			 
			case mqtt_socket_create:
			  printf("\r\n Opening socket to scriptr.io at %s : %s : % s", mqtt_scriptr_setup.hostname, mqtt_scriptr_setup.port, &mqtt_scriptr_setup.protocol);
		      int connect_status = spwf_socket_create (&n,mqtt_scriptr_setup.hostname, mqtt_scriptr_setup.port, &mqtt_scriptr_setup.protocol);
			  if(connect_status <0){
				  printf("\r\n [E]. Socket opening with scriptr.io MQTT broker failed. Please check MQTT configuration. Trying reconnection.... \r\n");
				  printf((char*)mqtt_scriptr_setup.hostname);
				  printf("\r\n");
				  printf((char*)(&mqtt_scriptr_setup.protocol));
				  printf("\r\n");
				  wifi_state=mqtt_socket_create;
				  HAL_Delay(2000);
			  }else{   
				  /* Initialize MQTT client structure */
				  MQTTClient(&c,&n, 4000, MQTT_write_buf, sizeof(MQTT_write_buf), MQTT_read_buf, sizeof(MQTT_read_buf));
				  wifi_state=mqtt_connect;
			  
				  printf("\r\n [D]. Created socket with MQTT broker. \r\n");
			  }    
			  break;
			  
			case mqtt_connect:     
			  options.MQTTVersion = 3;
			  options.clientID.cstring = (char*)mqtt_scriptr_setup.clientid;
			  //options.username.cstring = (char*)mqtt_scriptr_setup.username;
			  //options.password.cstring = (char*)mqtt_scriptr_setup.password;

			  if ( (ret_code = MQTTConnect(&c, &options)) != 0){  
				  printf("\r\n [E]. Client connection with scriptr.io MQTT broker failed with Error Code %d \r\n", (int)ret_code);
				  switch (ret_code)
				  {
						case C_REFUSED_PROT_V:
						  printf("Connection Refused, unacceptable protocol version \r\n");
						  break;
						case C_REFUSED_ID:  
						  printf("Connection Refused, identifier rejected \r\n");
						  break;
						case C_REFUSED_SERVER:
						  printf("Connection Refused, Server unavailable \r\n");
						  break;
						case C_REFUSED_UID_PWD:
						  printf("Connection Refused, bad user name or password \r\n");
						  break;
						case C_REFUSED_UNAUTHORIZED:   
						  printf("Connection Refused, not authorized \r\n");
						  break;
				  }  
				  spwf_socket_close (&n);
				  wifi_state = mqtt_socket_create;
				  HAL_Delay(2000);
				 /* Turn LED2 Off to indicate that we are now disconnected from MQTT broker */
				  NotifyLEDOff();
			  }else {  
				  //this is currently a publish only code
				  wifi_state = mqtt_pub;
				 /*if ( mqtt_scriptr_setup.ibm_mode == QUICKSTART ){
					 printf("\r\n [D]. Connected with IBM MQTT broker for Quickstart mode (only MQTT publish supported) \r\n");
					 // Quickstart mode. We only publish data.
					 wifi_state = mqtt_pub;
				 } else{
					 printf("\r\n [D]. Connected with IBM MQTT broker for Registered devices mode (requires account on IBM Watson IOT Platform service; publish/subscribe supported) \r\n");
					 // Registered device mode.
					 wifi_state = mqtt_sub;
				 }*/
				 /* Turn LED2 ON to indicate that we are now connected to IBM MQTT broker */
				 NotifyLEDOn();
			  }
			  HAL_Delay(1500);
			 break;
			 
			case mqtt_sub: 
			  printf("\r\n [D] mqtt sub  \r\n");
			  
			  /* Subscribe to topic */ 
			  /*
			  printf("\r\n [D] Subscribing topic:  \r\n");
			  printf((char *)mqtt_scriptr_setup.sub_topic);

			   if( MQTTSubscribe(&c, (char*)mqtt_scriptr_setup.sub_topic, mqtt_scriptr_setup.qos, messageArrived) < 0) {
				  printf("\r\n [E]. Subscribe with scriptr.io MQTT broker failed. Please check MQTT configuration. \r\n");
			   } else {
				 printf("\r\n [D] Subscribed to topic:  \r\n");
				 printf((char *)mqtt_scriptr_setup.sub_topic);
			   }
			   */
			   HAL_Delay(1500);   
			   wifi_state=mqtt_pub; 
			  break;
			  
			case mqtt_pub:
                          #ifndef __IKS01A1
                          if(sensorInUse == IIS2DH_X_0 || sensorInUse == LSM303AGR_X_0)
                          {
                            BSP_ACCELERO_Set_Active_Axis_Ext(ACCELERO_handle, Z_AXIS);
                            BSP_ACCELERO_Enable_HP_Filter_Ext(ACCELERO_handle, HPF_MODE_NORMAL, CUTOFF_MODE2);
                            BSP_ACCELERO_Set_INT1_DRDY_Ext(ACCELERO_handle, INT1_DRDY_ENABLED);
                            BSP_ACCELERO_ClearDRDY_Ext(ACCELERO_handle, Z_AXIS);
                            
                            acquireData();
                            do_fft();
                            
                            BSP_ACCELERO_Set_INT1_DRDY_Ext(ACCELERO_handle, INT1_DRDY_DISABLED);
                            BSP_ACCELERO_Disable_HP_Filter_Ext(ACCELERO_handle);
                            BSP_ACCELERO_Set_Active_Axis_Ext(ACCELERO_handle, ALL_ACTIVE);
                          }
                          #endif//__IKS01A1
			  prepare_json_pkt(json_buffer);

			  MQTT_msg.qos=QOS0;
			  MQTT_msg.dup=0;
			  MQTT_msg.retained=1;
			  MQTT_msg.payload= (char *) json_buffer;
			  MQTT_msg.payloadlen=strlen( (char *) json_buffer);  
			  
			  /* Publish MQTT message */
			  int pub_success = MQTTPublish(&c,(char*)mqtt_scriptr_setup.pub_topic,&MQTT_msg);
			  if ( pub_success < 0) {
				  printf("\r\n [E]. Failed to publish data. Reconnecting to MQTT broker.... %i \r\n", pub_success);
				  wifi_state=mqtt_connect;
			  }  else {

				  HAL_Delay(MQTT_PUBLISH_DELAY);

				  printf("\r\n [D]. Sensor data are published to scriptr.io\r\n");
				  printf(MQTT_msg.payload);
				  printf("\r\n");
				 /* Blink LED2 to indicate that sensor data is published */
				 NotifyLEDBlink(500);
			   } 
			   break;
		   
			case wifi_state_idle:
				printf(".");
				fflush(stdout);
				HAL_Delay(500);
				break;  
				
			case wifi_state_disconnected:        
				NotifyLEDOff();
				wifi_state=wifi_state_reconnect;
				break;
				
			default:
			  break;
			}     
		  
		  break; 
		}
  
		/****************************************** FFT part of main loop **************************************************/           
                #ifndef __IKS01A1
		case FFT:
		{	  
		  FFT_main();      
		  break;
		}
                #endif       // __IKS01A1
	} // end of switch (progFunction)
  }  
}


#if (CONFIG_USE_NFC==USE_NDEF_TEXT_RECORD)
/**
  * @brief  Read Access Point parameters from NFC
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t ReadParametersFromNFC(void) {
  status = MODULE_ERROR;
  uint32_t tickstart;

  tickstart = user_currentTimeGetTick();
  while ( (user_currentTimeGetTick() - tickstart ) < APP_TIMEOUT)  {
    if(TT4_ReadTextToken(&textToken) == SUCCESS) {
      status = MODULE_SUCCESS;
      ParseStringRecord(textToken.Message);
      return status;
    }
  }
  
  return status;
}
#elif (CONFIG_USE_NFC==USE_NDEF_WIFI_RECORD) 
/**
  * @brief  Read Access Point parameters from NFC
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t ReadWifiTokenFromNFC(void) {
  status = MODULE_ERROR;
  uint32_t tickstart;

  tickstart = user_currentTimeGetTick();
  while ( (user_currentTimeGetTick() - tickstart ) < APP_TIMEOUT)  {
    if(TT4_ReadWifiToken(&UnionWifiToken.TokenInfo) == SUCCESS) {
      status = MODULE_SUCCESS;
      /* copy user parameters from UnionWifiToken to UnionNFCToken */
      strcpy(UnionNFCToken.configParams.NetworkSSID, UnionWifiToken.TokenInfo.NetworkSSID);
      strcpy(UnionNFCToken.configParams.AuthenticationType, UnionWifiToken.TokenInfo.AuthenticationType);
      strcpy(UnionNFCToken.configParams.NetworkKey, UnionWifiToken.TokenInfo.NetworkKey);       
      return status;
    }
  }
      
  return status;
}
#endif


/**
  * @brief  Write URL to NFC
  * @param  char* UrlForNFC: string containing URL to write in NFC
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t WriteURLtoNFC(char* UrlForNFC)
{
  sURI_Info URI;
  System_Status_t status = MODULE_ERROR;
  
   /* Write URI */
   strcpy(URI.protocol,URI_ID_0x03_STRING);   
   strcpy(URI.URI_Message,(char *)UrlForNFC);
   strcpy(URI.Information,"\0");
              
   if(TT4_WriteURI(&URI) == SUCCESS) {
          printf("Written URL in NFC\r\n");
          printf((char *)UrlForNFC);
          printf("\r\n");
          status = MODULE_SUCCESS;
   } 
 
  return status; 
}

/**
  * @brief  Save Access Point parameters to FLASH
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t SaveSSIDPasswordToMemory(void)
{
  System_Status_t status = MODULE_ERROR;

  /* Reset Before The data in Memory */
  status = ResetSSIDPasswordInMemory();

  if(status==MODULE_SUCCESS) {
    /* Store in Flash Memory */
    uint32_t Address = BLUEMSYS_FLASH_ADD;
    int32_t WriteIndex;

   /* Unlock the Flash to enable the flash control register access *************/
   HAL_FLASH_Unlock();
   
   /* Write the Magic Number */
   {
     uint32_t MagicNumber = WIFI_CHECK_SSID_KEY;
      if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address,MagicNumber) == HAL_OK){
        Address = Address + 4;
      } else {
          printf("Error while writing in FLASH\r\n");
          status = MODULE_ERROR;
        }
   }
     
  /* Write the Wifi */
  for(WriteIndex=0;WriteIndex<UNION_DATA_SIZE_TEXT;WriteIndex++){
    if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address,UnionNFCToken.Data[WriteIndex]) == HAL_OK){
      Address = Address + 4;
    } else {
        printf("Error while writing in FLASH\r\n");
        status = MODULE_ERROR;
      }
    }       

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();
    
    printf("SaveSSIDPasswordToMemory => OK\r\n");
  }
  else
    printf("Error while resetting FLASH memory\r\n");
    
  return status;
}



/**
  * @brief  Erase Access Point parameters from FLASH
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t ResetSSIDPasswordInMemory(void)
{
  /* Reset Calibration Values in FLASH */
  System_Status_t status = MODULE_ERROR;

  /* Erase First Flash sector */
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;

  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = BLUEMSYS_FLASH_SECTOR;
  EraseInitStruct.NbSectors = 1;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    /* Error occurred while sector erase. 
      User can add here some code to deal with this error. 
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
      printf("Error while erasing FLASH memory\r\n");
  } else
      status = MODULE_SUCCESS;
  
  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return status;
}


/**
  * @brief  Read Access Point parameters from FLASH
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t ReCallSSIDPasswordFromMemory(void)
{
  System_Status_t status = MODULE_ERROR;
  
  uint32_t Address = BLUEMSYS_FLASH_ADD;
  __IO uint32_t data32 = *(__IO uint32_t*) Address;
  if(data32== WIFI_CHECK_SSID_KEY){
    int32_t ReadIndex;
    
  for(ReadIndex=0;ReadIndex<UNION_DATA_SIZE_TEXT;ReadIndex++){
    Address +=4;
    data32 = *(__IO uint32_t*) Address;
    UnionNFCToken.Data[ReadIndex]=data32;
  }  
    status = MODULE_SUCCESS;
  }
  else
    printf("\r\nFLASH Keyword not found.\r\n");
  
  return status;
}

/**
  * @brief  Read Access Point parameters from serial interface
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t wifi_get_AP_settings(void)
{
  uint8_t console_input[1], console_count=0;
  char console_ssid[NDEF_WIFI];
  char console_psk[NDEF_WIFI];
  WiFi_Priv_Mode mode; 
    
  
              printf("\r\nEnter the SSID: ");
              fflush(stdout);

              console_count=0;
              console_count=scanf("%s",console_ssid);
              printf("\r\n");
              // FIXME : Why 39. NDEF is 32
              if(console_count==NDEF_WIFI) 
              {
                        printf("Exceeded number of ssid characters permitted");
                        return MODULE_ERROR;
               }    

              printf("Enter the password: ");
              fflush(stdout);
              console_count=0;
              
              console_count=scanf("%s",console_psk);
              printf("\r\n");
              // FIXME : Why 19. NDEF is 32
              if(console_count==NDEF_WIFI) 
                    {
                        printf("Exceeded number of psk characters permitted.\r\n");
                        return MODULE_ERROR;
                    }    
              printf("Enter the authentication mode(0:Open, 1:WEP, 2:WPA2/WPA2-Personal): "); 
              fflush(stdout);
              scanf("%s",console_input);
              printf("\r\n");
              //printf("entered =%s\r\n",console_input);
              switch(console_input[0])
              {
                case '0':
                  mode = None;
                  break;
                case '1':
                  mode = WEP;
                  break;
                case '2':
                  mode = WPA_Personal;
                  break;
                default:
                  printf("\r\nWrong Entry. Priv Mode is not compatible\n");
                  return MODULE_ERROR;              
              }       
          
            /* use NDEF Text record data structures for obtaining input parameters from user. If we are using WIFI NDEF structure, we copy these parameters when needed. */
            memcpy(UnionNFCToken.configParams.NetworkSSID, console_ssid, strlen(console_ssid));
            memcpy(UnionNFCToken.configParams.NetworkKey, console_psk, strlen(console_psk));
            if (mode == None)
              memcpy(UnionNFCToken.configParams.AuthenticationType, "NONE", strlen("NONE"));
            else if (mode == WEP)
              memcpy(UnionNFCToken.configParams.AuthenticationType, "WEP", strlen("WEP"));
            else
              memcpy(UnionNFCToken.configParams.AuthenticationType, "WPA2", strlen("WPA2"));
            
            getScriptrParamsFromTerminal();
            return MODULE_SUCCESS;
}


/**
  * @brief  Initialize Nucleo board
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t InitNucleo(int i_usart_speed)
{
   System_Status_t status = MODULE_ERROR;
  
   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

   /* Configure the system clock */
   SystemClock_Config();

   /* configure the timers  */
   Timer_Config();

   UART_Configuration(115200);
   UART_Msg_Gpio_Init();
   USART_PRINT_MSG_Configuration(i_usart_speed);

   BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
   BSP_LED_Init(LED2);  // <hd>
   
  /* I2C Initialization */
  if(I2C_Global_Init()!=HAL_OK) {
    printf("I2C Init Error \r\n");
    return status;
  }
  else
    status = MODULE_SUCCESS;
   
   return status;
 }

/**
  * @brief  Initialize NFC board
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t InitNFC(void)
{
   System_Status_t status = MODULE_ERROR;
   int initFailCount = 0;
    /* Initialize the M24SR component */
  while (TT4_Init() != SUCCESS){
    initFailCount++;
    if(initFailCount==5) {
      return MODULE_NFC_NOT_PRESENT;
    }
  }
  status = MODULE_SUCCESS;
  return status;
}

/**
  * @brief  Initialize WiFi board
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t ConfigWiFi(void)
{
  System_Status_t status = MODULE_ERROR;

  /* Config WiFi : disable low power mode */
  config.power=wifi_active;
  config.power_level=high;
  config.dhcp=on;//use DHCP IP address
  config.web_server=WIFI_TRUE;  
  
  status = MODULE_SUCCESS;
  return status;
}  
  
/**
  * @brief  Initialize MQTT interface
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t InitMQTT(void)
{
  System_Status_t status = MODULE_ERROR;

  /* Initialize network interface for MQTT  */
  NewNetwork(&n);
  /* Initialize MQTT timers */
  status = MQTTtimer_init();
  if(status!=MODULE_SUCCESS)
  {
    printf("\r\n[E].Error in MQTTtimer_init \r\n");
    return status;
  }  

  status = MODULE_SUCCESS;
  return status;
} 


/**
  * @brief  Init timer for MQTT
  * 
  * @retval None
  */
static System_Status_t MQTTtimer_init(void){
  System_Status_t status = MODULE_ERROR;

 __TIM4_CLK_ENABLE();

  MQTTtimHandle.Instance = TIM4;
  MQTTtimHandle.Init.Period = 10-1;    //1 ms
  MQTTtimHandle.Init.Prescaler =  (uint32_t)(SystemCoreClock / 10000) - 1;
  MQTTtimHandle.Init.ClockDivision = 0;
  MQTTtimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;  
  if(HAL_TIM_Base_Init(&MQTTtimHandle) != HAL_OK)
  {
    return status;
  }
  
  if (HAL_TIM_Base_Start_IT(&MQTTtimHandle) != HAL_OK)
  {
    return status;
  }

   HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);
   HAL_NVIC_EnableIRQ(TIM4_IRQn);

   status = MODULE_SUCCESS;
   return status;
}

/**
  * @brief  Initialize sensors board
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t InitSensors(void)
{
   System_Status_t status = MODULE_ERROR;
 
#ifdef __IKS01A1
    /* Try to use LSM6DS3 DIL24 if present */
    if(BSP_ACCELERO_Init( LSM6DS3_X_0, &ACCELERO_handle)!=COMPONENT_OK){        
      /* otherwise try to use LSM6DS on board */
      if(BSP_ACCELERO_Init(LSM6DS0_X_0, &ACCELERO_handle)!=COMPONENT_OK){
        return status;
      }
    }
#else
  /* Try to use IIS2DH, LSM303AGR or LSM6DSL accelerometer */
  if(BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &ACCELERO_handle ) != COMPONENT_OK){
    return status;
  }        
#endif      

    /* gyro sensor */
    if(BSP_GYRO_Init( GYRO_SENSORS_AUTO, &GYRO_handle )!=COMPONENT_OK){
        return status;
    }

    /* magneto sensor */
    if(BSP_MAGNETO_Init( MAGNETOMETER_SENSOR_AUTO, &MAGNETO_handle )!=COMPONENT_OK){
        return status;
    }

    /* Force to use HTS221 */
    if(BSP_HUMIDITY_Init( HTS221_H_0, &HUMIDITY_handle )!=COMPONENT_OK){
        return status;
    }

    /* Force to use HTS221 */
    if(BSP_TEMPERATURE_Init( HTS221_T_0, &TEMPERATURE_handle )!=COMPONENT_OK){
        return status;
    }

    /* Try to use LPS25HB DIL24 if present, otherwise use LPS25HB on board */
    if(BSP_PRESSURE_Init( PRESSURE_SENSORS_AUTO, &PRESSURE_handle )!=COMPONENT_OK){
        return status;
    }
    
    /* Set ODR to 400Hz for FFT demo */
    if(BSP_ACCELERO_Set_ODR_Value( ACCELERO_handle, 400.0f ) != COMPONENT_OK)
    {
      return status;
    }

    /*  Enable all the sensors */
    BSP_ACCELERO_Sensor_Enable( ACCELERO_handle );
    BSP_GYRO_Sensor_Enable( GYRO_handle );
    BSP_MAGNETO_Sensor_Enable( MAGNETO_handle );
    BSP_HUMIDITY_Sensor_Enable( HUMIDITY_handle );
    BSP_TEMPERATURE_Sensor_Enable( TEMPERATURE_handle );
    BSP_PRESSURE_Sensor_Enable( PRESSURE_handle );

    status = MODULE_SUCCESS;
    return status;      
    
}



/**
* @brief  EXTI line detection callbacks
* @param  GPIO_Pin the pin connected to EXTI line
* @retval None
*/
#ifndef __IKS01A1
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  
  if( GPIO_Pin == M_INT1_O_PIN && sensorInUse == IIS2DH_X_0 ) {
    
    AXL_DRDY_received = 1;
    
  }

  if( GPIO_Pin == LSM303AGR_INT_O_PIN && sensorInUse == LSM303AGR_X_0 ) {
    
    AXL_DRDY_received = 1;
    
  }
}
#endif // __IKS01A1


/**
  * @brief  Wi-Fi callback activated when Wi-Fi is on 
  *   
  * @retval None
  */
void ind_wifi_on()
{
  printf("\r\n[D]. Wi-Fi on \r\n");
  wifi_state = wifi_state_ready;
}

/**
  * @brief  Wi-Fi callback activated when Wi-Fi is connected to AP 
  *   
  * @retval None
  */
void ind_wifi_connected()
{
  printf("\r\n [D] Connected to Wifi\r\n");
  wifi_state = mqtt_socket_create;
}

/**
  * @brief  Wi-Fi callback activated Wi-Fi is resuming from sleep mode 
  *   
  * @retval None
  */
void ind_wifi_resuming()
{
  printf("\r\n [E]. Wifi resuming from sleep user callback... \r\n");
}

/**
  * @brief  Wi-Fi callback activated in case of connection error
  *   
  * @retval None
  */
void ind_wifi_connection_error(WiFi_Status_t wifi_status)
{
  printf("\r\n [E]. WiFi connection error. Trying to reconnect to AP after some seconds ... \r\n"); 

  switch (wifi_status)
  {
     case WiFi_DE_AUTH:
         printf("[E] Error code WiFi_DE_AUTH  \r\n");        
         break;
     case WiFi_DISASSOCIATION:
         printf("[E] Error code WiFi_ DISASSOCIATION  \r\n");
         break;
     case WiFi_JOIN_FAILED:
         printf("[E] Error code WiFi_JOIN_FAILED  \r\n");
         break;
     case WiFi_SCAN_BLEWUP:
         printf("[E] Error code WiFi_ SCAN_BLEWUP \r\n");
         break;
     case WiFi_SCAN_FAILED:
         printf("[E] Error code WiFi_ SCAN_FAILED \r\n");
         break;    
     default:
         printf("[E] Undefined Error code \r\n");
         break;
  }
  
  wifi_state = wifi_state_disconnected;
}

/**
  * @brief  Wi-Fi callback activated when remote server is closed  
  * @param  socket_closed_id : socket identifier  
  * @retval None
  */
void ind_wifi_socket_client_remote_server_closed(uint8_t * socket_closed_id)
{
   printf("\r\n[E]. Remote disconnection from server. Trying to reconnect to MQTT broker... \r\n");
  // HAL_Delay (2000); 

   //spwf_socket_close()
   if (wifi_state != wifi_state_reconnect)
        wifi_state = mqtt_socket_create;
}

 /**
  * @brief  Prepare JSON packet with sensors data
  * @param  buffer : buffer that will contain sensor data in JSON format 
  * @retval None
  */
 void prepare_json_pkt (uint8_t * buffer)
{
      int32_t intPart, decPart;
      char tempbuff[40] = {'\0'};
      float PRESSURE_Value;
      float HUMIDITY_Value;
      float TEMPERATURE_Value;
      SensorAxes_t ACC_Value;
      SensorAxes_t GYR_Value;
      SensorAxes_t MAG_Value;
      #ifndef __IKS01A1 
      uint32_t FFTmaxAmpl;
      #endif   
//{"method":"Publish","params":{"apsdb.message": "{\"d\":{\"myName\":\"Nucleo\",\"A_Temperature\":23.29,\"A_Humidity\":52.59,\"A_Pressure\":955.84,\"Acc_X\":-37,\"Acc_Y\":27,\"Acc_Z\":974,\"GYR_X\":3220,\"GYR_Y\":-1400,\"GYR_Z\":980,\"MAG_X\":68,\"MAG_Y\":113,\"MAG_Z\":91}}","apsdb.channel": "stm32" }}

  strcpy((char *)buffer,"{\"method\":\"Publish\",\"params\":{\"apsdb.message\": \"");
  strcat((char *)buffer,"{\\\"d\\\":{\\\"myName\\\":\\\"Nucleo\\\"");
  #ifndef __IKS01A1     
  if(sensorInUse == IIS2DH_X_0 || sensorInUse == LSM303AGR_X_0)
  {
    FFTmaxAmpl = getFFTMaxFreqAmp(); 
          
    if(FFTmaxAmpl < 400){
      sprintf(tempbuff, ",\\\"Motor_status\\\":\\\"OK\\\"");
    } else if(FFTmaxAmpl > 400 && FFTmaxAmpl < 800 ){
      sprintf(tempbuff, ",\\\"Motor_status\\\":\\\"Warning\\\"");
    } else {
      sprintf(tempbuff, ",\\\"Motor_status\\\":\\\"Failure\\\"");
    }
    strcat((char *)buffer,tempbuff);
    
    sprintf(tempbuff, ",\\\"FFT_max_f\\\":%d", getFFTMaxFreq());
    strcat((char *)buffer,tempbuff);
    
    sprintf(tempbuff, ",\\\"FFT_max_f_amp\\\":%d", FFTmaxAmpl);
    strcat((char *)buffer,tempbuff);
  }
  #endif // __IKS01A1
  
      BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle,(float *)&TEMPERATURE_Value);
      MCR_BLUEMS_F2I_2D(TEMPERATURE_Value, intPart, decPart);
      sprintf(tempbuff, ",\\\"A_Temperature\\\":%lu.%lu",intPart, decPart);
      strcat((char *)buffer,tempbuff);
      
      BSP_HUMIDITY_Get_Hum(HUMIDITY_handle,(float *)&HUMIDITY_Value);
      MCR_BLUEMS_F2I_2D(HUMIDITY_Value, intPart, decPart);
      sprintf(tempbuff, ",\\\"A_Humidity\\\":%lu.%lu",intPart, decPart );
      strcat(  (char *)buffer,tempbuff);
      
      BSP_PRESSURE_Get_Press(PRESSURE_handle,(float *)&PRESSURE_Value);
      MCR_BLUEMS_F2I_2D(PRESSURE_Value, intPart, decPart);
      sprintf(tempbuff, ",\\\"A_Pressure\\\":%lu.%lu",intPart, decPart );
      strcat((char *)buffer,tempbuff);
      
      BSP_ACCELERO_Get_Axes(ACCELERO_handle,&ACC_Value);
      sprintf(tempbuff, ",\\\"Acc_X\\\":%d", (int)ACC_Value.AXIS_X);
      strcat((char *)buffer,tempbuff);
      sprintf(tempbuff, ",\\\"Acc_Y\\\":%d", (int)ACC_Value.AXIS_Y);
      strcat((char *)buffer,tempbuff);
      sprintf(tempbuff, ",\\\"Acc_Z\\\":%d", (int)ACC_Value.AXIS_Z);
      strcat((char *)buffer,tempbuff);
      
      BSP_GYRO_Get_Axes(GYRO_handle,&GYR_Value);
      sprintf(tempbuff, ",\\\"GYR_X\\\":%d", (int)GYR_Value.AXIS_X);
      strcat((char *)buffer,tempbuff);
      sprintf(tempbuff, ",\\\"GYR_Y\\\":%d", (int)GYR_Value.AXIS_Y);
      strcat((char *)buffer,tempbuff);
      sprintf(tempbuff, ",\\\"GYR_Z\\\":%d", (int)GYR_Value.AXIS_Z);
      strcat((char *)buffer,tempbuff);
     
      BSP_MAGNETO_Get_Axes(MAGNETO_handle,&MAG_Value);
      sprintf(tempbuff, ",\\\"MAG_X\\\":%d", (int)MAG_Value.AXIS_X);
      strcat((char *)buffer,tempbuff);
      sprintf(tempbuff, ",\\\"MAG_Y\\\":%d", (int)MAG_Value.AXIS_Y);
      strcat((char *)buffer,tempbuff);
      sprintf(tempbuff, ",\\\"MAG_Z\\\":%d", (int)MAG_Value.AXIS_Z);
      strcat((char *)buffer,tempbuff);
      
      strcat((char *)buffer,"}}");
      strcat((char *)buffer, "\",\"apsdb.channel\": \"stm32\" }}");
      return;
}


 /**
  * @brief  GET MAC Address from WiFi
  * @param  char* macadd : string containing MAC address
  * @retval None
  */
static System_Status_t Get_MAC_Add (char *macadd)
{
  uint8_t macaddstart[32];
  System_Status_t status = MODULE_ERROR; // 0 success
  int i,j;
       
  if (GET_Configuration_Value("nv_wifi_macaddr",(uint32_t *)macaddstart) != WiFi_MODULE_SUCCESS)
  {
      printf("Error retrieving MAC address \r\n");  
      return status; 
  }
  else
      status = MODULE_SUCCESS;
 
  macaddstart[17]='\0';
  printf("MAC orig: \r\n");  
  printf((char *)macaddstart);
  printf("\r\n");
  
  if (status == MODULE_SUCCESS)
  {  
        for(i=0,j=0;i<17;i++){
          if(macaddstart[i]!=':'){
            macadd[j]=macaddstart[i];
            j++;  
          } 
        }
        macadd[j]='\0';
  }
  
  return status;
}


/**
  * @brief  MQTT Callback function
  * @param  md : MQTT message received
  * @retval WiFi_Status_t
  */
void messageArrived(MessageData* md)
{
    MQTTMessage* message = md->message;
    uint16_t message_size = (uint16_t)message->payloadlen;
    *((char*)message->payload + message_size) = '\0';
    
    printf("\r\n [D]. MQTT payload received is: \r\n");
    printf((char*)message->payload);  
    printf("\r\n");
}


/**
  * @brief  MQTT Callback function
  * @param  md : MQTT message received
  * @retval WiFi_Status_t
  */
void messageArrived_echo(MessageData* md)
{
    MQTTMessage* message = md->message;
    
    printf("\r\n [D]. MQTT echo received is: \r\n");
    printf((char*)message->payload);  
    printf("\r\n");
}


/**
  * @brief  Configure Access Point parmaters (SSID, PWD, Authenticaation) :
  * @brief  1) Read from FLASH
  * @brief  2) If not available in FLASH or when User Button is pressed : read from NFC
  * @brief  3) If not available in FLASH or when User Button is pressed and if nothing is written in NFC : read from serial terminal
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t ConfigAPSettings(void)
{
   System_Status_t status = MODULE_ERROR;  

   printf("\r\nKeep pressed user button to set WiFi and scriptr.io parameters ");
   if(nfcStatus==MODULE_NFC_IN_USE)    printf("from NFC or ");
   printf("via serial terminal. Otherwise parameters saved in FLASH will be used.\r\n");  
   HAL_Delay(4000);
   
   /* User Button not pressed --> read parameters from Flash */
   if(BSP_PB_GetState(BUTTON_KEY) != GPIO_PIN_RESET) {
       /* Read from FLASH */
       if(ReCallSSIDPasswordFromMemory() > 0) {
          printf("Read user parameters from FLASH:\r\n");
          status = testUserParamSanity();            
       }
       else { 
          printf("\r\nNo data written in FLASH. Trying to obtain configuration parameters from user.\r\n");                           
          status = MODULE_NO_USER_PARAMS_ON_FLASH;
       }   
    } 
  
      if(status != MODULE_SUCCESS) {
        /* 
         We can reach here in following cases only:
           case 1: User button is pressed and NFC is being used. => read NFC
           case 2: User button is pressed and NFC is present. But there was some error while reading NFC. => read terminal
           case 3: User button is pressed and NFC not present. => read Terminal 				
           case 4: User button not pressed and there was error while reading flash. NFC is present. => read NFC
           case 5: User button not pressed and there was error while reading flash. NFC is not present. => read Terminal			
        */	 
             
        // case 1 and 4	
        if(nfcStatus==MODULE_NFC_IN_USE) {
          status = ReadUserParamsFromNFC();
        }
                
        // case 2, 3, 5
        if(status != MODULE_SUCCESS) {                
          if((status=getWiFiParamFromTerminal())!=MODULE_SUCCESS) {
            return status;
          }                               
        }				
      }
   
      printUserParams();   
      /* Save to FLASH */  
      printf("Now saving user params to flash memory\r\n");
      status = SaveSSIDPasswordToMemory();
      if(status!=MODULE_SUCCESS) {
            printf("\r\n[E]. Error in AP Settings\r\n");
            return status;
      }
      return status;
}

/**
* @brief  Read user parameters from NFC
* 
* @retval System_Status_t MODULE_SUCCESS if parameters are read succesfully otherwise suitable error code
*/
static System_Status_t ReadUserParamsFromNFC() {
	System_Status_t status = MODULE_ERROR;
        uint8_t URL_ST[64];
	
	#if (CONFIG_USE_NFC==USE_NDEF_TEXT_RECORD)
		printf("\r\nReading user params using NFC (NDEF TEXT)....\r\n");
		status = ReadParametersFromNFC();
	#elif (CONFIG_USE_NFC==USE_NDEF_WIFI_RECORD)  
		printf("\r\nReading user params using NFC (NDEF WIFI)....\r\n");
		status = ReadWifiTokenFromNFC();              
	#else
	  printf("\r\n[E]. Error in CONFIG_USE_NFC flag. \r\n");
		status = MODULE_ERROR;			  
	#endif       

	if(status==MODULE_SUCCESS) {
                // test sanity of user parameters
                printf("[D]. performing sanity check for user parameters \r\n");
                status = testUserParamSanity();  
                if(status==MODULE_SUCCESS) {        
                  /* Prevent to read wrong SSID/PWD from NFC next time */		
                  strcpy((char *)URL_ST, "www.st.com/stm32ode");
                  status = WriteURLtoNFC((char *)URL_ST);  
                  if(status!=MODULE_SUCCESS) { 
                                          printf("\r\n[E]. Error while writing URL to NFC \r\n");  
                                          return status;    
                  }						
                  status = MODULE_SUCCESS;	
                }
	}     
	else {
		printf("No User parameters found in NFC device. \r\n"); 
		status = MODULE_NO_USER_PARAMS_ON_NFC;	
	}
	return status;     
}

/**
* @brief  Read user parameters from Terminal
* 
* @retval System_Status_t MODULE_SUCCESS if parameters are read succesfully otherwise suitable error code
*/
static System_Status_t getWiFiParamFromTerminal() {
  // printf("\tgetWiFiParamFromTerminal\r\n");   
  System_Status_t status = MODULE_ERROR;
  // wait for 2 seconds
  HAL_Delay(2000);
  printf("\r\nRead parameters from serial terminal.\r\n");   
  /* Read from Serial.  */
  status = wifi_get_AP_settings();        
  if(status!=MODULE_SUCCESS)
      printf("\r\n[E]. Error in AP Settings\r\n");
  
  return status;     
}


/**
* @brief  Init program functionality
* @retval void
*/
int progFunctionInit(void){
  WiFi_Status_t status = WiFi_MODULE_ERROR;
  System_Status_t sysStatus = MODULE_ERROR;
  uint8_t DisplayName[32];
  #ifndef __IKS01A1 
  uint32_t measODR;
  uint32_t t1;
  uint32_t t2;
  uint32_t elapsed_mSec;
  uint32_t limit;
  uint32_t i;  
  
  if(sensorInUse == IIS2DH_X_0 || sensorInUse == LSM303AGR_X_0)
  { 
    printf("\r\nPress \"f\" or \"F\" key on your keyboard to run FFT project.\r\nOtherwise Cloud demo project with FFT extension will be run.\r\n");
    printf("(Go instantly to Cloud demo by pressing Enter button on your keyboard...)\r\n");
    fflush(stdout);
    Int_Current_Time1 = user_currentTimeGetTick();
    t1 = Int_Current_Time1;   
    elapsed_mSec = 0;    
    limit = 8000; // 8s
    i = 0;
    
    /* Wait until control button is pressed */
    RxBuffer = '\0';
    while( elapsed_mSec < limit ){
      HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1);      
      Int_Current_Time2 = user_currentTimeGetTick();
      t2 = Int_Current_Time2;
            
      if(RxBuffer == 'f' || RxBuffer == 'F' || RxBuffer == '\r')
      {
        break;
        }
      
      elapsed_mSec = t2 - t1;
      if(elapsed_mSec > i)
      {		   								
        printf("\r%d...", (limit - i)/1000);
          fflush(stdout);
        i += 1000;
      }   
      
    }    
    printf("\r    \r");
    fflush(stdout);
  } else {
    RxBuffer = '\0';
  }
  if( RxBuffer == 'f' ){
    /* set program to FFT default */
    
    printf("Starting FFT project...\r\n"); 
    HAL_Delay(2000);
    
    progFunction = FFT;
    
    if(BSP_ACCELERO_Disable_HP_Filter_Ext( ACCELERO_handle ) != COMPONENT_OK)
    {
      return 0;
    }
    if(BSP_ACCELERO_DeInit( &ACCELERO_handle ) != COMPONENT_OK)
    {
      return 0;
    }
    if(BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &ACCELERO_handle ) != COMPONENT_OK)
    {
      return 0;
    }
    if(BSP_ACCELERO_Sensor_Enable(ACCELERO_handle) != COMPONENT_OK)
    {
      return 0;
    }
    if(BSP_ACCELERO_Set_INT1_DRDY_Ext( ACCELERO_handle, INT1_DRDY_ENABLED ) != COMPONENT_OK)
    {
      return 0;
    }
    if(BSP_ACCELERO_Set_Active_Axis_Ext( ACCELERO_handle, Z_AXIS ) != COMPONENT_OK)
    {
      return 0;
    }
    
    /* Set ODR to 400Hz for FFT demo */
    if(BSP_ACCELERO_Set_ODR_Value( ACCELERO_handle, 400.0f ) != COMPONENT_OK)
    {
      return 0;
    }
 
    // init FFT
    init_FFT( e_FFT );
    return MODULE_SUCCESS;  
  } 
  
#endif // __IKS01A1
  
    /* set program to CLOUD default */   
    progFunction = CLOUD;        
    
    if(BSP_ACCELERO_DeInit( &ACCELERO_handle ) != COMPONENT_OK)
    {
      return 0;
    }
    if(BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &ACCELERO_handle ) != COMPONENT_OK)
    {
      return 0;
    }
    if(BSP_ACCELERO_Sensor_Enable(ACCELERO_handle) != COMPONENT_OK)
    {
      return 0;
    }
    #ifndef __IKS01A1 
    if(sensorInUse == IIS2DH_X_0 || sensorInUse == LSM303AGR_X_0)
    {
      if(BSP_ACCELERO_Enable_HP_Filter_Ext( ACCELERO_handle, HPF_MODE_NORMAL, CUTOFF_MODE2) != COMPONENT_OK)
      {
        return 0;
      }
      if(BSP_ACCELERO_Set_Active_Axis_Ext( ACCELERO_handle, Z_AXIS ) != COMPONENT_OK)
      {
        return 0;
      }
      if(BSP_ACCELERO_Set_INT1_DRDY_Ext(ACCELERO_handle, INT1_DRDY_ENABLED) != COMPONENT_OK)
      {
        return 0;
      }
      
      /* Set ODR to 400Hz for FFT demo */
      if(BSP_ACCELERO_Set_ODR_Value( ACCELERO_handle, 400.0f ) != COMPONENT_OK)
      {
        return 0;
      }
      
      // init FFT
      measODR = init_FFT( e_CLOUD );
      
      if(BSP_ACCELERO_Set_INT1_DRDY_Ext(ACCELERO_handle, INT1_DRDY_DISABLED) != COMPONENT_OK)
      {
        return 0;
      }    
      
      if(BSP_ACCELERO_Set_Active_Axis_Ext(ACCELERO_handle, ALL_ACTIVE) != COMPONENT_OK)
      {
        return 0;
      }    
      
      printf("Initially measured ODR for FFT extension = %d Hz\r\n", measODR);
    }
    #endif  //__IKS01A1
    
  /* Set WiFi AP parameters */  
  if((ssid!=NULL)&&(seckey!=NULL)) {
	printf("\r\nUsing SSID and PWD written in the code. "); 
	setDefaultConfig();
  } else {
	printf("\r\nStarting configure procedure for SSID and PWD....  "); 
	HAL_Delay(3000);      
	if ((sysStatus=ConfigAPSettings()) < 0) {  
	  printf("Failed to set AP settings.\r\n"); 
	  return sysStatus; 
	} else {
	  printf("\n\rAP settings set.\r\n");         
	}
    }

  
  /* WiFi Init */
  if (ConfigWiFi () < 0 ){  
    printf("Failed to Initialize WiFi.\r\n"); 
    return 0; 
  } else {
    printf("\n\rWiFi initialized.\r\n");   
  }

  /*  Init MQTT  intf               */
  if ((sysStatus=InitMQTT())!=MODULE_SUCCESS ){  
    printf("Failed to Initialize MQTT interface.\r\n"); 
    return sysStatus; 
  } else {
    printf("MQTT initialized.\r\n");   
  }

  /* Initialize WiFi module */
  wifi_state = wifi_state_idle;  
  if((status=wifi_init(&config)) != WiFi_MODULE_SUCCESS)
  {
    printf("[E].Error in wifi init. \r\n");
    return status;
  }    
  
  /*  Get MAC Address */  
  if((sysStatus=Get_MAC_Add((char *)DisplayName)) < 0){ 
     printf("[E]. Error while retrieving MAC address \r\n");  
     return status; 
  } else {
      printf("[D]. WiFi MAC address: \r\n");
      printf((char *)DisplayName);
  }  

  /* Config MQTT Infrastructure */
  Config_MQTT_SCRIPTR ( &mqtt_scriptr_setup, DisplayName );
  
 
 
 return MODULE_SUCCESS;  

}


/**
* @brief  Retrieve SysTick to increment counter
* 
* @retval tick value
*/
uint32_t user_currentTimeGetTick()
{
   return HAL_GetTick();
}  


/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSI)
 *            SYSCLK(Hz)                     = 84000000
 *            HCLK(Hz)                       = 84000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            HSI Frequency(Hz)              = 16000000
 *            PLL_M                          = 16
 *            PLL_N                          = 336
 *            PLL_P                          = 4
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale2 mode
 *            Flash Latency(WS)              = 2
 * 
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 

     clocked below the maximum system frequency, to update the voltage scaling value 

     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
   
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}




 /**
  * @brief  Time Handler for MQTT
  * 
  * @retval None
  */
void TIM4_IRQHandler(void)
{
   SysTickIntHandlerMQTT();   
   
   __HAL_TIM_CLEAR_FLAG(&MQTTtimHandle, TIM_FLAG_UPDATE);  
}

 /**
  * @brief  Turn on notificaion LED (LED2)
  * 
  * @retval None
  */
static void NotifyLEDOn(void)
{
 BSP_LED_On(LED2);
}

 /**
  * @brief  Turn off notificaion LED (LED2)
  * 
  * @retval None
  */
static void NotifyLEDOff(void)
{
 BSP_LED_Off(LED2);
}

 /**
  * @brief  Turn on notificaion LED (LED2)
  * @param  msPeriod time delay in milli seconds
  * @retval None
  */
static void NotifyLEDBlink(unsigned int msPeriod) 
{
   BSP_LED_Off(LED2);
   HAL_Delay(msPeriod);
   BSP_LED_On(LED2);
}

 /**
  * @brief  obtains Scriptr.io connection QOS from user through terminal.
  * @retval QOS (QOS0/QOS1/QOS2)
  */
static enum QoS getScriptrQoS(void) {
    /*
	char console_input[50];
   // QOS                  
    printf("\r\nQuality of service? ");
    fflush(stdout);
    console_count=0;
    console_count=scanf("%s",console_input);
    printf("\r\n");
    switch(console_input[0])    {
    case '0':
      UnionNFCToken.configParams.qos = QOS0;
      break;
    case '1':
      UnionNFCToken.configParams.qos = QOS1;
      break;
    case '2':
      UnionNFCToken.configParams.qos = QOS2;
      break;
    default:
      printf("\r\nWrong entry for QOS. \n selecting QOS0\n\r");
      UnionNFCToken.configParams.qos = QOS0;        
    }
    return UnionNFCToken.configParams.qos;
    */
}

 /**
  * @brief  obtains scriptr.io user name from user through terminal.
  * @retval void
  */
static void getScriptrUsername(void) {
    /*
	char console_input[50];
    // username                  
    printf("\r\nusername? ");
    fflush(stdout);
    console_count=0;
    console_count=scanf("%s",console_input);
    printf("\r\n");
    strcpy(UnionNFCToken.configParams.username, console_input);
    */
}

 /**
  * @brief  obtains scriptr.io secret token from user through terminal.
  * @retval void
  */
static void getScriptrPassword(void) {
    /*
	char console_input[50];
    // password                  
    printf("\r\npassword? ");
    fflush(stdout);
    console_count=0;
    console_count=scanf("%s",console_input);
    printf("\r\n");
    strcpy(UnionNFCToken.configParams.password, console_input);
    */
}

 /**
  * @brief  obtains scriptr.io host name from user through terminal.
  * @retval void
  */
static void getScriptrHostname(void) {
    /*
	char console_input[50];
    // hostname                  
    printf("\r\nhostname? ");
    fflush(stdout);
    console_count=0;
    console_count=scanf("%s",console_input);
    printf("\r\n");
    strcpy(UnionNFCToken.configParams.hostname, console_input);
    */
}    

/**
  * @brief  obtains scriptr.io script name from user through terminal.
  * @retval void
  */
static void getScriptrChannelName(void) {
    char console_input[50];
    // deviceid
    printf("\r\nChannel name? ");
    fflush(stdout);
    console_count=0;
    console_count=scanf("%s",console_input);
    printf("\r\n");
    strcpy(UnionNFCToken.configParams.channel, console_input);
}

/**
  * @brief  obtains scriptr.io script name from user through terminal.
  * @retval void
  */
static void getScriptrToken(void) {
    char console_input[50];
    // deviceid
    printf("\r\nAuthentication Token? ");
    fflush(stdout);
    console_count=0;
    console_count=scanf("%s",console_input);
    printf("\r\n");
    strcpy(UnionNFCToken.configParams.token, console_input);
}

 /**
  * @brief  obtains Wifi SSID from user through terminal.
  * @retval void
  */
static System_Status_t getWifiSSID(void) {
    char console_ssid[50];    
    printf("\r\nEnter the SSID: ");
    fflush(stdout);

    console_count=0;
    console_count=scanf("%s",console_ssid);
    printf("\r\n");
    // FIXME : Why 39. NDEF is 32
    if(console_count==NDEF_WIFI) 
    {
      printf("Exceeded number of ssid characters permitted");
      return MODULE_ERROR;
    }   
    strcpy(UnionNFCToken.configParams.NetworkSSID, console_ssid);  
    return MODULE_SUCCESS;
}

 /**
  * @brief  obtains Wifi security key from user through terminal.
  * @retval void
  */
static System_Status_t getWifiSEcurityKey(void) {
  char console_psk[50];  
  printf("Enter the Wifi password: ");
  fflush(stdout);
  console_count=0;

  console_count=scanf("%s",console_psk);
  printf("\r\n");
  // FIXME : Why 19. NDEF is 32
  if(console_count==NDEF_WIFI) 
  {
    printf("Exceeded number of psk characters permitted.\r\n");
    return MODULE_ERROR;
  }    
  strcpy(UnionNFCToken.configParams.NetworkKey, console_psk);  
  return MODULE_SUCCESS;
}

 /**
  * @brief  obtains Wifi authentication mode from user through terminal.
  * @retval void
  */
static void getWifiAuthMode(void) {
    char console_input[50]; 
    printf("Enter the authentication mode(0:Open, 1:WEP, 2:WPA2/WPA2-Personal): "); 
    fflush(stdout);
    scanf("%s",console_input);
    printf("\r\n");
    //printf("entered =%s\r\n",console_input);
    switch(console_input[0])
    {
      case '0':
        mode = None;
        strcpy(UnionNFCToken.configParams.AuthenticationType, "NONE");
        break;
      case '1':
        mode = WEP;
        strcpy(UnionNFCToken.configParams.AuthenticationType, "WEP");           
        break;
      case '2':
        mode = WPA_Personal;
        strcpy(UnionNFCToken.configParams.AuthenticationType, "WPA2");
        break;
      default:
        printf("\r\nWrong Entry. Priv Mode is not compatible. \nSetting mode to WPA2\n\r");
        strcpy(UnionNFCToken.configParams.AuthenticationType, "WPA2");                      
   }      
}

/**
  * @brief  get values of various scriptr.io configuration parameters from Terminal input
  * 
  * @retval System_Status_t (MODULE_SUCCESS/MODULE_ERROR)
  */
static System_Status_t getScriptrParamsFromTerminal(void) {
  //char console_input[50];
 

	printf("Please input scriptr.io account configuration parameters one by one.\r\n");
	getScriptrQoS();
	getScriptrUsername();
	getScriptrPassword();
	getScriptrHostname();
	getScriptrToken();
	getScriptrChannelName();
	return MODULE_SUCCESS;

}

/**
  * @brief  print values for configuration params
  * @retval None
 */
static void printUserParams() {
	//printf("\tprintUserParams\n\r");
	printf("\tSSID: %s\n\r",UnionNFCToken.configParams.NetworkSSID);
	printf("\tKey: %s\n\r",UnionNFCToken.configParams.NetworkKey);
	printf("\tAuthentication: %s\n\r",UnionNFCToken.configParams.AuthenticationType);
	printf("\tScriptr.io account configuration\n\r");
	printf("\tUsername: %s\n\r",UnionNFCToken.configParams.token);
	printf("\tChannel: %s\n\r",UnionNFCToken.configParams.channel);
	/*
	printf("\tUsername: %s\n\r",UnionNFCToken.configParams.username);
	printf("\tPassword: %s\n\r",UnionNFCToken.configParams.password);
	printf("\tHost name: %s\n\r",UnionNFCToken.configParams.hostname);
	printf("\tDevice name: %s\n\r",UnionNFCToken.configParams.clientid);
	*/
}

#if (CONFIG_USE_NFC==USE_NDEF_TEXT_RECORD)

 /**
  * @brief  Parse NFC string record
  * @param  str NFC string record to be parsed
  * @retval None
  */
static void ParseStringRecord(char* str) {
  char delim[] = ",";
  char *token = NULL;
  
  /* remove whitespaces from input string and set default values for parameters*/
  removeWhiteSpaces(str);
  setDefaultConfig();
  
  token = strtok(str, delim);
  
  while(token) {   
    str = str+strlen(token)+1;
    parseToken(token);
    token = strtok(str, delim);
  }
}

 /**
  * @brief  Parse token contained inside NFC string record and set values of NFC/Scriptr.io parameters
  * @param  tt token to be parsed
  * @retval None
  */
static void parseToken(char *tt) {
  char delim[] = "=";
  //int len1 = strlen(tt);
  char *tokenId = strtok(tt, delim);
  tt = tt+strlen(tt)+1;
  char *value = strtok(tt, delim);   
#if 0  
  printf("\t\ttokenId=%s\n\r", tokenId);  
  printf("\t\tvalue=%s\n\r", value);
#endif  
  if(value==NULL) {
    setParameterValue(tokenId, "");
  } else {
    setParameterValue(tokenId, value);  
  }
}


/**
  * @brief  set values of NFC/Watson parameters
  * @param  tokenId string identifying the token
  * @param  value value to be set for the token
  * @retval None
 */
static void setParameterValue(char* tokenId, char *value) {
    printf("\r\n %s=%s\r\n", tokenId, value);
    if(strcmp(tokenId, "qos")==0) {   
      if(strcmp(value, "0")==0) {
          //UnionNFCToken.configParams.qos = QOS0;
      } else if(strcmp(value, "1")==0) {
          //UnionNFCToken.configParams.qos = QOS1;
      } else if(strcmp(value, "2")==0) {
          //UnionNFCToken.configParams.qos = QOS2;
      }
    } else if(strcmp(tokenId, "username")==0) {
        //strcpy(UnionNFCToken.configParams.username, value);
    } else if(strcmp(tokenId, "password")==0) {
        //strcpy(UnionNFCToken.configParams.password, value);
    } else if(strcmp(tokenId, "hostname")==0) {
        //strcpy(UnionNFCToken.configParams.hostname, value);
    } else if(strcmp(tokenId, "deviceid")==0) {
        //strcpy(UnionNFCToken.configParams.clientid, value);
    } else if(strcmp(tokenId, "ssid")==0) {      
        strcpy(UnionNFCToken.configParams.NetworkSSID, value);  
    } else if(strcmp(tokenId, "seckey")==0) {
        strcpy(UnionNFCToken.configParams.NetworkKey, value);   
    } else if(strcmp(tokenId, "authtype")==0) {
        strcpy(UnionNFCToken.configParams.AuthenticationType, value); 
    } else if(strcmp(tokenId, "channel")==0) {
        strcpy(UnionNFCToken.configParams.channel, value);
    } else if(strcmp(tokenId, "token")==0) {
        strcpy(UnionNFCToken.configParams.token, value);
    } 
 } 

/**
  * @brief  remove whitespace characters from input string
  * @param  input input string
  * @param  output same as input string but all whitespaces removed
  * @retval None
 */
static void removeWhiteSpaces(char* input) {  
  char output[400];
  int outIndex = 0;
  int i=0;
  for(i=0; i<strlen(input); i++) {
    if(!isspace((int)input[i]))
      output[outIndex++] = input[i];
  }
  // add null character at the end
  output[outIndex++] = 0;
  
  strcpy(input, output);
}
#endif  

/**
  * @brief  set default values for configuration params
  * @retval None
 */
static void setDefaultConfig() {
//  UnionNFCToken.configParams.qos = QOS0;
  strcpy(UnionNFCToken.configParams.NetworkSSID, ssid);
  strcpy(UnionNFCToken.configParams.NetworkKey, seckey);
  strcpy(UnionNFCToken.configParams.AuthenticationType, "WPA2");  
} 


/**
  * @brief  print HW configuration on the terminal
  * @retval None
 */
static void ShowHwConfiguration() {
    printf("\n\rHW configuration detected:\r\n");  
    printf("\tNucleo-F401RE\r\n");      
    printf("\tX-Nucleo-IDW01M1\r\n");
    if(nfcStatus==MODULE_NFC_IN_USE) printf("\tX-Nucleo-NFC01A1\r\n");     
#ifdef __IKS01A1
    printf("\tX-Nucleo-IKS01A1\r\n");
#else
    printf("\tX-Nucleo-IKS01A2\r\n");    
#endif    
}


/**
  * @brief  check sanity of user parameters
  * @retval MODULE_SUCCESS is sanity test is passed, otherwise MODULE_ERROR
 */
static System_Status_t testUserParamSanity() {
    System_Status_t paramSanityStatus = MODULE_ERROR;  
    
    printf("\tPerforming user parameter sanity check\r\n");

    if(strlen(UnionNFCToken.configParams.NetworkSSID)==0) {
      printf("\tSanity check failed for input parameter \"NetworkSSID\"\n\r");
      getWifiSSID();
    }  
    if(strlen(UnionNFCToken.configParams.NetworkKey)==0) {
      printf("\tSanity check failed for input parameter \"NetworkKey\"\n\r");
      getWifiSEcurityKey();
    }  
    if(strlen(UnionNFCToken.configParams.AuthenticationType)==0) {
      printf("\tSanity check failed for input parameter \"AuthenticationType\"\n\r");
      getWifiAuthMode();
    }  

    printf("\tchecking \"registered\" mode user parameters\n\r");
    /*
    if((UnionNFCToken.configParams.qos!=(int)QOS0) && (UnionNFCToken.configParams.qos!=(int)QOS1) && (UnionNFCToken.configParams.qos!=(int)QOS2)) {
    	printf("\tSanity check failed for input parameter \"QOS\"\n\r");
        getScriptrQoS();
    }
    if(strlen(UnionNFCToken.configParams.username)==0) {
    	printf("\tSanity check failed for input parameter \"username\"\n\r");
		getScriptrUsername();
    }
    if(strlen(UnionNFCToken.configParams.password)==0) {
    	printf("\tSanity check failed for input parameter \"password\"\n\r");
        getScriptrPassword();
    }
    if(strlen(UnionNFCToken.configParams.hostname)==0) {
        printf("\tSanity check failed for input parameter \"hostname\"\n\r");
        getScriptrHostname();
    }
    if(strlen(UnionNFCToken.configParams.clientid)==0) {
        printf("\tSanity check failed for input parameter \"clientid\"\n\r");
        getScriptrDeviceName();
    }
	*/
    paramSanityStatus = MODULE_SUCCESS;
    return paramSanityStatus;
 } 


/**
 * @}
 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
