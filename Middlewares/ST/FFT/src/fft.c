/**
******************************************************************************
* @file    fft.c
* @author  Martin Polacek
* @version V1.0.4
* @date    13-January-2017
* @brief   FFT project driver
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "fft.h"
#include <string.h> // strlen
#include <stdio.h>  // sprintf
#include <math.h>   // trunc

#include "arm_const_structs.h"
#include "flat_top_win.h"


#define MAX_SAMPLES             1024
#define HEADER_LENGTH             82


#define MAG_200mg                200
#define MAG_500mg                500
#define MAG_1g                  1000
#define MAG_2g                  2000
#define MAG_4g                  4000
#define MAG_8g                  8000
#define MAG_16g                16000

typedef enum
{
  HEADER_MAIN,
  HEADER_ODR,
  HEADER_MAG,
  HEADER_AXIS,
  HEADER_FS,
  HEADER_SAMPLES,
  HEADER_OPMODE
} Menu_Header;

typedef enum
{
  eGUI_MAIN_MENU,
  eGUI_ODR_SET,
  eGUI_ODR_SET_ERR,
  eGUI_MAG_SET,
  eGUI_AXIS_SET,
  eGUI_AXIS_SET_ERR,
  eGUI_FS_SET,
  eGUI_FS_SET_ERR,
  eGUI_SAMPLES_SET,
  eGUI_OPMODE_SET,
  eGUI_OPMODE_SET_ERR,
  eGUI_HP_SWITCHED,
  eGUI_HP_SWITCHED_ERR,
  eGUI_HP2DC_NULL_SUBST,
  eGUI_PLOT_GRAPH,
  eGUI_PLOT_TABLE,
  eGUI_FIRST_ODR_MEAS
} GUI_status;


/* Extern variables ----------------------------------------------------------*/
#ifdef USE_FFT_CLOUD
extern UART_HandleTypeDef UartMsgHandle;
#else
extern char dataOut[256];
extern UART_HandleTypeDef UartHandle;
#define UartMsgHandle UartHandle
#endif

extern volatile uint32_t Int_Current_Time1;     /*!< Int_Current_Time1 Value */
extern volatile uint32_t Int_Current_Time2;     /*!< Int_Current_Time2 Value */
extern volatile uint8_t AXL_DRDY_received;
extern void *ACCELERO_handle;
extern uint8_t RxBuffer;


/* Select proper function for printing data to terminal ----------------------*/
#ifdef USE_FFT_CLOUD
#define DISP(...) printf(__VA_ARGS__)
#else
#define DISP(...) sprintf(dataOut, __VA_ARGS__); \
HAL_UART_Transmit(&UartHandle, (uint8_t*)dataOut, strlen(dataOut), 5000)
#endif


/* Settable variables --------------------------------------------------------*/
uint16_t    plot_Magnitude;
uint8_t     HP_Filter;                         // Disabled
uint8_t     switch_HP_to_DC_null;              // HP filter
/* Samples = 2^n */
uint16_t    Samples;


/* Private variables ---------------------------------------------------------*/
uint8_t     data_For_FFT_Ready;
uint8_t     table_displayed;
uint16_t    freqAxis[50];
float       ODR_measured;                      // Calculated value of ODR (for ODR calibration)
GUI_status  GUIstatus;
uint32_t    maxIndex;
float32_t   maxValue;
uint32_t    BinMaxIndex;
float32_t   BinMaxValue;
uint32_t    FTW_MaxIndex;
float32_t   FTW_MaxValue;


/* Input and Output buffer Declarations for FFT ------------------------------*/
static float32_t fftInput[MAX_SAMPLES];
static float32_t fftOutput[MAX_SAMPLES];
static float32_t fftTextPlotData[51];

/* Private function prototypes -----------------------------------------------*/
void calculateFreqAxis( void );
void HP_DC_changer( void );
void En_Dis_HP_or_DCnull( void );
void printMenuHeader( Menu_Header menuHeader );
void fft_plot_creator( int16_t n, char *c );
void plot_fft( void );
void plot_table( void );

/* Menu functions */
void mainMenu( void );
void ODRMenu( void );
void magnitudeMenu( void );
void axisMenu( void );
void fullScaleMenu( void );
void samplesMenu( void );
void opModeMenu(void);

/* ODR measurement functions */
void measODR(void);

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Initialize FFT demo
*
* @param  func Program functionality selector
* @retval measured ODR of the sensor
*/
uint32_t init_FFT( en_ProgFun func ){
  
  plot_Magnitude = MAG_1g;
  if( func == e_FFT){
    HP_Filter = 0;                      // Disabled
  } else {
    HP_Filter = 1;                      // Enabled
  }
  switch_HP_to_DC_null = 0;             // HP filter
  Samples = 512;
  data_For_FFT_Ready = 0;
  table_displayed = 1;
  ODR_measured = 0;
  GUIstatus = eGUI_MAIN_MENU;
  AXL_DRDY_received = 0;
  maxIndex = 0;
  maxValue = 0;
  BinMaxIndex = 0;
  BinMaxValue = 0;
  FTW_MaxIndex = 0;
  FTW_MaxValue = 0;
  
  /* Measeure and calculate ODR */
  measODR();
  GUIstatus = eGUI_FIRST_ODR_MEAS;
  
  /* Calculate values for graph x axis */
  calculateFreqAxis();
  
  return (uint32_t)(ODR_measured);
}


/**
* @brief  FFT_main function is to show how to use X_NUCLEO_IKS01A1 expansion board with IIS2DH or LSM303AGR to calculate FFT in Nucleo board and display it
*         on generic applications like TeraTerm or Putty.
*         Menu for set properties of AXL or graph is included in Project and Main menu is displayed after each start or reset.
*         
*         Extra control buttons:
*         1) Whenever user press "M", "m" or blue User button on Nucleo board Main menu is displayed
*         2) Whenever user press "h" or "H" in main menu, option [7] function will be switched between HP filter Enable/Disable and 
*            DC nulling Enable/Disable
*
*
*         Recomended size of terminal (e.g. Tera Term) is 83 x 33
*
*
* @param  None
* @retval Integer
*/
void FFT_main(void){
  if( GUIstatus != eGUI_PLOT_GRAPH && GUIstatus != eGUI_PLOT_TABLE)
  {    
    /* Display main menu in terminal */
    mainMenu();
  } else if ( data_For_FFT_Ready ){
    
    /* Calculate FFT */
    do_fft();  
    data_For_FFT_Ready = 0;
    
    if( GUIstatus == eGUI_PLOT_GRAPH ){ 
      /* Display graph in terminal */
      plot_fft();
      
    } else {
      if( table_displayed == 0 ){
        /* Display table in terminal */
        plot_table();
      }
    }                       
  } else {
    acquireData();
    
  }
  
  /* Go to main menu after "m" is pressed */
  RxBuffer = '\0';
  HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1);
  
  if(RxBuffer=='m' || RxBuffer == 'M'){
    GUIstatus = eGUI_MAIN_MENU;
  }
}


/**
* @brief  Display Manin menu in terminal
*
* User have theese options to choose by pressing proper button (number) on PCs keyboard:
* 1) Set ODR
* 2) Set Magnitude 
* 3) Set Sensing axis
* 4) Set Full scale
* 5) Set Operating mode
* 6) Set FFT Samples
* 7) Enable/Disable HP filter OR Enable/Disable DC nulling
* 8) Plot FFT graph
* 9) Plot FFT table
*
* Main menu include two status bars:
* 1) top status bar, that shows actual settings
* 2) bottom status bar, that inform user about recently done changes 
*
* @param  void
* @retval void
*/
void mainMenu(void){
  
  float ODR_Switch = 0;
  float fullScale = 0;
  OPER_MODE_t operatingMode;
  ACTIVE_AXIS_t axisActive;
  uint8_t tmp;
  
  /* Print menu header to terminal */
  printMenuHeader(HEADER_MAIN);
  
  BSP_ACCELERO_Get_FS( ACCELERO_handle, &fullScale);
  BSP_ACCELERO_Get_Active_Axis_Ext( ACCELERO_handle, &axisActive );
  
  //BSP_ACCELERO_Get_ODR( ACCELERO_handle, &ODR_Switch );
  BSP_ACCELERO_Read_Reg( ACCELERO_handle, 0x20, &tmp );
  tmp &= (0xF0);
  
  BSP_ACCELERO_Get_OpMode_Ext( ACCELERO_handle, &operatingMode );
  
  switch(tmp)
  {
  default:
  case 0x60:
    ODR_Switch = 200;
    break;
    
  case 0x70:
    ODR_Switch = 400;
    break;
    
  case 0x80:
    if(operatingMode == LOW_PWR_MODE){
      ODR_Switch = 1620;
    }
    break;
    
  case 0x90:
    if(operatingMode == LOW_PWR_MODE){
      ODR_Switch = 5376;
    } else {
      ODR_Switch = 1344;
    }
    break;
  }
  
  /* Actual settings */
  DISP("Actual settings:\r\n");
  
  DISP("ODR = %.0f Hz,", ODR_Switch);
  
  if( plot_Magnitude < MAG_1g ){
    DISP(" Mag. = %d mg,", plot_Magnitude);
  } else{
    DISP(" Mag. = %d g,", plot_Magnitude/MAG_1g);
  }
  
  
  switch( axisActive ){
  case X_AXIS:
    DISP(" Axis = X,");
    break;
    
  case Y_AXIS:
    DISP(" Axis = Y,");
    break;
    
  case Z_AXIS:
    DISP(" Axis = Z,");
    break;
    
  default: 
    break;
  }
  
  DISP(" FS = +-%.0f g,", fullScale);  
  
  switch( operatingMode ){
  case LOW_PWR_MODE:
    DISP(" OpMode = LPM,");
    break;
    
  case NORMAL_MODE:
    DISP(" OpMode = NM,");
    break;
    
  case HIGH_RES_MODE:
    DISP(" OpMode = HRM,");
    break;
    
  default: 
    break;
  }
  
  DISP(" Sam. = %d,", Samples);  
  
  if( switch_HP_to_DC_null ){
    if( HP_Filter ){
      DISP("\r\nDCnull = Enabled\r\n\r\n");
    } else {
      DISP("\r\nDCnull = Disabled\r\n\r\n");
    }
  } else {
    if( HP_Filter ){
      DISP("\r\nHPF = Enabled\r\n\r\n");
    } else {
      DISP("\r\nHPF = Disabled\r\n\r\n");
    }
  }  
  
  /* Options to be choosed */
  DISP("Options:\t\tNo. to be pressed:\r\n");  
  DISP("------------------------------------------\r\n");  
  DISP("\r\nSet ODR\t\t\t\"1\"\r\n");  
  DISP("\r\nSet Magnitude\t\t\"2\"\r\n");  
  DISP("\r\nSet Sensing axis\t\"3\"\r\n");  
  DISP("\r\nSet Full scale\t\t\"4\"\r\n");  
  DISP("\r\nSet Operating mode\t\"5\"\r\n");  
  DISP("\r\nSet FFT Samples\t\t\"6\"\r\n");  
  
  if( switch_HP_to_DC_null ){
    if( HP_Filter ){      
      DISP("\r\nDisable DC nulling\t\"7\"\r\n");     
      
    } else {      
      DISP("\r\nEnable DC nulling\t\"7\"\r\n");     
      
    } 
    
  } else {
    if( HP_Filter ){      
      DISP("\r\nDisable HP Filter\t\"7\"\r\n");      
      
    } else {      
      DISP("\r\nEnable HP Filter\t\"7\"\r\n");      
      
    } 
    
  }
  
  DISP("\r\nPlot FFT Graph\t\t\"8\"\r\n");  
  DISP("\r\nPlot FFT Table\t\t\"9\"\r\n");  
  
  /* Status message */
  if( GUIstatus == eGUI_ODR_SET ){
    
    DISP("\r\n\r\nODR was successfully set to %.0f Hz!\r\n(Measured ODR = %d Hz)\r\n", ODR_Switch, (int)ODR_measured);
    
    
  } else if( GUIstatus == eGUI_ODR_SET_ERR ){
    
    DISP("Error occurred during ODR set! Please restart nucleo board...\r\n");    
    Error_Handler();    
    
    
  } else if ( GUIstatus == eGUI_MAG_SET ){
    
    if( plot_Magnitude < MAG_1g ){
      DISP("\r\n\r\nMagnitude was successfully set to %d mg!\r\n", plot_Magnitude);
    } else{
      DISP("\r\n\r\nMagnitude was successfully set to %d g!\r\n", plot_Magnitude/MAG_1g);
    }
    
    
  } else if ( GUIstatus == eGUI_AXIS_SET ){
    
    switch( axisActive ){
    case X_AXIS:
      DISP("\r\n\r\nSensing axis was successfully set to X!\r\n");
      break;
      
    case Y_AXIS:
      DISP("\r\n\r\nSensing axis was successfully set to Y!\r\n");
      break;
      
    case Z_AXIS:
      DISP("\r\n\r\nSensing axis was successfully set to Z!\r\n");
      break;
      
    default: 
      break;
    }    
    
    
  } else if( GUIstatus == eGUI_AXIS_SET_ERR ){ 
    
    DISP("Error occurred during Active Axis set! Please restart nucleo board...\r\n");    
    Error_Handler();
    
    
  } else if ( GUIstatus == eGUI_FS_SET ){
    
    DISP("\r\n\r\nFull scale was successfully set to +-%.0f g!\r\n", fullScale);
    
  } else if( GUIstatus == eGUI_FS_SET_ERR ){
    
    DISP("Error occurred during FS set! Please restart nucleo board...\r\n");    
    Error_Handler();
    
    
  } else if ( GUIstatus == eGUI_OPMODE_SET ){
    
    switch( operatingMode ){
    case LOW_PWR_MODE:
      DISP("\r\n\r\nOperating mode was successfully set to Low power mode!\r\n");
      break;
      
    case NORMAL_MODE:
      DISP("\r\n\r\nOperating mode was successfully set to Normal mode!\r\n");
      break;
      
    case HIGH_RES_MODE:
      DISP("\r\n\r\nOperating mode was successfully set to High resolution mode!\r\n");
      break;
      
    default: 
      break;
    }
    
    
  } else if ( GUIstatus == eGUI_OPMODE_SET_ERR ){
    
    DISP("Error occurred during OpMode set! Please restart nucleo board...\r\n");    
    Error_Handler();
    
    
  } else if ( GUIstatus == eGUI_SAMPLES_SET ){
    
    DISP("\r\n\r\nFFT Samples was successfully set to %d!\r\n", Samples);    
    
    
  } else if ( GUIstatus == eGUI_HP_SWITCHED ){
    
    if( switch_HP_to_DC_null ){
      if( HP_Filter ){
        DISP("\r\n\r\nDC nulling was successfully Enabled!\r\n");
      } else {
        DISP("\r\n\r\nDC nulling was successfully Disabled!\r\n");
      }
    } else {
      if( HP_Filter ){
        DISP("\r\n\r\nHP filter was successfully Enabled!\r\n");
      } else {
        DISP("\r\n\r\nHP filter was successfully Disabled!\r\n");
      }
    }    
    
    
  } else if ( GUIstatus == eGUI_HP_SWITCHED_ERR ){
    
    DISP("\r\n\r\nError occurred during HP filter switch! Please restart nucleo board...\r\n");    
    Error_Handler();
    
    
  } else if ( GUIstatus == eGUI_HP2DC_NULL_SUBST ){
    
    if( switch_HP_to_DC_null ){
      DISP("\r\n\r\nOption HP Filter was successfully changed to DC nulling in main menu!\r\n");     
    } else {
      DISP("\r\n\r\nOption DC nulling was successfully changed to HP Filter in main menu!\r\n");
    }        
    
    
  } else if ( GUIstatus == eGUI_FIRST_ODR_MEAS ){
    
    DISP("\r\n\r\n(Initially measured ODR = %d Hz)\r\n", (int)ODR_measured);
    
  } 
  
  /* Wait until control button is pressed */
  RxBuffer = '\0';
  while(RxBuffer != '1' && RxBuffer != '2' && RxBuffer != '3' && RxBuffer != '4' && RxBuffer != '5' && RxBuffer != '6' && RxBuffer != '7' && RxBuffer != '8' && RxBuffer != '9' && RxBuffer != 'h' && RxBuffer != 'H' && RxBuffer != 'f' && RxBuffer != 'F'){
    HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1000);
    
  }
  
  
  /* Go to chosen menu */
  switch( RxBuffer ){
  case '1':
    ODRMenu();
    break;
    
  case '2':
    magnitudeMenu();
    break;
    
  case '3':
    axisMenu();
    break; 
    
  case '4':
    fullScaleMenu(); 
    break; 
    
  case '5':
    opModeMenu();
    break;
    
  case '6':
    samplesMenu();
    break;
    
  case '7':
    En_Dis_HP_or_DCnull(); 
    break;
    
  case '8':
    GUIstatus = eGUI_PLOT_GRAPH;
    break;
    
  case '9':
    GUIstatus = eGUI_PLOT_TABLE;
    data_For_FFT_Ready = 0;
    table_displayed = 0;
    break;
    
  case 'h':
    HP_DC_changer();
    break;
    
  case 'H':
    HP_DC_changer();
    break;
    
  default: 
    break;
  }
}


/**
* @brief  Display Set ODR menu in terminal
*
* User can change theese vlaues of ODR by pressing proper button (number) on PCs keyboard:
* 1) 200 Hz
* 2) 400 Hz 
* 3) 1344 Hz
* 4) 1620 Hz
* 5) 5376 kHz
*
* Set ODR menu include one status bars:
* 1) top status bar, that shows actually set value
*
* @param  void
* @retval void
*/
void ODRMenu(void){
  
  float ODR_Switch = 0;
  OPER_MODE_t operatingMode;
  uint8_t tmp;
  
  /* Print menu header to terminal */
  printMenuHeader(HEADER_ODR);
  
  //BSP_ACCELERO_Get_ODR( ACCELERO_handle, &ODR_Switch );
  BSP_ACCELERO_Read_Reg( ACCELERO_handle, 0x20, &tmp );
  tmp &= (0xF0);
  
  BSP_ACCELERO_Get_OpMode_Ext( ACCELERO_handle, &operatingMode );
  
  switch(tmp)
  {
  default:
  case 0x60:
    ODR_Switch = 200;
    break;
    
  case 0x70:
    ODR_Switch = 400;
    break;
    
  case 0x80:
    if(operatingMode == LOW_PWR_MODE){
      ODR_Switch = 1620;
    }
    break;
    
  case 0x90:
    if(operatingMode == LOW_PWR_MODE){
      ODR_Switch = 5376;
    } else {
      ODR_Switch = 1344;
    }
    break;
  }
  
  /* Actual settings */
  DISP("Actually set value:\r\n");
  DISP("ODR = %.0f Hz\r\n\r\n\r\n", ODR_Switch);
  
  /* Options to be choosed */
  DISP("ODR:\t\t\tNo. to be pressed:\r\n");  
  DISP("------------------------------------------\r\n");  
  DISP("\r\n200 Hz\t\t\t\"1\"\r\n");  
  DISP("\r\n400 Hz\t\t\t\"2\"\r\n");
  
  if(operatingMode == LOW_PWR_MODE){
    DISP("\r\n*1344 Hz\t\t\"3\"\r\n");
  } else {
    DISP("\r\n1344 Hz\t\t\t\"3\"\r\n");
  }
  
  if(operatingMode == LOW_PWR_MODE){
    DISP("\r\n1620 Hz\t\t\t\"4\"\r\n");
  } else {
    DISP("\r\n*1620 Hz\t\t\"4\"\r\n");
  }
  
  if(operatingMode == LOW_PWR_MODE){
    DISP("\r\n5376 Hz\t\t\t\"5\"\r\n");
  } else {
    DISP("\r\n*5376 Hz\t\t\"5\"\r\n");
  }
  
  DISP("\r\n\r\nReturn to Main menu\t\"0\"\r\n");  
  
  
  if(operatingMode == LOW_PWR_MODE){
    DISP("\r\n\r\n\r\n\r\n\r\n\r\n* You have to set Operating Mode to High resolution mode or Normal mode\r\n  to use this option\r\n");
  } else {
    DISP("\r\n\r\n\r\n\r\n\r\n\r\n* You have to set Operating Mode to Low power mode to use this option\r\n");
  }
  
  /* Wait until control button is pressed */
  RxBuffer = '\0';
  if(operatingMode == LOW_PWR_MODE){
    
    while(RxBuffer != '0' && RxBuffer != '1' && RxBuffer != '2' && RxBuffer != '4' && RxBuffer != '5'){
      HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1000);
      
    }
  } else {
    
    while(RxBuffer != '0' && RxBuffer != '1' && RxBuffer != '2' && RxBuffer != '3'){
      HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1000);
      
    }  
  }
  
  GUIstatus = eGUI_ODR_SET;
  
  /* Set chosen ODR and measure real value */
  switch( RxBuffer ){
  default:
  case '0':
    GUIstatus = eGUI_MAIN_MENU;
    break;
    
  case '1':
    if(BSP_ACCELERO_Read_Reg( ACCELERO_handle, 0x20, &tmp ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_ODR_SET_ERR;
    } else {
      
      tmp &= ~(0xF0);
      tmp |= 0x60;
      
      if(BSP_ACCELERO_Write_Reg( ACCELERO_handle, 0x20, tmp ) != COMPONENT_OK)
      {
        GUIstatus = eGUI_ODR_SET_ERR;
      }
    }
    break;
    
  case '2':
    if(BSP_ACCELERO_Read_Reg( ACCELERO_handle, 0x20, &tmp ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_ODR_SET_ERR;
    } else {
      
      tmp &= ~(0xF0);
      tmp |= 0x70;
      
      if(BSP_ACCELERO_Write_Reg( ACCELERO_handle, 0x20, tmp ) != COMPONENT_OK)
      {
        GUIstatus = eGUI_ODR_SET_ERR;
      }
    }
    break; 
    
  case '3':
    if(BSP_ACCELERO_Read_Reg( ACCELERO_handle, 0x20, &tmp ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_ODR_SET_ERR;
    } else {
      
      tmp &= ~(0xF0);
      tmp |= 0x90;
      
      if(BSP_ACCELERO_Write_Reg( ACCELERO_handle, 0x20, tmp ) != COMPONENT_OK)
      {
        GUIstatus = eGUI_ODR_SET_ERR;
      }
    }
    break; 
    
  case '4':
    if(BSP_ACCELERO_Read_Reg( ACCELERO_handle, 0x20, &tmp ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_ODR_SET_ERR;
    } else {
      
      tmp &= ~(0xF0);
      tmp |= 0x80;
      
      if(BSP_ACCELERO_Write_Reg( ACCELERO_handle, 0x20, tmp ) != COMPONENT_OK)
      {
        GUIstatus = eGUI_ODR_SET_ERR;
      }
    } 
    break;
    
  case '5':
    if(BSP_ACCELERO_Read_Reg( ACCELERO_handle, 0x20, &tmp ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_ODR_SET_ERR;
    } else {
      
      tmp &= ~(0xF0);
      tmp |= 0x90;
      
      if(BSP_ACCELERO_Write_Reg( ACCELERO_handle, 0x20, tmp ) != COMPONENT_OK)
      {
        GUIstatus = eGUI_ODR_SET_ERR;
      }
    }
    break; 
  }
  
  if( GUIstatus != eGUI_ODR_SET_ERR && GUIstatus != eGUI_MAIN_MENU )
  {
    measODR();  
    data_For_FFT_Ready = 0;
  }
}

/**
* @brief  Display Set magnitude menu in terminal
*
* User can change theese vlaues of graph magnitude by pressing proper button (number) on PCs keyboard:
* 1) 200 mg
* 2) 500 mg 
* 3) 1 g
* 4) 2 g
* 5) 4 g
* 6) 8 g
* 7) 16 g
*
* Set magnitude menu include one status bars:
* 1) top status bar, that shows actually set value
*
* @param  void
* @retval void
*/
void magnitudeMenu(void){
  /* Print menu header to terminal */
  printMenuHeader(HEADER_MAG);
  
  /* Actual settings */
  DISP("Actually set value:\r\n");
  
  if( plot_Magnitude < MAG_1g ){
    DISP("Mag = %d mg\r\n\r\n\r\n", plot_Magnitude);
  } else{
    DISP("Mag = %d g\r\n\r\n\r\n", plot_Magnitude/MAG_1g);
  }  
  
  
  /* Options to be choosed */
  DISP("Magnitude:\t\tNo. to be pressed:\r\n");  
  DISP("------------------------------------------\r\n");  
  DISP("\r\n200 mg\t\t\t\"1\"\r\n");  
  DISP("\r\n500 mg\t\t\t\"2\"\r\n");  
  DISP("\r\n1 g\t\t\t\"3\"\r\n");  
  DISP("\r\n2 g\t\t\t\"4\"\r\n");  
  DISP("\r\n4 g\t\t\t\"5\"\r\n");  
  DISP("\r\n8 g\t\t\t\"6\"\r\n");  
  DISP("\r\n16 g\t\t\t\"7\"\r\n");  
  DISP("\r\n\r\nReturn to Main menu\t\"0\"\r\n");  
  
  
  /* Wait until control button is pressed */
  RxBuffer = '\0';
  while(RxBuffer != '0' && RxBuffer != '1' && RxBuffer != '2' && RxBuffer != '3' && RxBuffer != '4' && RxBuffer != '5' && RxBuffer != '6' && RxBuffer != '7'){
    HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1000);
    
  }
  
  GUIstatus = eGUI_MAG_SET;
  
  /* Set chosen graph magnitude */
  switch( RxBuffer ){
  default:
  case '0':
    GUIstatus = eGUI_MAIN_MENU;
    break;
    
  case '1':
    plot_Magnitude = MAG_200mg;
    break;
    
  case '2':
    plot_Magnitude = MAG_500mg;
    break; 
    
  case '3':
    plot_Magnitude = MAG_1g;
    break; 
    
  case '4':
    plot_Magnitude = MAG_2g;
    break;
    
  case '5':
    plot_Magnitude = MAG_4g;
    break;
    
  case '6':
    plot_Magnitude = MAG_8g;
    break;
    
  case '7':
    plot_Magnitude = MAG_16g;
    break;    
  }  
}

/**
* @brief  Display Set sensing axis menu in terminal
*
* User can change sensing axis by pressing proper button (number) on PCs keyboard:
* 1) N (Euclidean norm)
* 1) X
* 2) Y 
* 3) Z
*
* Set sensing axis menu include one status bars:
* 1) top status bar, that shows actually set value
*
* @param  void
* @retval void
*/
void axisMenu(void){
  
  ACTIVE_AXIS_t axisActive;
  
  /* Print menu header to terminal */
  printMenuHeader(HEADER_AXIS);  
  
  BSP_ACCELERO_Get_Active_Axis_Ext( ACCELERO_handle, &axisActive );
  
  /* Actual settings */
  DISP("Actually set value:\r\n");
  
  switch( axisActive ){
  case X_AXIS:
    DISP("Axis = X\r\n\r\n\r\n");
    break;
    
  case Y_AXIS:
    DISP("Axis = Y\r\n\r\n\r\n");
    break;
    
  case Z_AXIS:
    DISP("Axis = Z\r\n\r\n\r\n");
    break;
    
  default:
    break;
  }
  
  
  /* Options to be choosed */
  DISP("Sensing axis:\t\tNo. to be pressed:\r\n");  
  DISP("------------------------------------------\r\n");  
  DISP("\r\nX-axis\t\t\t\"1\"\r\n");  
  DISP("\r\nY-axis\t\t\t\"2\"\r\n"); 
  DISP("\r\nZ-axis\t\t\t\"3\"\r\n");  
  DISP("\r\n\r\nReturn to Main menu\t\"0\"\r\n");    
  
  
  /* Wait until control button is pressed */
  RxBuffer = '\0';
  while(RxBuffer != '0' && RxBuffer != '1' && RxBuffer != '2' && RxBuffer != '3'){
    HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1000);
    
  }
  
  GUIstatus = eGUI_AXIS_SET;
  
  /* Set chosen sensing axis */
  switch( RxBuffer ){
  default:
  case '0':
    GUIstatus = eGUI_MAIN_MENU;
    break;
    
  case '1':
    if(BSP_ACCELERO_Set_Active_Axis_Ext( ACCELERO_handle, X_AXIS ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_AXIS_SET_ERR;
    } else {
      
      data_For_FFT_Ready = 0;   
    }
    break;
    
  case '2':
    if(BSP_ACCELERO_Set_Active_Axis_Ext( ACCELERO_handle, Y_AXIS ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_AXIS_SET_ERR;
    } else {
      
      data_For_FFT_Ready = 0;   
    }
    break; 
    
  case '3':
    if(BSP_ACCELERO_Set_Active_Axis_Ext( ACCELERO_handle, Z_AXIS ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_AXIS_SET_ERR;
    } else {
      
      data_For_FFT_Ready = 0;   
    }
    break; 
  }
}


/**
* @brief  Display Set Full scale menu in terminal
*
* User can change theese vlaues of AXL Full scale by pressing proper button (number) on PCs keyboard:
* 1) +-2 g
* 2) +-4 g
* 3) +-8 g
* 4) +-16 g
*
* Set Full scale menu include one status bars:
* 1) top status bar, that shows actually set value
*
* @param  void
* @retval void
*/
void fullScaleMenu(void){
  float fullScale = 0;
  
  /* Print menu header to terminal */
  printMenuHeader(HEADER_FS);
  
  BSP_ACCELERO_Get_FS( ACCELERO_handle, &fullScale);
  
  /* Actual settings */
  DISP("Actually set value:\r\n");
  DISP("FS = +-%.0f g\r\n\r\n\r\n", fullScale);
  
  
  /* Options to be choosed */
  DISP("Full scale:\t\tNo. to be pressed:\r\n");  
  DISP("------------------------------------------\r\n");  
  DISP("\r\n+-2 g\t\t\t\"1\"\r\n");  
  DISP("\r\n+-4 g\t\t\t\"2\"\r\n");  
  DISP("\r\n+-8 g\t\t\t\"3\"\r\n");  
  DISP("\r\n+-16 g\t\t\t\"4\"\r\n");  
  DISP("\r\n\r\nReturn to Main menu\t\"0\"\r\n");
  
  /* Wait until control button is pressed */
  RxBuffer = '\0';
  while(RxBuffer != '0' && RxBuffer != '1' && RxBuffer != '2' && RxBuffer != '3' && RxBuffer != '4'){
    HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1000);
    
  }
  
  GUIstatus = eGUI_FS_SET;
  
  /* Set chosen AXL Full scale */
  switch( RxBuffer ){  
  default:
  case '0':
    GUIstatus = eGUI_MAIN_MENU;
    break;
    
  case '1':
    if(BSP_ACCELERO_Set_FS_Value( ACCELERO_handle, 2.0f ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_FS_SET_ERR;
    }
    data_For_FFT_Ready = 0;
    break;
    
  case '2':
    if(BSP_ACCELERO_Set_FS_Value( ACCELERO_handle, 4.0f ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_FS_SET_ERR;
    }
    data_For_FFT_Ready = 0;
    break; 
    
  case '3':
    if(BSP_ACCELERO_Set_FS_Value( ACCELERO_handle, 8.0f ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_FS_SET_ERR;
    } 
    data_For_FFT_Ready = 0;
    break; 
    
  case '4':
    if(BSP_ACCELERO_Set_FS_Value( ACCELERO_handle, 16.0f ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_FS_SET_ERR;
    } 
    data_For_FFT_Ready = 0;
    break;    
  }  
}


/**
* @brief  Display Set Operating mode menu in terminal
*
* User can change theese vlaues of Operating Mode by pressing proper button (number) on PCs keyboard:
* 1) Low power
* 2) Normal
* 3) High res
*
* Set Operating mode menu include one status bar:
* 1) top status bar, that shows actually set value
*
* @param  void
* @retval void
*/
void opModeMenu(void){
  
  float ODR_Switch = 0;
  OPER_MODE_t operatingMode;
  uint8_t tmp;   
  /* Print menu header to terminal */
  printMenuHeader(HEADER_OPMODE);
  
  //BSP_ACCELERO_Get_ODR( ACCELERO_handle, &ODR_Switch );
  BSP_ACCELERO_Read_Reg( ACCELERO_handle, 0x20, &tmp );
  tmp &= (0xF0);
  
  BSP_ACCELERO_Get_OpMode_Ext( ACCELERO_handle, &operatingMode );
  
  switch(tmp)
  {
  default:
  case 0x60:
    ODR_Switch = 200;
    break;
    
  case 0x70:
    ODR_Switch = 400;
    break;
    
  case 0x80:
    if(operatingMode == LOW_PWR_MODE){
      ODR_Switch = 1620;
    }
    break;
    
  case 0x90:
    if(operatingMode == LOW_PWR_MODE){
      ODR_Switch = 5376;
    } else {
      ODR_Switch = 1344;
    }
    break;
  }
  
  /* Actual settings */
  DISP("Actually set value:\r\n");
  
  switch( operatingMode ){
  case LOW_PWR_MODE:
    DISP("OpMode = LPM\r\n\r\n\r\n");
    break;
    
  case NORMAL_MODE:
    DISP("OpMode = NM\r\n\r\n\r\n");
    break;
    
  case HIGH_RES_MODE:
    DISP("OpMode = HRM\r\n\r\n\r\n");
    break;
    
  default: 
    break;
  }
  
  /* Options to be choosed */
  DISP("Operating mode:\t\tNo. to be pressed:\r\n");
  DISP("------------------------------------------\r\n");
  
  if( ODR_Switch == 1344.0f ){
    DISP("\r\n*OpMode = Low power\t\"1\"\r\n");
  } else {
    DISP("\r\nOpMode = Low power\t\"1\"\r\n");
  }    
  if( ODR_Switch == 1620.0f || ODR_Switch == 5376.0f ){
    DISP("\r\n*OpMode = Normal\t\"2\"\r\n");
    DISP("\r\n*OpMode = High res\t\"3\"\r\n");
  } else {
    DISP("\r\nOpMode = Normal\t\t\"2\"\r\n");
    DISP("\r\nOpMode = High res\t\"3\"\r\n");
  }  
  DISP("\r\n\r\nReturn to Main menu\t\"0\"\r\n");
  
  
  if( ODR_Switch == 1344.0f || ODR_Switch == 1620.0f || ODR_Switch == 5376.0f )
  {
    DISP("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n* You have to set a common ODR for all OpModes (i.e. 200 Hz or 400 Hz) \r\n  to use this option\r\n");
    
  }
  
  
  /* Wait until control button is pressed */
  RxBuffer = '\0';
  if( ODR_Switch == 1344.0f ){
    
    while(RxBuffer != '0' && RxBuffer != '2' && RxBuffer != '3'){
      HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1000);
      
    }
  } else if( ODR_Switch == 1620.0f || ODR_Switch == 5376.0f ){
    
    while(RxBuffer != '0' && RxBuffer != '1'){
      HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1000);
      
    }
  } else {
    
    while(RxBuffer != '0' && RxBuffer != '1' && RxBuffer != '2' && RxBuffer != '3'){
      HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1000);
      
    }
  }  
  
  GUIstatus = eGUI_OPMODE_SET;
  
  /* Set chosen AXL Full scale */
  switch( RxBuffer ){  
  default:
  case '0':
    GUIstatus = eGUI_MAIN_MENU;
    break;
    
  case '1':
    if(BSP_ACCELERO_Set_OpMode_Ext( ACCELERO_handle, LOW_PWR_MODE ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_OPMODE_SET_ERR;
    }
    data_For_FFT_Ready = 0;
    break;
    
  case '2':
    if(BSP_ACCELERO_Set_OpMode_Ext( ACCELERO_handle, NORMAL_MODE ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_OPMODE_SET_ERR;
    }
    data_For_FFT_Ready = 0;
    break; 
    
  case '3':
    if(BSP_ACCELERO_Set_OpMode_Ext( ACCELERO_handle, HIGH_RES_MODE ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_OPMODE_SET_ERR;
    } 
    data_For_FFT_Ready = 0;
    break;  
  }  
}


/**
* @brief  Display Set FFT samples menu in terminal
*
* User can change theese vlaues of FFT samples by pressing proper button (number) on PCs keyboard:
* 1) 256
* 2) 512
* 3) 1024
*
* Set FFT samples menu include one status bars:
* 1) top status bar, that shows actually set value
*
* @param  void
* @retval void
*/
void samplesMenu(void){
  
  /* Print menu header to terminal */
  printMenuHeader(HEADER_SAMPLES);
  
  /* Actual settings */
  DISP("Actually set value:\r\n");
  DISP("Sam = %d\r\n\r\n\r\n", Samples);
  
  /* Options to be choosed */
  DISP("Number of samples:\tNo. to be pressed:\r\n");
  DISP("------------------------------------------\r\n");
  DISP("\r\n256\t\t\t\"1\"\r\n");
  DISP("\r\n512\t\t\t\"2\"\r\n");
  DISP("\r\n1024\t\t\t\"3\"\r\n");
  DISP("\r\n\r\nReturn to Main menu\t\"0\"\r\n");
  
  
  /* Wait until control button is pressed */
  RxBuffer = '\0';
  while(RxBuffer != '0' && RxBuffer != '1' && RxBuffer != '2' && RxBuffer != '3'){
    HAL_UART_Receive(&UartMsgHandle, &RxBuffer, 1, 1000);
    
  }  
  
  GUIstatus = eGUI_SAMPLES_SET;
  
  /* Set chosen FFT Samples and calculate values for graph x axis  */
  switch( RxBuffer ){
  default:
  case '0':
    GUIstatus = eGUI_MAIN_MENU;
    break;
    
  case '1':
    Samples = 256;
    calculateFreqAxis(); 
    data_For_FFT_Ready = 0;
    break;
    
  case '2':
    Samples = 512;
    calculateFreqAxis();
    data_For_FFT_Ready = 0;
    break;
    
  case '3':
    Samples = 1024;
    calculateFreqAxis();
    data_For_FFT_Ready = 0;    
  }  
}



/**
* @brief  Calculate FFT and create data array for FFT graph plot
* @param  None
* @retval None
*/
void do_fft( void ){
  float32_t fftTmp[MAX_SAMPLES];
  float32_t fftTmp2[MAX_SAMPLES];
  arm_rfft_fast_instance_f32 Real_FFT_Instance;
  uint32_t  ifftFlag = 0;
  uint16_t i, j;
  
  for(i=0; i<51; i++){
    fftTextPlotData[i] = 0;
  }  
  
  for(i=0; i<Samples; i++){
    fftTmp[i] = fftInput[i];
  }
  
  /* Calculate FFT */
  arm_rfft_fast_init_f32(&Real_FFT_Instance, Samples);
  arm_rfft_fast_f32(&Real_FFT_Instance, fftTmp, fftTmp2, ifftFlag);	
  
  /* Process the data through the Complex Magnitude Module for
  calculating the magnitude at each bin */
  arm_cmplx_mag_f32(fftTmp2, fftOutput, Samples);
  
  
  /* Flat Top window is supported only when HP filter is on */
  if( !switch_HP_to_DC_null && HP_Filter ){ 
    
    for(i=0; i<Samples; i++){
      fftTmp[i] = fftInput[i]*flatTopWin[i*(1024/Samples)];
    }    
    
    /* Calculate FFT using Flat Top window */
    arm_rfft_fast_init_f32(&Real_FFT_Instance, Samples);
    arm_rfft_fast_f32(&Real_FFT_Instance, fftTmp, fftTmp2, ifftFlag);	
    
    /* Process the data through the Complex Magnitude Module for
    calculating the magnitude at each bin */
    arm_cmplx_mag_f32(fftTmp2, fftTmp, Samples);
  }
  
  
  j = 0;
  
  for(i=0; i<Samples/2; i++){
    if( i == 0 ){
      if( switch_HP_to_DC_null && HP_Filter ){ 
        fftOutput[i] = 0;                               // if DC nulling enabled
        //fftTmp[i] = 0;
      } else {
        fftOutput[i] = (fftOutput[i]/(Samples))*1000;
        fftTmp[i] = ((fftTmp[i]/(Samples))*1000)*scaleFactor;
      }
    } else {
      fftOutput[i] = (fftOutput[i]/(Samples))*2000;       // output*2
      fftTmp[i] = ((fftTmp[i]/(Samples))*2000)*scaleFactor;
    }
    
    
    if( i == freqAxis[j] ){
      j++;
    }
    
    /* Consider only max value in frequency range of bin */
    if( fftOutput[i] > fftTextPlotData[j] ){
      fftTextPlotData[j] = fftOutput[i];
    }
  }
  
  /* Calculates maxValue and returns corresponding BIN value */
  arm_max_f32(fftOutput, Samples/2, &maxValue, &maxIndex);
  
  /* Calculates maxValue and returns corresponding BIN value */
  arm_max_f32(fftTextPlotData, 51, &BinMaxValue, &BinMaxIndex);
  
  
  /* Flat Top window is supported only when HP filter is on */
  if( !switch_HP_to_DC_null && HP_Filter ){   
    
    /* Calculates maxValue and returns corresponding BIN value */
    arm_max_f32(fftTmp, Samples/2, &FTW_MaxValue, &FTW_MaxIndex);
    
  }   
}


/**
* @brief  Plot FFT Table
* @param  None
* @retval None
*/
void plot_table(void){
  int16_t i = 0;
  
  DISP("\r\n\r\nInput [mg]:\tOutput [mg]:\tGraph data [mg]:\tODR [Hz]:\tSamples:\r\n");
  
  for(i=0; i<Samples; i++){
    
    if( i < 1 ){                        // all data
      DISP("%.f\t\t%4.2f\t\t%4.2f\t\t\t%d\t\t%d\r\n", fftInput[i]*1000, fftOutput[i], fftTextPlotData[i], (uint16_t)ODR_measured, Samples);
      
    }else if( i < 51 ){                 // all data but ODR and Samples
      DISP("%.f\t\t%4.2f\t\t%4.2f\r\n", fftInput[i]*1000, fftOutput[i], fftTextPlotData[i]);
      
    } else if( i < Samples/2 ){         // Input and Output data only
      DISP("%.f\t\t%4.2f\r\n", fftInput[i]*1000, fftOutput[i]);
      
    } else {                            // Input data only
      DISP("%.f\r\n", fftInput[i]*1000);
      
    }
  }
  
  table_displayed = 1;
  
}


/**
* @brief  Plot FFT Graph and max frequency + value
* @param  None
* @retval None
*/
void plot_fft(void){
  int16_t i, tmp;
  char plotData[53];
  int32_t range[2];
  int32_t max_f = 0;
  float tmp2;
  float max = -1e5; 
  float min = 1e5;
  float ampl = 0;
  
  
  // Calculate:
  
  // e.g.: for 416 Hz:
  // 0    20.8   41.6   62.4   83.2  104  124.8  145.6  166.4  187.2  208
  if( BinMaxIndex == 0 ){
    range[0] = 0;
    range[1] = (int32_t)(((ODR_measured/Samples)*freqAxis[BinMaxIndex])+0.5);
  } else if (BinMaxIndex == 50){
    range[0] = (int32_t)(((ODR_measured/Samples)*freqAxis[BinMaxIndex-1])+0.5);
    range[1] = (int32_t)((ODR_measured/2)+0.5);
  } else {
    range[0] = (int32_t)(((ODR_measured/Samples)*freqAxis[BinMaxIndex-1])+0.5);
    range[1] = (int32_t)(((ODR_measured/Samples)*freqAxis[BinMaxIndex])+0.5);
  }
  
  max_f = (int32_t)((maxIndex*ODR_measured)/(Samples));
  
  for( i=0; i<Samples; i++ ){
    if( fftInput[i] > max ){
      max = fftInput[i];
    }
    if( fftInput[i] < min ){
      min = fftInput[i];
    }
  } 
  
  ampl = ((max - min)*500);
  
  
  // Display:
  DISP("\033[2J\033[0;0H     A[mg]\r\n\r\n       ^\r\n       |\r\n       |\r\n");
  
  tmp = plot_Magnitude;
  for(i=0; i<11; i++){
    
    fft_plot_creator(tmp, &plotData[0]);
    if( i%2 != 0 ){
      DISP("      _%s\r\n", plotData);
    } else {
      
      if(tmp == 0){
        DISP("   %d __%s__ -> f[Hz]\r\n", tmp, plotData);
      } else if( tmp < 100 ){
        DISP("  %d __%s\r\n", tmp, plotData);
      } else if( tmp < 1000 ){
        DISP(" %d __%s\r\n", tmp, plotData);
      } else if( tmp < 10000 ){
        DISP("%d __%s\r\n", tmp, plotData);
      } else {
        tmp2 = tmp/1000;
        DISP(" %dK __%s\r\n", (int16_t)tmp2, plotData);
      } 
      
    }      
    tmp-=plot_Magnitude/10;
  }  
  
  for(i=0; i<11; i++){
    if( i==0 ){
      DISP("       |");
    }else{
      DISP("    |");
    }    
  }  
  
  tmp2 = ODR_measured/20;
  for (i=0; i<11; i++){
    tmp =(int16_t)((float)tmp2*i+0.5);
    
    if(tmp == 0){
      DISP("\r\n       0");
    }else if( tmp < 10 ){
      DISP("    %d ", tmp);
    }else if( tmp < 100 ){
      DISP("   %d", tmp);
    }else if( tmp < 1000){
      DISP("  %d", tmp);
    }else if( tmp < 10000 ){
      DISP(" %d", tmp);
    }    
  }    
  
  DISP("\r\n\r\n\r\nMax f = %d Hz\r\nMax value = %4.2f mg\r\n\r\n", max_f, maxValue);
  
  /* Flat Top window is supported only when HP filter is on */
  if( !switch_HP_to_DC_null && HP_Filter ){ 
    DISP("\r\nMax value (Flat Top window used): %4.2f mg\r\n\r\n", FTW_MaxValue);    
  }  
  DISP("\r\nMax f Bin no. %d\r\n[i.e. values %d to %d Hz]\r\n\r\n", BinMaxIndex, range[0], range[1]);  
  DISP("\r\n(Input signal amplitude = %.2f mg)\r\n", ampl);
}


/**
* @brief  Create FFT graph for plot
* @param  None
* @retval None
*/
void fft_plot_creator(int16_t n, char *c){
  uint16_t i; 
  
  for(i=0; i<52; i++){
    c[i]='\0';
  }
  
  for(i=0; i<51; i++){
    
    if(fftTextPlotData[i] > n){
      c[i] = '#';
      
      if( n == 0 & fftTextPlotData[i] < (plot_Magnitude/100) ){
        c[i] = '_';
        if(i == 0){
          c[i] = '|';
        }
      }
      
    }else{ 
      if( i == 0 ){
        c[i] = '|';
      }else{
        c[i] = ' ';
      }
      
    }
  } 
}


/**
* @brief  Measure ODR of AXL
* @param  None
* @retval None
*/
void measODR( void ){
  uint8_t  ODR_meas_enable = 1; 
  uint16_t ODR_meas_iter = 0;
  uint16_t ODR_meas_start_time = 0;
  uint16_t ODR_meas_stop_time = 0;
  uint16_t ODR_meas_samples = 150;                // number of measured samples for calculating ODR
  
  ACTIVE_AXIS_t axisActive;
  
  BSP_ACCELERO_Get_Active_Axis_Ext( ACCELERO_handle, &axisActive );
  BSP_ACCELERO_ClearDRDY_Ext( ACCELERO_handle, axisActive );
  
  while( ODR_meas_enable ){
    if( AXL_DRDY_received ){
      AXL_DRDY_received = 0;
      /* Get start time */
      if( ODR_meas_iter == 0){
        Int_Current_Time1 = user_currentTimeGetTick();
        ODR_meas_start_time = Int_Current_Time1;
      }
      
      /* Get stop time */ 
      if( ODR_meas_iter == ODR_meas_samples - 1){
        Int_Current_Time2 = user_currentTimeGetTick();
        ODR_meas_stop_time = Int_Current_Time2;
        ODR_meas_enable = 0;
        
      }
      
      /* Stop after measuring "ODR_meas_samples" values */ 
      if( ODR_meas_iter < ODR_meas_samples ){
        ODR_meas_iter++;
        BSP_ACCELERO_ClearDRDY_Ext( ACCELERO_handle, axisActive );
      }
    }    
  }
  
  /* Calculate measured ODR */
  ODR_measured = ((float)(1000*ODR_meas_samples)/(ODR_meas_stop_time - ODR_meas_start_time));
  
}


/**
* @brief  Calculate bins values for graph x axis
* @param  None
* @retval None
*/
void calculateFreqAxis( void ){
  int8_t i = 0;
  
  float delta = ((float)Samples/100);   // (samples/2)/50
  float tmp = (delta/2); 
  
  for(i=0; i<50; i++){
    freqAxis[i] = (int16_t)(tmp + 0.5);
    tmp += delta;
  }
  
}


/**
* @brief  Enable/Disable HP or DCnull
* @param  void
* @retval void
*/
void En_Dis_HP_or_DCnull( void ){
  
  if( switch_HP_to_DC_null ){
    
    if ( HP_Filter ){
      HP_Filter = 0;
    } else {
      HP_Filter = 1;
    }
    
    data_For_FFT_Ready = 0;
    GUIstatus = eGUI_HP_SWITCHED;
    
  } else {
    if ( HP_Filter ){
      
      /* Disable HP filter */
      if(BSP_ACCELERO_Disable_HP_Filter_Ext( ACCELERO_handle ) != COMPONENT_OK)
      {
        GUIstatus = eGUI_HP_SWITCHED_ERR;
      } else {
        
        HP_Filter = 0;  
        data_For_FFT_Ready = 0;
        GUIstatus = eGUI_HP_SWITCHED;
        
      }        
      
    } else {   
      
      /* Enable HP filter */
      if(BSP_ACCELERO_Enable_HP_Filter_Ext( ACCELERO_handle, HPF_MODE_NORMAL, CUTOFF_MODE2 ) != COMPONENT_OK)
      {
        GUIstatus = eGUI_HP_SWITCHED_ERR;
      } else {
        
        HP_Filter = 1;
        data_For_FFT_Ready = 0;
        GUIstatus = eGUI_HP_SWITCHED;
        
      }
    }
  }
}


/**
* @brief  Changes HP to DCnull and vice versa in main menu options
* @param  void
* @retval void
*/
void HP_DC_changer( void ){
  
  if( switch_HP_to_DC_null ){
    
    switch_HP_to_DC_null = 0;
    HP_Filter = 0;
    GUIstatus = eGUI_HP2DC_NULL_SUBST;
    
  } else {
    
    /* Disable HP filter */        
    if(BSP_ACCELERO_Disable_HP_Filter_Ext( ACCELERO_handle ) != COMPONENT_OK)
    {
      GUIstatus = eGUI_HP_SWITCHED_ERR;
    } else {
      
      switch_HP_to_DC_null = 1;
      HP_Filter = 0;
      data_For_FFT_Ready = 0;
      GUIstatus = eGUI_HP2DC_NULL_SUBST;
      
    } 
  }
}


/**
* @brief  Acquire Data from IIS2DH
* @param  void
* @retval void
*/
void acquireData( void ){
  uint16_t amountOfData = 0;
  int16_t data;
  uint16_t i;
  OPER_MODE_t opMode;
  float sensitivity = 0.0f;
  int16_t tmpArray[MAX_SAMPLES];
  ACTIVE_AXIS_t axisActive;
  
  BSP_ACCELERO_Get_OpMode_Ext( ACCELERO_handle, &opMode );
  BSP_ACCELERO_Get_Sensitivity( ACCELERO_handle, &sensitivity );
  BSP_ACCELERO_Get_Active_Axis_Ext( ACCELERO_handle, &axisActive );
  
  
  while( amountOfData != Samples ){
    if(AXL_DRDY_received){
      AXL_DRDY_received = 0;
      BSP_ACCELERO_Get_SuperRawAxes_Ext( ACCELERO_handle, &data, axisActive );      // get data
      
      tmpArray[amountOfData] = data; 
      
      amountOfData++;
    }
  }     
  
  for(i=0; i<Samples; i++){ 
    
    if( opMode == LOW_PWR_MODE){
      tmpArray[i] >>= 8;
      
      /* convert the 2's complement 8 bit to 2's complement 16 bit */
      if (tmpArray[i] & 0x0080){
        tmpArray[i] |= 0xFF00;
      }
    } else if( opMode == NORMAL_MODE){
      tmpArray[i] >>= 6;
      
      /* convert the 2's complement 10 bit to 2's complement 16 bit */
      if (tmpArray[i] & 0x0200){
        tmpArray[i] |= 0xFC00;
      }
    } else if( opMode == HIGH_RES_MODE){
      tmpArray[i] >>= 4;
      
      /* convert the 2's complement 12 bit to 2's complement 16 bit */
      if (tmpArray[i] & 0x0800){
        tmpArray[i] |= 0xF000;
      }
    }
    
    fftInput[i] = (float)((tmpArray[i] * sensitivity)/1000);
    
  }
  
  data_For_FFT_Ready = 1;
} 



/**
* @brief  Print menu header to terminal
*
* @param  menuHeader the number of menu to be printed to terminal
* @retval void
*/
void printMenuHeader( Menu_Header menuHeader ){
  
  char stars[HEADER_LENGTH+1] = {'\0'};
  char spaces[(HEADER_LENGTH+1)/2] = {'\0'};
  char *menuName;
  uint8_t menuNameLength = 0;
  uint8_t i = 0;
  
  switch ( menuHeader ){
  case HEADER_MAIN:
    menuName = "Main Menu";
    break;
    
  case HEADER_ODR:
    menuName = "Set ODR Menu";
    break;
    
  case HEADER_MAG:
    menuName = "Set magnitude Menu";
    break;
    
  case HEADER_AXIS:
    menuName = "Set sensing axis Menu";
    break;
    
  case HEADER_FS:
    menuName = "Set Full scale Menu";
    break;
    
  case HEADER_SAMPLES:
    menuName = "Set FFT samples Menu";
    break;
    
  case HEADER_OPMODE:
    menuName = "Set Operating mode Menu";
    break;
    
  default:
    break;
  }
  
  while(menuName[menuNameLength]!='\0'){
    menuNameLength++;
  }
  
  for( i=0; i<HEADER_LENGTH; i++ ){
    stars[i] = '*';
  }
  
  if( menuNameLength%2 != 0 && HEADER_LENGTH%2 == 0 ||  menuNameLength%2 == 0 && HEADER_LENGTH%2 != 0 ){
    
    for( i=0; i<((HEADER_LENGTH - (menuNameLength + 1) - 4)/2); i++ ){
      spaces[i] = ' ';
    }    
    
  } else {
    
    for( i=0; i<((HEADER_LENGTH - menuNameLength - 4)/2); i++ ){
      spaces[i] = ' ';
    }
    
  }
  
  DISP("\033[2J\033[0;0H%s\r\n", stars);
  
  if( menuNameLength%2 != 0 && HEADER_LENGTH%2 == 0 ||  menuNameLength%2 == 0 && HEADER_LENGTH%2 != 0 ){    
    DISP("** %s%s%s**\r\n", spaces, menuName, spaces);
    
  } else {    
    DISP("**%s%s%s**\r\n", spaces, menuName, spaces);       
  }  
  DISP("%s\r\n\r\n", stars);
}


/**
* @brief  Get max frequency value.
*
* @param  none
* @retval calculated max frequency
*/
uint32_t getFFTMaxFreq( void )
{  
  return (uint32_t)((maxIndex*ODR_measured)/(Samples));
}


/**
* @brief  Get amplitude of max frequency using Flat Top window.
*
* @param  none
* @retval calculated max frequency
*/
uint32_t getFFTMaxFreqAmp( void )
{
  return (uint32_t)(FTW_MaxValue);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/