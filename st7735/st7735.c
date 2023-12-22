/*

  ******************************************************************************
  * @file 			( фаил ):   ST7735.c
  * @brief 		( описание ):  	
  ******************************************************************************
  * @attention 	( внимание ):
  ******************************************************************************
  
*/

#include <ST7735.h>


uint16_t ST7735_X_Start = ST7735_XSTART;	
uint16_t ST7735_Y_Start = ST7735_YSTART;

uint16_t ST7735_Width = 0;
uint16_t ST7735_Height = 0;

#if FRAME_BUFFER
// массив буфер кадра
	uint16_t buff_frame[ST7735_WIDTH*ST7735_HEIGHT] = { 0x0000, };
#endif

static void ST7735_ExecuteCommandList(const uint8_t *addr);
static void ST7735_Unselect(void);
static void ST7735_Select(void);
static void ST7735_SendCmd(uint8_t Cmd);
static void ST7735_SendData(uint8_t Data );
static void ST7735_SendDataMASS(uint8_t* buff, size_t buff_size);
static void ST7735_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
static void ST7735_RamWrite(uint16_t *pBuff, uint32_t Len);
static void ST7735_ColumnSet(uint16_t ColumnStart, uint16_t ColumnEnd);
static void ST7735_RowSet(uint16_t RowStart, uint16_t RowEnd);
static void SwapInt16Values(int16_t *pValue1, int16_t *pValue2);
static void ST7735_DrawLine_Slow(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);


// based on Adafruit ST7735 library for Arduino
static const uint8_t
  init_cmds1[] = {            // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
     ST7735_ROTATION,        //     row addr/col addr, bottom to top refresh*******************************************************************************************************
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
     ST7735_ColorMode_16bit },//     16-bit color

#if (defined(ST7735_IS_128X128))
  init_cmds2[] = {            // Init for 7735R, part 2 (1.44" display)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00>>8, 0x00&0xff,     //     XSTART = 0>>8, 0&0xff
      0x00>>8, 0x7F&0xff,     //     XEND = (128-1)>>8, (128-1)&0xff
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00>>8, 0x00&0xff,     //     XSTART = 0>>8, 0&0xff
      0x00>>8, 0x7F&0xff },   //     XEND = (128-1)>>8, (128-1)&0xff
#endif // ST7735_IS_128X128
	  

								

#ifdef ST7735_IS_160X80
  init_cmds2[] = {            // Init for 7735S, part 2 (160x80 display)
    3,                        //  3 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00>>8, 0x00&0xff,     //     XSTART = 0>>8, 0&0xff
      0x00>>8, 0x4F&0xff,     //     XEND = (80-1)>>8, (80-1)&0xff
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00>>8, 0x00&0xff,     //     XSTART = 0>>8, 0&0xff
      0x00>>8, 0x9F&0xff ,    //     XEND = (160-1)>>8, (160-1)&0xff
    ST7735_INVON, 0 },        //  3: Invert colors
#endif




  init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay


//==============================================================================
// Процедура инициализации дисплея
//==============================================================================
void ST7735_Init(void){
	
	// Задержка после подачи питания
	// если при старте не всегда запускаеться дисплей увеличиваем время задержки
	HAL_Delay(600);	

	ST7735_Width = ST7735_WIDTH;
	ST7735_Height = ST7735_HEIGHT;


    ST7735_Select();
	
    ST7735_HardReset(); 
	ST7735_ExecuteCommandList(init_cmds1);
	
#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X80))
    ST7735_ExecuteCommandList(init_cmds2);
#endif

	ST7735_ExecuteCommandList(init_cmds3);
	
    ST7735_Unselect();
	
#if FRAME_BUFFER
	ST7735_ClearFrameBuffer();
#endif
}
//==============================================================================


//==============================================================================
// Процедура управления SPI
//==============================================================================
static void ST7735_Select(void) {
	
    #ifdef CS_PORT
			//-- если захотим переделать под HAL ------------------	
			#ifdef ST7735_SPI_HAL
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
			#endif
			//-----------------------------------------------------
			
			//-- если захотим переделать под CMSIS  ---------------
			#ifdef ST7735_SPI_CMSIS
				CS_GPIO_Port->BSRR = ( CS_Pin << 16 );
			#endif
			//-----------------------------------------------------
	#endif
	
}
//==============================================================================


//==============================================================================
// Процедура управления SPI
//==============================================================================
static void ST7735_Unselect(void) {
	
    #ifdef CS_PORT
			//-- если захотим переделать под HAL ------------------	
			#ifdef ST7735_SPI_HAL
				HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
			#endif
			//-----------------------------------------------------
			
			//-- если захотим переделать под CMSIS  ---------------
			#ifdef ST7735_SPI_CMSIS
					 CS_GPIO_Port->BSRR = CS_Pin;
			#endif
			//-----------------------------------------------------
	#endif
	
}
//==============================================================================


//==============================================================================
// Процедура отправки данных для инициализации дисплея
//==============================================================================
static void ST7735_ExecuteCommandList(const uint8_t *addr) {
	
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--) {
        uint8_t cmd = *addr++;
        ST7735_SendCmd(cmd);

        numArgs = *addr++;
        // If high bit set, delay follows args
        ms = numArgs & DELAY;
        numArgs &= ~DELAY;
        if(numArgs) {
            ST7735_SendDataMASS((uint8_t*)addr, numArgs);
            addr += numArgs;
        }

        if(ms) {
            ms = *addr++;
            if(ms == 255) ms = 500;
            HAL_Delay(ms);
        }
    }
}
//==============================================================================


//==============================================================================
// Процедура вывода цветного изображения на дисплей
//==============================================================================
void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
	
    if((x >= ST7735_Width) || (y >= ST7735_Height)){
		return;
	}
	
    if((x + w - 1) >= ST7735_Width){
		return;
	}
	
    if((y + h - 1) >= ST7735_Height){
		return;
	}
	
#if FRAME_BUFFER	// если включен буфер кадра
		for( uint16_t i = 0; i < h; i++ ){
			for( uint16_t j = 0; j < w; j++ ){
				buff_frame[( y + i ) * ST7735_Width + x + j] = *data;
				data++;
			}
		}
#else	//если попиксельный вывод	
	
	ST7735_SetWindow(x, y, x+w-1, y+h-1);
	
	ST7735_Select();
	
	ST7735_SendDataMASS((uint8_t*)data, sizeof(uint16_t)*w*h);
	
	ST7735_Unselect();
#endif
}
//==============================================================================


//==============================================================================
// Процедура аппаратного сброса дисплея (ножкой RESET)
//==============================================================================
void ST7735_HardReset(void){

	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);	
	HAL_GPIO_WritePin(RES_GPIO_Port, RES_Pin, GPIO_PIN_SET);
}
//==============================================================================


//==============================================================================
// Процедура отправки команды в дисплей
//==============================================================================
__inline static void ST7735_SendCmd(uint8_t Cmd){	
	
	//-- если захотим переделать под HAL ------------------	
	#ifdef ST7735_SPI_HAL
	
		// pin DC LOW
		HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);	
		
		HAL_SPI_Transmit(&ST7735_SPI_HAL, &Cmd, 1, HAL_MAX_DELAY);
		while(HAL_SPI_GetState(&ST7735_SPI_HAL) != HAL_SPI_STATE_READY){};
		 
		// pin DC HIGH
		 HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
		 
	#endif
	//-----------------------------------------------------
	
	//-- если захотим переделать под CMSIS  ---------------------------------------------
	#ifdef ST7735_SPI_CMSIS
		
		// pin DC LOW
		DC_GPIO_Port->BSRR = ( DC_Pin << 16 );
		
		//======  FOR F-SERIES ===========================================================
			
			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((ST7735_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			// Ждем, пока не освободится буфер передатчика
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (ST7735_SPI_CMSIS->SR & SPI_SR_TXE) == RESET ){};
		
			// заполняем буфер передатчика 1 байт информации--------------
			*((__IO uint8_t *)&ST7735_SPI_CMSIS->DR) = Cmd;
			
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (ST7735_SPI_CMSIS->SR & (SPI_SR_TXE | SPI_SR_BSY)) != SPI_SR_TXE ){};
			
			//Ждем, пока SPI освободится от предыдущей передачи
			//while((ST7735_SPI_CMSIS->SR&SPI_SR_BSY)){};	
			
			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
		//================================================================================
		
/*		//======  FOR H-SERIES ===========================================================

			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((ST7735_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			SET_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_CSTART);	// ST7735_SPI_CMSIS->CR1 |= SPI_CR1_CSTART;
			
			// ждем пока SPI будет свободна------------
			//while (!(ST7735_SPI_CMSIS->SR & SPI_SR_TXP)){};		
		
			// передаем 1 байт информации--------------
			*((__IO uint8_t *)&ST7735_SPI_CMSIS->TXDR )  = Cmd;
				
			// Ждать завершения передачи---------------
			while (!( ST7735_SPI_CMSIS -> SR & SPI_SR_TXC )){};
			
			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
*/		//================================================================================
		
		// pin DC HIGH
		DC_GPIO_Port->BSRR = DC_Pin;
		
	#endif
	//-----------------------------------------------------------------------------------
	
}
//==============================================================================


//==============================================================================
// Процедура отправки данных (параметров) в дисплей 1 BYTE
//==============================================================================
__inline static void ST7735_SendData(uint8_t Data ){

	//-- если захотим переделать под HAL ------------------
	#ifdef ST7735_SPI_HAL
	
		HAL_SPI_Transmit(&ST7735_SPI_HAL, &Data, 1, HAL_MAX_DELAY);
		while(HAL_SPI_GetState(&ST7735_SPI_HAL) != HAL_SPI_STATE_READY){};
		
	#endif
	//-----------------------------------------------------
	
	
	//-- если захотим переделать под CMSIS  ---------------------------------------------
	#ifdef ST7735_SPI_CMSIS
		
		//======  FOR F-SERIES ===========================================================
			
			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((ST7735_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			// Ждем, пока не освободится буфер передатчика
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (ST7735_SPI_CMSIS->SR & SPI_SR_TXE) == RESET ){};
		
			// передаем 1 байт информации--------------
			*((__IO uint8_t *)&ST7735_SPI_CMSIS->DR) = Data;
			
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (ST7735_SPI_CMSIS->SR & (SPI_SR_TXE | SPI_SR_BSY)) != SPI_SR_TXE ){};
			
			// Ждем, пока не освободится буфер передатчика
			//while((ST7735_SPI_CMSIS->SR&SPI_SR_BSY)){};
			
			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
		//================================================================================
		
/*		//======  FOR H-SERIES ===========================================================

			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((ST7735_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			SET_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_CSTART);	// ST7735_SPI_CMSIS->CR1 |= SPI_CR1_CSTART;
			
			// ждем пока SPI будет свободна------------
			//while (!(ST7735_SPI_CMSIS->SR & SPI_SR_TXP)){};		
		
			// передаем 1 байт информации--------------
			*((__IO uint8_t *)&ST7735_SPI_CMSIS->TXDR )  = Data;
				
			// Ждать завершения передачи---------------
			while (!( ST7735_SPI_CMSIS -> SR & SPI_SR_TXC )){};
			
			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
*/		//================================================================================
		
	#endif
	//-----------------------------------------------------------------------------------

}
//==============================================================================


//==============================================================================
// Процедура отправки данных (параметров) в дисплей MASS
//==============================================================================
__inline static void ST7735_SendDataMASS(uint8_t* buff, size_t buff_size){

	//-- если захотим переделать под HAL ------------------
	#ifdef ST7735_SPI_HAL
	
		if( buff_size <= 0xFFFF ){
			HAL_SPI_Transmit(&ST7735_SPI_HAL, buff, buff_size, HAL_MAX_DELAY);
		}
		else{
			while( buff_size > 0xFFFF ){
				HAL_SPI_Transmit(&ST7735_SPI_HAL, buff, 0xFFFF, HAL_MAX_DELAY);
				buff_size-=0xFFFF;
				buff+=0xFFFF;
			}
			HAL_SPI_Transmit(&ST7735_SPI_HAL, buff, buff_size, HAL_MAX_DELAY);
		}
		
		while(HAL_SPI_GetState(&ST7735_SPI_HAL) != HAL_SPI_STATE_READY){};
		
	#endif
	//-----------------------------------------------------
	
	
		//-- если захотим переделать под CMSIS  ---------------------------------------------
	#ifdef ST7735_SPI_CMSIS	

		//======  FOR F-SERIES ===========================================================
			
			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((ST7735_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			// Ждем, пока не освободится буфер передатчика
			// while((ST7735_SPI_CMSIS->SR&SPI_SR_BSY)){};
			
			while( buff_size ){
				
				// Ждем, пока не освободится буфер передатчика
				// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
				while( (ST7735_SPI_CMSIS->SR & SPI_SR_TXE) == RESET ){};
			
				// передаем 1 байт информации--------------
				*((__IO uint8_t *)&ST7735_SPI_CMSIS->DR) = *buff++;

				buff_size--;

			}
			
			// TXE(Transmit buffer empty) – устанавливается когда буфер передачи(регистр SPI_DR) пуст, очищается при загрузке данных
			while( (ST7735_SPI_CMSIS->SR & (SPI_SR_TXE | SPI_SR_BSY)) != SPI_SR_TXE ){};
				
			// Ждем, пока не освободится буфер передатчика
			// while((ST7735_SPI_CMSIS->SR&SPI_SR_BSY)){};
			
			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
		//================================================================================
		
/*		//======  FOR H-SERIES ===========================================================

			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 &= ~SPI_CR1_SPE;
			// Enable SPI
			if((ST7735_SPI_CMSIS->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE){
				// If disabled, I enable it
				SET_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);	// ST7735_SPI_CMSIS->CR1 |= SPI_CR1_SPE;
			}
			
			SET_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_CSTART);	// ST7735_SPI_CMSIS->CR1 |= SPI_CR1_CSTART;
			
			// ждем пока SPI будет свободна------------
			//while (!(ST7735_SPI_CMSIS->SR & SPI_SR_TXP)){};		
			
			while( buff_size ){
		
				// передаем 1 байт информации--------------
				*((__IO uint8_t *)&ST7735_SPI_CMSIS->TXDR )  = *buff++;
				
				// Ждать завершения передачи---------------
				while (!( ST7735_SPI_CMSIS -> SR & SPI_SR_TXC )){};

				buff_size--;

			}
			
			// Disable SPI	
			//CLEAR_BIT(ST7735_SPI_CMSIS->CR1, SPI_CR1_SPE);
			
*/		//================================================================================
		
	#endif
	//-----------------------------------------------------------------------------------

}
//==============================================================================


//==============================================================================
// Процедура включения режима сна
//==============================================================================
void ST7735_SleepModeEnter( void ){
	
	ST7735_Select(); 
	
	ST7735_SendCmd(ST7735_SLPIN);
	
	ST7735_Unselect();
	
	HAL_Delay(250);
}
//==============================================================================


//==============================================================================
// Процедура отключения режима сна
//==============================================================================
void ST7735_SleepModeExit( void ){
	
	ST7735_Select(); 
	
	ST7735_SendCmd(ST7735_SLPOUT);
	
	ST7735_Unselect();
	
	HAL_Delay(250);
}
//==============================================================================


//==============================================================================
// Процедура включения/отключения режима частичного заполнения экрана
//==============================================================================
void ST7735_InversionMode(uint8_t Mode){
	
  ST7735_Select(); 
	
  if (Mode){
    ST7735_SendCmd(ST7735_INVON);
  }
  else{
    ST7735_SendCmd(ST7735_INVOFF);
  }
  
  ST7735_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура закрашивает экран цветом color
//==============================================================================
void ST7735_FillScreen(uint16_t color){
	
  ST7735_FillRect(0, 0,  ST7735_Width, ST7735_Height, color);
}
//==============================================================================


//==============================================================================
// Процедура очистки экрана - закрашивает экран цветом черный
//==============================================================================
void ST7735_Clear(void){
	
  ST7735_FillRect(0, 0,  ST7735_Width, ST7735_Height, 0);
}
//==============================================================================


//==============================================================================
// Процедура заполнения прямоугольника цветом color
//==============================================================================
void ST7735_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
	
  if ((x >= ST7735_Width) || (y >= ST7735_Height)){
	  return;
  }
  
  if ((x + w) > ST7735_Width){	  
	  w = ST7735_Width - x;
  }
  
  if ((y + h) > ST7735_Height){
	  h = ST7735_Height - y;
  }
 
#if FRAME_BUFFER	// если включен буфер кадра
	for( uint16_t i = 0; i < h; i++ ){
		for( uint16_t j = 0; j < w; j++ ){
			buff_frame[( y + i ) * ST7735_Width + x + j] = ((color & 0xFF)<<8) | (color >> 8 );
		}
	}
#else	//если попиксельный вывод 
  ST7735_SetWindow(x, y, x + w - 1, y + h - 1);

	ST7735_RamWrite(&color, (h * w));
#endif	
}
//==============================================================================


//==============================================================================
// Процедура установка границ экрана для заполнения
//==============================================================================
static void ST7735_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
	
	ST7735_Select();
	
	ST7735_ColumnSet(x0, x1);
	ST7735_RowSet(y0, y1);
	
	// write to RAM
	ST7735_SendCmd(ST7735_RAMWR);
	
	ST7735_Unselect();
	
}
//==============================================================================


//==============================================================================
// Процедура записи данных в дисплей
//==============================================================================
static void ST7735_RamWrite(uint16_t *pBuff, uint32_t Len){
	
  ST7735_Select();
	
  uint8_t buff[2];
  buff[0] = *pBuff >> 8;
  buff[1] = *pBuff & 0xFF;
  
  while (Len--){
	  ST7735_SendDataMASS( buff, 2);
  } 
  
  ST7735_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура установки начального и конечного адресов колонок
//==============================================================================
static void ST7735_ColumnSet(uint16_t ColumnStart, uint16_t ColumnEnd){
	
  if (ColumnStart > ColumnEnd){
    return;
  }
  
  if (ColumnEnd > ST7735_Width){
    return;
  }
  
  ColumnStart += ST7735_X_Start;
  ColumnEnd += ST7735_X_Start;
 
  ST7735_SendCmd(ST7735_CASET);
  ST7735_SendData(ColumnStart >> 8);  
  ST7735_SendData(ColumnStart & 0xFF);  
  ST7735_SendData(ColumnEnd >> 8);  
  ST7735_SendData(ColumnEnd & 0xFF); 

}
//==============================================================================


//==============================================================================
// Процедура установки начального и конечного адресов строк
//==============================================================================
static void ST7735_RowSet(uint16_t RowStart, uint16_t RowEnd){
	
  if (RowStart > RowEnd){
    return;
  }
  
  if (RowEnd > ST7735_Height){
    return;
  }
  
  RowStart += ST7735_Y_Start;
  RowEnd += ST7735_Y_Start;
 
  ST7735_SendCmd(ST7735_RASET);
  ST7735_SendData(RowStart >> 8);  
  ST7735_SendData(RowStart & 0xFF);  
  ST7735_SendData(RowEnd >> 8);  
  ST7735_SendData(RowEnd & 0xFF);  

}
//==============================================================================


//==============================================================================
// Процедура управления подсветкой (ШИМ)
//==============================================================================
void ST7735_SetBL(uint8_t Value){
	
//  if (Value > 100)
//    Value = 100;

//	tmr2_PWM_set(ST77xx_PWM_TMR2_Chan, Value);

}
//==============================================================================


//==============================================================================
// Процедура включения/отключения питания дисплея
//==============================================================================
void ST7735_DisplayPower(uint8_t On){
	
  ST7735_Select(); 
	
  if (On){
    ST7735_SendCmd(ST7735_DISPON);
  }
  else{
    ST7735_SendCmd(ST7735_DISPOFF);
  }
  
  ST7735_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура рисования прямоугольника ( пустотелый )
//==============================================================================
void ST7735_DrawRectangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
	
  ST7735_DrawLine(x1, y1, x1, y2, color);
  ST7735_DrawLine(x2, y1, x2, y2, color);
  ST7735_DrawLine(x1, y1, x2, y1, color);
  ST7735_DrawLine(x1, y2, x2, y2, color);
	
}
//==============================================================================


//==============================================================================
// Процедура вспомогательная для --- Процедура рисования прямоугольника ( заполненый )
//==============================================================================
static void SwapInt16Values(int16_t *pValue1, int16_t *pValue2){
	
  int16_t TempValue = *pValue1;
  *pValue1 = *pValue2;
  *pValue2 = TempValue;
}
//==============================================================================


//==============================================================================
// Процедура рисования прямоугольника ( заполненый )
//==============================================================================
void ST7735_DrawRectangleFilled(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t fillcolor) {
	
  if (x1 > x2){
    SwapInt16Values(&x1, &x2);
  }
  
  if (y1 > y2){
    SwapInt16Values(&y1, &y2);
  }
  
  ST7735_FillRect(x1, y1, x2 - x1, y2 - y1, fillcolor);
}
//==============================================================================


//==============================================================================
// Процедура вспомогательная для --- Процедура рисования линии
//==============================================================================
static void ST7735_DrawLine_Slow(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
	
  const int16_t deltaX = abs(x2 - x1);
  const int16_t deltaY = abs(y2 - y1);
  const int16_t signX = x1 < x2 ? 1 : -1;
  const int16_t signY = y1 < y2 ? 1 : -1;

  int16_t error = deltaX - deltaY;

  ST7735_DrawPixel(x2, y2, color);

  while (x1 != x2 || y1 != y2) {
	  
    ST7735_DrawPixel(x1, y1, color);
    const int16_t error2 = error * 2;
 
    if (error2 > -deltaY) {
		
      error -= deltaY;
      x1 += signX;
    }
    if (error2 < deltaX){
		
      error += deltaX;
      y1 += signY;
    }
  }
}
//==============================================================================


//==============================================================================
// Процедура рисования линии
//==============================================================================
void ST7735_DrawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {

  if (x1 == x2){

    if (y1 > y2){
      ST7735_FillRect(x1, y2, 1, y1 - y2 + 1, color);
	}
    else{
      ST7735_FillRect(x1, y1, 1, y2 - y1 + 1, color);
	}
	
    return;
  }
  
  if (y1 == y2){
    
    if (x1 > x2){
      ST7735_FillRect(x2, y1, x1 - x2 + 1, 1, color);
	}
    else{
      ST7735_FillRect(x1, y1, x2 - x1 + 1, 1, color);
	}
	
    return;
  }
  
  ST7735_DrawLine_Slow(x1, y1, x2, y2, color);
}
//==============================================================================


//==============================================================================
// Процедура рисования линии с указаным углом и длиной
//==============================================================================
void ST7735_DrawLineWithAngle(int16_t x, int16_t y, uint16_t length, double angle_degrees, uint16_t color) {
    // Преобразование угла в радианы
    double angle_radians = (360.0 - angle_degrees) * PI / 180.0;

    // Вычисление конечных координат
    int16_t x2 = x + length * cos(angle_radians) + 0.5;
    int16_t y2 = y + length * sin(angle_radians) + 0.5;

    // Используем существующую функцию для рисования линии
    ST7735_DrawLine(x, y, x2, y2, color);
}
//==============================================================================


//==============================================================================
// Процедура рисования треугольника ( пустотелый )
//==============================================================================
void ST7735_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color){
	/* Draw lines */
	ST7735_DrawLine(x1, y1, x2, y2, color);
	ST7735_DrawLine(x2, y2, x3, y3, color);
	ST7735_DrawLine(x3, y3, x1, y1, color);
}
//==============================================================================


//==============================================================================
// Процедура рисования треугольника ( заполненый )
//==============================================================================
void ST7735_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color){
	
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
	curpixel = 0;
	
	deltax = abs(x2 - x1);
	deltay = abs(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} 
	else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} 
	else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} 
	else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		ST7735_DrawLine(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}
//==============================================================================


//==============================================================================
// Процедура окрашивает 1 пиксель дисплея
//==============================================================================
void ST7735_DrawPixel(int16_t x, int16_t y, uint16_t color){
	
  if ((x < 0) ||(x >= ST7735_Width) || (y < 0) || (y >= ST7735_Height)){
    return;
  }

#if FRAME_BUFFER	// если включен буфер кадра
	buff_frame[y * ST7735_Width + x] = ((color & 0xFF)<<8) | (color >> 8 );
#else	//если попиксельный вывод
  ST7735_SetWindow(x, y, x, y);
  ST7735_RamWrite(&color, 1);
 #endif
}
//==============================================================================


//==============================================================================
// Процедура рисования круг ( заполненый )
//==============================================================================
void ST7735_DrawCircleFilled(int16_t x0, int16_t y0, int16_t radius, uint16_t fillcolor) {
	
  int x = 0;
  int y = radius;
  int delta = 1 - 2 * radius;
  int error = 0;

  while (y >= 0){
	  
    ST7735_DrawLine(x0 + x, y0 - y, x0 + x, y0 + y, fillcolor);
    ST7735_DrawLine(x0 - x, y0 - y, x0 - x, y0 + y, fillcolor);
    error = 2 * (delta + y) - 1;

    if (delta < 0 && error <= 0) {
		
      ++x;
      delta += 2 * x + 1;
      continue;
    }
	
    error = 2 * (delta - x) - 1;
		
    if (delta > 0 && error > 0) {
		
      --y;
      delta += 1 - 2 * y;
      continue;
    }
	
    ++x;
    delta += 2 * (x - y);
    --y;
  }
}
//==============================================================================


//==============================================================================
// Процедура рисования круг ( пустотелый )
//==============================================================================
void ST7735_DrawCircle(int16_t x0, int16_t y0, int16_t radius, uint16_t color) {
	
  int x = 0;
  int y = radius;
  int delta = 1 - 2 * radius;
  int error = 0;

  while (y >= 0){
	  
    ST7735_DrawPixel(x0 + x, y0 + y, color);
    ST7735_DrawPixel(x0 + x, y0 - y, color);
    ST7735_DrawPixel(x0 - x, y0 + y, color);
    ST7735_DrawPixel(x0 - x, y0 - y, color);
    error = 2 * (delta + y) - 1;

    if (delta < 0 && error <= 0) {
		
      ++x;
      delta += 2 * x + 1;
      continue;
    }
	
    error = 2 * (delta - x) - 1;
		
    if (delta > 0 && error > 0) {
		
      --y;
      delta += 1 - 2 * y;
      continue;
    }
	
    ++x;
    delta += 2 * (x - y);
    --y;
  }
}
//==============================================================================


//==============================================================================
// рисуем элипс
//==============================================================================
void ST7735_DrawEllipse(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color) {
    int x, y;
    for (float angle = 0; angle <= 360; angle += 0.1) {
        x = x0 + radiusX * cos(angle * PI / 180);
        y = y0 + radiusY * sin(angle * PI / 180);
        ST7735_DrawPixel(x, y, color);
    }
}
//==============================================================================


//==============================================================================
// рисуем элипс под указаным углом наклона
//==============================================================================
void ST7735_DrawEllipseWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color) {
    float cosAngle = cos((360.0 - angle_degrees) * PI / 180);
    float sinAngle = sin((360.0 - angle_degrees) * PI / 180);

    for (int16_t t = 0; t <= 360; t++) {
        float radians = t * PI / 180.0;
        int16_t x = radiusX * cos(radians);
        int16_t y = radiusY * sin(radians);

        int16_t xTransformed = x0 + cosAngle * x - sinAngle * y;
        int16_t yTransformed = y0 + sinAngle * x + cosAngle * y;

        ST7735_DrawPixel(xTransformed, yTransformed, color);
    }
}
//==============================================================================


//==============================================================================
// рисуем элипс закрашенный
//==============================================================================
void ST7735_DrawEllipseFilled(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, uint16_t color) {
	int x, y;

	for (y = -radiusY; y <= radiusY; y++) {
			for (x = -radiusX; x <= radiusX; x++) {
					if ((x * x * radiusY * radiusY + y * y * radiusX * radiusX) <= (radiusX * radiusX * radiusY * radiusY)) {
							ST7735_DrawPixel(x0 + x, y0 + y, color);
					}
			}
	}
}
//==============================================================================


//==============================================================================
// рисуем элипс закрашенный под указаным углом наклона
//==============================================================================
void ST7735_DrawEllipseFilledWithAngle(int16_t x0, int16_t y0, int16_t radiusX, int16_t radiusY, float angle_degrees, uint16_t color) {
   float cosAngle = cos((360.0 - angle_degrees) * PI / 180.0);
    float sinAngle = sin((360.0 - angle_degrees) * PI / 180.0);

    for (int16_t y = -radiusY; y <= radiusY; y++) {
        for (int16_t x = -radiusX; x <= radiusX; x++) {
          float xTransformed = cosAngle * x - sinAngle * y;
          float yTransformed = sinAngle * x + cosAngle * y;

					if ((x * x * radiusY * radiusY + y * y * radiusX * radiusX) <= (radiusX * radiusX * radiusY * radiusY)){
             ST7735_DrawPixel(x0 + xTransformed, y0  + yTransformed, color);
          }
        }
    }
}
//==============================================================================


//==============================================================================
// Процедура рисования символа ( 1 буква или знак )
//==============================================================================
void ST7735_DrawChar(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, unsigned char ch){
	
	uint32_t i, b, j;
	
	uint32_t X = x, Y = y;
	
	uint8_t xx, yy;
	
	if( multiplier < 1 ){
		multiplier = 1;
	}

	/* Check available space in LCD */
	if ( ST7735_Width >= ( x + Font->FontWidth) || ST7735_Height >= ( y + Font->FontHeight)){

	
		/* Go through font */
		for (i = 0; i < Font->FontHeight; i++) {		
			
			if( ch < 127 ){			
				b = Font->data[(ch - 32) * Font->FontHeight + i];
			}
			
			else if( (uint8_t) ch > 191 ){
				// +96 это так как латинские символы и знаки в шрифтах занимают 96 позиций
				// и если в шрифте который содержит сперва латиницу и спец символы и потом 
				// только кирилицу то нужно добавлять 95 если шрифт 
				// содержит только кирилицу то +96 не нужно
				b = Font->data[((ch - 192) + 96) * Font->FontHeight + i];
			}
			
			else if( (uint8_t) ch == 168 ){	// 168 символ по ASCII - Ё
				// 160 эллемент ( символ Ё ) 
				b = Font->data[( 160 ) * Font->FontHeight + i];
			}
			
			else if( (uint8_t) ch == 184 ){	// 184 символ по ASCII - ё
				// 161 эллемент  ( символ ё ) 
				b = Font->data[( 161 ) * Font->FontHeight + i];
			}
			//-------------------------------------------------------------------
			
			//----  Украинская раскладка ----------------------------------------------------
			else if( (uint8_t) ch == 170 ){	// 168 символ по ASCII - Є
				// 162 эллемент ( символ Є )
				b = Font->data[( 162 ) * Font->FontHeight + i];
			}
			else if( (uint8_t) ch == 175 ){	// 184 символ по ASCII - Ї
				// 163 эллемент  ( символ Ї )
				b = Font->data[( 163 ) * Font->FontHeight + i];
			}
			else if( (uint8_t) ch == 178 ){	// 168 символ по ASCII - І
				// 164 эллемент ( символ І )
				b = Font->data[( 164 ) * Font->FontHeight + i];
			}
			else if( (uint8_t) ch == 179 ){	// 184 символ по ASCII - і
				// 165 эллемент  ( символ і )
				b = Font->data[( 165 ) * Font->FontHeight + i];
			}
			else if( (uint8_t) ch == 186 ){	// 184 символ по ASCII - є
				// 166 эллемент  ( символ є )
				b = Font->data[( 166 ) * Font->FontHeight + i];
			}
			else if( (uint8_t) ch == 191 ){	// 168 символ по ASCII - ї
				// 167 эллемент ( символ ї )
				b = Font->data[( 167 ) * Font->FontHeight + i];
			}
			//-----------------------------------------------------------------------------
			
			for (j = 0; j < Font->FontWidth; j++) {
				
				if ((b << j) & 0x8000) {
					
					for (yy = 0; yy < multiplier; yy++){
						for (xx = 0; xx < multiplier; xx++){
								ST7735_DrawPixel(X+xx, Y+yy, TextColor);
						}
					}
					
				} 
				else if( TransparentBg ){
					
					for (yy = 0; yy < multiplier; yy++){
						for (xx = 0; xx < multiplier; xx++){
								ST7735_DrawPixel(X+xx, Y+yy, BgColor);
						}
					}
					
				}
				X = X + multiplier;
			}
			X = x;
			Y = Y + multiplier;
		}
	
	}
}
//==============================================================================


//==============================================================================
// Процедура рисования строки
//==============================================================================
void ST7735_print(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, char *str){	
	
	if( multiplier < 1 ){
		multiplier = 1;
	}
	
	unsigned char buff_char;
	
	uint16_t len = strlen(str);
	
	while (len--) {
		
		//---------------------------------------------------------------------
		// проверка на кириллицу UTF-8, если латиница то пропускаем if
		// Расширенные символы ASCII Win-1251 кириллица (код символа 128-255)
		// проверяем первый байт из двух ( так как UTF-8 ето два байта )
		// если он больше либо равен 0xC0 ( первый байт в кириллеце будет равен 0xD0 либо 0xD1 именно в алфавите )
		if ( (uint8_t)*str >= 0xC0 ){	// код 0xC0 соответствует символу кириллица 'A' по ASCII Win-1251
			
			// проверяем какой именно байт первый 0xD0 либо 0xD1---------------------------------------------
			switch ((uint8_t)*str) {
				case 0xD0: {
					// увеличиваем массив так как нам нужен второй байт
					str++;
					// проверяем второй байт там сам символ
					if ((uint8_t)*str >= 0x90 && (uint8_t)*str <= 0xBF){ buff_char = (*str) + 0x30; }	// байт символов А...Я а...п  делаем здвиг на +48
					else if ((uint8_t)*str == 0x81) { buff_char = 0xA8; break; }		// байт символа Ё ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x84) { buff_char = 0xAA; break; }		// байт символа Є ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x86) { buff_char = 0xB2; break; }		// байт символа І ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x87) { buff_char = 0xAF; break; }		// байт символа Ї ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					break;
				}
				case 0xD1: {
					// увеличиваем массив так как нам нужен второй байт
					str++;
					// проверяем второй байт там сам символ
					if ((uint8_t)*str >= 0x80 && (uint8_t)*str <= 0x8F){ buff_char = (*str) + 0x70; }	// байт символов п...я	елаем здвиг на +112
					else if ((uint8_t)*str == 0x91) { buff_char = 0xB8; break; }		// байт символа ё ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x94) { buff_char = 0xBA; break; }		// байт символа є ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x96) { buff_char = 0xB3; break; }		// байт символа і ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x97) { buff_char = 0xBF; break; }		// байт символа ї ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					break;
				}
			}
			//------------------------------------------------------------------------------------------------
			
			// уменьшаем еще переменную так как израсходывали 2 байта для кириллицы
			len--;
			
			ST7735_DrawChar(x, y, TextColor, BgColor, TransparentBg, Font, multiplier, buff_char);
		}
		//---------------------------------------------------------------------
		else{
			ST7735_DrawChar(x, y, TextColor, BgColor, TransparentBg, Font, multiplier, *str);
		}
		
		x = x + (Font->FontWidth * multiplier);
		/* Increase string pointer */
		str++;
	}
}
//==============================================================================


//==============================================================================
// Процедура рисования символа с указаным углом ( 1 буква или знак )
//==============================================================================
void ST7735_DrawCharWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, unsigned char ch){
	
	uint32_t i, b, j;
	
	uint32_t X = x, Y = y;
	
	uint8_t xx, yy;
	
	// Преобразуем угол в радианы
	double radians = (360.0 - angle_degrees) * PI / 180.0;

	// Вычисляем матрицу поворота
	double cosTheta = cos(radians);
	double sinTheta = sin(radians);

	// Переменные для преобразованных координат
	double newX, newY;
	
	if( multiplier < 1 ){
		multiplier = 1;
	}

	/* Check available space in LCD */
	if (ST7735_Width >= ( x + Font->FontWidth) || ST7735_Height >= ( y + Font->FontHeight)){

			/* Go through font */
			for (i = 0; i < Font->FontHeight; i++) {		
				
				if( ch < 127 ){			
					b = Font->data[(ch - 32) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch > 191 ){
					// +96 это так как латинские символы и знаки в шрифтах занимают 96 позиций
					// и если в шрифте который содержит сперва латиницу и спец символы и потом 
					// только кирилицу то нужно добавлять 95 если шрифт 
					// содержит только кирилицу то +96 не нужно
					b = Font->data[((ch - 192) + 96) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch == 168 ){	// 168 символ по ASCII - Ё
					// 160 эллемент ( символ Ё ) 
					b = Font->data[( 160 ) * Font->FontHeight + i];
				}
				
				else if( (uint8_t) ch == 184 ){	// 184 символ по ASCII - ё
					// 161 эллемент  ( символ ё ) 
					b = Font->data[( 161 ) * Font->FontHeight + i];
				}
				//-------------------------------------------------------------------
				
				//----  Украинская раскладка ----------------------------------------------------
				else if( (uint8_t) ch == 170 ){	// 168 символ по ASCII - Є
					// 162 эллемент ( символ Є )
					b = Font->data[( 162 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 175 ){	// 184 символ по ASCII - Ї
					// 163 эллемент  ( символ Ї )
					b = Font->data[( 163 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 178 ){	// 168 символ по ASCII - І
					// 164 эллемент ( символ І )
					b = Font->data[( 164 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 179 ){	// 184 символ по ASCII - і
					// 165 эллемент  ( символ і )
					b = Font->data[( 165 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 186 ){	// 184 символ по ASCII - є
					// 166 эллемент  ( символ є )
					b = Font->data[( 166 ) * Font->FontHeight + i];
				}
				else if( (uint8_t) ch == 191 ){	// 168 символ по ASCII - ї
					// 167 эллемент ( символ ї )
					b = Font->data[( 167 ) * Font->FontHeight + i];
				}
				//-----------------------------------------------------------------------------
			
				for (j = 0; j < Font->FontWidth; j++) {
					if ((b << j) & 0x8000) {
							// Применяем поворот к координатам
							newX = cosTheta * (X - x) - sinTheta * (Y - y) + x;
							newY = sinTheta * (X - x) + cosTheta * (Y - y) + y;

							for (yy = 0; yy < multiplier; yy++) {
									for (xx = 0; xx < multiplier; xx++) {
											ST7735_DrawPixel(newX + xx, newY + yy, TextColor);
									}
							}
					} else if (TransparentBg) {
							// Аналогично для фона
							newX = cosTheta * (X - x) - sinTheta * (Y - y) + x + 0.5;
							newY = sinTheta * (X - x) + cosTheta * (Y - y) + y + 0.5;

							for (yy = 0; yy < multiplier; yy++) {
									for (xx = 0; xx < multiplier; xx++) {
											ST7735_DrawPixel(newX + xx, newY + yy, BgColor);
									}
							}
					}
					X = X + multiplier;
				}
				X = x;
				Y = Y + multiplier;
			}
	}
}
//==============================================================================


//==============================================================================
// Процедура рисования строки с указаным углом
//==============================================================================
void ST7735_printWithAngle(uint16_t x, uint16_t y, uint16_t TextColor, uint16_t BgColor, uint8_t TransparentBg, FontDef_t* Font, uint8_t multiplier, double angle_degrees, char *str){	
	
	if( multiplier < 1 ){
		multiplier = 1;
	}
	
	unsigned char buff_char;
	
	uint16_t len = strlen(str);
	
	while (len--) {
		
		//---------------------------------------------------------------------
		// проверка на кириллицу UTF-8, если латиница то пропускаем if
		// Расширенные символы ASCII Win-1251 кириллица (код символа 128-255)
		// проверяем первый байт из двух ( так как UTF-8 ето два байта )
		// если он больше либо равен 0xC0 ( первый байт в кириллеце будет равен 0xD0 либо 0xD1 именно в алфавите )
		if ( (uint8_t)*str >= 0xC0 ){	// код 0xC0 соответствует символу кириллица 'A' по ASCII Win-1251
			
			// проверяем какой именно байт первый 0xD0 либо 0xD1---------------------------------------------
			switch ((uint8_t)*str) {
				case 0xD0: {
					// увеличиваем массив так как нам нужен второй байт
					str++;
					// проверяем второй байт там сам символ
					if ((uint8_t)*str >= 0x90 && (uint8_t)*str <= 0xBF){ buff_char = (*str) + 0x30; }	// байт символов А...Я а...п  делаем здвиг на +48
					else if ((uint8_t)*str == 0x81) { buff_char = 0xA8; break; }		// байт символа Ё ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x84) { buff_char = 0xAA; break; }		// байт символа Є ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x86) { buff_char = 0xB2; break; }		// байт символа І ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x87) { buff_char = 0xAF; break; }		// байт символа Ї ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					break;
				}
				case 0xD1: {
					// увеличиваем массив так как нам нужен второй байт
					str++;
					// проверяем второй байт там сам символ
					if ((uint8_t)*str >= 0x80 && (uint8_t)*str <= 0x8F){ buff_char = (*str) + 0x70; }	// байт символов п...я	елаем здвиг на +112
					else if ((uint8_t)*str == 0x91) { buff_char = 0xB8; break; }		// байт символа ё ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x94) { buff_char = 0xBA; break; }		// байт символа є ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x96) { buff_char = 0xB3; break; }		// байт символа і ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					else if ((uint8_t)*str == 0x97) { buff_char = 0xBF; break; }		// байт символа ї ( если нужнф еще символы добавляем тут и в функции DrawChar() )
					break;
				}
			}
			//------------------------------------------------------------------------------------------------
			// уменьшаем еще переменную так как израсходывали 2 байта для кириллицы
			len--;
			
			ST7735_DrawCharWithAngle(x, y, TextColor, BgColor, TransparentBg, Font, multiplier, angle_degrees, buff_char);
		}
		//---------------------------------------------------------------------
		else{
			ST7735_DrawCharWithAngle(x, y, TextColor, BgColor, TransparentBg, Font, multiplier, angle_degrees, *str);
		}
		// Смещаем начальные координаты с каждым символом с учетом угла
    x += (Font->FontWidth * multiplier * cos((360.0 - angle_degrees) * PI / 180.0) + 0.5);
    y += (Font->FontWidth * multiplier * sin((360.0 - angle_degrees) * PI / 180.0) + 0.5);

		/* Increase string pointer */
		str++;
	}
}
//==============================================================================



//==============================================================================
// Процедура ротации ( положение ) дисплея
//==============================================================================
// па умолчанию 1 режим ( всего 1, 2, 3, 4 )
void ST7735_rotation( uint8_t rotation ){
	
	ST7735_Select();
	
	ST7735_SendCmd(ST7735_MADCTL);


	  switch (rotation) {

		case 1:	//****************************************************************************************************************
			
			//===========================================================================================
			// 1.44" 128 x 128 display, default orientation
			#ifdef ST7735_IS_128X128
				ST7735_SendData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR);
				ST7735_Width = 128;
				ST7735_Height = 128;
				ST7735_X_Start = 2;
				ST7735_Y_Start = 3;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
		
			//===========================================================================================
			// 0.96" mini 160 x 80 display (it's unlikely you want the default orientation)
			#ifdef ST7735_IS_160X80
				ST7735_SendData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR);
				ST7735_Width = 80;
				ST7735_Height = 160;
				ST7735_X_Start = 26;
				ST7735_Y_Start = 1;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
			
			//===========================================================================================
			// 1.8" 160 x 128 AliExpress/eBay  display, default orientation
			#ifdef ST7735_IS_160X128_V1
				ST7735_SendData(ST7735_MADCTL_MX | ST7735_MADCTL_MY);
				ST7735_Width = 128;
				ST7735_Height = 160;
				ST7735_X_Start = 0;
				ST7735_Y_Start = 0;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
			
			//===========================================================================================
			// 1.8" 160 x 128 WaveShare ST7735S-based display, default orientation
			#ifdef ST7735_IS_160X128_V2
				ST7735_SendData(ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB);
				ST7735_Width = 128;
				ST7735_Height = 160;
				ST7735_X_Start = 2;
				ST7735_Y_Start = 1;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
		
		 break;	//****************************************************************************************************************
		
		case 2:	//****************************************************************************************************************
			
			//===========================================================================================
			// 1.44" 128 x 128 display, rotate right
			#ifdef ST7735_IS_128X128
				ST7735_SendData(ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR);
				ST7735_Width = 128;
				ST7735_Height = 128;
				ST7735_X_Start = 3;
				ST7735_Y_Start = 2;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
		
			//===========================================================================================
			// 0.96" mini 160 x 80 display rotate right 
			#ifdef ST7735_IS_160X80
				ST7735_SendData(ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR);
				ST7735_Width = 160;
				ST7735_Height = 80;
				ST7735_X_Start = 1;
				ST7735_Y_Start = 26;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
			
			//===========================================================================================
			// 1.8" 160 x 128 AliExpress/eBay  display, rotate right
			#ifdef ST7735_IS_160X128_V1
				ST7735_SendData(ST7735_MADCTL_MY | ST7735_MADCTL_MV);
				ST7735_Width = 160;
				ST7735_Height = 128;
				ST7735_X_Start = 0;
				ST7735_Y_Start = 0;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
			
			//===========================================================================================
			// 1.8" 160 x 128 WaveShare ST7735S-based display, rotate right
			#ifdef ST7735_IS_160X128_V2
				ST7735_SendData(ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_RGB);
				ST7735_Width = 160;
				ST7735_Height = 128;
				ST7735_X_Start = 1;
				ST7735_Y_Start = 2;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
		
		 break;	//****************************************************************************************************************
		
	   case 3:	//****************************************************************************************************************
		   
			//===========================================================================================
			// 1.44" 128 x 128 display, upside down
			#ifdef ST7735_IS_128X128
				ST7735_SendData(ST7735_MADCTL_BGR);
				ST7735_Width = 128;
				ST7735_Height = 128;
				ST7735_X_Start = 2;
				ST7735_Y_Start = 1;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
	   
			//===========================================================================================
			// 0.96" mini 160 x 80 display upside down 
			#ifdef ST7735_IS_160X80
				ST7735_SendData(ST7735_MADCTL_BGR);
				ST7735_Width = 80;
				ST7735_Height = 160;
				ST7735_X_Start = 26;
				ST7735_Y_Start = 1;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
			
			//===========================================================================================
			// 1.8" 160 x 128 AliExpress/eBay  display, upside down
			#ifdef ST7735_IS_160X128_V1
				ST7735_SendData(0);
				ST7735_Width = 128;
				ST7735_Height = 160;
				ST7735_X_Start = 0;
				ST7735_Y_Start = 0;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
			
			//===========================================================================================
			// 1.8" 160 x 128 WaveShare ST7735S-based display, upside down
			#ifdef ST7735_IS_160X128_V2
				ST7735_SendData(ST7735_MADCTL_RGB);
				ST7735_Width = 128;
				ST7735_Height = 160;
				ST7735_X_Start = 2;
				ST7735_Y_Start = 1;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
	   
		 break;	//****************************************************************************************************************
	   
	   case 4:	//****************************************************************************************************************
		   
			//===========================================================================================
			// 1.44" 128 x 128 display, rotate left
			#ifdef ST7735_IS_128X128
				ST7735_SendData(ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR);
				ST7735_Width = 128;
				ST7735_Height = 128;
				ST7735_X_Start = 1;
				ST7735_Y_Start = 2;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
	   
			//===========================================================================================
			// 0.96" mini 160 x 80 display rotate left
			#ifdef ST7735_IS_160X80
				ST7735_SendData(ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR);
				ST7735_Width = 160;
				ST7735_Height = 80;
				ST7735_X_Start = 1;
				ST7735_Y_Start = 26;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
			
			//===========================================================================================
			// 1.8" 160 x 128 AliExpress/eBay  display, rotate left
			#ifdef ST7735_IS_160X128_V1
				ST7735_SendData(ST7735_MADCTL_MX | ST7735_MADCTL_MV);
				ST7735_Width = 160;
				ST7735_Height = 128;
				ST7735_X_Start = 0;
				ST7735_Y_Start = 0;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
			
			//===========================================================================================
			// 1.8" 160 x 128 WaveShare ST7735S-based display, rotate left
			#ifdef ST7735_IS_160X128_V2
				ST7735_SendData(ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_RGB);
				ST7735_Width = 160;
				ST7735_Height = 128;
				ST7735_X_Start = 1;
				ST7735_Y_Start = 2;
				ST7735_FillScreen(0);
			#endif
			//===========================================================================================
	   
		 break;	//****************************************************************************************************************
	   
	   default:	//****************************************************************************************************************
		 break;	//****************************************************************************************************************
	  }
	  
	  ST7735_Unselect();
}
//==============================================================================


//==============================================================================
// Процедура рисования иконки монохромной
//==============================================================================
void ST7735_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color){

    int16_t byteWidth = (w + 7) / 8; 	// Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for(int16_t j=0; j<h; j++, y++){
		
        for(int16_t i=0; i<w; i++){
			
            if(i & 7){
               byte <<= 1;
            }
            else{
               byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }
			
            if(byte & 0x80){
				ST7735_DrawPixel(x+i, y, color);
			}
        }
    }
}
//==============================================================================


//==============================================================================
// Процедура рисования иконки монохромной с указаным углом
//==============================================================================
void ST7735_DrawBitmapWithAngle(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color, double angle_degrees) {
    // Преобразование угла в радианы
    double angle_radians = (360.0 - angle_degrees) * PI / 180.0;

    // Вычисление матрицы поворота
    double cosTheta = cos(angle_radians);
    double sinTheta = sin(angle_radians);

    // Ширина и высота повернутого изображения
    int16_t rotatedW = round(fabs(w * cosTheta) + fabs(h * sinTheta));
    int16_t rotatedH = round(fabs(h * cosTheta) + fabs(w * sinTheta));

    // Вычисление центральных координат повернутого изображения
    int16_t centerX = x + w / 2;
    int16_t centerY = y + h / 2;

    // Проходим по каждому пикселю изображения и рисуем его повернутым
    for (int16_t j = 0; j < h; j++) {
        for (int16_t i = 0; i < w; i++) {
            // Вычисление смещения от центра
            int16_t offsetX = i - w / 2;
            int16_t offsetY = j - h / 2;

            // Применение матрицы поворота
            int16_t rotatedX = round(centerX + offsetX * cosTheta - offsetY * sinTheta);
            int16_t rotatedY = round(centerY + offsetX * sinTheta + offsetY * cosTheta);

            // Проверка находится ли пиксель в пределах экрана
            if (rotatedX >= 0 && rotatedX < ST7735_Width && rotatedY >= 0 && rotatedY < ST7735_Height) {
                // Получение цвета пикселя из исходного изображения
                uint8_t byteWidth = (w + 7) / 8;
                uint8_t byte = (*(const unsigned char*)(&bitmap[j * byteWidth + i / 8]));
                if (byte & (0x80 >> (i & 7))) {
                    // Рисование пикселя на экране
                    ST7735_DrawPixel(rotatedX, rotatedY, color);
                }
            }
        }
    }
}
//==============================================================================


//==============================================================================
// Процедура рисования прямоугольник с закругленніми краями ( заполненый )
//==============================================================================
void ST7735_DrawFillRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color) {
	
	int16_t max_radius = ((width < height) ? width : height) / 2; // 1/2 minor axis
  if (cornerRadius > max_radius){
    cornerRadius = max_radius;
	}
	
  ST7735_DrawRectangleFilled(x + cornerRadius, y, x + cornerRadius + width - 2 * cornerRadius, y + height, color);
  // draw four corners
  ST7735_DrawFillCircleHelper(x + width - cornerRadius - 1, y + cornerRadius, cornerRadius, 1, height - 2 * cornerRadius - 1, color);
  ST7735_DrawFillCircleHelper(x + cornerRadius, y + cornerRadius, cornerRadius, 2, height - 2 * cornerRadius - 1, color);
}
//==============================================================================

//==============================================================================
// Процедура рисования половины окружности ( правая или левая ) ( заполненый )
//==============================================================================
void ST7735_DrawFillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color) {

  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;
  int16_t px = x;
  int16_t py = y;

  delta++; // Avoid some +1's in the loop

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    if (x < (y + 1)) {
      if (corners & 1){
        ST7735_DrawLine(x0 + x, y0 - y, x0 + x, y0 - y - 1 + 2 * y + delta, color);
			}
      if (corners & 2){
        ST7735_DrawLine(x0 - x, y0 - y, x0 - x, y0 - y - 1 + 2 * y + delta, color);
			}
    }
    if (y != py) {
      if (corners & 1){
        ST7735_DrawLine(x0 + py, y0 - px, x0 + py, y0 - px - 1 + 2 * px + delta, color);
			}
      if (corners & 2){
        ST7735_DrawLine(x0 - py, y0 - px, x0 - py, y0 - px - 1 + 2 * px + delta, color);
			}
			py = y;
    }
    px = x;
  }
}
//==============================================================================																		

//==============================================================================
// Процедура рисования четверти окружности (закругление, дуга) ( ширина 1 пиксель)
//==============================================================================
void ST7735_DrawCircleHelper(int16_t x0, int16_t y0, int16_t radius, int8_t quadrantMask, uint16_t color)
{
    int16_t f = 1 - radius ;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * radius;
    int16_t x = 0;
    int16_t y = radius;

    while (x <= y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
				
        x++;
        ddF_x += 2;
        f += ddF_x;

        if (quadrantMask & 0x4) {
            ST7735_DrawPixel(x0 + x, y0 + y, color);
            ST7735_DrawPixel(x0 + y, y0 + x, color);;
        }
        if (quadrantMask & 0x2) {
						ST7735_DrawPixel(x0 + x, y0 - y, color);
            ST7735_DrawPixel(x0 + y, y0 - x, color);
        }
        if (quadrantMask & 0x8) {
						ST7735_DrawPixel(x0 - y, y0 + x, color);
            ST7735_DrawPixel(x0 - x, y0 + y, color);
        }
        if (quadrantMask & 0x1) {
            ST7735_DrawPixel(x0 - y, y0 - x, color);
            ST7735_DrawPixel(x0 - x, y0 - y, color);
        }
    }
}
//==============================================================================		

//==============================================================================
// Процедура рисования прямоугольник с закругленніми краями ( пустотелый )
//==============================================================================
void ST7735_DrawRoundRect(int16_t x, int16_t y, uint16_t width, uint16_t height, int16_t cornerRadius, uint16_t color) {
	
	int16_t max_radius = ((width < height) ? width : height) / 2; // 1/2 minor axis
  if (cornerRadius > max_radius){
    cornerRadius = max_radius;
	}
	
  ST7735_DrawLine(x + cornerRadius, y, x + cornerRadius + width -1 - 2 * cornerRadius, y, color);         // Top
  ST7735_DrawLine(x + cornerRadius, y + height - 1, x + cornerRadius + width - 1 - 2 * cornerRadius, y + height - 1, color); // Bottom
  ST7735_DrawLine(x, y + cornerRadius, x, y + cornerRadius + height - 1 - 2 * cornerRadius, color);         // Left
  ST7735_DrawLine(x + width - 1, y + cornerRadius, x + width - 1, y + cornerRadius + height - 1 - 2 * cornerRadius, color); // Right
	
  // draw four corners
	ST7735_DrawCircleHelper(x + cornerRadius, y + cornerRadius, cornerRadius, 1, color);
  ST7735_DrawCircleHelper(x + width - cornerRadius - 1, y + cornerRadius, cornerRadius, 2, color);
	ST7735_DrawCircleHelper(x + width - cornerRadius - 1, y + height - cornerRadius - 1, cornerRadius, 4, color);
  ST7735_DrawCircleHelper(x + cornerRadius, y + height - cornerRadius - 1, cornerRadius, 8, color);
}
//==============================================================================

//==============================================================================
// Процедура рисования линия толстая ( последний параметр толщина )
//==============================================================================
void ST7735_DrawLineThick(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint8_t thick) {
	const int16_t deltaX = abs(x2 - x1);
	const int16_t deltaY = abs(y2 - y1);
	const int16_t signX = x1 < x2 ? 1 : -1;
	const int16_t signY = y1 < y2 ? 1 : -1;

	int16_t error = deltaX - deltaY;

	if (thick > 1){
		ST7735_DrawCircleFilled(x2, y2, thick >> 1, color);
	}
	else{
		ST7735_DrawPixel(x2, y2, color);
	}

	while (x1 != x2 || y1 != y2) {
		if (thick > 1){
			ST7735_DrawCircleFilled(x1, y1, thick >> 1, color);
		}
		else{
			ST7735_DrawPixel(x1, y1, color);
		}

		const int16_t error2 = error * 2;
		if (error2 > -deltaY) {
			error -= deltaY;
			x1 += signX;
		}
		if (error2 < deltaX) {
			error += deltaX;
			y1 += signY;
		}
	}
}
//==============================================================================		


//==============================================================================
// линия толстая нужной длины и указаным углом поворота (0-360) ( последний параметр толшина )
//==============================================================================
void ST7735_DrawLineThickWithAngle(int16_t x, int16_t y, int16_t length, double angle_degrees, uint16_t color, uint8_t thick) {
    double angleRad = (360.0 - angle_degrees) * PI / 180.0;
    int16_t x2 = x + (int16_t)(cos(angleRad) * length) + 0.5;
    int16_t y2 = y + (int16_t)(sin(angleRad) * length) + 0.5;

    ST7735_DrawLineThick(x, y, x2, y2, color, thick);
}
//==============================================================================


//==============================================================================
// Процедура рисования дуга толстая ( часть круга )
//==============================================================================
void ST7735_DrawArc(int16_t x0, int16_t y0, int16_t radius, int16_t startAngle, int16_t endAngle, uint16_t color, uint8_t thick) {
	
    int16_t xLast = -1, yLast = -1;

    if (startAngle > endAngle) {
        // Рисование первой части дуги от startAngle до 360 градусов
        for (int16_t angle = startAngle; angle <= 360; angle += 2) {
            float angleRad = (float)(360 - angle) * PI / 180;
            int x = cos(angleRad) * radius + x0;
            int y = sin(angleRad) * radius + y0;

            if (xLast != -1 && yLast != -1) {
                if (thick > 1) {
                    ST7735_DrawLineThick(xLast, yLast, x, y, color, thick);
                } else {
                    ST7735_DrawLine(xLast, yLast, x, y, color);
                }
            }

            xLast = x;
            yLast = y;
        }

        // Рисование второй части дуги от 0 до endAngle
        for (int16_t angle = 0; angle <= endAngle; angle += 2) {
            float angleRad = (float)(360 - angle) * PI / 180;
            int x = cos(angleRad) * radius + x0;
            int y = sin(angleRad) * radius + y0;

            if (xLast != -1 && yLast != -1) {
                if (thick > 1) {
                    ST7735_DrawLineThick(xLast, yLast, x, y, color, thick);
                } else {
                    ST7735_DrawLine(xLast, yLast, x, y, color);
                }
            }

            xLast = x;
            yLast = y;
        }
    } else {
        // Рисование дуги от startAngle до endAngle
        for (int16_t angle = startAngle; angle <= endAngle; angle += 2) {
            float angleRad = (float)(360 - angle) * PI / 180;
            int x = cos(angleRad) * radius + x0;
            int y = sin(angleRad) * radius + y0;

            if (xLast != -1 && yLast != -1) {
                if (thick > 1) {
                    ST7735_DrawLineThick(xLast, yLast, x, y, color, thick);
                } else {
                    ST7735_DrawLine(xLast, yLast, x, y, color);
                }
            }

            xLast = x;
            yLast = y;
        }
    }
}
//==============================================================================

#if FRAME_BUFFER
	//==============================================================================
	// Процедура вывода буффера кадра на дисплей
	//==============================================================================
	void ST7735_Update(void){
		
			ST7735_SetWindow(0, 0, ST7735_Width-1, ST7735_Height-1);
		
			ST7735_Select();
		
			ST7735_SendDataMASS((uint8_t*)buff_frame, sizeof(uint16_t)*ST7735_Width*ST7735_Height);
		
			ST7735_Unselect();
	}
	//==============================================================================
	
	//==============================================================================
	// Процедура очистка только буфера кадра  ( при етом сам экран не очищаеться )
	//==============================================================================
	void ST7735_ClearFrameBuffer(void){
		memset((uint8_t*)buff_frame, 0x00, ST7735_Width*ST7735_Height*sizeof(uint16_t) );
	}
	//==============================================================================
#endif

//#########################################################################################################################
//#########################################################################################################################



/************************ (C) COPYRIGHT GKP *****END OF FILE****/
