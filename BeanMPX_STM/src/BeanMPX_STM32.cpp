// 
// Includes
//
#include <Arduino.h>
#include <BeanMPX.h>

TIM_HandleTypeDef htim1;

//
// Statics
//
BeanMPX *BeanMPX::active_object = 0;
uint8_t rxFlag = 0;

#ifdef _DEBUG
uint8_t debug_pin1 = (1<<PD3); // pin 3
uint8_t debug_pin2 = (1<<PD4); // pin 4
uint8_t debug_pin3 = (1<<PD5); // pin 5
#endif


//
// CRC
//
uint8_t BeanMPX::gencrc(uint8_t bytes[], uint8_t length) { // calculate CRC by byte
  uint8_t crc = 0;
  for (uint8_t i = 0; i < length; i++) {
    uint8_t b = bytes[i];
    /* XOR-in next input byte */
    uint8_t data = (b ^ crc);
    /* get current CRC value = remainder */    
	crc = pgm_read_word_near(crctable + data);
  }
  return crc;
}

uint8_t BeanMPX::checkcrc(volatile uint8_t bytes[], uint8_t length) {
  uint8_t crc = 0;
  for (uint8_t i = 1; i < length; i++) { // skips SOF 
    uint8_t b = bytes[i];
    uint8_t data = (b ^ crc);    
	crc = pgm_read_word_near(crctable + data);
  }  
  return crc;
}

//
// Receive helpers
//
void BeanMPX::storeReceivedBit(uint8_t rx_pin_val, bool no_stuffing_bit /* = false */) {
  if (no_stuffing_bit) {
    if (rx_pin_val) {
      d |= i;
    }

    i >>= 1;
    return;
  }

  if (s0 == 5 || s1 == 5) { // skip stuffing bit
    i <<= 1;
  } else {
    if (rx_pin_val) {
      d |= i;
    }
  }

  if (rx_pin_val) {
    s0++;
    s1 = 0;
  } else {
    s1++;
    s0 = 0;
  }

  i >>= 1;
}

void BeanMPX::storeReceivedByte() {  
  _receive_buffer[_buffer_index] = d;   
  d = 0;
  i = 0x80;
  _buffer_index++;
  if (_buffer_index > BUFFER_SIZE) {
	_buffer_index = 0;
	is_listining = false;
	msg_stage = 0;	  
  }    
}

void BeanMPX::storeMessage(volatile uint8_t *msg, uint8_t len, uint8_t msg_type) { // Store Message to mailbox
  if (mailbox_fill_level < MAILBOX_SIZE) {
	mailbox[mailbox_fill_level][0] = len;
	mailbox[mailbox_fill_level][1] = msg_type;
	memcpy((void*)&mailbox[mailbox_fill_level][2], (void*)msg, len);
	mailbox_fill_level++;
  }
}

void BeanMPX::getMsg(uint8_t *buffer, uint8_t buffer_len) {	
	memcpy(buffer, mailbox[0], buffer_len); // get first from mail box
	
	for (int i = 0; i < mailbox_fill_level && i < MAILBOX_SIZE; i++) { // shift all messages up
		memcpy(mailbox[i], mailbox[i+1], BUFFER_SIZE);
	}
	if (mailbox_fill_level > 0) {
		mailbox_fill_level--;
	}
}

uint8_t BeanMPX::getStatus() {	
	return mailbox_fill_level;
}


//
// The receive routine called by the interrupt handler
//
void BeanMPX::receive() {
  if (is_transmitting) {
    return;
  }
  
  if (s0 > 7 || s1 > 7) {
	_buffer_index = 0;
	is_listining = false;
	msg_stage = 0;	
	s0 = 0;
	s1 = 0;
	return;
  } 

  uint8_t rx_pin_val;
  rx_pin_val = rxFlag; //*_receivePortRegister & _receiveBitMask;
  checkRxFlag(&rxFlag);

  switch (msg_stage) {
    case 0:
      storeReceivedBit(rx_pin_val, true);

      if (!i) {
        d &= 1;
        storeReceivedByte();
        msg_stage++;
      }
      break;
    case 1:
      storeReceivedBit(rx_pin_val);

      if (!i) {
        msg_length = d & 0x0f;
        if (msg_length > 13) {
          msg_stage = 0;
          is_listining = false;

          timerStop(); //*_timerInterruptMaskRegister = 0; // disable timer interrupts
          
          _buffer_index = 0;
        }
        storeReceivedByte();
        msg_stage++;
        break;
      }
      break;
    case 2:
      storeReceivedBit(rx_pin_val);

      if (!i) {
        storeReceivedByte();
        if (msg_length == 1) {
          msg_stage++;
          break;
        }
        msg_length--;
      }
      break;
    case 3:
      storeReceivedBit(rx_pin_val);

      if (!i) {
        storeReceivedByte();
        s0 = 0;
        s1 = 0;
        msg_stage++;
        break;
      }
      break;
    case 4:
      storeReceivedBit(rx_pin_val, true);
      if (!i) {
        if (checkcrc(_receive_buffer, _buffer_index) == 0) {
          ack = 0x40;
        }
        else {
          ack = 0x80;
        }
        if (d == 0x7E) {
          for (int a = 0; a < sizeof(acknowledge_did); a++) {
            if (_receive_buffer[2] == acknowledge_did[a]) {
              is_transmit_ack = true;
              a = sizeof(acknowledge_did);
            }
          }
        }
        storeReceivedByte();
        msg_stage++;
        break;
      }
      break;
    case 5:
      storeReceivedBit(rx_pin_val, true);

      if (!i || (i & 0x1f) > 0) {
        storeReceivedByte();

        timerStop(); //*_timerInterruptMaskRegister = 0; // disable timer interrupt

        msg_stage = 0;
        is_listining = false;
		busy_timeout = millis();

        storeMessage(_receive_buffer, _buffer_index, 'R');
        _buffer_index = 0;

        break;
      }
      break;
  }
}

void BeanMPX::receiveAcknowledge() {
  if (!is_receive_ack) return;

  uint8_t rx_pin_val;
  rx_pin_val = rxFlag; //*_receivePortRegister & _receiveBitMask;
  checkRxFlag(&rxFlag);

  if (rx_pin_val) {
    rsp |= k;
  }

  k >>= 1;

  if (!k) {
    k = 0x80;
	
	_transmit_buffer[_tx_buffer_index] = rsp;
	storeMessage(_transmit_buffer, _tx_buffer_index + 1, 'T');	

    if (rsp != 0x40 && tx_retry) {
      _tx_buffer_index = 0;
      j = 1;
      tx_retry--;
    } else {
      _tx_buffer_len = 0;
      tx_retry = 2;
    }
    is_receive_ack = false;
	busy_timeout = millis();
    rsp = 0;    
  }
}


//
// The transmit routine called by the interrupt handler
//
void BeanMPX::transmit() {
  if (!(_tx_buffer_index < _tx_buffer_len)) {
    if (_tx_buffer_len > 1) {
      is_receive_ack = true;
      return;
    } else {
      if (is_transmitting) {
        is_transmitting = false;
		busy_timeout = millis();
        if (!is_listining) {
          timerStop(); //*_timerInterruptMaskRegister = 0; // disable timer interrupt
        }
        txSafetyState(); // safety
      }
      return;
    }
  }

  is_transmitting = true;

  uint8_t tx_pin_val;
  tx_pin_val = _transmit_buffer[_tx_buffer_index] & j;

  if (_tx_buffer_index < _tx_buffer_len - 1 && (tx_s0 == 5 || tx_s1 == 5)) {
    j <<= 1;
    if (tx_s0 == 5) {
	  tx_pin_val = 0; // bug fix to count stuffing bit	 
	    setTxPinLow(); //*_transmitPortRegister &= ~_transmitBitMask;
    } else {	  
	  tx_pin_val = 1; // bug fix to count stuffing bit
	    setTxPinHigh(); //*_transmitPortRegister |= _transmitBitMask; 
    }
  } else {	  
	if (tx_pin_val) {
	    setTxPinHigh(); //*_transmitPortRegister |= _transmitBitMask;
    } else {      	  
	    setTxPinLow(); //*_transmitPortRegister &= ~_transmitBitMask;
    }    
  }

  if (tx_pin_val) {
    tx_s0++;
    tx_s1 = 0;
  } else {
    tx_s1++;
    tx_s0 = 0;
  }

  j >>= 1;
  if (!j) {
    j = 0x80;
    _tx_buffer_index++;
  }
}

void BeanMPX::transmitAcknowledge() {
  if (!is_transmit_ack) {
    return;
  }
  
  uint8_t tx_pin_val;
  tx_pin_val = ack & l;
   
  
  if (tx_pin_val) {
	  setTxPinHigh(); //*_transmitPortRegister |= _transmitBitMask;
  } else {
	  setTxPinLow(); //*_transmitPortRegister &= ~_transmitBitMask;   	
  }  

  l >>= 1;
  if (!l || (l & 0x1f) > 0) {
    is_transmit_ack = false;
	busy_timeout = millis();
    l = 0x80; 
	txSafetyState(); // safety
  }
}


// 
//  sync pulse
//
void BeanMPX::syncPulse() {
  uint8_t rx_pin_val;
  rx_pin_val = rxFlag; //*_receivePortRegister & _receiveBitMask;
  checkRxFlag(&rxFlag);

  if (!is_listining && rx_pin_val && !is_transmitting) {
    msg_stage = 0;
    i = 2;
    is_listining = true;
	
	  timerStart(); //*_timerInterruptMaskRegister |= _timerInterruptMask; // enable timer interrupts	
  }
    
  if (rx_pin_val != _rx_prev_val) {
	_rx_prev_val = rx_pin_val;
	setTimerCounter();  
  }  
}



//
//	tx safety state
//

void BeanMPX::txSafetyState() {
  setTxPinLow();  //*_transmitPortRegister &= ~_transmitBitMask; // set tx pin LOW   
}


//
// Static handle functions
//
// handle timer1

inline void BeanMPX::handle_rx() {

  active_object->receive();	
  
}

inline void BeanMPX::handle_rx_ack() {
  
  active_object->receiveAcknowledge();	
  
}

inline void BeanMPX::handle_tx() {

  active_object->transmit();	

}

inline void BeanMPX::handle_tx_ack() {  

  active_object->transmitAcknowledge();	
    
}


// handle pin interrupt

inline void BeanMPX::handle_sync() {	

	active_object->syncPulse();			

}


BeanMPX::BeanMPX() {
  // set acknowledge DIDs?
}

//
// Public methods
//

void BeanMPX::begin(uint8_t rx, uint8_t tx, bool use_timer2) {	  

  HAL_Init();

  MX_TIM1_Init();   
  MX_GPIO_Init();
  txSafetyState();  
	active_object = this; 
}

void BeanMPX::ackMsg(const uint8_t *destintion_id, uint8_t len) {	
	memcpy(acknowledge_did, destintion_id, len);	
}


// Read from buffer

uint8_t BeanMPX::available() 
{
	return mailbox_fill_level;
}


void BeanMPX::sendMsg(const uint8_t *data, uint16_t datalen) 
{  
  uint8_t frame[datalen + 4];
  tx_s0 = 0;
  tx_s1 = 0;

  frame[0] = 0x20 + datalen; // PRI ML
  for (uint8_t i = 0; i < datalen; i++) {
    frame[i + 1] = data[i]; // DID, MID, D0, D1, ...
  }

  frame[datalen + 1] = gencrc(frame, datalen + 1); // CRC
  frame[datalen + 2] = 0x7e; // EOF

  for (uint8_t i = sizeof(frame)-1; i > 0; i--) {
    frame[i] = frame[i - 1];
  }

  frame[0] = 1;

  memcpy((void*)_transmit_buffer, frame, sizeof(frame));
  _tx_buffer_len = sizeof(frame);
  _tx_buffer_index = 0;

  j = 1;
  
  setTimerCounter(); // set timer counter sync 	
  timerStart(); //*_timerInterruptMaskRegister |= _timerInterruptMask; // enable timer interrupt
}

//
// Interrupt handling
//

// ISR(PCINT0_vect) {
//   #ifdef _DEBUG
//   PORTD ^= debug_pin3;    
//   #endif
//   BeanMPX::handle_sync();
// }

//
// Enable Pin Interrupt
//

// void BeanMPX::pciSetup(byte pin) {
//   // *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); // enable pin   //
//   // PCIFR |= bit(digitalPinToPCICRbit(pin)); // clear any outstanding interrupt //
//   // PCICR |= bit(digitalPinToPCICRbit(pin)); // enable interrupt for the group  //
// }

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  rxFlag = 1;
  BeanMPX::handle_sync();
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

// 
//  set timer counter
//

void BeanMPX::setTimerCounter() {

  // Change the ARR value on-the-fly
  TIM1->CR1 &= ~TIM_CR1_CEN; // Disable Timer 1
  TIM1->ARR = 104;          // Set the new auto-reload value
  TIM1->SR &= ~TIM_SR_UIF;   // Clear the update event flag
  TIM1->CR1 |= TIM_CR1_CEN;  // Enable Timer 1 again

}

void checkRxFlag(uint8_t *flag)
{
  if(*flag)
  {
    *flag = 0;
  }
}

void setTxPinHigh(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

void setTxPinLow(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

void timerStop(void)
{
  HAL_TIM_Base_Stop_IT(&htim1);
}

void timerStart(void)
{
  HAL_TIM_Base_Start_IT(&htim1);
}

/**
  * @brief This function handles TIM1 capture compare interrupt.
  */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */
  if(htim1.Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
  	// HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    //OCR1A
    BeanMPX::handle_rx();
    BeanMPX::handle_rx_ack();

    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1); // Clear the interrupt flag for Channel 1

  }

  if(htim1.Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
  	// HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    //OCR1B
    BeanMPX::handle_tx();
    BeanMPX::handle_tx_ack();
  
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC2); // Clear the interrupt flag for Channel 1

  }

  /* USER CODE END TIM1_CC_IRQn 0 */
  // HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */

  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 52;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

