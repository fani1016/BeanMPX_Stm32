// 
// Includes
//
#include <Arduino.h>
#include <BeanMPX.h>

TIM_HandleTypeDef htim2;

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
  TIM2_Init();
  GPIO_Init();
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

// External Interrupt ISR Handler CallBackFun
void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR8) {
	rxFlag = 1;
	BeanMPX::handle_sync();
	EXTI->PR |= EXTI_PR_PR8; // Clear the interrupt flag
    }
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void GPIO_Init(void) {
    // Enable GPIOB clock
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // Configure PB8 as input with pull-up
    GPIOB->CRH &= ~GPIO_CRH_MODE8; // Clear mode bits
    GPIOB->CRH |= GPIO_CRH_CNF8_1; // Set input with pull-up mode
    GPIOB->ODR |= GPIO_ODR_ODR8;   // Enable internal pull-up

    // Configure PB9 as general-purpose output push-pull
    GPIOB->CRH &= ~GPIO_CRH_MODE9; // Clear mode bits
    GPIOB->CRH |= GPIO_CRH_MODE9_0; // Set output mode
    GPIOB->CRH &= ~GPIO_CRH_CNF9;  // Set general-purpose output push-pull mode

    // Enable AFIO clock
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    // Map PB8 to EXTI8
    AFIO->EXTICR[2] |= AFIO_EXTICR3_EXTI8_PB;

    // Enable interrupt on EXTI8
    EXTI->IMR |= EXTI_IMR_MR8;
    EXTI->FTSR |= EXTI_FTSR_TR8;

    // Enable EXTI9_5 interrupt in NVIC
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

// 
//  set timer counter
//
void BeanMPX::setTimerCounter() {
  TIM2->CNT = 3734;
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
  TIM2->DIER &= ~TIM_DIER_CC1IE;
  TIM2->DIER &= ~TIM_DIER_CC2IE;
  // TIM2->CR1 &= ~TIM_CR1_CEN;    // Stop the timer
}

void timerStart(void)
{
  TIM2->DIER |= (TIM_DIER_CC1IE | TIM_DIER_CC2IE);
  // TIM2->CR1 |= TIM_CR1_CEN;    // Start the timer
}

/**
  * @brief This function handles TIM2 output compare interrupt.
  */
void TIM2_IRQHandler(void)
{
  if (TIM2->SR & TIM_SR_CC1IF)
  {
    BeanMPX::handle_rx();
    BeanMPX::handle_rx_ack();
    TIM2->SR &= ~TIM_SR_CC1IF;
  }

  if (TIM2->SR & TIM_SR_CC2IF)
  {
    BeanMPX::handle_tx();
    BeanMPX::handle_tx_ack();
    TIM2->SR &= ~TIM_SR_CC2IF;
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void TIM2_Init(void)
{
    // Enable Timer 2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure Timer 2 for CCR1 and CCR2 compare mode
    TIM2->CCMR1 = (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                   TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
    
    // Set the prescaler value for a 72MHz clock
    TIM2->PSC = 0;

    // Set the auto-reload value for the maximum timer period
    TIM2->ARR = 0xFFFF;

    // Set the compare value for Channel 1 (OCR1A) for the desired time interval
    TIM2->CCR1 = 3734; // Assuming a desired time interval of 51.875 microseconds

    // Set the compare value for Channel 2 (OCR1B) for the desired time interval
    TIM2->CCR2 = 7312; // Assuming a desired time interval of 101.563 microseconds

    // Enable compare interrupts for CCR1 and CCR2
    TIM2->DIER |= (TIM_DIER_CC1IE | TIM_DIER_CC2IE);

    // Enable Timer 2 interrupt
    NVIC_EnableIRQ(TIM2_IRQn);

    // Start Timer 2
    TIM2->CR1 |= TIM_CR1_CEN;
}


