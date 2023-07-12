BeanMPX - Arduino Core for STM32
================================

Bean muliplex is a communication protocol developed by Toyota for body control system in passenger cars. Its a single wire 12 volt serial communication. As bean multiplex isn't very common i created bean multiplex interpreter using an Arduino.

> Why do this?

The ecu is responsible to feed information to the combination meter, if say the ecu was replaced for an aftermarket unit the temperature gauge will not work.

#### Demo: https://youtu.be/FXD5fauDTvY
#### Bridge: https://youtu.be/dopTZ3F5d6s


This is same library created by @fiztech-code, which is modified for Arduino Core for STM32.

## Code

To begin work with BeanMPX, download the ZIP file and include it into the Arduino IDE. 
Next open the example sketch provided. 


### Init
There is optional parameter which can be set, its the Acknowledge Messages.
BeanMPX can send acknowledge response to messages with a particular destination id. With bean.begin() function you can specify which pins to use for rx/tx also you can also set to use timer1(16bit) or timer2(8bit)

```C++
  bean.ackMsg((const uint8_t[]) {0xFE}, 1); // Acknowledge Messages, Length
  bean.begin(PB8, PB9, 0); // rx pin, tx pin, timer (0 or 1);
```

### Receive
Receive message: memcpy into a buffer, 1st item is length of entire message, 2nd item is message type: R/T (Receive/Transmit), followed by entire bean frame. 

### Transmit
Sending message, requires Destination id, Message id, and Data. Priority, Message length, CRC, and End of Message bytes will be generated.

```C++
  bean.sendMsg(engTemp, sizeof(engTemp));
```




