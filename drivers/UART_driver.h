/*
 * UART_driver.h
 *
 * Created: 5/12/2016 9:58:46 PM
 *  Author: Cookie Factory
 */ 


#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

void UART_init(void);

// Buffer for communication
struct Buffer
{
	uint8_t bufferSize;
	uint8_t counter;
	char bufferData[6];
};

void bufferWrite(struct Buffer *buffer, uint8_t writeData);
void bufferRead(struct Buffer *buffer, uint8_t readData);

void messageEncode(char message[]);
void messageDecode(char message[]);
void messageValidate(char message_param[], uint8_t message_value_int);

#endif /* UART_DRIVER_H_ */