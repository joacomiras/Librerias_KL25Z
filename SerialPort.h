#ifndef SERIALPORT_H
#define SERIALPORT_H

void SerialInit();
void Serial_SendChar(char c);
void Serial_SendString(const char *str);
void Serial_SendInt (int num);

#endif