#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>
#include "MKL25Z4.h"

// Direcciones I2C del módulo LCD
#define LCD_ADDRESS 0x27
// Definición manual de PORT_PCR_ODE_MASK
#define PORT_PCR_ODE_MASK (1u << 5)
// Comandos LCD
#define LCD_COMMAND_CLEAR_DISPLAY 0x01
#define LCD_COMMAND_RETURN_HOME 0x02
#define LCD_COMMAND_DISPLAY_ON 0x0C
#define LCD_COMMAND_DISPLAY_OFF 0x08
#define LCD_COMMAND_CURSOR_ON 0x0E
#define LCD_COMMAND_CURSOR_OFF 0x0C
#define LCD_COMMAND_BLINK_ON 0x0F
#define LCD_COMMAND_BLINK_OFF 0x0C
// Definir set cursor
#define LCD_NUM_ROWS 4
#define LCD_NUM_COLUMNS 20

extern const uint8_t LCD_LINE_ADDRESS[LCD_NUM_ROWS];
#define LCD_SET_DD_RAM_ADDRESS 0x80

void lcd_init(void);
void lcd_send_command(unsigned char command);
void lcd_send_data(unsigned char data);

// Función para enviar un carácter al LCD
void lcd_send_character(uint8_t character);


// Función para imprimir una cadena de texto en el LCD
void lcd_print(const char* text);

// Función para borrar el LCD
void lcd_clear();

// Función para crear caracter
void create_char(uint8_t index, const uint8_t* data);

// Función para establecer la posición del cursor en el LCD
void lcd_set_cursor(uint8_t row, uint8_t column);

// Función para desplazar todo el texto hacia la derecha en el LCD
void lcd_scroll_right();

// Función para desplazar todo el texto hacia la izquierda en el LCD
void lcd_scroll_left();


#endif  // LCD_I2C_H
