#include "lcd_i2c.h"
#include "MKL25Z4.h"

const uint8_t LCD_LINE_ADDRESS[LCD_NUM_ROWS] = {0x00, 0x40};
#define LCD_SET_DD_RAM_ADDRESS 0x80

// ...
// Función para enviar un byte a través de I2C
void i2c_send_byte(uint8_t data) {
    I2C0->D = data;
    while (!(I2C0->S & I2C_S_IICIF_MASK));
    I2C0->S |= I2C_S_IICIF_MASK;
}

// Función para enviar un comando al LCD
void lcd_send_command(uint8_t command) {
    i2c_send_byte(0x00);  // Modo comando
    i2c_send_byte(command);
}

// Función para enviar un carácter al LCD
void lcd_send_character(uint8_t character) {
    i2c_send_byte(0x40);  // Modo datos
    i2c_send_byte(character);
}


// Función para inicializar el LCD
void lcd_init() {
    // Inicializar I2C
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB->PCR[0] = PORT_PCR_MUX(2) | PORT_PCR_ODE_MASK;
    PORTB->PCR[1] = PORT_PCR_MUX(2) | PORT_PCR_ODE_MASK;
    I2C0->F = 0x14;  // Configurar velocidad de 100kHz
    I2C0->C1 |= I2C_C1_IICEN_MASK;

    // Inicializar LCD
    lcd_send_command(0x33);
    lcd_send_command(0x32);
    lcd_send_command(0x28);
    lcd_send_command(LCD_COMMAND_DISPLAY_OFF);
    lcd_send_command(LCD_COMMAND_CLEAR_DISPLAY);
    lcd_send_command(LCD_COMMAND_CURSOR_OFF);
    lcd_send_command(LCD_COMMAND_BLINK_OFF);
    lcd_send_command(LCD_COMMAND_RETURN_HOME);
    lcd_send_command(LCD_COMMAND_DISPLAY_ON);
}

// Función para imprimir una cadena de texto en el LCD
void lcd_print(const char* text) {
    while (*text) {
        lcd_send_character(*text++);
    }
}

// Función para borrar el LCD
void lcd_clear() {
    lcd_send_command(LCD_COMMAND_CLEAR_DISPLAY);
}

// Función para crear un caracter
void create_char(uint8_t index, const uint8_t* data) {
    lcd_send_command(0x40 + (index * 8));  // Set CGRAM address for custom character

    // Send custom character data
    for (uint8_t i = 0; i < 8; i++) {
        lcd_send_character(data[i]);
    }
}

// Función para establecer donde se empieza a escribir
void lcd_set_cursor(uint8_t row, uint8_t column) {
    // Calcular la dirección del cursor basada en la fila y columna especificadas
    uint8_t address = LCD_LINE_ADDRESS[row] + column;

    // Enviar el comando de establecer la dirección del cursor
    lcd_send_command(LCD_SET_DD_RAM_ADDRESS | address);
    
}
// Funciones de desplazamiento

void lcd_scroll_right() {
    lcd_send_command(0x1C);  // Comando de desplazamiento hacia la derecha
}

void lcd_scroll_left() {
    lcd_send_command(0x18);  // Comando de desplazamiento hacia la izquierda
}

