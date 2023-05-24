#include "lcd_i2c.h"
#include "MKL25Z4.h"

const uint8_t LCD_LINE_ADDRESS[LCD_NUM_ROWS] = {0x00, 0x40};
#define LCD_SET_DD_RAM_ADDRESS 0x80

#define LCD_ADDR 0x27

/* Función para permitir tiempos de espera necesarios en la comunicacion I2C */

void i2c_delay(void) {
    volatile int i;
    for (i = 0; i < 50; i++) {
        __asm("nop");
    }
}

/* Se encarga de iniciar la comunicación I2C */

void i2c_start(void) {
    I2C0_C1 |= I2C_C1_MST_MASK | I2C_C1_TX_MASK;
    i2c_delay();
}

/* Se encarga de detener la comunicación I2C */

void i2c_stop(void) {
    I2C0_C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK);
    i2c_delay();
}

/* Se utiliza para enviar un byte a traves del bus I2C */

void i2c_write_byte(unsigned char byte) {
// Se carga el byte en el registro de datos I2C0_D
    I2C0_D = byte;
// Se espera a que se complete la transferencia
    while (!(I2C0_S & I2C_S_IICIF_MASK));
    i2c_delay();
// Se borra la transferencia
    I2C0_S |= I2C_S_IICIF_MASK;
}

/* Se envia un nibble */

void lcd_send_nibble(unsigned char nibble) {
    i2c_write_byte((nibble & 0xF0) | (1 << 3) | (1 << 2));
    i2c_write_byte((nibble & 0xF0) | (1 << 3));
    i2c_write_byte((nibble & 0xF0) | (1 << 2));
}

/* Se envia un byte */

void lcd_send_byte(unsigned char address, unsigned char byte) {
    lcd_send_nibble(address);
    lcd_send_nibble(byte);
}

/* Se envían comandos */

void lcd_command(unsigned char command) {
    lcd_send_byte(0x00, command);
}

/* Se envían datos */

void lcd_data(unsigned char data) {
    lcd_send_byte(0x40, data);
}

/* Se inicializa la comunicación I2C */

void lcd_init(void) {
// Se activa el reloj I2C y se configuran los pines para el bus I2C
    SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB_PCR0 = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTB_PCR1 = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    I2C0_C1 = 0;
    I2C0_C2 = 0;
// Frecuencia de 100 kHz para el bus I2C
    I2C0_F = 0x14;
    I2C0_C1 |= I2C_C1_IICEN_MASK;
    lcd_command(0x33);
    lcd_command(0x32);
    lcd_command(0x28);
    lcd_command(0x0C);
    lcd_command(0x06);
    lcd_command(0x01);
}

/* */

void lcd_send_command(unsigned char command) {
    lcd_command(command);
}

/* */

void lcd_send_data(unsigned char data) {
    lcd_data(data);
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
