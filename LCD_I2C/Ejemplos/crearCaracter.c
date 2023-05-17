#include "MKL25Z4.h"
#include "LCD_I2C.h"
#include "Delay.h"

    // Definir un carácter personalizado "C"
    uint8_t customCharC[] = {
        0x0E,
        0x11,
        0x11,
        0x11,
        0x0A,
        0x04,
        0x00,
        0x00
    };


// Función principal
int main() {
    // Configurar y inicializar el LCD
    lcd_init();


    // Crear el carácter personalizado en el índice 0
    create_custom_character(0, customCharC);

    // Mostrar el carácter personalizado en la posición 0,0 del LCD
    lcd_set_cursor(0, 0);
    lcd_send_character(0);

    while (1) {
        // Bucle infinito
    }
}
