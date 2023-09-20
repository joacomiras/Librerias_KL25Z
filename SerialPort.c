#include "MKL25Z4.h"
#include "SerialPort.h"

void SerialInit() {
    // Habilitar el reloj para UART0 en el Sistema Clock Gating Control Register 4 (SIM_SCGC4).
    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;

    // Configurar el clock source para UART0 (seleccionar el clock MCGFLLCLK).
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);

    // Habilitar el clock para el puerto de los pines UART0 en el Puerto Control Register (SIM_SCGC5).
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

    // Configurar los pines de transmisión (PTA2) y recepción (PTA1) como UART0.
    PORTA->PCR[1] = PORT_PCR_MUX(2); // PTA1 como RX
    PORTA->PCR[2] = PORT_PCR_MUX(2); // PTA2 como TX

    // Configurar la velocidad de baudios.
    UART0->BDH = 0;
    UART0->BDL = 26; // Baud rate = 9600 (para 48MHz de MCGFLLCLK)

    // Habilitar la transmisión y recepción.
    UART0->C2 |= UART_C2_TE_MASK | UART_C2_RE_MASK;
}

void Serial_SendChar(char c) {
    while (!(UART0->S1 & UART_S1_TDRE_MASK)); // Esperar hasta que el registro de datos esté vacío.
    UART0->D = c; // Enviar el carácter.
}

void Serial_SendString(const char *str) {
    while (*str) {
        Serial_SendChar(*str);
        str++;
    }
}
void Serial_SendInt (int num) {
    char buffer[16]; // Un buffer lo suficientemente grande para almacenar el número como una cadena.
    sprintf(buffer, "%d", num); // Convierte el número a una cadena.
    Serial_SendString(buffer); // Envía la cadena por el puerto serie.
}




