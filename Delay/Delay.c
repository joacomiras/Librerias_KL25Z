#include "MKL25Z4.h"

// Función para realizar un retardo en milisegundos
void delay_ms(uint32_t milliseconds) {
    // Calcular el número de ciclos necesarios para el retardo
    uint32_t cycles = milliseconds * (SystemCoreClock / 1000);

    // Realizar el retardo
    for (volatile uint32_t i = 0; i < cycles; i++) {
        __NOP();
    }
}

// Función para realizar un retardo en microsegundos
void delay_us(uint32_t microseconds) {
    // Calcular el número de ciclos necesarios para el retardo
    uint32_t cycles = microseconds * (SystemCoreClock / 1000000);

    // Realizar el retardo
    for (volatile uint32_t i = 0; i < cycles; i++) {
        __NOP();
    }
}
