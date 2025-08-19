#ifndef CONFIG_BTN_H
#define CONFIG_BTN_H

// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6

// --- Handler único de interrupções dos botões ---
void gpio_irq_handler(uint gpio, uint32_t events) {
    reset_usb_boot(0, 0);
}

void init_btn_callback(){
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    // Fim do trecho para modo BOOTSEL com botão B
}

#endif