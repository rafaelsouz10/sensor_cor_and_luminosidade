#include <stdio.h>
#include <stdlib.h> 
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bh1750_light_sensor.h"
#include "lib/config_btn.h"

// I2C
#define I2C_PORT i2c0
#define I2C_SDA  0
#define I2C_SCL  1

// Endereços
#define BH1750_ADDR 0x23
#define GY33_ADDR   0x29

// TCS34725 (GY-33) regs
#define ENABLE_REG   0x80
#define ATIME_REG    0x81
#define CONTROL_REG  0x8F
#define CDATA_REG    0x94 //  "Clear"
#define RDATA_REG    0x96 //  "Red"
#define GDATA_REG    0x98 //  "Green"
#define BDATA_REG    0x9A //  "Blue"

// Variáveis globais
volatile uint16_t rgb_r16 = 0, rgb_g16 = 0, rgb_b16 = 0, rgb_c16 = 0;
volatile uint8_t  rgb_r8 = 0, rgb_g8 = 0, rgb_b8 = 0;
volatile uint16_t lux_val = 0;
volatile char cor_nome[12] = "Indefinido";

// Util
static inline uint8_t clamp_u8(int v) {
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

const char* classify_color(uint8_t r8, uint8_t g8, uint8_t b8) {
    const int DELTA = 30;

    if (r8 < 30 && g8 < 30 && b8 < 30) return "Preto";
    if (r8 > 180 && g8 > 180 && b8 > 180) return "Branco";

    if (r8 > g8 + DELTA && r8 > b8 + DELTA) return "Vermelho";
    if (g8 > r8 + DELTA && g8 > b8 + DELTA) return "Verde";
    if (b8 > r8 + DELTA && b8 > g8 + DELTA) return "Azul";

    if (r8 > 100 && g8 > 100 && b8 < 80)  return "Amarelo";
    if (g8 > 100 && b8 > 100 && r8 < 80)  return "Ciano";
    if (r8 > 100 && b8 > 100 && g8 < 80)  return "Magenta";

    if ((abs((int)r8 - (int)g8) < 20) && (abs((int)g8 - (int)b8) < 20)) return "Cinza";

    return "Indefinido";
}

// Função para escrever um valor em um registro do GY-33
void gy33_write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, GY33_ADDR, buffer, 2, false);
}

// Função para ler um valor de um registro do GY-33
uint16_t gy33_read_register16(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, GY33_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, GY33_ADDR, buf, 2, false);
    return (uint16_t)((buf[1] << 8) | buf[0]); // Combina os bytes em um valor de 16 bits
}

void gy33_init(void) {
    gy33_write_register(ENABLE_REG, 0x03);  // Ativa o sensor (Power ON e Ativação do ADC)
    gy33_write_register(ATIME_REG, 0xF5);   // Tempo de integração (ajusta a sensibilidade) D5 => 103ms
    gy33_write_register(CONTROL_REG, 0x00); // Configuração de ganho padrão (1x) (pode ir até 60x)
}

void gy33_read_raw(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
    *c = gy33_read_register16(CDATA_REG);
    *r = gy33_read_register16(RDATA_REG);
    *g = gy33_read_register16(GDATA_REG);
    *b = gy33_read_register16(BDATA_REG);
}

int main() {
    stdio_init_all();

    // Botão B callback para modo bootsel
    init_btn_callback();

    // I2C0 a 400kHz
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa sensores
    bh1750_power_on(I2C_PORT);
    gy33_init();

    sleep_ms(200);

    while (true) {
        // BH1750
        lux_val = bh1750_read_measurement(I2C_PORT);

        // GY-33 RAW
        gy33_read_raw((uint16_t*)&rgb_r16, (uint16_t*)&rgb_g16, (uint16_t*)&rgb_b16, (uint16_t*)&rgb_c16);

        // Normalização
        if (rgb_c16 > 0) {
            rgb_r8 = clamp_u8((int)((uint32_t)rgb_r16 * 255u / rgb_c16));
            rgb_g8 = clamp_u8((int)((uint32_t)rgb_g16 * 255u / rgb_c16));
            rgb_b8 = clamp_u8((int)((uint32_t)rgb_b16 * 255u / rgb_c16));
        } else {
            rgb_r8 = rgb_g8 = rgb_b8 = 0;
        }

        // Classificação
        const char* cor_detectada = classify_color(rgb_r8, rgb_g8, rgb_b8);
        snprintf((char*)cor_nome, sizeof(cor_nome), "%s", cor_detectada);

        uint32_t ts = to_ms_since_boot(get_absolute_time());

        printf("[TS=%lums] Lux=%u lx | R=%u G=%u B=%u C=%u | RGB8 R=%u G=%u B=%u | Cor=%s\n",
               ts, lux_val,
               rgb_r16, rgb_g16, rgb_b16, rgb_c16,
               rgb_r8, rgb_g8, rgb_b8,
               cor_nome);

        sleep_ms(200);
    }
}
