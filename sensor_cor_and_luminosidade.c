#include <stdio.h>
#include <stdlib.h>   // abs()
#include <math.h>     // fmaxf, fminf
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"   // <— novo (buzzer)
#include "hardware/pwm.h"    // <— novo (buzzer)
#include "bh1750_light_sensor.h"
#include "lib/config_btn.h"

// ====================== I2C ======================
#define I2C_PORT i2c0
#define I2C_SDA  0
#define I2C_SCL  1

// Endereços
#define BH1750_ADDR 0x23
#define GY33_ADDR   0x29

// ================== TCS34725 (GY-33) ==================
#define ENABLE_REG   0x80
#define ATIME_REG    0x81
#define CONTROL_REG  0x8F
#define CDATA_REG    0x94
#define RDATA_REG    0x96
#define GDATA_REG    0x98
#define BDATA_REG    0x9A

// ================== Buzzer (PWM) ==================
#define BUZZER      21
#define LUX_LIMIT   15     // limite para alerta de baixa luz (ajuste conforme seu ambiente)
#define LUX_HYST    3      // histerese para sair do estado de alerta

// ================== Variáveis globais ==================
volatile uint16_t rgb_r16 = 0, rgb_g16 = 0, rgb_b16 = 0, rgb_c16 = 0;
volatile uint8_t  rgb_r8 = 0, rgb_g8 = 0, rgb_b8 = 0; // normalizado por (R+G+B)
volatile uint16_t lux_val = 0;
volatile char cor_nome[12] = "Indefinido";

// ================== Utils ==================
static inline uint8_t clamp_u8(int v) {
    if (v < 0)   return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

// RGB(0..1) -> HSV: h(0..360), s(0..1), v(0..1)
static void rgb_to_hsv(float r, float g, float b, float* h_deg, float* s, float* v) {
    float maxv = fmaxf(r, fmaxf(g, b));
    float minv = fminf(r, fminf(g, b));
    float delta = maxv - minv;

    *v = maxv;
    *s = (maxv > 0.0f) ? (delta / maxv) : 0.0f;

    float h;
    if (delta < 1e-6f) {
        h = 0.0f; // neutro
    } else if (maxv == r) {
        h = 60.0f * fmodf(((g - b) / delta), 6.0f);
    } else if (maxv == g) {
        h = 60.0f * (((b - r) / delta) + 2.0f);
    } else { // maxv == b
        h = 60.0f * (((r - g) / delta) + 4.0f);
    }
    if (h < 0.0f) h += 360.0f;
    *h_deg = h;
}

static const char* classify_color_hsv(uint8_t r8, uint8_t g8, uint8_t b8, uint16_t lux, uint16_t c16) {
    // Regras para rejeitar leituras ruins
    if (lux < 8 || c16 < 50) return "Indefinido"; // luz ambiente muito baixa

    float r = r8 / 255.0f, g = g8 / 255.0f, b = b8 / 255.0f;
    float h, s, v;
    rgb_to_hsv(r, g, b, &h, &s, &v);

    // Preto / Branco / Cinza por V e S
    if (v < 0.10f) return "Preto";
    if (s < 0.12f) {
        if (v > 0.85f) return "Branco";
        return "Cinza";
    }

    // Faixas de matiz (Hue) em graus
    if ((h >= 0 && h < 20) || (h >= 345 && h <= 360)) return "Vermelho";
    if (h >= 20 && h < 70)   return "Amarelo";
    if (h >= 70 && h < 170)  return "Verde";
    if (h >= 170 && h < 210) return "Ciano";
    if (h >= 210 && h < 270) return "Azul";
    if (h >= 270 && h < 345) return "Magenta";
    return "Indefinido";
}

// ================== GY-33 I2C ==================
static void gy33_write_register(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, GY33_ADDR, buffer, 2, false);
}

static uint16_t gy33_read_register16(uint8_t reg) {
    uint8_t buf[2];
    i2c_write_blocking(I2C_PORT, GY33_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, GY33_ADDR, buf, 2, false);
    return (uint16_t)((buf[1] << 8) | buf[0]);
}

static void gy33_init(void) {
    gy33_write_register(ENABLE_REG, 0x03);   // Power ON + ADC enable
    gy33_write_register(ATIME_REG, 0xD5);    // ~103 ms (use 0xF5 para ~26 ms se precisar mais rápido)
    gy33_write_register(CONTROL_REG, 0x00);  // Gain 1x (ajuste se baixo sinal)
}

static void gy33_read_raw(uint16_t* r, uint16_t* g, uint16_t* b, uint16_t* c) {
    *c = gy33_read_register16(CDATA_REG);
    *r = gy33_read_register16(RDATA_REG);
    *g = gy33_read_register16(GDATA_REG);
    *b = gy33_read_register16(BDATA_REG);
}

// ================== Buzzer: funções PWM ==================
static void init_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(slice_num, 125.0f);        // base 1 MHz
    pwm_set_wrap(slice_num, 1000);            // TOP 1000 => 1 kHz default
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0);
    pwm_set_enabled(slice_num, true);
}

static void set_buzzer_tone(uint gpio, uint freq) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    if (freq == 0) { // proteção
        pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0);
        return;
    }
    uint top = 1000000 / freq;                // 1 MHz / freq
    if (top < 10) top = 10;                   // evita TOP muito baixo
    pwm_set_wrap(slice_num, top);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), top / 2); // 50% duty
}

static void stop_buzzer(uint gpio) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0);
}

// pequenos helpers de padrão sonoro (bloqueantes, curtos)
static void beep_once(uint gpio, uint freq, uint ms) {
    set_buzzer_tone(gpio, freq);
    sleep_ms(ms);
    stop_buzzer(gpio);
}

static void beep_triple(uint gpio, uint freq, uint on_ms, uint off_ms) {
    for (int i = 0; i < 3; i++) {
        set_buzzer_tone(gpio, freq);
        sleep_ms(on_ms);
        stop_buzzer(gpio);
        if (i < 2) sleep_ms(off_ms);
    }
}

// ================== Main ==================
int main() {
    stdio_init_all();
    sleep_ms(50);

    init_btn_callback();

    // I2C0 a 400kHz
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Sensores
    bh1750_power_on(I2C_PORT);
    gy33_init();

    // Buzzer
    init_pwm(BUZZER);

    sleep_ms(200);

    // Cabeçalho CSV opcional
    printf("timestamp_ms,lux,r16,g16,b16,c16,r8,g8,b8,color\n");

    // Estados de histerese (para disparar só na transição)
    bool low_light_active = false;
    bool red_intense_active = false;

    while (true) {
        // BH1750
        lux_val = bh1750_read_measurement(I2C_PORT);

        // GY-33 (raw)
        gy33_read_raw((uint16_t*)&rgb_r16, (uint16_t*)&rgb_g16, (uint16_t*)&rgb_b16, (uint16_t*)&rgb_c16);

        // Normalização por soma (R+G+B)
        uint32_t sum = (uint32_t)rgb_r16 + (uint32_t)rgb_g16 + (uint32_t)rgb_b16;
        if (sum > 0) {
            rgb_r8 = clamp_u8((int)((rgb_r16 * 255u) / sum * 3u));
            rgb_g8 = clamp_u8((int)((rgb_g16 * 255u) / sum * 3u));
            rgb_b8 = clamp_u8((int)((rgb_b16 * 255u) / sum * 3u));
        } else {
            rgb_r8 = rgb_g8 = rgb_b8 = 0;
        }

        // Classificação HSV (nome da cor)
        const char* cname = classify_color_hsv(rgb_r8, rgb_g8, rgb_b8, lux_val, rgb_c16);
        snprintf((char*)cor_nome, sizeof(cor_nome), "%s", cname);

        // ===== Lógica de ALERTAS com buzzer =====
        // 1) Baixa luminosidade (histerese simples)
        if (!low_light_active && lux_val < LUX_LIMIT) {
            low_light_active = true;
            // bip único curto (880 Hz)
            beep_once(BUZZER, 880, 500);
        } else if (low_light_active && lux_val > (LUX_LIMIT + LUX_HYST)) {
            low_light_active = false;
        }

        // 2) Vermelho intenso: nome "Vermelho" + S e V altos
        float hr, sr, vr;
        rgb_to_hsv(rgb_r8 / 255.0f, rgb_g8 / 255.0f, rgb_b8 / 255.0f, &hr, &sr, &vr);

        bool is_red_name = (cname[0]=='V' && cname[1]=='e'); // "Vermelho" (barato e rápido)
        bool red_is_intense = is_red_name && (sr >= 0.60f) && (vr >= 0.55f);

        if (!red_intense_active && red_is_intense) {
            red_intense_active = true;
            // padrão triplo (1200 Hz) para destacar
            beep_triple(BUZZER, 1200, 90, 70);
        } else if (red_intense_active && (!is_red_name || (sr < 0.55f || vr < 0.50f))) {
            // solta histerese para sair do estado
            red_intense_active = false;
        }

        // ===== Saídas =====
        uint32_t ts = to_ms_since_boot(get_absolute_time());
        printf("[TS=%lums] Lux=%u lx | RAW R=%u G=%u B=%u C=%u | RGB8 R=%u G=%u B=%u | Cor=%s\n",
               (unsigned long)ts, lux_val,
               rgb_r16, rgb_g16, rgb_b16, rgb_c16,
               rgb_r8, rgb_g8, rgb_b8, cor_nome);

        // CSV
        printf("%lu,%u,%u,%u,%u,%u,%u,%u,%u,%s\n",
               (unsigned long)ts, lux_val,
               rgb_r16, rgb_g16, rgb_b16, rgb_c16,
               rgb_r8, rgb_g8, rgb_b8, cor_nome);

        sleep_ms(200);
    }
    return 0;
}
