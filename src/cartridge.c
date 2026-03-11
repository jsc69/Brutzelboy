#include "cartridge.h"

#include <string.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <esp_timer.h>

static const char *TAG = "CARTRIDGE";

// ── Interner Zustand ─────────────────────────────────────────────
static cartridge_config_t s_cfg;
static bool s_initialized = false;

// ── Hilfsfunktionen ──────────────────────────────────────────────

static int aw9523_write(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    esp_err_t err = i2c_master_write_to_device(
        s_cfg.i2c_port, addr, buf, 2, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C write 0x%02X reg=0x%02X: %s", addr, reg, esp_err_to_name(err));
        return CARTRIDGE_ERR_I2C;
    }
    return CARTRIDGE_OK;
}

static int aw9523_read(uint8_t addr, uint8_t reg, uint8_t *value) {
    esp_err_t err = i2c_master_write_read_device(
        s_cfg.i2c_port, addr, &reg, 1, value, 1, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C read 0x%02X reg=0x%02X: %s", addr, reg, esp_err_to_name(err));
        return CARTRIDGE_ERR_I2C;
    }
    return CARTRIDGE_OK;
}

// Wartet bis gpio_num den Zustand expected_level annimmt.
// Timeout in µs. Gibt CARTRIDGE_OK oder CARTRIDGE_ERR_TIMEOUT zurück.
static int wait_for_pin(int gpio_num, int expected_level, uint32_t timeout_us) {
    int64_t deadline = esp_timer_get_time() + timeout_us;
    while (gpio_get_level((gpio_num_t)gpio_num) != expected_level) {
        if (esp_timer_get_time() >= deadline) {
            return CARTRIDGE_ERR_TIMEOUT;
        }
        // Kurze Pause um den Bus nicht zu blockieren
        esp_rom_delay_us(10);
    }
    return CARTRIDGE_OK;
}

// ── Initialisierung ──────────────────────────────────────────────

static int aw9523_init_device(uint8_t addr, bool is_u2) {
    // Soft-Reset
    int ret = aw9523_write(addr, AW9523_REG_RESET, 0x00);
    if (ret != CARTRIDGE_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    if (is_u2) {
        // U2: P0 = Datenbus (alle Inputs), P1 = Steuerleitungen
        // Power-Enable-Bit (P1_6) als Ausgang, Rest Input
        uint8_t dir_p1 = 0xFF;  // alle Input
        if (s_cfg.power_enable_port == 1) {
            dir_p1 &= ~(1 << s_cfg.power_enable_bit);  // als Ausgang
        }
        ret  = aw9523_write(addr, AW9523_REG_DIR_P0, 0xFF);   // P0 alle Input
        ret |= aw9523_write(addr, AW9523_REG_DIR_P1, dir_p1);

        // Power-Enable initial HIGH → ATmega einschalten
        if (s_cfg.power_enable_port == 1) {
            uint8_t out_p1 = s_cfg.power_enable_active_high
                             ? (1 << s_cfg.power_enable_bit)
                             : 0x00;
            ret |= aw9523_write(addr, AW9523_REG_OUTPUT_P1, out_p1);
        }
    } else {
        // U1: P0 = Adressbus (alle Inputs), P1 = Steuerleitungen (alle Inputs)
        ret  = aw9523_write(addr, AW9523_REG_DIR_P0, 0xFF);
        ret |= aw9523_write(addr, AW9523_REG_DIR_P1, 0xFF);
    }

    // Push-Pull für alle Pins
    ret |= aw9523_write(addr, AW9523_REG_MODE_P0, 0xFF);
    ret |= aw9523_write(addr, AW9523_REG_MODE_P1, 0xFF);

    return ret;
}

int cartridge_init(const cartridge_config_t *cfg) {
    if (!cfg) return CARTRIDGE_ERR_INIT;
    memcpy(&s_cfg, cfg, sizeof(cartridge_config_t));

    ESP_LOGI(TAG, "Init U1=0x%02X U2=0x%02X", s_cfg.addr_u1, s_cfg.addr_u2);

    // AW9523B initialisieren
    int ret = aw9523_init_device(s_cfg.addr_u1, false);
    if (ret != CARTRIDGE_OK) {
        ESP_LOGE(TAG, "U1 init failed");
        return ret;
    }
    ret = aw9523_init_device(s_cfg.addr_u2, true);
    if (ret != CARTRIDGE_OK) {
        ESP_LOGE(TAG, "U2 init failed");
        return ret;
    }

    // Handshake-Pins konfigurieren (falls gewünscht)
    if (s_cfg.handshake.wr_pin >= 0) {
        gpio_set_direction((gpio_num_t)s_cfg.handshake.wr_pin, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)s_cfg.handshake.wr_pin, 0);
        ESP_LOGI(TAG, "Handshake WR: GPIO%d", s_cfg.handshake.wr_pin);
    }
    if (s_cfg.handshake.rd_pin >= 0) {
        gpio_set_direction((gpio_num_t)s_cfg.handshake.rd_pin, GPIO_MODE_INPUT);
        ESP_LOGI(TAG, "Handshake RD: GPIO%d", s_cfg.handshake.rd_pin);
    }
    if (s_cfg.handshake.cs_pin >= 0) {
        gpio_set_direction((gpio_num_t)s_cfg.handshake.cs_pin, GPIO_MODE_INPUT);
        ESP_LOGI(TAG, "Handshake CS: GPIO%d", s_cfg.handshake.cs_pin);
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Cartridge ready");
    return CARTRIDGE_OK;
}

void cartridge_deinit(void) {
    if (!s_initialized) return;
    if (s_cfg.handshake.wr_pin >= 0)
        gpio_reset_pin((gpio_num_t)s_cfg.handshake.wr_pin);
    if (s_cfg.handshake.rd_pin >= 0)
        gpio_reset_pin((gpio_num_t)s_cfg.handshake.rd_pin);
    if (s_cfg.handshake.cs_pin >= 0)
        gpio_reset_pin((gpio_num_t)s_cfg.handshake.cs_pin);
    s_initialized = false;
}

// ── Direkter Registerzugriff ─────────────────────────────────────

int cartridge_read_reg(uint8_t addr, uint8_t reg, uint8_t *value) {
    return aw9523_read(addr, reg, value);
}

int cartridge_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    return aw9523_write(addr, reg, value);
}

// ── Lesen ────────────────────────────────────────────────────────

int cartridge_read(uint8_t bank, uint32_t *value) {
    if (!s_initialized || !value) return CARTRIDGE_ERR_INIT;

    bool use_handshake = (s_cfg.handshake.wr_pin >= 0)
                      && (s_cfg.handshake.rd_pin >= 0);

    // Handshake: WR HIGH = "ESP32 bereit zum Lesen"
    if (use_handshake) {
        gpio_set_level((gpio_num_t)s_cfg.handshake.wr_pin, 1);
        // Warten auf RD HIGH = "Daten gültig"
        int ret = wait_for_pin(s_cfg.handshake.rd_pin, 1,
                               CARTRIDGE_HANDSHAKE_TIMEOUT_US);
        if (ret != CARTRIDGE_OK) {
            gpio_set_level((gpio_num_t)s_cfg.handshake.wr_pin, 0);
            ESP_LOGW(TAG, "read: RD-ACK timeout (bank=%d)", bank);
            return CARTRIDGE_ERR_TIMEOUT;
        }
    }

    int ret = CARTRIDGE_OK;
    if (bank == 0) {
        // Adressbus: U1.P0 (8 Bit Low) — für 16-Bit-Erweiterung
        // wäre U1.P1 das High-Byte, hier als zukünftige Option vorbereitet
        uint8_t lo = 0;
        ret = aw9523_read(s_cfg.addr_u1, AW9523_REG_INPUT_P0, &lo);
        *value = lo;
    } else {
        // Datenbus: U2.P0 (8 Bit)
        uint8_t data = 0;
        ret = aw9523_read(s_cfg.addr_u2, AW9523_REG_INPUT_P0, &data);
        *value = data;
    }

    // Handshake: WR LOW = "gelesen, fertig"
    if (use_handshake) {
        gpio_set_level((gpio_num_t)s_cfg.handshake.wr_pin, 0);
        // Warten bis Sender RD wieder LOW zieht
        wait_for_pin(s_cfg.handshake.rd_pin, 0,
                     CARTRIDGE_HANDSHAKE_TIMEOUT_US);  // Timeout hier ignorieren
    }

    return ret;
}

// ── Schreiben ────────────────────────────────────────────────────

int cartridge_write(uint8_t bank, uint32_t value) {
    if (!s_initialized) return CARTRIDGE_ERR_INIT;

    bool use_handshake = (s_cfg.handshake.wr_pin >= 0)
                      && (s_cfg.handshake.rd_pin >= 0);

    int ret = CARTRIDGE_OK;
    if (bank == 0) {
        // Adressbus: U1.P0 als Ausgang schalten und schreiben
        ret  = aw9523_write(s_cfg.addr_u1, AW9523_REG_DIR_P0,    0x00);
        ret |= aw9523_write(s_cfg.addr_u1, AW9523_REG_OUTPUT_P0, (uint8_t)(value & 0xFF));
    } else {
        // Datenbus: U2.P0 als Ausgang schalten und schreiben
        ret  = aw9523_write(s_cfg.addr_u2, AW9523_REG_DIR_P0,    0x00);
        ret |= aw9523_write(s_cfg.addr_u2, AW9523_REG_OUTPUT_P0, (uint8_t)(value & 0xFF));
    }
    if (ret != CARTRIDGE_OK) return ret;

    // Handshake: WR HIGH = "Daten bereit"
    if (use_handshake) {
        gpio_set_level((gpio_num_t)s_cfg.handshake.wr_pin, 1);
        // Warten auf RD HIGH = "Empfänger hat gelesen"
        ret = wait_for_pin(s_cfg.handshake.rd_pin, 1,
                           CARTRIDGE_HANDSHAKE_TIMEOUT_US);
        if (ret != CARTRIDGE_OK) {
            gpio_set_level((gpio_num_t)s_cfg.handshake.wr_pin, 0);
            ESP_LOGW(TAG, "write: RD-ACK timeout (bank=%d)", bank);
            // Bus wieder als Input freigeben
        } else {
            gpio_set_level((gpio_num_t)s_cfg.handshake.wr_pin, 0);
            wait_for_pin(s_cfg.handshake.rd_pin, 0,
                         CARTRIDGE_HANDSHAKE_TIMEOUT_US);
        }
    }

    // Bus wieder als Input freigeben (nicht-destruktiv für andere Leser)
    if (bank == 0) {
        aw9523_write(s_cfg.addr_u1, AW9523_REG_DIR_P0, 0xFF);
    } else {
        aw9523_write(s_cfg.addr_u2, AW9523_REG_DIR_P0, 0xFF);
    }

    return ret;
}

// ── Hilfsfunktionen ──────────────────────────────────────────────

int cartridge_set_power(bool on) {
    if (!s_initialized) return CARTRIDGE_ERR_INIT;
    if (s_cfg.power_enable_port < 0) return CARTRIDGE_ERR_INIT;

    uint8_t reg = (s_cfg.power_enable_port == 0)
                  ? AW9523_REG_OUTPUT_P0
                  : AW9523_REG_OUTPUT_P1;
    uint8_t addr = s_cfg.addr_u2;

    // Aktuellen Ausgangszustand lesen
    uint8_t current = 0;
    // Wir haben keinen Shadow-Register, daher direkt schreiben:
    // Das Bit setzen oder löschen, Rest auf 0 (sicher da andere P1-Bits Inputs sind)
    uint8_t val = s_cfg.power_enable_active_high
                  ? (on ? (1 << s_cfg.power_enable_bit) : 0x00)
                  : (on ? 0x00 : (1 << s_cfg.power_enable_bit));
    (void)current;

    ESP_LOGI(TAG, "Power %s (reg=0x%02X val=0x%02X)", on ? "ON" : "OFF", reg, val);
    return aw9523_write(addr, reg, val);
}

bool cartridge_is_present(void) {
    if (!s_initialized) return false;
    // U1.P1_7 = Identifier-Bit: HIGH wenn Cartridge eingesteckt
    uint8_t p1 = 0;
    if (aw9523_read(s_cfg.addr_u1, AW9523_REG_INPUT_P1, &p1) != CARTRIDGE_OK)
        return false;
    return (p1 & 0x80) != 0;
}

bool cartridge_command_pending(void) {
    if (!s_initialized) return false;
    if (s_cfg.handshake.cs_pin < 0) return false;
    return gpio_get_level((gpio_num_t)s_cfg.handshake.cs_pin) == 1;
}
