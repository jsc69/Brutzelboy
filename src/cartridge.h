#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <driver/i2c.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

// ================================================================
// cartridge.h — Generische GBC-Cartridge-Schnittstelle
//
// Liest/schreibt Daten über zwei AW9523B I²C-Portexpander,
// die an der GBC-Cartridge-Schnittstelle hängen.
//
// Bus-Layout (Standard-Konfiguration BrutzelBoy V1.3):
//   U1 @ I²C 0x58 — P0: Adressbus A0-A15 (16 Bit)
//                    P1: Steuerleitungen + Identifier-Bit (P1_7)
//   U2 @ I²C 0x5A — P0: Datenbus D0-D7 (8 Bit, direkt)
//                    P1: Steuerleitungen (WR, RD, CS, CLK, etc.)
//
// Handshake (optional, wählbare Pins über cartridge_config_t):
//   WR-Pin (Ausgang ESP32): "Daten bereit / Bereit zum Lesen"
//   RD-Pin (Eingang ESP32): "ACK vom Sender / Kommando gültig"
//   CS-Pin (Eingang ESP32): "Kommando pending" Flag
//
// Die Funktionen read(bank) und write(bank, value) sind bewusst
// generisch gehalten:
//   bank=0 → Adressbus (U1.P0, 16 Bit)
//   bank=1 → Datenbus  (U2.P0,  8 Bit)
//   Für direkten Registerzugriff: cartridge_read_reg() / cartridge_write_reg()
// ================================================================

// ── AW9523B Konstanten ───────────────────────────────────────────
#define AW9523_REG_INPUT_P0   0x00
#define AW9523_REG_INPUT_P1   0x01
#define AW9523_REG_OUTPUT_P0  0x02
#define AW9523_REG_OUTPUT_P1  0x03
#define AW9523_REG_DIR_P0     0x04   // 1=Input, 0=Output
#define AW9523_REG_DIR_P1     0x05
#define AW9523_REG_MODE_P0    0x12   // 1=Push-Pull, 0=Open-Drain
#define AW9523_REG_MODE_P1    0x13
#define AW9523_REG_RESET      0x7F   // Schreibe 0x00 für Soft-Reset

// Standard I²C-Adressen (BrutzelBoy V1.3)
#define CARTRIDGE_ADDR_U1     0x58   // Adressbus (CARD_A0-A15)
#define CARTRIDGE_ADDR_U2     0x5A   // Datenbus  (CARD_D0-D7)

// Handshake-Timeout in µs
#define CARTRIDGE_HANDSHAKE_TIMEOUT_US  5000

// ── Handshake-Konfiguration ──────────────────────────────────────
// Pins auf -1 setzen um Handshake zu deaktivieren.
typedef struct {
    int     wr_pin;     // ESP32 → Sender:   "Frame bereit / Bereit zum Lesen"
                        // (-1 = kein Handshake)
    int     rd_pin;     // Sender → ESP32:   "ACK / Kommando gültig"
                        // (-1 = kein Handshake)
    int     cs_pin;     // Sender → ESP32:   "Kommando pending" Flag
                        // (-1 = nicht verwendet)
} cartridge_handshake_t;

// Handshake deaktivieren (Polling-Modus)
#define CARTRIDGE_NO_HANDSHAKE  { .wr_pin = -1, .rd_pin = -1, .cs_pin = -1 }

// ── Cartridge-Konfiguration ──────────────────────────────────────
typedef struct {
    // I²C
    i2c_port_t  i2c_port;       // I²C-Bus (I2C_NUM_0 oder I2C_NUM_1)
    uint8_t     addr_u1;        // I²C-Adresse U1 (Adressbus)
    uint8_t     addr_u2;        // I²C-Adresse U2 (Datenbus)

    // Handshake-Pins (optional)
    cartridge_handshake_t handshake;

    // U2.P1 Steuerbit: ESP32-Ausgang der den ATmega 5V aktiviert
    // -1 = nicht vorhanden / nicht verwenden
    int8_t      power_enable_port;  // Port (0=P0, 1=P1) des Power-Enable-Bits
    uint8_t     power_enable_bit;   // Bit-Nummer innerhalb des Ports
    bool        power_enable_active_high;
} cartridge_config_t;

// Standard-Konfiguration für BrutzelBoy V1.3
// i2c_port muss vom Aufrufer gesetzt werden (I2C muss schon initialisiert sein)
#define CARTRIDGE_DEFAULT_CONFIG(port) {        \
    .i2c_port = (port),                         \
    .addr_u1  = CARTRIDGE_ADDR_U1,              \
    .addr_u2  = CARTRIDGE_ADDR_U2,              \
    .handshake = CARTRIDGE_NO_HANDSHAKE,        \
    .power_enable_port        = 1,              \
    .power_enable_bit         = 6,              \
    .power_enable_active_high = true,           \
}

// ── Rückgabewerte ────────────────────────────────────────────────
#define CARTRIDGE_OK            0
#define CARTRIDGE_ERR_I2C      -1
#define CARTRIDGE_ERR_TIMEOUT  -2
#define CARTRIDGE_ERR_INIT     -3

// ── API ──────────────────────────────────────────────────────────

/**
 * @brief Cartridge initialisieren.
 *        I²C muss bereits initialisiert sein.
 * @param cfg  Pointer auf Konfigurationsstruktur
 * @return CARTRIDGE_OK oder Fehlercode
 */
int cartridge_init(const cartridge_config_t *cfg);

/**
 * @brief Cartridge deinitialisieren (Pins freigeben).
 */
void cartridge_deinit(void);

/**
 * @brief Lese einen Bus-Bank.
 *        bank=0 → U1.P0 (Adressbus, 16 Bit: P0-High + P0-Low)
 *        bank=1 → U2.P0 (Datenbus, 8 Bit)
 *
 *        Mit Handshake: WR-Pin wird gesetzt, auf RD-ACK gewartet,
 *        dann gelesen und WR wieder zurückgenommen.
 *        Ohne Handshake: direktes I²C-Lesen.
 *
 * @param bank      0 = Adressbus (U1), 1 = Datenbus (U2)
 * @param value     Ausgabepuffer für gelesenen Wert
 * @return CARTRIDGE_OK, CARTRIDGE_ERR_I2C oder CARTRIDGE_ERR_TIMEOUT
 */
int cartridge_read(uint8_t bank, uint32_t *value);

/**
 * @brief Schreibe einen Bus-Bank.
 *        bank=0 → U1.P0 (Adressbus, 16 Bit)
 *        bank=1 → U2.P0 (Datenbus, 8 Bit)
 *
 *        Mit Handshake: Daten anlegen, WR-Pin setzen,
 *        auf RD-ACK warten, WR zurücknehmen.
 *        Ohne Handshake: direktes I²C-Schreiben.
 *
 * @param bank      0 = Adressbus (U1), 1 = Datenbus (U2)
 * @param value     Zu schreibender Wert
 * @return CARTRIDGE_OK, CARTRIDGE_ERR_I2C oder CARTRIDGE_ERR_TIMEOUT
 */
int cartridge_write(uint8_t bank, uint32_t value);

/**
 * @brief Direkter Zugriff: lese ein AW9523B-Register.
 * @param addr      I²C-Adresse (z.B. CARTRIDGE_ADDR_U1)
 * @param reg       Register-Adresse (z.B. AW9523_REG_INPUT_P0)
 * @param value     Ausgabepuffer
 * @return CARTRIDGE_OK oder CARTRIDGE_ERR_I2C
 */
int cartridge_read_reg(uint8_t addr, uint8_t reg, uint8_t *value);

/**
 * @brief Direkter Zugriff: schreibe ein AW9523B-Register.
 * @param addr      I²C-Adresse
 * @param reg       Register-Adresse
 * @param value     Zu schreibender Wert
 * @return CARTRIDGE_OK oder CARTRIDGE_ERR_I2C
 */
int cartridge_write_reg(uint8_t addr, uint8_t reg, uint8_t value);

/**
 * @brief ATmega 5V-Versorgung ein- oder ausschalten.
 *        Nur verfügbar wenn power_enable_bit konfiguriert ist.
 * @param on  true = ATmega einschalten
 * @return CARTRIDGE_OK oder CARTRIDGE_ERR_INIT
 */
int cartridge_set_power(bool on);

/**
 * @brief Prüfe ob eine Cartridge eingesteckt ist.
 *        Liest U1.P1_7 (Identifier-Bit, immer HIGH wenn Cartridge da).
 * @return true wenn Cartridge erkannt
 */
bool cartridge_is_present(void);

/**
 * @brief Prüfe ob ein Kommando vom Sender pending ist (CS-Pin HIGH).
 *        Nur sinnvoll wenn cs_pin konfiguriert ist.
 * @return true wenn Kommando ansteht
 */
bool cartridge_command_pending(void);

#ifdef __cplusplus
}
#endif
