#ifndef MCP23S17_H
#define MCP23S17_H

#include <stdint.h>
#include <stddef.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Callbacks MCU (à fournir par l'utilisateur)
// ---------------------------------------------------------------------------
typedef void (*mcp23s17_cs_low_t)(void *user_data);
typedef void (*mcp23s17_cs_high_t)(void *user_data);
typedef void (*mcp23s17_spi_transfer_t)(const uint8_t *tx, uint8_t *rx, uint16_t len, void *user_data);
typedef void (*mcp23s17_delay_ms_t)(uint32_t ms, void *user_data);

// ---------------------------------------------------------------------------
// Handle du driver (une instance = un MCP23S17)
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t                  addr_pins;     // A2 A1 A0 (0..7)
    void                    *user_data;     // ton handle SPI, GPIO, etc.

    mcp23s17_cs_low_t        cs_low;
    mcp23s17_cs_high_t       cs_high;
    mcp23s17_spi_transfer_t  spi_transfer;
    mcp23s17_delay_ms_t      delay_ms;

    SemaphoreHandle_t        mutex;         // NULL = pas de protection, sinon mutex FreeRTOS
} mcp23s17_handle_t;

// ---------------------------------------------------------------------------
// Registres (BANK=0)
// ---------------------------------------------------------------------------
#define MCP23S17_IODIRA         0x00
#define MCP23S17_IODIRB         0x01
#define MCP23S17_IPOLA          0x02
#define MCP23S17_IPOLB          0x03
#define MCP23S17_GPINTENA       0x04
#define MCP23S17_GPINTENB       0x05
#define MCP23S17_DEFVALA        0x06
#define MCP23S17_DEFVALB        0x07
#define MCP23S17_INTCONA        0x08
#define MCP23S17_INTCONB        0x09
#define MCP23S17_IOCON          0x0A
#define MCP23S17_GPPUA          0x0C
#define MCP23S17_GPPUB          0x0D
#define MCP23S17_INTFA          0x0E
#define MCP23S17_INTFB          0x0F
#define MCP23S17_INTCAPA        0x10
#define MCP23S17_INTCAPB        0x11
#define MCP23S17_GPIOA          0x12
#define MCP23S17_GPIOB          0x13
#define MCP23S17_OLATA          0x14
#define MCP23S17_OLATB          0x15

#define MCP23S17_IOCON_HAEN     (1<<3)
#define MCP23S17_IOCON_SEQOP    (1<<5)

// Ports
typedef enum {
    MCP23S17_PORTA = 0,
    MCP23S17_PORTB = 1
} mcp23s17_port_t;

// ---------------------------------------------------------------------------
// Fonctions publiques (retour: 1 = OK, 0 = erreur)
// ---------------------------------------------------------------------------

void MCP23S17_Init(void);

int mcp23s17_write_reg(mcp23s17_handle_t *handle, uint8_t reg, uint8_t value);
int mcp23s17_read_reg(mcp23s17_handle_t *handle, uint8_t reg, uint8_t *value);

int mcp23s17_pin_mode(mcp23s17_handle_t *handle, uint8_t pin, int input);
int mcp23s17_pullup(mcp23s17_handle_t *handle, uint8_t pin, int enable);
int mcp23s17_digital_write(mcp23s17_handle_t *handle, uint8_t pin, int level);
int mcp23s17_digital_read(mcp23s17_handle_t *handle, uint8_t pin, int *level);

int mcp23s17_write_port(mcp23s17_handle_t *handle, mcp23s17_port_t port, uint8_t value);
int mcp23s17_read_port(mcp23s17_handle_t *handle, mcp23s17_port_t port, uint8_t *value);

int mcp23s17_int_enable(mcp23s17_handle_t *handle, uint8_t pin, int enable);
int mcp23s17_int_get_captured(mcp23s17_handle_t *handle, mcp23s17_port_t port, uint8_t *value);
int mcp23s17_int_get_flag(mcp23s17_handle_t *handle, mcp23s17_port_t port, uint8_t *flag);

void MCP23S17_SetAllPinsLow(void);
void MCP23S17_SetAllPinsHigh(void);
void MCP23S17_SetLed(uint8_t ledIndex);


#ifdef __cplusplus
}
#endif

#endif // MCP23S17_H
