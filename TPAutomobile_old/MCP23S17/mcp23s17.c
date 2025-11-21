#include "mcp23s17.h"
#include "FreeRTOS.h"
#include "task.h"

// ---------------------------------------------------------------------------
// Op-codes SPI
// ---------------------------------------------------------------------------
#define OPCODE_WRITE(addr)   (0x40 | (((addr) & 0x07) << 1))
#define OPCODE_READ(addr)    (0x41 | (((addr) & 0x07) << 1))

// ---------------------------------------------------------------------------
// Macro de prise de mutex (timeout 10 ticks)
// ---------------------------------------------------------------------------
#define TAKE_MUTEX(h) do { \
    if ((h)->mutex) { \
        if (xSemaphoreTake((h)->mutex, pdMS_TO_TICKS(10)) != pdTRUE) return 0; \
    } \
} while(0)

#define RELEASE_MUTEX(h) do { \
    if ((h)->mutex) xSemaphoreGive((h)->mutex); \
} while(0)

// ---------------------------------------------------------------------------
// Transfert SPI bas niveau
// ---------------------------------------------------------------------------
static int spi_transfer_bytes(mcp23s17_handle_t *h, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (!h || !h->spi_transfer || !h->cs_low || !h->cs_high) return 0;

    h->cs_low(h->user_data);
    h->spi_transfer(tx, rx, len, h->user_data);
    h->cs_high(h->user_data);
    return 1;
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
int mcp23s17_init(mcp23s17_handle_t *h, int haen_enable)
{
    if (!h || !h->delay_ms) return 0;

    TAKE_MUTEX(h);

    h->delay_ms(10, h->user_data);

    uint8_t iocon = 0x00;
    if (haen_enable) iocon |= MCP23S17_IOCON_HAEN;

    // Écrire IOCON deux fois
    int ret = mcp23s17_write_reg(h, MCP23S17_IOCON, iocon);
    if (ret) {
        h->delay_ms(1, h->user_data);
        ret = mcp23s17_write_reg(h, MCP23S17_IOCON, iocon);
    }

    // Tout en entrée, pull-ups off
    if (ret) ret = mcp23s17_write_reg(h, MCP23S17_IODIRA, 0xFF);
    if (ret) ret = mcp23s17_write_reg(h, MCP23S17_IODIRB, 0xFF);
    if (ret) ret = mcp23s17_write_reg(h, MCP23S17_GPPUA, 0x00);
    if (ret) ret = mcp23s17_write_reg(h, MCP23S17_GPPUB, 0x00);

    RELEASE_MUTEX(h);
    return ret;
}

// ---------------------------------------------------------------------------
// Registres
// ---------------------------------------------------------------------------
int mcp23s17_write_reg(mcp23s17_handle_t *h, uint8_t reg, uint8_t value)
{
    if (!h) return 0;
    TAKE_MUTEX(h);

    uint8_t tx[3] = { OPCODE_WRITE(h->addr_pins), reg, value };
    int ret = spi_transfer_bytes(h, tx, NULL, 3);

    RELEASE_MUTEX(h);
    return ret;
}

int mcp23s17_read_reg(mcp23s17_handle_t *h, uint8_t reg, uint8_t *value)
{
    if (!h || !value) return 0;
    TAKE_MUTEX(h);

    uint8_t tx[2] = { OPCODE_READ(h->addr_pins), reg };
    uint8_t rx[2];
    int ret = spi_transfer_bytes(h, tx, rx, 2);
    if (ret) *value = rx[1];

    RELEASE_MUTEX(h);
    return ret;
}

// ---------------------------------------------------------------------------
// API haut niveau
// ---------------------------------------------------------------------------
int mcp23s17_pin_mode(mcp23s17_handle_t *h, uint8_t pin, int input)
{
    uint8_t reg = (pin < 8) ? MCP23S17_IODIRA : MCP23S17_IODIRB;
    uint8_t bit = 1 << (pin & 7);
    uint8_t val;
    if (!mcp23s17_read_reg(h, reg, &val)) return 0;
    if (input) val |= bit; else val &= ~bit;
    return mcp23s17_write_reg(h, reg, val);
}

int mcp23s17_pullup(mcp23s17_handle_t *h, uint8_t pin, int enable)
{
    uint8_t reg = (pin < 8) ? MCP23S17_GPPUA : MCP23S17_GPPUB;
    uint8_t bit = 1 << (pin & 7);
    uint8_t val;
    if (!mcp23s17_read_reg(h, reg, &val)) return 0;
    if (enable) val |= bit; else val &= ~bit;
    return mcp23s17_write_reg(h, reg, val);
}

int mcp23s17_digital_write(mcp23s17_handle_t *h, uint8_t pin, int level)
{
    uint8_t reg = (pin < 8) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    uint8_t bit = 1 << (pin & 7);
    uint8_t val;
    if (!mcp23s17_read_reg(h, reg, &val)) return 0;
    if (level) val |= bit; else val &= ~bit;
    return mcp23s17_write_reg(h, reg, val);
}

int mcp23s17_digital_read(mcp23s17_handle_t *h, uint8_t pin, int *level)
{
    uint8_t reg = (pin < 8) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    uint8_t val;
    if (!mcp23s17_read_reg(h, reg, &val)) return 0;
    if (level) *level = (val >> (pin & 7)) & 1;
    return 1;
}

int mcp23s17_write_port(mcp23s17_handle_t *h, mcp23s17_port_t port, uint8_t value)
{
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    return mcp23s17_write_reg(h, reg, value);
}

int mcp23s17_read_port(mcp23s17_handle_t *h, mcp23s17_port_t port, uint8_t *value)
{
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_GPIOA : MCP23S17_GPIOB;
    return mcp23s17_read_reg(h, reg, value);
}

int mcp23s17_int_enable(mcp23s17_handle_t *h, uint8_t pin, int enable)
{
    uint8_t reg = (pin < 8) ? MCP23S17_GPINTENA : MCP23S17_GPINTENB;
    uint8_t bit = 1 << (pin & 7);
    uint8_t val;
    if (!mcp23s17_read_reg(h, reg, &val)) return 0;
    if (enable) val |= bit; else val &= ~bit;
    return mcp23s17_write_reg(h, reg, val);
}

int mcp23s17_int_get_captured(mcp23s17_handle_t *h, mcp23s17_port_t port, uint8_t *value)
{
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_INTCAPA : MCP23S17_INTCAPB;
    return mcp23s17_read_reg(h, reg, value);
}

int mcp23s17_int_get_flag(mcp23s17_handle_t *h, mcp23s17_port_t port, uint8_t *flag)
{
    uint8_t reg = (port == MCP23S17_PORTA) ? MCP23S17_INTFA : MCP23S17_INTFB;
    return mcp23s17_read_reg(h, reg, flag);
}
