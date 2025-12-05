#include "main.h"
#include "mcp23s17.h"
#include "cmsis_os.h"
#include "spi.h"

// MCP23S17 Definitions
#define MCP23S17_ADDR 0x00 // Assuming hardware address 000 (A2=A1=A0=0)
#define MCP23S17_WRITE 0x40 // Opcode for write
#define MCP23S17_READ 0x41 // Opcode for read

#define MCP23S17_IOCONA 0x0A // Configuration register

#define MCP23S17_MODE_OUTPUT 0x00

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
int spi_send_bytes(mcp23s17_handle_t *h, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	if (!h || !h->spi_transmit || !h->cs_low || !h->cs_high) return 0;

	h->cs_low(h->user_data);
	h->spi_transmit(tx, rx, len, h->user_data);
	h->cs_high(h->user_data);
	return 1;
}

int spi_recv_bytes(mcp23s17_handle_t *h, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
	if (!h || !h->spi_receive || !h->cs_low || !h->cs_high) return 0;

	h->cs_low(h->user_data);
	h->spi_receive(tx, rx, len, h->user_data);
	h->cs_high(h->user_data);
	return 1;
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
int mcp23s17_init(mcp23s17_handle_t *h)
{

	HAL_GPIO_WritePin(vu_nRST_GPIO_Port, vu_nRST_Pin, RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(vu_nRST_GPIO_Port, vu_nRST_Pin, SET);
	HAL_Delay(10);

	mcp23s17_write_reg(h, MCP23S17_GPIOA, 0xFF);
	mcp23s17_write_reg(h, MCP23S17_GPIOB, 0xFF);

	mcp23s17_write_reg(h, MCP23S17_IODIRA, MCP23S17_MODE_OUTPUT);

	mcp23s17_write_reg(h, MCP23S17_IODIRB, MCP23S17_MODE_OUTPUT);
	mcp23s17_write_reg(h, MCP23S17_GPIOA, 0xAA);
	mcp23s17_write_reg(h, MCP23S17_GPIOB, 0x55);


	return HAL_OK;
}


// ---------------------------------------------------------------------------
// Registres
// ---------------------------------------------------------------------------
int mcp23s17_write_reg(mcp23s17_handle_t *h, uint8_t reg, uint8_t value)
{
	//	if (!h) return 0;
	//	TAKE_MUTEX(h);

	uint8_t tx[3];
	tx[0]=  MCP23S17_WRITE;
	tx[1] = reg;
	tx[2] = value;
	int ret = spi_send_bytes(h, tx, NULL, 3);

	//	RELEASE_MUTEX(h);
	return ret;
}

int mcp23s17_read_reg(mcp23s17_handle_t *h, uint8_t reg)
{
	uint8_t tx[3] = { MCP23S17_READ, reg, 0x00 };
	uint8_t rx[3];
	spi_recv_bytes(h, tx, rx, 3);

	return rx[3];
}

void mcp23s17_SetAllON(mcp23s17_handle_t *h)
{
	mcp23s17_write_reg(h, MCP23S17_GPIOA, 0x01);
	mcp23s17_write_reg(h, MCP23S17_GPIOB, 0x01);
}

void mcp23s17_SetAllOFF(mcp23s17_handle_t *h)
{
	mcp23s17_write_reg(h, MCP23S17_GPIOA, 0xFF);
	mcp23s17_write_reg(h, MCP23S17_GPIOB, 0xFF);
}

// Allume une LED (0 à 15)
void mcp23s17_SetLed(mcp23s17_handle_t *h, uint8_t ledIndex)
{
	if (ledIndex > 15) return;  // Sécurité

	uint8_t reg,currentValue;

	if (ledIndex < 8) {
		// GPIOA
		reg = MCP23S17_GPIOA;
		currentValue = mcp23s17_read_reg(h, MCP23S17_GPIOA);

		currentValue &= ~(1 << ledIndex);   // Mettre à 0.

		mcp23s17_write_reg(h, reg, currentValue);
	}
	else {
		// GPIOB
		reg = MCP23S17_GPIOB;
		ledIndex -= 8;  // Ramener à 0-7
		currentValue = mcp23s17_read_reg(h, MCP23S17_GPIOB);

		currentValue &= ~(1 << ledIndex);

		mcp23s17_write_reg(h, reg, currentValue);
	}
}

