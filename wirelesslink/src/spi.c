//#include "spi.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME spi
LOG_MODULE_REGISTER(LOG_MODULE_NAME);



/** @brief Transmit buffer initialized with the specified message ( @ref MSG_TO_SEND ). */
static uint8_t m_tx_buffer[64] = {0};

/** @brief Receive buffer defined with the size to store specified message ( @ref MSG_TO_SEND ). */
static uint8_t m_rx_buffer[64] = {0};


static struct spi_buf spi_tx_buf ={.buf = m_tx_buffer, .len = sizeof(m_tx_buffer)} ;
static const struct spi_buf_set spi_tx_bufs = {.buffers = &spi_tx_buf, .count = 1};
static struct spi_buf spi_rx_buf ={.buf = m_rx_buffer, .len = sizeof(m_rx_buffer)} ;
static const struct spi_buf_set spi_rx_bufs = {.buffers = &spi_rx_buf, .count = 1};



#define SPI_OP  SPI_OP_MODE_MASTER  | SPI_WORD_SET(8) | SPI_LINES_SINGLE

static const struct spi_dt_spec spi = SPI_DT_SPEC_GET(DT_NODELABEL(medradio), SPI_OP, 0);
   


void spi_init(void)
{

   if (!spi_is_ready_dt(&spi)) {
        LOG_ERR("Device medradio SPI not ready, aborting test");
        return;
    }

}




 void spi_transmit(uint8_t* tx_buf, uint8_t* rx_buf, uint8_t len) 
 {	

 	memcpy(m_tx_buffer, tx_buf, len);  //copy to SPI buffer
	spi_tx_buf.len = len;
	spi_rx_buf.len = len;

	//LOG_HEXDUMP_INF(m_tx_buffer, len, "Message to send");
	
	int ret = spi_transceive_dt(&spi, &spi_tx_bufs, &spi_rx_bufs);
	if (ret) { 
		LOG_INF("spi_read status: %d", ret); 
	}

	//LOG_HEXDUMP_INF(m_rx_buffer, len, "Message received");
	memcpy(rx_buf, m_rx_buffer, len);  //copy from SPI buffer 
	

	
 }



// /**
// @brief transmit single byte and retrieve response
// See Erata: http://infocenter.nordicsemi.com/pdf/nRF52832_Errata_v1.1.pdf
// Item 58, this outputs 2 bytes instead of 1.  For our purposes it doesn't matter
// */
 uint8_t spi_transmit_byte(uint8_t tx_byte) 
 {

	uint8_t rx_byte;
	
	spi_transmit(&tx_byte, &rx_byte, 1);
	
	return rx_byte;
 }











