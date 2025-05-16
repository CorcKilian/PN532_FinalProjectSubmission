#include "spi.h"
#include "eeng1030_lib.h"
#include <stdio.h>   // for printf, scanf, etc.
#include <stdlib.h>  // for malloc, free, exit
#include <string.h>  // for strcmp, strlen, etc.

/**
 * initSPI
 *
 * Initializes the SPI1 peripheral on the STM32.
 * Configures SPI1 pins and control registers for master mode operation.
 *
 * @param spi Pointer to SPI peripheral (e.g., SPI1).
 */
void initSPI(SPI_TypeDef *spi) {
    /*
        I/O List:
        PA7  : SPI1 MOSI : Alternative function 5		
        PA1  : SPI1 SCLK : Alternative function 5    
        PA11 : SPI1 MISO : Alternative function 5
    */

    int drain;	
    RCC->APB2ENR |= (1 << 12); // turn on SPI1

    // Now configure the SPI interface        
    pinMode(GPIOA, 7, 2);        
    pinMode(GPIOA, 1, 2);
    pinMode(GPIOA, 11, 2);
    selectAlternateFunction(GPIOA, 7, 5);        
    selectAlternateFunction(GPIOA, 1, 5);
    selectAlternateFunction(GPIOA, 11, 5);

    drain = spi->SR; // dummy read of SR to clear MODF	

    // enable SSM, set SSI, enable SPI, PCLK/2, MSB First Master   
    spi->CR1 = (1 << 9) + (1 << 8) + (1 << 7) + (1 << 6) + (1 << 2) + (1 << 3) + (1 << 5) + (1 << 4); 
    spi->CR1 &= ~(1 << 0); // ensuring correct settings for PN532
    spi->CR1 &= ~(1 << 1); 
    spi->CR1 &= ~(7 << 3); // ensuring correct settings for display
    spi->CR1 |= (6 << 3); 
    spi->CR2 = (1 << 10) + (1 << 9) + (1 << 8); // configure for 8-bit operation
}

void initSPIMode3(SPI_TypeDef *spi) {
    /*
        I/O List:
        PA7  : SPI1 MOSI : Alternative function 5		
        PA1  : SPI1 SCLK : Alternative function 5    
        PA11 : SPI1 MISO : Alternative function 5
    */

    int drain;	
    RCC->APB2ENR |= (1 << 12); // turn on SPI1

    // Now configure the SPI interface        
    pinMode(GPIOA, 7, 2);        
    pinMode(GPIOA, 1, 2);
    pinMode(GPIOA, 11, 2);
    selectAlternateFunction(GPIOA, 7, 5);        
    selectAlternateFunction(GPIOA, 1, 5);
    selectAlternateFunction(GPIOA, 11, 5);

    drain = spi->SR; // dummy read of SR to clear MODF	

    // enable SSM, set SSI, enable SPI, PCLK/2, MSB First Master  
    
    spi->CR1 = (1 << 9) + (1 << 8) + (1 << 7) + (1 << 6) + (1 << 2) + (1 << 3) + (1 << 4); 
    spi->CR1 |= (1 << 0); // ensuring correct settings for display
    spi->CR1 |= (1 << 1); 
    spi->CR1 &= ~(7 << 3); // ensuring correct settings for display
    spi->CR1 |= (6 << 3); 
    spi->CR2 = (1 << 10) + (1 << 9) + (1 << 8); // configure for 8-bit operation
}

/**
 * transferSPI8
 *
 * Sends and receives 1 byte over SPI.
 *
 * @param spi Pointer to SPI peripheral.
 * @param data Byte to transmit.
 * @return Received byte.
 */
uint8_t transferSPI8(SPI_TypeDef *spi, uint8_t data) {
    uint8_t ReturnValue;
    volatile uint8_t *preg = (volatile uint8_t*)&spi->DR;

    while ((spi->SR & (1 << 7)) != 0); // wait if busy
    *preg = data;
    while ((spi->SR & (1 << 7)) != 0); // wait for transfer to finish
    ReturnValue = *preg;

    return ReturnValue;
}

/**
 * transferSPI16
 *
 * Sends and receives 16 bits over SPI.
 *
 * @param spi Pointer to SPI peripheral.
 * @param data 16-bit data to transmit.
 * @return 16-bit data received.
 */
uint16_t transferSPI16(SPI_TypeDef *spi, uint16_t data) {
    uint32_t ReturnValue;    	
    while ((spi->SR & (1 << 7)) != 0); // wait if busy
    spi->DR = data;    
    while ((spi->SR & (1 << 7)) != 0); // wait for transfer to finish
    ReturnValue = spi->DR;

    return (uint16_t)ReturnValue;
}

/**
 * spi_exchange
 *
 * Sends and/or receives multiple bytes over SPI using full-duplex transfer.
 *
 * @param spi Pointer to SPI peripheral.
 * @param d_out Pointer to output data buffer (data to transmit).
 * @param d_out_len Length of output data in bytes.
 * @param d_in Pointer to input data buffer (to store received data).
 * @param d_in_len Length of input data to read (usually same or less than d_out_len).
 * @return Currently always returns 0.
 */
uint8_t spi_exchange(SPI_TypeDef *spi, uint8_t d_out[], uint32_t d_out_len, uint8_t d_in[], uint32_t d_in_len) {
    unsigned Timeout = 1000000;
    unsigned index = 0;
    uint8_t ReturnValue = 0;

    while (d_out_len--) {
        transferSPI8(spi, d_out[index]);
        index++;
    }

    index = 0;
    while (d_in_len--) {
        d_in[index] = transferSPI8(spi, 0xFF);
        index++;
    }

    return ReturnValue; 
}

/**
 * waitForByte_USART2
 *
 * Blocks until a byte is received over USART2.
 *
 * @return Received byte.
 */
char waitForByte_USART2(void) {
    while ((USART2->ISR & USART_ISR_RXNE) == 0);  // Wait until RXNE (Receive Not Empty) is set
    return USART2->RDR;  // Read the received byte
}

/**
 * delaySPI
 *
 * Simple busy-wait loop used for SPI timing delays.
 *
 * @param dly Number of cycles to delay.
 */
void delaySPI(volatile uint32_t dly) {
    while (dly--);
}
void SetSPIMode(uint8_t cpha, uint8_t cpol)
{

    // Clear CPHA (bit 0) and CPOL (bit 1)
    SPI1->CR1 &= ~((1 << 0) | (1 << 1));

    // Set CPHA and CPOL based on input
    SPI1->CR1 |= ((cpha & 0x01) << 0) | ((cpol & 0x01) << 1);
    //need to send two bytes before sending actual info, else it will send the first 2 bytes as garbage.	
    transferSPI8(SPI1,0xFF);
    transferSPI8(SPI1,0xFF); 
}
