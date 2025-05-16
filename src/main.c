#include "spi.h"
#include "eeng1030_lib.h"
#include "key_library.h"
#include <stm32l432xx.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include "PN532.h"
#include "auth.h"
#include "display.h"

void setup(void);
void delay(volatile uint32_t dly);
void initSerial(uint32_t baudrate);
void eputc(char c);

char c = 0;

/**
 * Main entry point. Initializes system, PN532, and loops for UID actions.
 */
int main(void) {
    setup();

    GPIOA->ODR &= ~(1 << 8);  // reset the board
    delay_ms(20);            // wait a little bit
    GPIOA->ODR |= (1 << 8);   // stop the reset
    delay_ms(20);              // wait a little bit
    GPIOB->ODR &= ~(1 << 0);  // pull Slave select low for the PN532
    delay_ms(20);              // Wait a little bit
    GPIOB->ODR |= (1 << 0);   // Pull slave select high
    delay_ms(100);             // wait a little bit for everything to be up and running


    typedef enum {
        STATE_WAIT_INPUT,
        STATE_ADD_UID,
        STATE_AUTH_UID,
        STATE_REMOVE_UID,
        STATE_READ_SPIREG,
        STATE_READ_TRANS,
        STATE_AUTH_SEC,
        STATE_CRACK_SEC,
        STATE_DUMP,
        STATE_TEST
    } SystemState;


    sendACK();

    uint8_t sam_cfg[] = {
        PN532_COMMAND_SAMCONFIGURATION,
        0x01,  // Normal mode
        0x14,  // Timeout = 100 ms (0x14 × 5ms)
        0x01   // Enable IRQ
    };

    // Initialise SAM
    sendCommand(sam_cfg, sizeof(sam_cfg));

    if (PN532_readACK()) {
        printf("ACK Read Succesfully.\r\n");
    }

    
    uint8_t rx_buf[PN532_MAX_FRAME_LEN];
    uint8_t rx_len = 0;
    uint8_t atqa[2];
    // Pn532_readResponse() will return true if successful.
    // It also waits until IRQ is pulled low.
    if (PN532_readResponse(rx_buf, &rx_len)) {
        printf("SAMConfig verification (0x15 expected):\r\n");
        for (uint8_t i = 0; i < rx_len; i++) {
            printf("Data[%d] = 0x%02X\n", i, rx_buf[i]);
        }
    }

    read_SPIcontrol_register();

    // End initialisation of SAM

    write_SPIcontrol_register(0x2C); // this sets the PN532 to SPI mode 3 which the display also uses
    delay_ms(100);           // wait a little bit for everything to be up and running

    init_display();
    SystemState state = STATE_WAIT_INPUT;
    PN532_TagInfo tag;
    printTextToggle("UID Authentication System",5,5,RGBToWord(128,255,128),0);

    // Enter the main loop
    while (1) {
        switch (state) {
            case STATE_WAIT_INPUT:
                printf("\r\n--- UID Authentication System ---\r\n");
                printf("1. Add UID to Whitelist\r\n");
                printf("2. Authenticate UID\r\n");
                printf("3. Remove UID from Whitelist\r\n");
                printf("4. Read SPIcontrol Register\r\n");
                printf("5. Read Transmission Register\r\n");
                printf("6. Auth Sector - Write block\r\n");
                printf("7. Auth Sector - Clear Block\r\n"); 
                printf("8. Dump settngs\r\n");
                printf("9. Test\r\n");
                printf("Select option: ");
                char cmd = waitForByte_USART2();
                printf("%c\r\n", cmd);  // Echo back selection

                if      (cmd == '1') state = STATE_ADD_UID;
                else if (cmd == '2') state = STATE_AUTH_UID;
                else if (cmd == '3') state = STATE_REMOVE_UID;
                else if (cmd == '4') state = STATE_READ_SPIREG;
                else if (cmd == '5') state = STATE_READ_TRANS;
                else if (cmd == '6') state = STATE_AUTH_SEC;
                else if (cmd == '7') state = STATE_CRACK_SEC; 
                else if (cmd == '8') state = STATE_DUMP;  
                else if (cmd == '9') state = STATE_TEST;
                else {
                    printf("Invalid input.\r\n");
                    state = STATE_WAIT_INPUT;
                }
                break;

            case STATE_ADD_UID:
                printf("Place tag to ADD...\r\n");
                if (PN532_scanForTag(&tag)) {
                    add_uid(&tag); // messaging handled in auth.c
                } else {
                    printf("❌ No tag detected.\r\n");
                }
                state = STATE_WAIT_INPUT;
                break;

            case STATE_AUTH_UID:
                printf("Place tag to AUTHENTICATE...\r\n");
                if (PN532_scanForTag(&tag)) {
                    if (is_uid_authorized(&tag))
                        printf("✅ Access Granted.\r\n");
                    else
                        printf("❌ Access Denied.\r\n");
                } else {
                    printf("❌ No tag detected.\r\n");
                }
                state = STATE_WAIT_INPUT;
                break;

            case STATE_REMOVE_UID:
                printf("Place tag to REMOVE from whitelist...\r\n");
                if (PN532_scanForTag(&tag)) {
                    remove_uid(&tag); // messaging handled in auth.c
                } else {
                    printf("❌ No tag detected.\r\n");
                }
                state = STATE_WAIT_INPUT;
                break;

                case STATE_READ_SPIREG:
                    read_SPIcontrol_register();
                    state = STATE_WAIT_INPUT;
                    break;

                case STATE_READ_TRANS:
                    Read_TransReg(); 
                    Read_RxModeReg();
                    disableRxCRC();
                    Read_RxModeReg();
                    state = STATE_WAIT_INPUT;
                break;

                case STATE_AUTH_SEC:
                        // we are going to auth a sector and then write to it.
                    if (PN532_scanForTag(&tag)) {
                        uint8_t command[14] = {
                            0x40,         // inDataExchange
                            0x01,         // Target
                            0x60,         // Authenticate A
                            0x22,         // Block 34   // sector 8 block 2 far out of the way for safety
                            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // Key A
                            tag.uid[0], tag.uid[1], tag.uid[2], tag.uid[3]
                        };

                        
                        if(PN532_sendRawPrint(command,sizeof(command))){
                            uint8_t command[] = {
                                0x40,   // InDataExchange
                                0x01,   // Target number (always 1)
                                0x30,   // READ command
                                0x22    // Block address to read
                            };

                            uint8_t response[64];
                            uint8_t response_len = 0;

                            PN532_sendRawGet(command, sizeof(command), response, &response_len);

                            if (response_len >= 16) {
                                printf("Block Data: ");
                                for (int i = 0; i < 16; i++) {
                                    printf("%02X ", response[i]);
                                }
                                printf("\r\n");
                                

                                uint8_t write_cmd[] = {
                                    0x40,  // InDataExchange
                                    0x01,  // Target (always 1)
                                    0xA0,  // MIFARE Classic WRITE command
                                    0x22,  // Block address (same block as before)
                                    // 16 bytes of data to write:
                                    0xDE, 0xAD, 0xBE, 0xEF,
                                    0xCA, 0xFE, 0xBA, 0xBE,
                                    0x11, 0x22, 0x33, 0x44,
                                    0x55, 0x66, 0x77, 0x88
                                };

                                if(PN532_sendRawPrint(write_cmd,sizeof(write_cmd))){
                                    response_len = 0;
                                    PN532_sendRawGet(command, sizeof(command), response, &response_len);
                                    printf("Block Data: ");
                                    for (int i = 0; i < 16; i++) {
                                        printf("%02X ", response[i]);
                                    }
                                    printf("\r\n");
                                }
                            } else {
                                printf("Failed to read block.\r\n");
                            }
                        }
                        
                    }
                    state = STATE_WAIT_INPUT;
                    break;
                
                case STATE_CRACK_SEC:
                 
                        // we are going to auth a sector and then write to it.
                    if (PN532_scanForTag(&tag)) {
                        uint8_t command[14] = {
                            0x40,         // inDataExchange
                            0x01,         // Target
                            0x60,         // Authenticate A
                            0x22,         // Block 34   // sector 8 block 2 far out of the way for safety
                            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,  // Key A
                            tag.uid[0], tag.uid[1], tag.uid[2], tag.uid[3]
                        };

                        
                        if(PN532_sendRawPrint(command,sizeof(command))){
                            uint8_t command[] = {
                                0x40,   // InDataExchange
                                0x01,   // Target number (always 1)
                                0x30,   // READ command
                                0x22    // Block address to read
                            };

                            uint8_t response[64];
                            uint8_t response_len = 0;

                            PN532_sendRawGet(command, sizeof(command), response, &response_len);

                            if (response_len >= 16) {
                                printf("Block Data: ");
                                for (int i = 0; i < 16; i++) {
                                    printf("%02X ", response[i]);
                                }
                                printf("\r\n");
                                

                                uint8_t write_cmd[] = {
                                    0x40,  // InDataExchange
                                    0x01,  // Target (always 1)
                                    0xA0,  // MIFARE Classic WRITE command
                                    0x22,  // Block address (same block as before)
                                    // 16 bytes of data to write:
                                    0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00
                                };

                                if(PN532_sendRawPrint(write_cmd,sizeof(write_cmd))){
                                    response_len = 0;
                                    PN532_sendRawGet(command, sizeof(command), response, &response_len);
                                    printf("Block Data: ");
                                    for (int i = 0; i < 16; i++) {
                                        printf("%02X ", response[i]);
                                    }
                                    printf("\r\n");
                                }
                            } else {
                                printf("Failed to read block.\r\n");
                            }
                        }
                        
                    }
                    state = STATE_WAIT_INPUT;
                    break;


                    case STATE_DUMP:
                    if(PN532_scanForTag(&tag)){
                        dumpRadioSettings();
                    }
                    state = STATE_WAIT_INPUT;
                    break;
                    
                    
                    case STATE_TEST:
                    
                    if(PN532_setTimerPresetMax()){
                        /* 1. Wait for a card */
                        if (!PN532_scanForTag(&tag)) {
                            printf("❌ No tag detected.\r\n");
                            state = STATE_WAIT_INPUT;
                            break;
                        }
                        /* 2. Turn off the PN532’s auto-CRC/parity */
                        
                        /* 3. Build a 4-byte frame: [CMD, BLOCK] + [CRC_LSB, CRC_MSB] */
                        uint8_t frame[4];
                        frame[0] = 0x60;               // AUTH A
                        frame[1] = 0x20;               // block 4
                        compute_crc_a(frame, 2, &frame[2]);
                        
                        /* 4. Pack those 4 bytes (with parity) into a bit-stream */
                        size_t  n             = sizeof(frame);              // ==4

                        uint8_t txbuf[(n*9 + 7)/8];                         // 36 bits → 5 bytes
                        size_t  bytes_to_send = pack_with_manual_parity(frame, n, txbuf);
                        
                        printf("pack_with_manual_parity: \r\n");
                        for (uint8_t i = 0; i < sizeof(txbuf); i++) {
                            printf(" 0x%02X", txbuf[i]);
                        }
                        
                        //-------------------------------
                        // Disabling neccesary registers
                        //------------------------------
                        disableRxCRC();
                        disableTxCRC();
                        modify_registerGeneric(0x63, 0x0D, 1<<4, 1<<4);  // ParityDisable=1
                        
                        //-------------------------------
                        // Preparing The Command
                        //-------------------------------
                        uint8_t cmd[1 + sizeof(txbuf)];
                        cmd[0] = PN532_COMMAND_INCOMMUNICATETHRU;
                        memcpy(&cmd[1], txbuf, bytes_to_send);
                        uint8_t response[PN532_MAX_FRAME_LEN];
                        uint8_t response_len;
                        uint8_t plain[64];
                        uint8_t plain_len;
                        printf("Command: \r\n");
                          for (uint8_t i = 0; i < sizeof(cmd); i++) {
                                printf(" 0x%02X", cmd[i]);
                            }

 
                        //------------------------------------
                        // Getting Token RB (nonce) from Card
                        //------------------------------------

                        if (PN532_sendRawGet(cmd, sizeof(cmd), response, &response_len)) {
 
                            printf("Response:");
                            for (uint8_t i = 0; i < response_len; i++) {
                                printf(" 0x%02X", response[i]);
                            }
                            printf("\r\n");

                        if (PN532_stripParity(response + 1, response_len - 1, plain, &plain_len)) {
                               
                                printf("Nt (correct this is known to be correct):");
                                for (uint8_t i = 0; i < plain_len; i++) {
                                    printf(" 0x%02X", plain[i]);
                                }
                                printf("\r\n");
                                
                            } else {
                                // error: either no ACK or malformed response
                        
                                printf("Failure at the if statment after recieving RB\r\n");
                            }
        

                        } else {
                            puts("Parity-encoded frame looks malformed.");
                        }
                    
                            
                           
                        
                        printf("\r\n");
                        restore_link_layer_defaults();

                    }
                    
                    state = STATE_WAIT_INPUT;
                    break;
        }

    }
}

/**
 * setup
 *
 * Initializes clocks, GPIOs, SPI, USART, and enables interrupts.
 */
void setup() {
    initClocks();    
    RCC->AHB2ENR |= (1 << 0) + (1 << 1); // enable GPIOA and GPIOB
    pinMode(GPIOB, 0, 1); // This is the SS line
    pinMode(GPIOB, 1, 0); // This is the IRQ line
    pinMode(GPIOA, 8, 1); // This is the Reset PN532 line

    // This adds a pull-up resistor to PB1 to prevent it from floating.
    GPIOB->PUPDR &= ~(3 << (1 * 2)); // Clear bits 3:2 for PB1           
    GPIOB->PUPDR |=  (1 << (1 * 2)); // Set PB1 to pull-up mode (01)

    initSerial(9600);
    SysTick->LOAD = 80000-1; // Systick clock = 80MHz. 80000000/80000=1000
	SysTick->CTRL = 7; // enable systick counter and its interrupts
	SysTick->VAL = 10; // start from a low number so we don't wait for ages for first interrupt
    initSPI(SPI1);
    __asm(" cpsie i "); // enable interrupts globally

}

/**
 * Initializes USART2 with a specified baudrate.
 *
 * @param baudrate Desired baud rate (e.g., 9600).
 */
void initSerial(uint32_t baudrate) {
    RCC->AHB2ENR |= (1 << 0); // make sure GPIOA is turned on
    pinMode(GPIOA, 2, 2); // alternate function mode for PA2
    selectAlternateFunction(GPIOA, 2, 7); // AF7 = USART2 TX
    pinMode(GPIOA, 15, 2); // This sets pin A15 to alternative function
    selectAlternateFunction(GPIOA, 15, 3); // This sets up the alternative function for USART2 RX
    RCC->APB1ENR1 |= (1 << 17); // turn on USART2

    const uint32_t CLOCK_SPEED = 80000000;    
    uint32_t BaudRateDivisor = CLOCK_SPEED / baudrate;

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = (1 << 12); // disable over-run errors
    USART2->BRR = BaudRateDivisor;
    USART2->CR1 = (1 << 3);  // enable the transmitter
    USART2->CR1 |= (1 << 2); // enable the receiver
    USART2->CR1 |= (1 << 0); // enable USART
}

/**
 * Redirects printf() to USART2 for output.
 */
int _write(int file, char *data, int len) {
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno = EBADF;
        return -1;
    }
    while (len--) {
        eputc(*data);    
        data++;
    }    
    return 0;
}

/**
 * Sends a single character out over USART2.
 *
 * @param c Character to transmit.
 */
void eputc(char c) {
    while ((USART2->ISR & (1 << 6)) == 0); // wait for ongoing transmission to finish
    USART2->TDR = c;
}

/**
 * Delays for a number of CPU cycles.
 *
 * @param dly Loop count (not time-calibrated).
 */
void delay(volatile uint32_t dly) {
    while (dly--);
}
