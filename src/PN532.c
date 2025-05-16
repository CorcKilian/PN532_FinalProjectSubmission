#include "PN532.h"
#include <stm32l432xx.h>
#include "eeng1030_lib.h"
#include "spi.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>   // for printf, scanf, etc.
#include <stdlib.h>  // for malloc, free, exit
#include <string.h>  // for strcmp, strlen, etc.


uint16_t count = 0;

/**
 * delayPN
 *
 * Simple blocking delay used in PN532-related timing.
 *
 * @param dly Arbitrary delay count (not calibrated to real time).
 */
void delayPN(volatile uint32_t dly) {
    while (dly--);
}

/**
 * PN532_SS_LOW
 *
 * Pulls the PN532 SPI Slave Select line LOW.
 */
void PN532_SS_LOW() {
    GPIOB->ODR &= ~(1 << 0);
}

/**
 * PN532_SS_HIGH
 *
 * Pulls the PN532 SPI Slave Select line HIGH.
 */
void PN532_SS_HIGH() {
    GPIOB->ODR |= (1 << 0);
}

/**
 * sendCommand
 *
 * Sends a command frame to the PN532 over SPI.
 * Based on Adafruit's PN532 library implementation.
 *
 * @param cmd Pointer to command buffer (excluding TFI).
 * @param cmdlen Length of the command (excluding TFI).
 */
void sendCommand(const uint8_t *cmd, uint8_t cmdlen) {
    // Actual command length must include TFI
    uint8_t frameLen = cmdlen + 1;

    PN532_SS_LOW();
    delayPN(PN532_Delay80MHZ);  // Ensure PN532 is awake

    transferSPI8(SPI1, PN532_SPI_DATAWRITE); // Notify PN532 of write

    // Write Preamble, Start Code
    transferSPI8(SPI1, PN532_PREAMBLE);
    transferSPI8(SPI1, PN532_PREAMBLE);
    transferSPI8(SPI1, PN532_STARTCODE2);

    // Write Length and LCS
    transferSPI8(SPI1, frameLen);
    transferSPI8(SPI1, (0x100 - frameLen) & 0xFF);

    // Write TFI (Host to PN532 identifier)
    transferSPI8(SPI1, PN532_HOSTTOPN532);

    uint8_t checksum = PN532_HOSTTOPN532; // Start with TFI

    for (uint8_t i = 0; i < cmdlen; i++) {
        transferSPI8(SPI1, cmd[i]);      // Send command byte
        checksum += cmd[i];              // Update checksum
    }

    transferSPI8(SPI1, ((~checksum + 1) & 0xFF)); // DCS
    transferSPI8(SPI1, PN532_POSTAMBLE);         // Postamble

    PN532_SS_HIGH();
}

/**
 * PN532_readACK
 *
 * Waits for and verifies PN532 ACK response pattern.
 *
 * @return true if valid ACK received, false otherwise.
 */
bool PN532_readACK(void) {
    const uint8_t ACK_PATTERN[6] = { 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00 };
    uint8_t window[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    uint8_t byte;
    bool ack_found = false;

    while (PN532_IRQcheck); // wait for IRQ

    PN532_SS_LOW();
    delayPN(PN532_Delay80MHZ);

    // Read up to 12 bytes to search for valid ACK frame
    for (int i = 0; i < 12; i++) {
        byte = transferSPI8(SPI1, 0xFF);

        for (int j = 0; j < 5; j++)
            window[j] = window[j + 1];
        window[5] = byte;

        bool match = true;
        for (int j = 0; j < 6; j++) {
            if (window[j] != ACK_PATTERN[j]) {
                match = false;
                break;
            }
        }

        if (match) {
            ack_found = true;
            break;
        }
    }

    PN532_SS_HIGH();
    return ack_found;
}

/**
 * PN532_readResponse
 *
 * Reads a full response frame from the PN532.
 *
 * @param data_out Pointer to buffer to store response payload (excluding TFI).
 * @param data_len_out Pointer to store number of payload bytes.
 * @return true if successful and response is valid, false otherwise.
 */
bool PN532_readResponse(uint8_t *data_out, uint8_t *data_len_out) {
    uint8_t byte;
    uint8_t len, lcs, tfi;
    uint8_t dcs, postamble;
    uint8_t frame_data[PN532_MAX_FRAME_LEN];
    uint8_t sync_window[3] = { 0xFF, 0xFF, 0xFF };
    uint8_t sync_found = 0;

    uint32_t elapsed = 0;
    while (PN532_IRQcheck && elapsed < PN532_RESPONSE_TIMEOUT_MS) {
        delayPN_ms(1);
        elapsed++;
    }

    if (PN532_IRQcheck) {
        *data_len_out = 0;
        return false;  // timeout
    }

    PN532_SS_LOW();
    delayPN(PN532_Delay80MHZ);
    transferSPI8(SPI1, PN532_SPI_DATAREAD);  // SPI read command

    // Find frame preamble
    for (int i = 0; i < 8; i++) {
        byte = transferSPI8(SPI1, 0xFF);
        sync_window[0] = sync_window[1];
        sync_window[1] = sync_window[2];
        sync_window[2] = byte;

        if (sync_window[0] == 0x00 && sync_window[1] == 0x00 && sync_window[2] == 0xFF) {
            sync_found = 1;
            break;
        }
    }

    if (!sync_found)
        goto fail;

    // Read LEN, LCS, TFI
    len = transferSPI8(SPI1, 0xFF);
    lcs = transferSPI8(SPI1, 0xFF);
    tfi = transferSPI8(SPI1, 0xFF);

    if ((uint8_t)(len + lcs) != 0x00 || len < 1 || len > PN532_MAX_FRAME_LEN)
        goto fail;

    for (int i = 0; i < len - 1; i++)
        frame_data[i] = transferSPI8(SPI1, 0xFF);

    dcs = transferSPI8(SPI1, 0xFF);
    postamble = transferSPI8(SPI1, 0xFF);

    PN532_SS_HIGH();

    uint8_t sum = tfi;
    for (int i = 0; i < len - 1; i++)
        sum += frame_data[i];

    if ((uint8_t)(sum + dcs) != 0x00 || postamble != 0x00)
        goto fail;

    for (int i = 0; i < len - 1; i++)
        data_out[i] = frame_data[i];

    *data_len_out = len - 1;
    return true;

fail:
    PN532_SS_HIGH();
    *data_len_out = 0;
    return false;
}

/**
 * PN532_scanForTag
 *
 * Sends an InListPassiveTarget command and parses tag information.
 *
 * @param tag_info Pointer to PN532_TagInfo struct to populate.
 * @return true if a valid tag is detected and parsed, false otherwise.
 */
bool PN532_scanForTag(PN532_TagInfo *tag_info) {
    const uint8_t inlist_cmd[] = {
        0x4A,  // INLISTPASSIVETARGET
        0x01,  // Max number of targets
        0x00   // Baud rate: 106 kbps (Type A)
    };

    for (uint8_t attempt = 0; attempt < PN532_SCAN_MAX_RETRIES; attempt++) {
        sendCommand(inlist_cmd, sizeof(inlist_cmd));

        if (!PN532_readACK()) {
            delayPN_ms(PN532_SCAN_RETRY_DELAY);
            continue;
        }

        uint8_t buf[32] = {0};
        uint8_t len = 0;

        if (!PN532_readResponse(buf, &len)) {
            delayPN_ms(PN532_SCAN_RETRY_DELAY);
            continue;
        }

        if (len < 7 || buf[0] != 0x4B)
            continue;

        tag_info->nb_targets    = buf[1];
        tag_info->target_number = buf[2];
        tag_info->sens_res[0]   = buf[3];
        tag_info->sens_res[1]   = buf[4];
        tag_info->sel_res       = buf[5];
        tag_info->uid_len       = buf[6];

        if (tag_info->uid_len > sizeof(tag_info->uid))
            return false;

        for (uint8_t i = 0; i < tag_info->uid_len; i++) {
            tag_info->uid[i] = buf[7 + i];
        }

        tag_info->response_len = len;
        for (uint8_t i = 0; i < len; i++) {
            tag_info->raw_response[i] = buf[i];
        }

        return true;
    }

    return false;
}

/**
 * delayPN_ms
 *
 * Millisecond blocking delay using NOP loops.
 *
 * @param ms Number of milliseconds to delay.
 */
void delayPN_ms(uint32_t ms) {
    volatile uint32_t count = ms * (SystemCoreClock / 1000);
    while (count--) {
        __NOP(); // Prevent compiler optimization
    }
}

/**
 * read_SPIcontrol_register
 *
 * Reads the SPIcontrol register (0xFFA9) from the PN532 using the ReadRegister command.
 * This is useful for checking or debugging the current SPI configuration (e.g., CPOL/CPHA bits).
 *
 * The function sends a ReadRegister command frame over SPI, waits for the ACK,
 * then reads and parses the response. If successful, it prints the register value.
 *
 * @note Uses sendCommand(), PN532_readACK(), and PN532_readResponse().
 * @note Register 0xA9 is part of the internal SFR space, so the high byte is 0xFF.
 */
void read_SPIcontrol_register(void) {
    uint8_t readreg_cmd[] = {
        PN532_COMMAND_READREGISTER,  // 0x06
        0xFF,  // High byte of address
        0xA9   // Low byte of address (SPIcontrol SFR)
    };

    // Send ReadRegister command
    sendCommand(readreg_cmd, sizeof(readreg_cmd));

    // Wait for ACK
    if (!PN532_readACK()) {
        printf("❌ No ACK for ReadRegister command.\r\n");
        return;
    }

    // Read the response
    uint8_t rx_buf[PN532_MAX_FRAME_LEN] = {0};
    uint8_t rx_len = 0;

    if (!PN532_readResponse(rx_buf, &rx_len)) {
        printf("❌ Failed to read response.\r\n");
        return;
    }

    if (rx_len >= 2 && rx_buf[0] == PN532_COMMAND_READREGISTER + 1) {
        printf("✅ SPIcontrol (0xA9) register value: 0x%02X\r\n", rx_buf[1]);
    } else {
        printf("⚠️ Unexpected response.\r\n");
        for (uint8_t i = 0; i < rx_len; i++) {
            printf("RX[%d] = 0x%02X\r\n", i, rx_buf[i]);
        }
    }
}

/**
 * write_SPIcontrol_register
 *
 * Writes a value to the SPIcontrol register (0xFFA9) using the WriteRegister command.
 * This lets you change SPI settings like CPOL and CPHA to match other devices.
 *
 * Example: Setting value to 0x3F sets SPI Mode 3 (CPOL=1, CPHA=1)
 * @param value The 8-bit value to write to the SPIcontrol register.
 */
void write_SPIcontrol_register(uint8_t value) {
    uint8_t writereg_cmd[] = {
        PN532_COMMAND_WRITEREGISTER,  // 0x08
        0xFF,
        0xA9,
        value
    };

    // Send the command
    sendCommand(writereg_cmd, sizeof(writereg_cmd));

    // Wait for ACK
    if (!PN532_readACK()) {
        printf("❌ No ACK for WriteRegister command.\r\n");
        return;
    }
    printf("✅ ACK received for WriteRegister.\r\n");
    // set the approptiate SPIMode
    SetSPIMode((value >> 3) & 0x01, (value >> 2) & 0x01);



    // Read response (no new command needed)
    uint8_t rx_buf[PN532_MAX_FRAME_LEN] = {0};
    uint8_t rx_len = 0;

    if (PN532_readResponse(rx_buf, &rx_len)) {
        if (rx_len >= 1 && rx_buf[0] == PN532_COMMAND_WRITEREGISTER + 1) {
            printf("✅ WriteRegister response received.\r\n");
        } else {
            printf("⚠️ Unexpected WriteRegister response:\r\n");
            for (uint8_t i = 0; i < rx_len; i++) {
                printf("  RX[%u] = 0x%02X\r\n", i, rx_buf[i]);
            }
        }
    } else {
        printf("❌ Failed to read WriteRegister response.\r\n");
    }
}


void sendACK(void)
{
    PN532_SS_LOW();
    delayPN(PN532_Delay80MHZ);  // Ensure PN532 is awake
    transferSPI8(SPI1, 0x01);
    transferSPI8(SPI1, 0x00);
    transferSPI8(SPI1, 0x00);
    transferSPI8(SPI1, 0xFF);
    transferSPI8(SPI1, 0x00);
    transferSPI8(SPI1, 0xFF);
    transferSPI8(SPI1, 0x00);
    PN532_SS_HIGH();
    
}



/**
 * check_CRC_transmission_status
 *
 * Reads the CIU_TxMode register (0x6302) to determine if CRC generation is enabled.
 * Bit 7 (TxCRCEn) = 1 means PN532 appends CRC to outgoing data.
 */
void Read_TransReg(void) {
    uint8_t readreg_cmd[] = {
        PN532_COMMAND_READREGISTER,  // 0x06
        0x63,  // High byte of 0x6302
        0x02   // Low byte of 0x6302 (CIU_TxMode)
    };

    // Send ReadRegister command
    sendCommand(readreg_cmd, sizeof(readreg_cmd));

    // Wait for ACK
    if (!PN532_readACK()) {
        printf("❌ No ACK for ReadRegister (TxMode).\r\n");
        return;
    }

    // Read response
    uint8_t rx_buf[PN532_MAX_FRAME_LEN] = {0};
    uint8_t rx_len = 0;

    if (!PN532_readResponse(rx_buf, &rx_len)) {
        printf("❌ Failed to read TxMode register response.\r\n");
        return;
    }

    if (rx_len >= 2 && rx_buf[0] == PN532_COMMAND_READREGISTER + 1) {
        uint8_t txmode_val = rx_buf[1];
        printf("📡 CIU_TxMode (0x6302) value: 0x%02X\r\n", txmode_val);

        if (txmode_val & (1 << 7)) {
            printf("✅ CRC Generation is ENABLED (TxCRCEn = 1)\r\n");
        } else {
            printf("❌ CRC Generation is DISABLED (TxCRCEn = 0)\r\n");
        }
    } else {
        printf("⚠️ Unexpected TxMode register response:\r\n");
        for (uint8_t i = 0; i < rx_len; i++) {
            printf("  RX[%u] = 0x%02X\r\n", i, rx_buf[i]);
        }
    }
}


/**
 * Read_RxModeReg
 *
 * Reads CIU_RxMode (0x6303) and shows whether the PN532
 * expects a CRC on every incoming frame (bit 7 = RxCRCEn).
 */
void Read_RxModeReg(void)
{
    uint8_t readreg_cmd[] = {
        PN532_COMMAND_READREGISTER,   // 0x06
        0x63,                         // high byte of 0x6303
        0x03                          // low  byte of 0x6303 (CIU_RxMode)
    };

    /* --- send command ---------------------------------------------------- */
    sendCommand(readreg_cmd, sizeof(readreg_cmd));

    if (!PN532_readACK()) {
        printf("❌ No ACK for ReadRegister (RxMode).\r\n");
        return;
    }

    /* --- read response --------------------------------------------------- */
    uint8_t rx_buf[PN532_MAX_FRAME_LEN] = {0};
    uint8_t rx_len = 0;

    if (!PN532_readResponse(rx_buf, &rx_len)) {
        printf("❌ Failed to read RxMode register response.\r\n");
        return;
    }

    /* --- parse ----------------------------------------------------------- */
    if (rx_len >= 2 && rx_buf[0] == (PN532_COMMAND_READREGISTER + 1)) {
        uint8_t rxmode_val = rx_buf[1];
        printf("📡 CIU_RxMode (0x6303) value: 0x%02X\r\n", rxmode_val);

        if (rxmode_val & (1 << 7)) {
            printf("✅ CRC Checking is ENABLED (RxCRCEn = 1)\r\n");
        } else {
            printf("❌ CRC Checking is DISABLED (RxCRCEn = 0)\r\n");
        }
    } else {
        printf("⚠️ Unexpected RxMode register response:\r\n");
        for (uint8_t i = 0; i < rx_len; i++) {
            printf("  RX[%u] = 0x%02X\r\n", i, rx_buf[i]);
        }
    }
}

bool disableRxCRC(void)
{
    // WRITE_REGISTER frame:
    //   PN532_COMMAND_WRITEREGISTER,
    //   high byte = 0x63, low byte = 0x03  → CIU_RxMode
    //   value = 0x00 to clear bit7 (RxCRCEn=0) and keep framing=ISO14443A@106kbps
    uint8_t wr[] = {
        PN532_COMMAND_WRITEREGISTER,
        0x63, 0x03,
        0x00
    };

    // send it, wait for ACK
    sendCommand(wr, sizeof wr);
    if (!PN532_readACK()) {
        printf("❌ No ACK disabling RxCRC\n");
        return false;
    }

    // read and ignore the single-byte response
    uint8_t buf[8];
    uint8_t len = 0;
    if (!PN532_readResponse(buf, &len)) {
        printf("❌ No response after disabling RxCRC\n");
        return false;
    }

    return true;
}

bool disableTxCRC(void)
{
    // WRITE_REGISTER frame:
    //   PN532_COMMAND_WRITEREGISTER,
    //   high byte = 0x63, low byte = 0x03  → CIU_RxMode
    //   value = 0x00 to clear bit7 (RxCRCEn=0) and keep framing=ISO14443A@106kbps
    uint8_t wr[] = {
        PN532_COMMAND_WRITEREGISTER,
        0x63, 0x02,
        0x00
    };

    // send it, wait for ACK
    sendCommand(wr, sizeof wr);
    if (!PN532_readACK()) {
        printf("❌ No ACK disabling TxCRC\n");
        return false;
    }

    // read and ignore the single-byte response
    uint8_t buf[8];
    uint8_t len = 0;
    if (!PN532_readResponse(buf, &len)) {
        printf("❌ No response after disabling TxCRC\n");
        return false;
    }

    return true;
}

static bool readRegister(uint8_t hi, uint8_t lo, uint8_t *value) {
    // Build READ_REGISTER frame
    uint8_t rd[] = {
        PN532_COMMAND_READREGISTER,
        hi, lo
    };
    sendCommand(rd, sizeof rd);
    if (!PN532_readACK()) {
        printf("❌ No ACK for READ_REGISTER 0x%02X%02X\n", hi, lo);
        return false;
    }

    uint8_t rsp[PN532_MAX_FRAME_LEN];
    uint8_t len = 0;
    if (!PN532_readResponse(rsp, &len)) {
        printf("❌ No response for READ_REGISTER 0x%02X%02X\n", hi, lo);
        return false;
    }
    // Expect rsp[0] == PN532_COMMAND_READREGISTER+1 and rsp[1]==value
    if (len < 2 || rsp[0] != (PN532_COMMAND_READREGISTER + 1)) {
        printf("⚠️ Unexpected READ_REGISTER resp for 0x%02X%02X\n", hi, lo);
        return false;
    }
    *value = rsp[1];
    return true;
}

void dumpRadioSettings(void) {
    struct {
        const char *name;
        uint8_t     hi, lo;
    } regs[] = {
        // CIU_Mode..CIU_RxThreshold
        { "CIU_Mode",          0x63, 0x01 },
        { "CIU_TxMode",        0x63, 0x02 },
        { "CIU_RxMode",        0x63, 0x03 },
        { "CIU_TxControl",     0x63, 0x04 },
        { "CIU_TxAuto",        0x63, 0x05 },
        { "CIU_TxSel",         0x63, 0x06 },
        { "CIU_RxSel",         0x63, 0x07 },
        { "CIU_RxThreshold",   0x63, 0x08 },
        { "CIU_Demod",         0x63, 0x09 },
        // Felica / NFC registers (often not used in plain MIFARE)
        { "CIU_FelNFC1",       0x63, 0x0A },
        { "CIU_FelNFC2",       0x63, 0x0B },
        { "CIU_MifNFC",        0x63, 0x0C },
        { "CIU_ManualRCV",     0x63, 0x0D },
        { "CIU_TypeB",         0x63, 0x0E },

        // CRC result registers
        { "CIU_CRCResultMSB",  0x63, 0x11 },
        { "CIU_CRCResultLSB",  0x63, 0x12 },

        // Modulation, pulse and timing
        { "CIU_GsNOff",        0x63, 0x13 },
        { "CIU_ModWidth",      0x63, 0x14 },
        { "CIU_TxBitPhase",    0x63, 0x15 },
        { "CIU_RFCfg",         0x63, 0x16 },
        { "CIU_GsNOn",         0x63, 0x17 },
        { "CIU_CWGsP",         0x63, 0x18 },
        { "CIU_ModGsP",        0x63, 0x19 },

        // Timer configuration & state
        { "CIU_TMode",         0x63, 0x1A },
        { "CIU_TPrescaler",    0x63, 0x1B },
        { "CIU_TReload_hi",    0x63, 0x1C },
        { "CIU_TReload_lo",    0x63, 0x1D },
        { "CIU_TCounterVal_hi",0x63, 0x1E },
        { "CIU_TCounterVal_lo",0x63, 0x1F },

        // Test registers
        { "CIU_TestSel1",      0x63, 0x21 },
        { "CIU_TestSel2",      0x63, 0x22 },
        { "CIU_TestPinEn",     0x63, 0x23 },
        { "CIU_TestPinValue",  0x63, 0x24 },
        { "CIU_TestBus",       0x63, 0x25 },
        { "CIU_AutoTest",      0x63, 0x26 },
        { "CIU_Version",       0x63, 0x27 },
        { "CIU_AnalogTest",    0x63, 0x28 },
        { "CIU_TestDAC1",      0x63, 0x29 },
        { "CIU_TestDAC2",      0x63, 0x2A },

        // Interrupt-related
        { "CIU_CommIEn",       0x63, 0x32 },
        { "CIU_DivIEn",        0x63, 0x33 },
        { "CIU_CommIrq",       0x63, 0x34 },
        { "CIU_DivIrq",        0x63, 0x35 },

        // Error & status
        { "CIU_Error",         0x63, 0x36 },  // last-command error flags
        { "CIU_FIFOData",      0x63, 0x39 },  // next byte out of FIFO
        { "CIU_FIFOLevel",     0x63, 0x3A },  // how many bytes in FIFO
        { "CIU_WaterLevel",    0x63, 0x3B },  // threshold for hi/lo alerts
        { "CIU_Control",       0x63, 0x3C },  // misc controls (timer start/stop…)
        { "CIU_BitFraming",    0x63, 0x3D },  // start-send, align, last-bits :contentReference[oaicite:0]{index=0}&#8203;:contentReference[oaicite:1]{index=1}
        { "CIU_Coll",          0x63, 0x3E },  // anticollision bit pos :contentReference[oaicite:2]{index=2}&#8203;:contentReference[oaicite:3]{index=3}
        { "CIU_Status1",       0x63, 0x3F },  // FIFO hi/lo, RF events, timer running…
        { "CIU_Status2",       0x63, 0x38 },  // crypto, TgActive, RF good :contentReference[oaicite:4]{index=4}&#8203;:contentReference[oaicite:5]{index=5}
    };
    uint8_t val;
    for (size_t i = 0; i < sizeof(regs)/sizeof(*regs); i++) {
        if (readRegister(regs[i].hi, regs[i].lo, &val)) {
            printf("%-16s (0x%02X%02X): 0x%02X\n",
                   regs[i].name, regs[i].hi, regs[i].lo, val);
        } else {
            printf("❌ Failed to read %s (0x%02X%02X)\n",
                   regs[i].name, regs[i].hi, regs[i].lo);
        }
    }
}



/**
 * read_registerGeneric
 *
 * Reads one 8-bit PN532 register.
 *
 * @param addr_high High byte of the 16-bit CIU register address
 * @param addr_low  Low  byte of the 16-bit CIU register address
 * @param value_out Pointer that will receive the register content on success
 * @return          true  if the read succeeded and value_out is valid,
 *                  false otherwise
 */
bool read_registerGeneric(uint8_t addr_high, uint8_t addr_low, uint8_t *value_out)
{
    static const uint8_t READREG_CMD = PN532_COMMAND_READREGISTER; // 0x06

    uint8_t readreg_cmd[] = { READREG_CMD, addr_high, addr_low };

    /* 1. Send ReadRegister command frame */
    sendCommand(readreg_cmd, sizeof readreg_cmd);

    /* 2. Wait for ACK */
    if (!PN532_readACK())
        return false;

    /* 3. Read response frame */
    uint8_t rx_buf[PN532_MAX_FRAME_LEN];
    uint8_t rx_len = 0;

    if (!PN532_readResponse(rx_buf, &rx_len))
        return false;

    /* 4. Verify response and extract register value            *
     *    Response payload layout:                              *
     *    [0]  = READREGISTER + 1  (command code echoed back)   *
     *    [1]  = register content                               */
    if (rx_len >= 2 && rx_buf[0] == (READREG_CMD + 1)) {
        *value_out = rx_buf[1];
        return true;
    }

    return false;   /* malformed or unexpected response */
}
void write_registerGeneric(uint8_t addr_high, uint8_t addr_low, uint8_t value) {
    uint8_t write_cmd[] = {
        PN532_COMMAND_WRITEREGISTER,  // 0x08
        addr_high,
        addr_low,
        value
    };

    sendCommand(write_cmd, sizeof(write_cmd));

    if (!PN532_readACK()) {
        printf("❌ No ACK for WriteRegister to 0x%02X%02X\r\n", addr_high, addr_low);
        return;
    }

    uint8_t response[PN532_MAX_FRAME_LEN] = {0};
    uint8_t len = 0;

    if (!PN532_readResponse(response, &len)) {
        printf("❌ Failed to read response after WriteRegister to 0x%02X%02X\r\n", addr_high, addr_low);
        return;
    }

    if (len >= 1 && response[0] == (PN532_COMMAND_WRITEREGISTER + 1)) {
        printf("✅ WriteRegister to 0x%02X%02X successful. Value written: 0x%02X\r\n",
               addr_high, addr_low, value);
    } else {
        printf("⚠️ Unexpected response to WriteRegister at 0x%02X%02X\r\n", addr_high, addr_low);
        for (uint8_t i = 0; i < len; i++) {
            printf("  RX[%u] = 0x%02X\r\n", i, response[i]);
        }
    }
}

/**
 * PN532_sendRawPrint
 * ----------------------------------------------------------------
 *  Sends any PN532 command (excluding the TFI), waits for ACK,
 *  then reads and prints the full PN532 response payload.
 *
 *  cmd     : pointer to command body (no 0xD4 TFI byte)
 *  cmdlen  : length of cmd[]
 *
 *  returns true  if an ACK was received (response is still printed)
 *          false if ACK timed out or was invalid (no response)
 * ----------------------------------------------------------------
 */
bool PN532_sendRawPrint(const uint8_t *cmd, uint8_t cmdlen)
{
    if (!cmd || cmdlen == 0) {
        return false;
    }

    /* 1) send the frame */
    sendCommand(cmd, cmdlen);

    /* 2) wait for ACK; if it fails, bail out */
    if (!PN532_readACK()) {
        printf("❌ PN532 did not ACK the command.\r\n");
        return false;
    }

    /* 3) read whatever comes back into a local buffer */
    uint8_t buffer[PN532_MAX_FRAME_LEN];
    uint8_t length = 0;
    if (!PN532_readResponse(buffer, &length)) {
        printf("❌ Failed to read PN532 response frame.\r\n");
        return true;  // ACK was OK; we return true but note the read error
    }

    /* 4) print the raw response bytes */
    printf("⇦  PN532 raw response (%u byte%s):\r\n",
           length, (length == 1) ? "" : "s");
    for (uint8_t i = 0; i < length; i++) {
        printf("  [0x%02X] 0x%02X\r\n", i, buffer[i]);
    }

    return true;
}
/**
 * @brief  Max out the CIU timer prescaler (12 bits → 0xFFF) without
 *         disturbing the other flags in CIU_TMode.
 * @return true on success (ACKs + valid responses), false on any failure
 */
bool PN532_setTimerPresetMax(void)
{
    uint8_t tmode;

    // 1) Read the current CIU_TMode (0x631A)
    if (!readRegister(0x63, 0x1A, &tmode)) {
        // ❌ No ACK or bad response
        return false;
    }

    // 2) OR in the lower nibble (TPrescaler_Hi[3:0] = 0xF)
    tmode |= 0x0F;

    // 3) Write it back
    write_registerGeneric(0x63, 0x1A, tmode);
    //    (this will send the WriteRegister command, wait for ACK, etc.)

    // 4) Now set the low-byte of the prescaler to 0xFF
    write_registerGeneric(0x63, 0x1B, 0xFF);

    return true;
}

/**
 * modify_registerGeneric
 *
 * Read a PN532 CIU register, change only the bits in `mask` to the
 * corresponding bits in `value_bits`, and write it back.
 *
 * @param addr_high    high byte of the 16-bit register address
 * @param addr_low     low  byte of the 16-bit register address
 * @param mask         bitmask of which bits we want to change (1 = change)
 * @param value_bits   bits to set within the mask (must already be shifted)
 * @return             true on success, false on any I/O error
 */
bool modify_registerGeneric(uint8_t addr_high,
    uint8_t addr_low,
    uint8_t mask,
    uint8_t value_bits)
{
uint8_t cur;
// 1) Read the current register value
if (!read_registerGeneric(addr_high, addr_low, &cur))
return false;

// 2) Compute new value: clear bits under mask, then OR in desired bits
uint8_t newv = (cur & ~mask) | (value_bits & mask);

// 3) If unchanged, we're done
if (newv == cur)
return true;

// 4) Otherwise write back the modified value
write_registerGeneric(addr_high, addr_low, newv);

// 5) Optionally re-read to confirm, or assume success if no error printed
return true;
}
/**
 * compute_crc_a
 *
 * ISO14443-A CRC-A (polynomial x^16 + x^12 + x^5 + 1).
 * Initial register value = 0x6363 (LSB=0x63, MSB=0x63).
 *
 * @param data     pointer to input bytes
 * @param len      number of bytes in data[]
 * @param crc_out  pointer to 2-byte array; on return:
 *                 crc_out[0] = CRC-A LSB
 *                 crc_out[1] = CRC-A MSB
 */
void compute_crc_a(const uint8_t *data, size_t len, uint8_t crc_out[2])
{
    uint16_t crc = 0x6363;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc = crc >> 1;
            }
        }
    }

    crc_out[0] = (uint8_t)(crc & 0xFF);       // CRC-A LSB
    crc_out[1] = (uint8_t)((crc >> 8) & 0xFF);// CRC-A MSB
}

/** compute odd parity (ISO14443-A) over one byte */
static inline uint8_t iso14443_parity(uint8_t b) {
    return __builtin_parity(b) ^ 1;
}

/** set one bit (v=0/1) at bitpos in buf[], MSB-first ordering */
static void set_tx_bit(uint8_t *buf, size_t bitpos, uint8_t v) {
    size_t  bytei = bitpos >> 3;
    uint8_t biti  =  bitpos & 7;        /* 0 = LSB, 7 = MSB */
    if (v) buf[bytei] |=  (1u << biti);
    else   buf[bytei] &= ~(1u << biti);
}

/**
 * pack_with_manual_parity
 *
 * Packs N data bytes + parity into txbuf, sets TxLastBits, and returns
 * the number of bytes to send via InCommunicateThru.
 *
 * @param data     pointer to N bytes of payload
 * @param len      number of bytes in data[]
 * @param txbuf    output buffer; must be at least (len*9+7)/8 bytes
 * @return         number of bytes you should send into FIFO
 */
size_t pack_with_manual_parity(const uint8_t *data,
                               size_t len,
                               uint8_t *txbuf)
{
    // 1) total bits = 9 bits per input byte
    size_t total_bits = len * 9;
    // 2) how many bytes that is (rounded up)
    size_t byte_len = (total_bits + 7) / 8;
    // 3) clear buffer
    memset(txbuf, 0, byte_len);

    // 4) pack each byte + its parity bit
    size_t bitpos = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t d = data[i];
        for (int b = 0; b < 8; b++) {          
            set_tx_bit(txbuf, bitpos++, (d >> b) & 1);
        }
        
        /*for (int b = 7; b >= 0; b--) {
            set_tx_bit(txbuf, bitpos++, (d >> b) & 1);
        }*/
        // 1 parity bit
        set_tx_bit(txbuf, bitpos++, iso14443_parity(d));
    }

    // 5) compute TxLastBits (0 => full last byte)
    uint8_t last_bits = (uint8_t)(total_bits % 8);

    // 6) program CIU_BitFraming.TxLastBits (reg 0x633D: mask=0x07)
    modify_registerGeneric(0x63, 0x3D, 0x07, last_bits);

    return byte_len;
}
/**
 * restore_link_layer_defaults
 *
 * Re-enable ISO14443-A parity checking/generation and
 * CRC generation/checking on the PN532.
 */
void restore_link_layer_defaults(void)
{
    // 1) Re-enable parity (clear ParityDisable bit)
    //    CIU_ManualRCV at 0x630D, bit 4 = ParityDisable
    modify_registerGeneric(
        0x63, 0x0D,        // addr_high, addr_low
        1 << 4,            // mask = bit 4
        0                  // value_bits = clear bit 4
    );

    // 2) Re-enable TX CRC generation
    //    CIU_TxAuto at 0x6305, bit 7 = TxCRCEn
    modify_registerGeneric(
        0x63, 0x05,        // addr_high, addr_low
        1 << 7,            // mask = bit 7
        1 << 7             // value_bits = set bit 7
    );

    // 3) Re-enable RX CRC checking
    //    CIU_Mode at 0x6300, bit 6 = RxCRCEn
    modify_registerGeneric(
        0x63, 0x00,        // addr_high, addr_low
        1 << 6,            // mask = bit 6
        1 << 6             // value_bits = set bit 6
    );
}


/**
 * PN532_sendRawGet
 * ------------------------------------------------------------
 *  Sends a PN532 command (body only, no 0xD4 TFI), waits
 *  for ACK, reads the full response frame, and returns the
 *  payload bytes.
 *
 *  @param cmd        pointer to the command body
 *  @param cmdlen     length of cmd[]
 *  @param out        buffer that receives the payload bytes
 *  @param outlen     pointer that receives the payload length
 *  @return           true  – ACK received and response valid
 *                    false – any error (ACK timeout, bad frame…)
 * ------------------------------------------------------------
 */
bool PN532_sendRawGet(const uint8_t *cmd, uint8_t cmdlen,
                      uint8_t *out, uint8_t *outlen)
{
    if (!cmd || cmdlen == 0 || !out || !outlen) {
        return false;
    }

    /* 1) send the command frame */
    sendCommand(cmd, cmdlen);

    /* 2) wait for ACK from the PN532 */
    if (!PN532_readACK()) {
        return false;
    }

    /* 3) read the response frame
           buf[0]          = RESP_CMD
           buf[1 .. n-2]   = payload bytes
           buf[n-1]        = DCS                                       */
    uint8_t buf[PN532_MAX_FRAME_LEN];
    uint8_t len = 0;
    if (!PN532_readResponse(buf, &len)) {
        return false;
    }

    /* len must include at least RESP_CMD and DCS */
    if (len < 2) {
        return false;
    }

    /* 4) verify the response command code matches request + 1 */
    if (buf[0] != (uint8_t)(cmd[0] + 1)) {
        return false;
    }

    /* 5) copy just the payload (strip RESP_CMD and DCS) */
    uint8_t datalen = len - 1;             // payload length
    if (datalen > 0) {
        memcpy(out, &buf[1], datalen);     // payload starts at buf[1]
    }
    *outlen = datalen;

    return true;
}


bool PN532_stripParity(const uint8_t *src, uint16_t src_len,
                       uint8_t *dst,     uint16_t *dst_len) {
    if (!src || !dst || !dst_len) return false;

    uint32_t bit_stream = 0;
    uint32_t total_bits = src_len * 8;
    uint32_t bit_buffer_size = src_len * 8;  // holds all reversed bits

    // Allocate enough space to hold all bits (max 512 bits for src_len = 64)
    uint8_t bit_buffer[bit_buffer_size];
    uint32_t bit_pos = 0;

    // Step 1: reverse each byte and store its bits in bit_buffer
    for (uint16_t i = 0; i < src_len; i++) {
        uint8_t rev = reverse_byte(src[i]);
        for (int j = 7; j >= 0; j--) {
            bit_buffer[bit_pos++] = (rev >> j) & 0x01;
        }
    }

    // Step 2: extract 8 bits, skip 1 bit, repeat
    uint16_t max_out_bytes = (src_len * 8) / 9;
    uint16_t produced = 0;
    uint32_t read_pos = 0;

    while ((read_pos + 8) <= bit_pos && produced < max_out_bytes) {
        uint8_t out_byte = 0;
        for (int i = 0; i < 8; i++) {
            out_byte <<= 1;
            out_byte |= bit_buffer[read_pos++];
        }
        dst[produced++] = reverse_byte(out_byte);
        read_pos++;  // skip 1 bit
    }

    *dst_len = produced;
    return true;
}

static uint8_t reverse_byte(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}