#ifndef __PN532_H__
#define __PN532_H__

#include <stdint.h>
#include <stm32l432xx.h>
#include <string.h> 
#include <stdbool.h>

void PN532_SS_LOW(void);
void PN532_SS_HIGH(void);
void sendCommand(const uint8_t *cmd, uint8_t cmdlen);
void checkStatus(uint8_t type);
bool PN532_readResponse(uint8_t *data_out, uint8_t *data_len_out);
bool PN532_readACK(void);
void delayPN_ms(uint32_t ms);
void delayPN(volatile uint32_t dly);
void read_SPIcontrol_register(void);
void write_SPIcontrol_register(uint8_t value);
void sendACK(void);
bool PN532_authenticateBlock(uint8_t block, char key_type, const uint8_t *key, const uint8_t *uid, uint8_t *nt_out);
void Read_TransReg(void);
bool PN532_requestAuthNonce(uint8_t block, char key_type, uint8_t *nt_out);
void Read_RxModeReg(void);
bool disableRxCRC(void);
void dumpRadioSettings(void);
bool PN532_sendRawGet(const uint8_t *cmd, uint8_t cmdlen,
    uint8_t *out, uint8_t *outlen);
bool modify_registerGeneric(uint8_t addr_high,
    uint8_t addr_low,
    uint8_t mask,
    uint8_t value_bits);
size_t pack_with_manual_parity(const uint8_t *data,
        size_t len,
        uint8_t *txbuf);

static void set_tx_bit(uint8_t *buf, size_t bitpos, uint8_t v);
void compute_crc_a(const uint8_t *data, size_t len, uint8_t crc_out[2]);
static inline uint8_t iso14443_parity(uint8_t b);
void restore_link_layer_defaults(void);
bool PN532_AuthKnownCapture(uint8_t  block,
    char     keyType,
    const uint8_t key[6],
    const uint8_t uid[4],
    uint8_t  Nt1[4],
    uint8_t  Nr[4],
    uint8_t  At1[4],
    uint8_t  Ar1[4]);

bool read_registerGeneric(uint8_t addr_high, uint8_t addr_low, uint8_t *value_out);
void write_registerGeneric(uint8_t addr_high, uint8_t addr_low, uint8_t value);
bool pn532_deselectTarget(uint8_t tg);
bool pn532_selectTarget(uint8_t tg);
bool PN532_setTimerPresetMax(void);
bool PN532_sendRawPrint(const uint8_t *cmd, uint8_t cmdlen);

bool PN532_authenticateBlock(uint8_t block,
                             char    key_type,
                             const uint8_t *key,
                             const uint8_t *uid,
                             uint8_t *nt_out);


bool PN532_captureUnknownAuth(uint8_t  block,
    char     keyType,     /* 'A' or 'B' */
    uint8_t  Nt2_out[4],  /* 4-byte buffer */
    uint8_t  At2_out[4]);  /* 4-byte buffer */

typedef struct {
    uint8_t nb_targets;        // Number of tags detected
    uint8_t target_number;     // Tag ID assigned by PN532
    uint8_t sens_res[2];       // ATQA (SENS_RES)
    uint8_t sel_res;           // SAK (SEL_RES)
    uint8_t uid_len;           // UID length in bytes
    uint8_t uid[10];           // UID data
    uint8_t raw_response[32];  // Optional: raw full buffer
    uint8_t response_len;      // Length of raw response
} PN532_TagInfo;
bool PN532_scanForTag(PN532_TagInfo *tag_info);
bool disableTxCRC(void);
bool stripParity(const uint8_t *src, uint16_t src_len,
                 uint8_t *dst, uint16_t *dst_len);
static uint8_t reverse_byte(uint8_t b);

#define PN532_IRQcheck                      (GPIOB->IDR & (1 << 1))    // Port B1 is the IRQ port
#define PN532_Delay80MHZ                    (3000)
#define PN532_MaxWait                       (3000)
#define PN532_MAX_FRAME_LEN                 (253)  // LEN (max 255-1) includes TFI (1 byte), so DATA = 254 - 1 = 253
#define PN532_SCAN_RETRY_DELAY              (100)  // delay in ms before trying to detect a card again
#define PN532_SCAN_TIMEOUT_MS               (100)  // how long to wait before resending the inlistPassiveTarget command
#define PN532_SCAN_MAX_RETRIES              (3)    // how many retries before giving up
#define PN532_RESPONSE_TIMEOUT_MS           (200)  // timeout for the get response function


// PN532 Frame Constants
#define PN532_PREAMBLE                      (0x00)
#define PN532_STARTCODE1                    (0x00)
#define PN532_STARTCODE2                    (0xFF)
#define PN532_POSTAMBLE                     (0x00)

#define PN532_HOSTTOPN532                   (0xD4)
#define PN532_PN532TOHOST                   (0xD5)

// PN532 Commands
#define PN532_COMMAND_DIAGNOSE              (0x00)
#define PN532_COMMAND_GETFIRMWAREVERSION    (0x02)
#define PN532_COMMAND_GETGENERALSTATUS      (0x04)
#define PN532_COMMAND_READREGISTER          (0x06)
#define PN532_COMMAND_WRITEREGISTER         (0x08)
#define PN532_COMMAND_READGPIO              (0x0C)
#define PN532_COMMAND_WRITEGPIO             (0x0E)
#define PN532_COMMAND_SETSERIALBAUDRATE     (0x10)
#define PN532_COMMAND_SETPARAMETERS         (0x12)
#define PN532_COMMAND_SAMCONFIGURATION      (0x14)
#define PN532_COMMAND_POWERDOWN             (0x16)
#define PN532_COMMAND_RFCONFIGURATION       (0x32)
#define PN532_COMMAND_RFREGULATIONTEST      (0x58)
#define PN532_COMMAND_INJUMPFORDEP          (0x56)
#define PN532_COMMAND_INJUMPFORPSL          (0x46)
#define PN532_COMMAND_INLISTPASSIVETARGET   (0x4A)
#define PN532_COMMAND_INATR                 (0x50)
#define PN532_COMMAND_INPSL                 (0x4E)
#define PN532_COMMAND_INDATAEXCHANGE        (0x40)
#define PN532_COMMAND_INCOMMUNICATETHRU     (0x42)
#define PN532_COMMAND_INDESELECT            (0x44)
#define PN532_COMMAND_INRELEASE             (0x52)
#define PN532_COMMAND_INSELECT              (0x54)
#define PN532_COMMAND_INAUTOPOLL            (0x60)
#define PN532_COMMAND_TGINITASTARGET        (0x8C)
#define PN532_COMMAND_TGSETGENERALBYTES     (0x92)
#define PN532_COMMAND_TGGETDATA             (0x86)
#define PN532_COMMAND_TGSETDATA             (0x8E)
#define PN532_COMMAND_TGSETMETADATA         (0x94)
#define PN532_COMMAND_TGGETINITIATORCOMMAND (0x88)
#define PN532_COMMAND_TGRESPONSETOINITIATOR (0x90)
#define PN532_COMMAND_TGGETTARGETSTATUS     (0x8A)

// PN532 Responses
#define PN532_RESPONSE_INDATAEXCHANGE       (0x41)
#define PN532_RESPONSE_INLISTPASSIVETARGET  (0x4B)

// Wakeup Command
#define PN532_WAKEUP                        (0x55)

// SPI-specific constants
#define PN532_SPI_STATREAD                  (0x02)
#define PN532_SPI_DATAWRITE                 (0x01)
#define PN532_SPI_DATAREAD                  (0x03)
#define PN532_SPI_READY                     (0x01)

// I2C-specific constants
#define PN532_I2C_ADDRESS                   (0x48 >> 1)
#define PN532_I2C_READBIT                   (0x01)
#define PN532_I2C_BUSY                      (0x00)
#define PN532_I2C_READY                     (0x01)
#define PN532_I2C_READYTIMEOUT              (20)

// Mifare constants
#define PN532_MIFARE_ISO14443A              (0x00)

#define MIFARE_CMD_AUTH_A                   (0x60)
#define MIFARE_CMD_AUTH_B                   (0x61)
#define MIFARE_CMD_READ                     (0x30)
#define MIFARE_CMD_WRITE                    (0xA0)
#define MIFARE_CMD_TRANSFER                 (0xB0)
#define MIFARE_CMD_DECREMENT                (0xC0)
#define MIFARE_CMD_INCREMENT                (0xC1)
#define MIFARE_CMD_STORE                    (0xC2)
#define MIFARE_ULTRALIGHT_CMD_WRITE         (0xA2)

// NDEF URI Prefixes
#define NDEF_URIPREFIX_NONE                 (0x00)
#define NDEF_URIPREFIX_HTTP_WWWDOT          (0x01)
#define NDEF_URIPREFIX_HTTPS_WWWDOT         (0x02)
#define NDEF_URIPREFIX_HTTP                 (0x03)
#define NDEF_URIPREFIX_HTTPS                (0x04)
#define NDEF_URIPREFIX_TEL                  (0x05)
#define NDEF_URIPREFIX_MAILTO               (0x06)
#define NDEF_URIPREFIX_FTP_ANONAT           (0x07)
#define NDEF_URIPREFIX_FTP_FTPDOT           (0x08)
#define NDEF_URIPREFIX_FTPS                 (0x09)
#define NDEF_URIPREFIX_SFTP                 (0x0A)
#define NDEF_URIPREFIX_SMB                  (0x0B)
#define NDEF_URIPREFIX_NFS                  (0x0C)
#define NDEF_URIPREFIX_FTP                  (0x0D)
#define NDEF_URIPREFIX_DAV                  (0x0E)
#define NDEF_URIPREFIX_NEWS                 (0x0F)
#define NDEF_URIPREFIX_TELNET               (0x10)
#define NDEF_URIPREFIX_IMAP                 (0x11)
#define NDEF_URIPREFIX_RTSP                 (0x12)
#define NDEF_URIPREFIX_URN                  (0x13)
#define NDEF_URIPREFIX_POP                  (0x14)
#define NDEF_URIPREFIX_SIP                  (0x15)
#define NDEF_URIPREFIX_SIPS                 (0x16)
#define NDEF_URIPREFIX_TFTP                 (0x17)
#define NDEF_URIPREFIX_BTSPP                (0x18)
#define NDEF_URIPREFIX_BTL2CAP              (0x19)
#define NDEF_URIPREFIX_BTGOEP               (0x1A)
#define NDEF_URIPREFIX_TCPOBEX              (0x1B)
#define NDEF_URIPREFIX_IRDAOBEX             (0x1C)
#define NDEF_URIPREFIX_FILE                 (0x1D)
#define NDEF_URIPREFIX_URN_EPC_ID           (0x1E)
#define NDEF_URIPREFIX_URN_EPC_TAG          (0x1F)
#define NDEF_URIPREFIX_URN_EPC_PAT          (0x20)
#define NDEF_URIPREFIX_URN_EPC_RAW          (0x21)
#define NDEF_URIPREFIX_URN_EPC              (0x22)
#define NDEF_URIPREFIX_URN_NFC              (0x23)

#endif /* __PN532_H__ */
