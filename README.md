## Introduction

This repository expands on the [PN532-STM32-SecuritySystem](https://github.com/CorcKilian/PN532-STM32-SecuritySystem) project, focusing on low-level NFC communication and authentication protocols.

The primary objective of this project is to explore the implementation of the **MIFARE Classic 3-pass authentication protocol** using an **STM32 microcontroller** interfaced with a **PN532 NFC module** over SPI. Unlike typical high-level library abstractions, this project delves into the internal mechanics of the authentication process, providing a clear, step-by-step approach to understanding how MIFARE Classic cards perform mutual authentication, as well as partially performing a low level 3 pass Authentification sequence.

After successful high level authentication, the system performs secure **read and write operations to specific memory blocks** on the MIFARE card. This enables practical applications such as access control, secure data storage, and card-based identification systems.

## Theory

### MIFARE Classic 3-Pass Authentication

MIFARE Classic cards use a proprietary 3-pass mutual authentication protocol based on the Crypto1 stream cipher. The goal of this protocol is to verify that both the reader and the tag possess a shared secret key (Key A or Key B) before granting access to specific memory sectors.

The authentication process follows these steps:

![Error](Images/MAuth1.png)

1. **Reader → Tag**:  
   The reader sends an authentication command:
   - `0x60` for Key A or `0x61` for Key B
   - Target block address
   - CRC

2. **Tag → Reader**:  
   The tag replies with a 32-bit random number (`R_B`), acting as its challenge.

![Error](Images/MAuth2.png)

3. **Reader → Tag**:  
   The reader computes an 8-byte response:
   - Encrypts the tag's challenge `R_B` with the shared secret key.
   - Appends its own 32-bit random number `R_A`, rotated left by 1 bit.
   - Sends `E_K(R_B || R_A')` back to the tag.

4. **Tag → Reader**:  
   The tag responds with `E_K(R_A'')`, which is the reader's nonce rotated again and encrypted.
   - The reader verifies this to confirm mutual authentication.

After successful authentication, a secure session is established. All subsequent communication, including read and write operations, is encrypted using the active Crypto1 cipher stream.

In summary:
- **Reader initiates** the authentication and proves knowledge of the key.
- **Tag responds** with challenges and verifies the reader’s knowledge.
- Only after mutual verification can memory operations proceed.

### PN532 Commands for Authentication

The PN532 communicates with MIFARE Classic cards via commands defined by the NFC standard and its own instruction set.

The typical authentication sequence involves:

1. **InListPassiveTarget (0x4A)**  
   - Initializes communication and selects the target tag.
   - Returns a Target Number used for further commands.

2. **InDataExchange (0x40)**  
   - Used to send a MIFARE Classic authentication request.
   - Frame format:
     - `0x60` or `0x61` (Key A or B)
     - Block address
     - Key bytes (6 bytes, already stored on PN532)
     - UID (4 bytes)
   - PN532 handles the 3-pass exchange internally and reports success/failure.

For advanced control, **InCommunicateThru (0x42)** can be used to manually send raw frames, effectively turning the PN532 into a transparent transceiver.

To fully control the bit-level behavior of the PN532, several configuration steps are required:

- **Disable CRC generation/checking**:  
  - Clear `TxCRCEn` in `CIU_TxMode` register.
  - Clear `RxCRCEn` in `CIU_RxMode` register.

- **Disable Parity handling**:  
  - Set `ParityDisable` in `CIU_ManualRCV` register.

- **Control partial byte transmission**:  
  - Adjust `TxLastBits` in `CIU_BitFramingReg` to specify how many valid bits are in the final byte (e.g., for non-byte-aligned commands).

These settings are applied using the **WriteRegister** command before issuing `InCommunicateThru`. This allows precise control over the PN532's RF interface, essential for manual 3-pass authentication experiments, protocol fuzzing, or cryptanalysis.


## Usage Examples

### Reading a Block After Authentication

Once a MIFARE Classic sector is successfully authenticated (via InDataExchange or manual methods), the PN532 can read a specific block using InDataExchange.

The typical read command frame is:

- `0x40` → InDataExchange command
- Target Number from InListPassiveTarget
- `0x30` → MIFARE Read command
- Block Address (0x00 - 0x3F)

Example:

```c
uint8_t read_block_cmd[] = {
    0x40,                    // InDataExchange
    tag_info.target_number,  // Target number
    0x30,                    // MIFARE Read command
    0x08                     // Block address (example: block 8)
};

PN532_sendRawPrint(read_block_cmd, sizeof(read_block_cmd));
```

The PN532 will respond with 16 bytes of block data if successful.

### Writing to a Block After Authentication

Writing to a block requires sending a MIFARE Write command followed by 16 bytes of data to write.

The typical write command frame is:

- `0x40` → InDataExchange
- Target Number from InListPassiveTarget
- `0xA0` → MIFARE Write command
- Block Address

Example:

```c
uint8_t write_block_cmd[] = {
    0x40,                    // InDataExchange
    tag_info.target_number,  // Target number
    0xA0,                    // MIFARE Write command
    0x08                     // Block address (example: block 8)
};

PN532_sendRawPrint(write_block_cmd, sizeof(write_block_cmd));
```

After sending the write command, the PN532 expects a second frame with the 16 bytes of data:

```c
uint8_t write_data_cmd[] = {
    0x40,                    // InDataExchange
    tag_info.target_number,  // Target number
    // 16 bytes of data to write
    0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02, 0x03,
    0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B
};

PN532_sendRawPrint(write_data_cmd, sizeof(write_data_cmd));
```

If the operation is successful, the PN532 will respond with a status OK response.

### Important Notes

- The block address must be within an authenticated sector.
- Sector trailer blocks (the last block in each sector) should be handled carefully as they contain key data and access bits.
- Authentication must be redone if the card or sector changes.

### Summary Flow

1. Authenticate sector with InDataExchange or manual 3-pass method.
2. Issue Read or Write commands using InDataExchange.
3. For writes, follow up with a 16-byte data frame.
4. Verify PN532 responses for success/failure.



By disabling automated CRC, parity, and adjusting bit framing, the PN532 becomes a transparent ISO14443A transceiver, giving full control of low-level data exchanges to the microcontroller.

### Usage of Low Level Commands

This project provides low-level functions to directly interface with the PN532 registers and send raw data frames for precise control during authentication experiments or protocol analysis.

#### Setting and Modifying Registers

Registers can be read, written, and modified using helper functions:

**Reading a Register:**

```c
uint8_t rxmode_val;
if (read_registerGeneric(0x63, 0x03, &rxmode_val)) {
    printf("CIU_RxMode (0x6303): 0x%02X\n", rxmode_val);
}
```

**Writing to a Register:**

```c
write_registerGeneric(0x63, 0x02, 0x00);  // Disable TxCRCEn in CIU_TxMode
```

**Modifying Specific Bits:**

```c
modify_registerGeneric(0x63, 0x0D, (1 << 4), (1 << 4));  // Set ParityDisable in CIU_ManualRCV
modify_registerGeneric(0x63, 0x3D, 0x07, 0x05);          // Set TxLastBits to 5 bits in CIU_BitFramingReg
```

These allow precise control over PN532's link layer behavior.

#### Sending Raw Commands: sendRawPrint()

`PN532_sendRawPrint()` sends a command body (excluding TFI), waits for ACK, and prints the raw response bytes.

**Example: Sending REQA Command via InCommunicateThru**

```c
uint8_t reqa_cmd[] = { 0x42, 0x26 };  // InCommunicateThru + REQA

PN532_sendRawPrint(reqa_cmd, sizeof(reqa_cmd));
```

This function prints the full PN532 response, useful for debugging low-level exchanges.

#### Sending Raw Commands and Receiving Parsed Payload: sendRawGet()

`PN532_sendRawGet()` sends a command, waits for ACK, and returns the response payload (excluding protocol headers) in a user-provided buffer.

**Example: Reading CIU_Status1 Register**

```c
uint8_t readreg_cmd[] = { 0x06, 0x63, 0x3F };  // ReadRegister for CIU_Status1
uint8_t payload[16];
uint8_t payload_len = 0;

if (PN532_sendRawGet(readreg_cmd, sizeof(readreg_cmd), payload, &payload_len)) {
    printf("CIU_Status1 (0x633F): 0x%02X\n", payload[0]);
}
```

#### Typical Low-Level Workflow
1. Configure registers as needed (CRC disable, parity disable, TxLastBits).
2. Use `PN532_sendRawPrint()` for exploratory communication with full debug output.
3. Use `PN532_sendRawGet()` when you need to extract and process specific response payloads.
4. After low-level experiments, restore defaults with:
   
```c
restore_link_layer_defaults();
```

#### Key Points
- All register addresses follow the 0x63xx space for CIU.
- Read/write/modify helpers abstract the frame building for ReadRegister/WriteRegister commands.
- `sendRawPrint` is for quick testing with printed results.
- `sendRawGet` is for structured data extraction.
- 


