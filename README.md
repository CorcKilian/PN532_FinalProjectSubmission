## Introduction

This repository expands on the [PN532-STM32-SecuritySystem](https://github.com/CorcKilian/PN532-STM32-SecuritySystem) project, focusing on low-level NFC communication and authentication protocols.

The primary objective of this project is to explore the implementation of the **MIFARE Classic 3-pass authentication protocol** using an **STM32 microcontroller** interfaced with a **PN532 NFC module** over SPI. Unlike typical high-level library abstractions, this project delves into the internal mechanics of the authentication process, providing a clear, step-by-step approach to understanding how MIFARE Classic cards perform mutual authentication, as well as partially performing a low level 3 pass Authentification sequence.

After successful high level authentication, the system performs secure **read and write operations to specific memory blocks** on the MIFARE card. This enables practical applications such as access control, secure data storage, and card-based identification systems.

## Theory

### MIFARE Classic 3-Pass Authentication

MIFARE Classic cards use a proprietary 3-pass mutual authentication protocol based on the Crypto1 stream cipher. The goal of this protocol is to verify that both the reader and the tag possess a shared secret key (Key A or Key B) before granting access to specific memory sectors.

The authentication process follows these steps:

1. **Reader → Tag**:  
   The reader sends an authentication command:
   - `0x60` for Key A or `0x61` for Key B
   - Target block address
   - CRC

2. **Tag → Reader**:  
   The tag replies with a 32-bit random number (`R_B`), acting as its challenge.

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

By disabling automated CRC, parity, and adjusting bit framing, the PN532 becomes a transparent ISO14443A transceiver, giving full control of low-level data exchanges to the microcontroller.

