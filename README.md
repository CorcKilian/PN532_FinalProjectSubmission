## Introduction

This repository expands on the [PN532-STM32-SecuritySystem](https://github.com/CorcKilian/PN532-STM32-SecuritySystem) project, focusing on low-level NFC communication and authentication protocols.

The primary objective of this project is to manually implement the **MIFARE Classic 3-pass authentication protocol** using an **STM32 microcontroller** interfaced with a **PN532 NFC module** over SPI. Unlike typical high-level library abstractions, this project delves into the internal mechanics of the authentication process, providing a clear, step-by-step approach to understanding how MIFARE Classic cards perform mutual authentication.

After successful authentication, the system performs secure **read and write operations to specific memory blocks** on the MIFARE card. This enables practical applications such as access control, secure data storage, and card-based identification systems.

The project emphasizes clean code structure, modular design, and detailed documentation to facilitate learning and ease of maintenance.
