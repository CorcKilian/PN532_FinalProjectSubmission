#ifndef KEY_LIBRARY_H
#define KEY_LIBRARY_H

#include <stdint.h>
#include <stdbool.h>

/// Lookup a key by UID, sector number, and key type ('A' or 'B').
/// @param uid      4‑byte card UID
/// @param sector   sector index (0–15 for a 1 K MIFARE Classic)
/// @param type     'A' or 'B'
/// @param key_out  6‑byte buffer to receive the key
/// @return true if we had a matching entry and that sector/key is non‑zero
bool get_known_key(const uint8_t uid[4],
                   uint8_t       sector,
                   char          type,
                   uint8_t       key_out[6]);

#endif // KEY_LIBRARY_H
