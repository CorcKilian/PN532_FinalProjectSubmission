#include "key_library.h"
#include <string.h>  // memcpy, memcmp

#define MAX_SECTORS 16

/// One card’s known keys for up to 16 sectors
typedef struct {
    uint8_t uid[4];
    uint8_t keyA[MAX_SECTORS][6];
    uint8_t keyB[MAX_SECTORS][6];
} CardKeys;

// ——————————————————————————————————————————————————————————
// /* <<< ADD NEW CARDS HERE >>> */
// For each new card, copy the *entire* block below, fill in .uid[], and
// supply any known KeyA or KeyB per sector.  Leave unknown sectors at {0}.
//——————————————————————————————————————————————————————————
static const CardKeys key_db[] = {
    {
      // example card #1
      .uid = { 0x04, 0xA2, 0x1C, 0xB7 },
      // KeyA for sectors 0 and 2; all other sectors stay {0}
      .keyA = {
         /* sector 0 */ { 0xA0,0xA1,0xA2,0xA3,0xA4,0xA5 },
         /* sector 1 */ { 0 },  // unknown
         /* sector 2 */ { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF },
         /* sectors 3–15 … */ 
      },
      // KeyB only for sector 0; others = {0}
      .keyB = {
         /* sector 0 */ { 0xD3,0xF7,0xD3,0xF7,0xD3,0xF7 },
         /* sectors 1–15 = {0} */
      }
    },

    {
      // example card #2
      .uid = { 0xDE,0xAD,0xBE,0xEF },
      .keyA = {
         /* sector 0 */ { 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF },
         /* sector 1 */ { 0x11,0x22,0x33,0x44,0x55,0x66 },
         /* … rest = {0} */
      },
      .keyB = {
         /* e.g. sector 3 */ [3] = { 0x99,0x88,0x77,0x66,0x55,0x44 },
         /* rest = {0} */
      }
    }

    // — add more CardKeys entries here —
};

static const size_t key_db_len = sizeof(key_db) / sizeof(key_db[0]);

bool get_known_key(const uint8_t uid[4],
                   uint8_t       sector,
                   char          type,
                   uint8_t       key_out[6])
{
    if (sector >= MAX_SECTORS) return false;

    for (size_t i = 0; i < key_db_len; i++) {
        if (memcmp(uid, key_db[i].uid, 4) == 0) {
            if (type == 'A') {
                // check non‑zero to avoid spuriously returning {0,0,0,0,0,0}
                if (key_db[i].keyA[sector][0] == 0) return false;
                memcpy(key_out, key_db[i].keyA[sector], 6);
                return true;
            } else if (type == 'B') {
                if (key_db[i].keyB[sector][0] == 0) return false;
                memcpy(key_out, key_db[i].keyB[sector], 6);
                return true;
            }
            return false;
        }
    }
    return false;  // no matching UID
}
