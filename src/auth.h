#ifndef AUTH_H
#define AUTH_H

#include <stdint.h>
#include <stdbool.h>
#include "PN532.h"

#define MAX_UIDS     16
#define UID_MAX_LEN  10

typedef struct {
    uint8_t uid[UID_MAX_LEN];
    uint8_t len;
} UID;

bool add_uid(const PN532_TagInfo *tag);
bool is_uid_authorized(const PN532_TagInfo *tag);
bool remove_uid(const PN532_TagInfo *tag);



#endif  // AUTH_H
