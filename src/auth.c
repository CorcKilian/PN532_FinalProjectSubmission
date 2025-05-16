#include "auth.h"
#include <string.h>  // for memcmp, memcpy
#include "eeng1030_lib.h"
#include <stdio.h>
#include "PN532.h"


static UID whitelist[MAX_UIDS];
static uint8_t whitelist_count = 0;

/**
 * Adds a UID to the whitelist if it's not already present.
 *
 * @param tag Pointer to PN532_TagInfo containing UID and its length.
 * @return true if the UID was added, false if duplicate or error.
 */
bool add_uid(const PN532_TagInfo *tag) {
    if (whitelist_count >= MAX_UIDS) {
        printf("⚠️ Whitelist full, cannot add UID.\r\n");
        return false;
    }

    if (tag->uid_len > UID_MAX_LEN) {
        printf("⚠️ UID length (%u bytes) exceeds maximum allowed (%u bytes).\r\n",
               tag->uid_len, UID_MAX_LEN);
        return false;
    }

    for (uint8_t i = 0; i < whitelist_count; i++) {
        if (tag->uid_len == whitelist[i].len &&
            memcmp(tag->uid, whitelist[i].uid, tag->uid_len) == 0) {
            printf("⚠️ UID already exists in whitelist.\r\n");
            return false;  // Already exists
        }
    }

    whitelist[whitelist_count].len = tag->uid_len;
    memcpy(whitelist[whitelist_count].uid, tag->uid, tag->uid_len);
    whitelist_count++;
    return true;
}

/**
 * Checks if the UID is present in the whitelist.
 *
 * @param tag Pointer to PN532_TagInfo with UID and its length.
 * @return true if authorized, false otherwise.
 */
bool is_uid_authorized(const PN532_TagInfo *tag) {
    if (tag->uid_len > UID_MAX_LEN) {
        printf("⚠️ UID length (%u bytes) invalid for authorization.\r\n",
               tag->uid_len);
        return false;
    }

    for (uint8_t i = 0; i < whitelist_count; i++) {
        if (tag->uid_len == whitelist[i].len &&
            memcmp(tag->uid, whitelist[i].uid, tag->uid_len) == 0) {
            return true;
        }
    }
    return false;
}

/**
 * Removes a UID from the whitelist if it exists.
 *
 * @param tag Pointer to PN532_TagInfo with UID and its length.
 * @return true if removed, false if not found.
 */
bool remove_uid(const PN532_TagInfo *tag) {
    if (tag->uid_len > UID_MAX_LEN) {
        printf("⚠️ UID length (%u bytes) invalid for removal.\r\n",
               tag->uid_len);
        return false;
    }

    for (uint8_t i = 0; i < whitelist_count; i++) {
        if (tag->uid_len == whitelist[i].len &&
            memcmp(tag->uid, whitelist[i].uid, tag->uid_len) == 0) {
            if (i < whitelist_count - 1) {
                memmove(&whitelist[i], &whitelist[i + 1],
                        sizeof(UID) * (whitelist_count - i - 1));
            }
            whitelist_count--;
            return true;
        }
    }
    printf("⚠️ UID not found, cannot remove.\r\n");
    return false;
}
