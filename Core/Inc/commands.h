#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Process a single ASCII command (one line, no trailing \n required).
void Commands_ProcessLine(const char *line);

// Optional: simple line buffer to assemble from CDC_Receive_FS()
typedef struct {
    char buf[128];
    uint16_t len;
} CmdLineBuffer;

void Commands_Reset(CmdLineBuffer *lb);
bool Commands_Push(CmdLineBuffer *lb, const uint8_t *data, uint32_t n); // returns true if a full line is ready
const char* Commands_GetLine(CmdLineBuffer *lb); // valid until next reset

#ifdef __cplusplus
}
#endif
