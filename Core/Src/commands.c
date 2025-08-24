#include "commands.h"
#include "controller.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>

extern Controller g_controller;

static int to_upper_inplace(char *s) {
    int i=0; for (; s[i]; ++i) s[i] = (char)toupper((unsigned char)s[i]); return i;
}

static char* next_tok(char **save) {
    char *t = strtok_r(NULL, " \t\r\n", save);
    return t;
}

static int parse_float(const char *t, float *out) {
    if (!t) return 0;
    char *end=0; double v=strtod(t, &end);
    if (end==t || (*end && !isspace((unsigned char)*end))) return 0;
    *out = (float)v; return 1;
}

void Commands_ProcessLine(const char *line) {
    // Work on a local mutable copy
    char tmp[128];
    size_t L = strlen(line);
    if (L >= sizeof(tmp)) { printf("ERR too long\r\n"); return; }
    memcpy(tmp, line, L+1);

    // Tokenize
    char *save=NULL;
    char *t0 = strtok_r(tmp, " \t\r\n", &save);
    if (!t0) { printf("ERR empty\r\n"); return; }
    to_upper_inplace(t0);

    if (strcmp(t0, "SWEEP")==0) {
        float f1,f2,fs;
        char *a=next_tok(&save), *b=next_tok(&save), *c=next_tok(&save);
        if (!parse_float(a,&f1)||!parse_float(b,&f2)||!parse_float(c,&fs) || fs<=0.0f || f2<f1) {
            printf("ERR SWEEP args\r\n"); return;
        }
        g_controller.sweep_start(&g_controller, f1,f2,fs);
        return;
    }

    if (strcmp(t0, "OUT")==0) {
        char *a=next_tok(&save); if (!a) { printf("ERR OUT arg\r\n"); return; }
        to_upper_inplace(a);
        if (strcmp(a,"ON")==0) {
            float f; char *b=next_tok(&save);
            if (!parse_float(b,&f) || f<=0.0f) { printf("ERR OUT ON freq\r\n"); return; }
            g_controller.set_freq(&g_controller, f);
            g_controller.set_output(&g_controller, true);
            printf("OK OUT ON %.6g\r\n", (double)f);
            return;
        } else if (strcmp(a,"OFF")==0) {
            g_controller.set_output(&g_controller, false);
            printf("OK OUT OFF\r\n");
            return;
        } else { printf("ERR OUT arg\r\n"); return; }
    }

    if (strcmp(t0, "QUAD")==0) {
        char *a=next_tok(&save);
        if (a) { to_upper_inplace(a); }
        if (a && strcmp(a,"STREAM")==0) {
            char *b=next_tok(&save); // ON|OFF
            if (!b) { printf("ERR QUAD STREAM arg\r\n"); return; }
            to_upper_inplace(b);
            if (strcmp(b,"ON")==0)       g_controller.quad_stream(&g_controller,true);
            else if (strcmp(b,"OFF")==0) g_controller.quad_stream(&g_controller,false);
            else { printf("ERR QUAD STREAM arg\r\n"); }
            return;
        }
        // default: one-shot
        g_controller.quad_once(&g_controller);
        return;
    }

    if (strcmp(t0, "PID")==0) {
        char *a=next_tok(&save); if (!a) { printf("ERR PID arg\r\n"); return; }
        to_upper_inplace(a);
        if      (strcmp(a,"ON")==0)  g_controller.pid_enable(&g_controller, true);
        else if (strcmp(a,"OFF")==0) g_controller.pid_enable(&g_controller, false);
        else printf("ERR PID arg\r\n");
        return;
    }

    if (strcmp(t0,"SWEEPSTOP")==0) { g_controller.sweep_stop(&g_controller); return; }

    printf("ERR unknown\r\n");
}

// ===== line buffer (optional but handy) =====
void Commands_Reset(CmdLineBuffer *lb) { lb->len = 0; lb->buf[0] = 0; }

bool Commands_Push(CmdLineBuffer *lb, const uint8_t *data, uint32_t n) {
    for (uint32_t i=0;i<n;i++) {
        if (lb->len+1 >= sizeof(lb->buf)) { // drop if overflow
            lb->len = 0; lb->buf[0]=0;
            continue;
        }
        char c = (char)data[i];
        if (c=='\r') continue;
        lb->buf[lb->len++] = c;
        if (c=='\n') { lb->buf[lb->len-1] = 0; return true; }
    }
    lb->buf[lb->len] = 0;
    return false;
}
const char* Commands_GetLine(CmdLineBuffer *lb) { return lb->buf; }
