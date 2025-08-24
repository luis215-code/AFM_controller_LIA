#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Controller Controller;

typedef struct {
    float f_start;
    float f_end;
    float f_step;
    bool  active;
    float next_f;
} SweepState;

struct Controller {
    // public “properties”
    float   out_freq_hz;
    bool    out_enabled;
    bool    pid_enabled;
    bool    quad_stream_enabled;

    SweepState sweep;

    // “methods”
    void (*init)(Controller* self);
    void (*set_freq)(Controller* self, float f_hz);
    void (*set_output)(Controller* self, bool on);
    void (*pid_enable)(Controller* self, bool on);
    void (*quad_stream)(Controller* self, bool on);
    void (*quad_once)(Controller* self);      // single X,Y send

    void (*sweep_start)(Controller* self, float f_start, float f_end, float f_step);
    void (*sweep_tick)(Controller* self);     // advance sweep non-blocking in main loop/ISR-safe
    void (*sweep_stop)(Controller* self);
};

extern Controller g_controller;
extern volatile bool sweep_step_request;

// Hooks you already have / provide elsewhere:
void set_dds_frequency(float freq_hz);         // your existing
void init_sine_lut(float amplitude);           // your existing
void Send_XY_Over_USB_Raw(float x, float y);   // your existing

// Tiny helpers to centralize USB text replies


// LIA data you update in callbacks (declared in main.c)
extern volatile bool usb_ready_to_transmit;
extern float X, Y;

#ifdef __cplusplus
}
#endif
