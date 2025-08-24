#include "main.h"
#include "controller.h"
#include "usb_device.h"     // makes hUsbDeviceFS visible in most Cube projects
#include "usbd_cdc_if.h"    // for CDC_Transmit_FS
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern DAC_HandleTypeDef hdac1;   // DAC handle from main.c
extern ADC_HandleTypeDef hadc1;
extern uint16_t dac_buffer[];     // your DMA buffer
extern uint16_t adc_buffer[];     // your DMA buffer
#ifndef DAC_MAX
#define DAC_MAX 4095
#endif

extern USBD_HandleTypeDef hUsbDeviceFS;
// ==== bridge to your globals from main.c ====
extern float current_frequency;
extern volatile uint8_t sweeping;
extern volatile uint8_t sweep_done;

// If you want to explicitly gate DAC output by ON/OFF, wrap here.
// Currently, your DAC/ADC start at boot; we keep that behavior, but
// still track “out_enabled” so higher layers know the state.

static void ctrl_init(Controller* self) {
    self->out_freq_hz = current_frequency;
    self->out_enabled = true;
    self->pid_enabled = false;
    self->quad_stream_enabled = false;
    self->sweep = (SweepState){0};
}

static void ctrl_set_freq(Controller* self, float f_hz) {
    if (f_hz <= 0.0f) return;
    self->out_freq_hz = f_hz;
    current_frequency  = f_hz;
    set_dds_frequency(f_hz);
    fill_buffer(0, DDS_BUFFER_SIZE);
}

static void ctrl_set_output(Controller* self, bool on) {
    self->out_enabled = on;

    if (on) {


        // Try to restart DMA: stop first to be safe
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        HAL_ADC_Stop_DMA(&hadc1);
        // Reprogram frequency to current setting (safe) and restart DAC DMA
        set_dds_frequency(self->out_freq_hz);
        fill_buffer(0, DDS_BUFFER_SIZE);
        // small delay to let hardware settle (optional)
        // HAL_Delay(1);
        // 4) Start both DMAs (CALL THEM SEPARATELY!)
        HAL_StatusTypeDef st_dac = HAL_DAC_Start_DMA(
            &hdac1, DAC_CHANNEL_1, (uint32_t*)dac_buffer, DDS_BUFFER_SIZE, DAC_ALIGN_12B_R);

        HAL_StatusTypeDef st_adc = HAL_ADC_Start_DMA(
            &hadc1, (uint32_t*)adc_buffer, DDS_BUFFER_SIZE);

        // Restart DAC DMA with same buffer - ensure the buffer pointer/size are valid
        if (st_dac != HAL_OK || st_adc != HAL_OK)
         {

        	printf("ERR OUT ON: DAC DMA start failed\r\n");
            return;
        }
        printf("OK OUT ON %.1f\r\n", (double)self->out_freq_hz);
    } else {
        // Stop the DMA feeding the DAC
        if (HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1) != HAL_OK && HAL_ADC_Stop_DMA(&hadc1)!= HAL_OK) {
            // still continue; report error
            printf("WARN OUT OFF: DAC DMA stop failed\r\n");
        }

        // Start DAC in non-DMA mode and set midscale (so it outputs a defined DC)
        if (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) {
            printf("ERR OUT OFF: DAC start failed\r\n");
        } else {
            // Set midscale (silence). use DAC_ALIGN_12B_R and half-scale
            HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_MAX / 2);
        }
        printf("OK OUT OFF\r\n");
    }
}

static void ctrl_pid_enable(Controller* self, bool on) {
    self->pid_enabled = on;
    printf("OK PID %s\r\n", on ? "ON" : "OFF");
}

static void ctrl_quad_stream(Controller* self, bool on) {
    self->quad_stream_enabled = on;
    printf("OK QUAD STREAM %s\r\n", on ? "ON" : "OFF");
}

static void ctrl_quad_once(Controller* self) {
    // Your ADC callback computes X,Y & sets usb_ready_to_transmit.
    // Here we do a one-shot immediate send using current X,Y.
    Send_XY_Over_USB_Raw(X, Y);
    printf("OK QUAD X=%.3f Y=%.3f\r\n", (double)X, (double)Y);
}

static void ctrl_sweep_start(Controller* self, float f_start, float f_end, float f_step) {
    if (!(f_step > 0 && f_end >= f_start)) {
        printf("ERR SWEEP args\r\n");
        return;
    }
    self->sweep = (SweepState){ .f_start=f_start, .f_end=f_end, .f_step=f_step, .active=true, .next_f=f_start };
    sweeping   = 1;
    sweep_done = 0;
    self->set_freq(self, f_start);
    printf("OK SWEEP %.1f %.1f %.1f\r\n", (double)f_start, (double)f_end, (double)f_step);
}

static void ctrl_sweep_tick(Controller* self) {
    if (!self->sweep.active) return;

    // Only advance when an ADC buffer has completed and requested a step.
    // Use a local copy to avoid races; sweep_step_request is volatile.
    if (!sweep_step_request) return;

    // consume the request (clear it) before doing longer ops
    sweep_step_request = false;

    // compute next frequency
    float next = self->sweep.next_f + self->sweep.f_step;
    current_frequency  = next;
    set_dds_frequency(next);
    // guard against floating point accumulation error
    if (next > self->sweep.f_end + 1e-9f) {
        // finished
        self->sweep.active = false;
        sweeping   = 0;
        sweep_done = 1;
        printf("OK SWEEP DONE\r\n");
        return;
    }

    // apply new frequency
    self->set_freq(self, next);
    self->sweep.next_f = next;

    // notify host of new freq (optional)
    printf("F=%.1f\r\n", (double)next);
}


static void ctrl_sweep_stop(Controller* self) {
    self->sweep.active = false;
    sweeping   = 0;
    sweep_done = 1;
    printf("OK SWEEP STOP\r\n");
}

// ====== Public instance ======
Controller g_controller = {
    .init         = ctrl_init,
    .set_freq     = ctrl_set_freq,
    .set_output   = ctrl_set_output,
    .pid_enable   = ctrl_pid_enable,
    .quad_stream  = ctrl_quad_stream,
    .quad_once    = ctrl_quad_once,
    .sweep_start  = ctrl_sweep_start,
    .sweep_tick   = ctrl_sweep_tick,
    .sweep_stop   = ctrl_sweep_stop,
};


int _write(int file, char *ptr, int len) {
    // Send over USB CDC
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        CDC_Transmit_FS((uint8_t*)ptr, len);
    }
    return len;
}
