#pragma once
#include <stdint.h>

#define RMS_WINDOW_SAMPLES 40  /* 200 ms at 200 Hz */

typedef struct {
    float    buf[RMS_WINDOW_SAMPLES];
    uint16_t head;
    uint16_t count;  /* valid samples, max = RMS_WINDOW_SAMPLES */
} RingBuf;

void  ringbuf_init(RingBuf *rb);
void  ringbuf_push(RingBuf *rb, float val);
float ringbuf_rms(const RingBuf *rb);
