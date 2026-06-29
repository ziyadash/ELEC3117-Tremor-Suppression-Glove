#include "ring_buffer.h"
#include <math.h>
#include <string.h>

void ringbuf_init(RingBuf *rb)
{
    memset(rb->buf, 0, sizeof(rb->buf));
    rb->head  = 0;
    rb->count = 0;
}

void ringbuf_push(RingBuf *rb, float val)
{
    rb->buf[rb->head] = val;
    rb->head = (rb->head + 1u) % RMS_WINDOW_SAMPLES;
    if (rb->count < RMS_WINDOW_SAMPLES)
        rb->count++;
}

float ringbuf_rms(const RingBuf *rb)
{
    if (rb->count == 0)
        return 0.0f;

    float sum = 0.0f;
    for (uint16_t i = 0; i < rb->count; i++)
        sum += rb->buf[i] * rb->buf[i];

    return sqrtf(sum / (float)rb->count);
}
