#ifndef _DRV_UTF_H_
#define _DRV_UTF_H_

#include <stdint.h>
#include <rtthread.h>
#include <board.h>
#include <stdio.h>

rt_size_t utf16_strnlen(const uint16_t *str, rt_size_t maxlen);

uint16_t *utf8_to_utf16(const uint8_t *utf8str, uint16_t *utf16buf, uint32_t bufsize);
uint8_t float_to_utf16(float num, uint8_t decimal_places, uint16_t *output, uint16_t output_size);

#endif // _DRV_UTF_CHANGE_H_
