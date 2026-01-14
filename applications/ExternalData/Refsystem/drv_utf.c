#include "drv_utf.h"

// 计算UTF-8字符的字节长度
static uint8_t utf8_char_length(uint8_t ch)
{
    if (ch < 0x80)
        return 1;
    if ((ch >> 5) == 0x06)
        return 2;
    if ((ch >> 4) == 0x0E)
        return 3;
    if ((ch >> 3) == 0x1E)
        return 4;
    return 0; // 无效字符
}

// Function to calculate the length of a UTF-16 string
rt_size_t utf16_strnlen(const uint16_t *str, rt_size_t maxlen)
{
    const uint16_t *s = str;
    while (*s && s - str < maxlen)
    {
        s++;
    }
    // return s - str;//返回uint16_t的长度
    return ((uint8_t *)s - (uint8_t *)str); // 返回字节长度
}

// 转换UTF-8字符串到UTF-16
uint16_t *utf8_to_utf16(const uint8_t *utf8str, uint16_t *utf16buf, uint32_t bufsize)
{
    uint32_t utf16_idx = 0;
    while (*utf8str && utf16_idx < bufsize - 1)
    {
        uint8_t ch = (uint8_t)*utf8str;
        uint8_t len = utf8_char_length(ch);
        uint32_t codepoint = 0;

        if (len == 1)
        {
            codepoint = ch;
        }
        else if (len == 2)
        {
            codepoint = ((utf8str[0] & 0x1F) << 6) | (utf8str[1] & 0x3F);
        }
        else if (len == 3)
        {
            codepoint = ((utf8str[0] & 0x0F) << 12) | ((utf8str[1] & 0x3F) << 6) | (utf8str[2] & 0x3F);
        }
        else if (len == 4)
        {
            codepoint = ((utf8str[0] & 0x07) << 18) | ((utf8str[1] & 0x3F) << 12) | ((utf8str[2] & 0x3F) << 6) | (utf8str[3] & 0x3F);
        }

        if (codepoint <= 0xFFFF)
        {
            utf16buf[utf16_idx++] = (uint16_t)codepoint;
        }
        else
        {
            codepoint -= 0x10000;
            utf16buf[utf16_idx++] = (uint16_t)((codepoint >> 10) + 0xD800);
            if (utf16_idx < bufsize - 1)
            {
                utf16buf[utf16_idx++] = (uint16_t)((codepoint & 0x3FF) + 0xDC00);
            }
        }

        utf8str += len;
    }

    utf16buf[utf16_idx] = 0; // 终止符
    return utf16buf;
}

// Function to convert a float to a pseudo-UTF-16 string
uint8_t float_to_utf16(float num, uint8_t decimal_places, uint16_t *output, uint16_t output_size)
{
    // Check for valid decimal places
    if (output == RT_NULL || output_size == 0)
    {
        return 0; // Failure
    }

    // Temporary buffer to hold the formatted string
    uint8_t temp[100];

    // Format the float to a string with the specified number of decimal places
    snprintf((void *)temp, sizeof(temp), "%.*f", decimal_places, num);

    // Convert ASCII string to UTF-16 by converting each char to unsigned short
    uint16_t i;
    for (i = 0; temp[i] != '\0' && i < output_size - 1; ++i)
    {
        output[i] = (uint8_t)temp[i];
    }

    // Ensure null termination of the UTF-16 string
    if (i >= output_size - 1)
    {
        return 0; // Failure due to insufficient output buffer size
    }

    output[i] = 0;
    return 1; // Success
}
