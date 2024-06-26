/**
 * @file FastIO.hpp
 * @brief fast IO macros
 */
#ifndef FASTIO_HPP__
#define FASTIO_HPP__

#include "Arduino.h"

// accurate delay functions
#define delay25ns __asm__ __volatile__("nop;nop;nop;nop;nop;nop;")
#define delay50ns __asm__ __volatile__("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;")
#define delay100ns __asm__ __volatile__("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;nop;")
#define delay200ns \
    delay100ns;    \
    delay100ns
#define delay500ns \
    delay100ns;    \
    delay100ns;    \
    delay100ns;    \
    delay100ns;    \
    delay100ns
#define delay1us \
    delay500ns;  \
    delay500ns

// fast switching of output
// set and unset
#define PRIMITIVE_CAT(x, y) x##y
#define setbit(b) (GPIO.out_w1ts |= PRIMITIVE_CAT(BIT, b))
#define clrbit(b) (GPIO.out_w1tc |= PRIMITIVE_CAT(BIT, b))

// fast read
#define fastread(b) ((PRIMITIVE_CAT(BIT, b) & GPIO.in) ? 1 : 0)

// setbit + coarse delay
#define delaylow100ns(b) \
    clrbit(b);           \
    clrbit(b);
#define delayhigh100ns(b) \
    setbit(b);            \
    setbit(b);

#define delaylow500ns(b) \
    delaylow100ns(b);    \
    delaylow100ns(b);    \
    delaylow100ns(b);    \
    delaylow100ns(b);    \
    delaylow100ns(b);
#define delayhigh500ns(b) \
    delayhigh100ns(b);    \
    delayhigh100ns(b);    \
    delayhigh100ns(b);    \
    delayhigh100ns(b);    \
    delayhigh100ns(b);

#define delaylow1us(b) \
    delaylow500ns(b);  \
    delaylow500ns(b);
#define delayhigh1us(b) \
    delayhigh500ns(b);  \
    delayhigh500ns(b);

#endif