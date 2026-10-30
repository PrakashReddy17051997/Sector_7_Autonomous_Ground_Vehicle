#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232
extern uint32_t g_ms; inline uint32_t millis(){return g_ms;}
struct SerialT{ template<class...A> void printf(const char*f,A...a){::fprintf(stderr,f,a...);} void println(const char*s){fprintf(stderr,"%s\n",s);} };  // firmware messages -> stderr
extern SerialT Serial;
