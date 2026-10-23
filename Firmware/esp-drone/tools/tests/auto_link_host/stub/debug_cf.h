#pragma once
#include <stdio.h>
#define DEBUG_PRINTI(fmt, ...) printf("I " fmt "\n", ##__VA_ARGS__)
#define DEBUG_PRINTW(fmt, ...) printf("W " fmt "\n", ##__VA_ARGS__)
