#ifndef MORSE_MAP_H
#define MORSE_MAP_H

typedef struct {
    const char* code;
    char letter;
} MorseMap;

extern MorseMap morse_table[];

#endif /* morse_map.h */
