#include "morse_utils.h"
#include "morse_map.h"
#include <string.h>

char morse_to_char(const char* code) {
    for (int i = 0; i < morse_table_size; ++i) {
        if (strcmp(code, morse_table[i].code) == 0) {
            return morse_table[i].letter;
        }
    }
    return '?';
}

// Function 2
char* morse_to_text(const char* morse_sentence) {
    char* buffer = strdup(morse_sentence);
    char* token = strtok(buffer, " ");
    
    char* result = malloc(101); //I'm assuming max 100 characters
    int pos = 0;

    while (token != NULL) {
        char letter = morse_to_char(token);
        result[pos++] = letter;
        token = strtok(NULL, " ");
    }

    result[pos] = '\0';
    free(buffer);
    return result;
}