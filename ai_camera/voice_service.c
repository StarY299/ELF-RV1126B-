#include <stdio.h>
#include <stdlib.h>
#include "voice_service.h"

int play_voice(const char *voice_file)
{
    char cmd[512];

    if (voice_file == NULL) {
        printf("[VOICE] voice_file is NULL\n");
        return -1;
    }

    snprintf(cmd, sizeof(cmd), "aplay -q %s 2>/dev/null", voice_file);
    printf("[VOICE] ▶ %s\n", voice_file);

    return system(cmd);
}
