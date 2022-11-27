#pragma once

#include "common/lite_pipe.h"
#include "common/radio.h"

typedef struct {
  LitePipe *pipes[RADIO_NUM_PIPES];
} RadioRxConfig;

void RadioRxThread(const RadioRxConfig *config, void *arg2, void *arg3);
bool RadioRxGetErrorStatus(void);