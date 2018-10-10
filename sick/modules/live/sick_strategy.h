#ifndef _SICK_STRATEGY_H_
#define _SICK_STRATEGY_H_

#include <stdint.h>

#define SICK_STRATEGY_STATE_INITIAL 0
#define SICK_STRATEGY_STATE_BLOCKED 1
#define SICK_STRATEGY_STATE_MOVING_TO_CART 2
#define SICK_STRATEGY_STATE_WAITING_CART 3
#define SICK_STRATEGY_STATE_ALIGN 4
#define SICK_STRATEGY_STATE_GRABBING 5
#define SICK_STRATEGY_STATE_RETURNING 6
#define SICK_STRATEGY_STATE_RELEASING 7

typedef struct {
    uint8_t current;
    uint8_t old;
} sick_strategy_t;

typedef void (*sick_strategy_receive_data_callback)(sick_strategy_t *state);

void init_sick_strategy();
void shutdown_sick_strategy();

void register_sick_strategy_callback(sick_strategy_receive_data_callback callback);
void unregister_sick_strategy_callback(sick_strategy_receive_data_callback callback);

#endif
