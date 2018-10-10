#ifndef _SICK_STRATEGY_H_
#define _SICK_STRATEGY_H_

#include <stdint.h>

typedef struct {
    uint8_t state;
} sick_strategy_t;

typedef void (*sick_strategy_receive_data_callback)(sick_strategy_t *state);

void init_sick_strategy();
void shutdown_sick_strategy();

void register_sick_strategy_callback(sick_strategy_receive_data_callback callback);
void unregister_sick_strategy_callback(sick_strategy_receive_data_callback callback);

#endif
