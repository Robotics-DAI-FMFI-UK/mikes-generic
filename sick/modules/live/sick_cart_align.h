#ifndef _SICK_CART_ALIGN_H_
#define _SICK_CART_ALIGN_H_

#define SICK_CART_ALIGN_SUCCESS      0
#define SICK_CART_ALIGN_FAIL         1
#define SICK_CART_ALIGN_TIMEOUT      2

typedef struct {
    int status;
} sick_cart_align_t;

typedef void (*sick_cart_align_receive_data_callback)(sick_cart_align_t *result);

void init_sick_cart_align();
void shutdown_sick_cart_align();

void align_robot_to_cart(long timeout);

void register_sick_cart_align_callback(sick_cart_align_receive_data_callback callback);
void unregister_sick_cart_align_callback(sick_cart_align_receive_data_callback callback);

#endif
