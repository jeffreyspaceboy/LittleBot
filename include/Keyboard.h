/*---KEYBOARD_H---*/
#ifndef KEYBOARD_H
#define KEYBOARD_H
/*----------------------------------------------------------------------------*/
/*    Module:       Keyboard.h                                                */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-22                                                */
/*----------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


/* LOCAL INCLUDES */
#include "Definitions.h"

/* STANDARD INCLUDES */
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

typedef struct keyfunc_t {
    int key; 
    void (*func)();
}keyfunc_t;

typedef struct keystate_t {
    uint64_t last_us, delta_us;
    int last_ch;
}keystate_t;

typedef struct Keyboard_t {
    bool enabled;
    keyfunc_t key_functions[KEY_MAX_COUNT];
    int key_functions_count;
    void (*default_func)();
    keystate_t key_state;
    pthread_t thread;
    pthread_mutex_t mutex;
}Keyboard_t;

typedef enum keyeevent_t {
    eKEY_DOWN,
    eKEY_UP,
    eKEY_REPEAT,
    eKEY_NOKEY,
}keyeevent_t;

Keyboard_t keyboard_init(keyfunc_t key_functions[KEY_MAX_COUNT], int key_functions_count,  void (*default_func)());

int keyboard_create_thread(Keyboard_t *keyboard);

int keyboard_del(Keyboard_t *keyboard);

uint64_t get_us();

keyeevent_t check_key(keystate_t *kstate, int ch);

void *keyboard_control_thread(void *arg);

#ifdef __cplusplus
}
#endif
#endif
/*---KEYBOARD_H---*/