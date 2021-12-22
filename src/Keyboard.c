/*---KEYBOARD_C---*/
/*----------------------------------------------------------------------------*/
/*    Module:       Keyboard.c                                                */
/*    Author:       Jeffrey Fisher II                                         */
/*    Created:      2021-12-22                                                */
/*----------------------------------------------------------------------------*/

/* LOCAL INCLUDES */
#include "Keyboard.h"

/* STANDARD INCLUDES */
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>

/* NON-STANDARD INCLUDES */
#include <ncurses.h>
#include <pigpio.h>

Keyboard_t keyboard_init(keyfunc_t key_functions[KEY_MAX_COUNT], int key_functions_count, void (*default_func)()){
    Keyboard_t new_keyboard = {
        .enabled = false,
        .key_functions_count = key_functions_count,
        .default_func = default_func,
    };
    if(pthread_mutex_init(&new_keyboard.mutex, NULL) != 0){
        printf("%s Keyboard mutex init failed.\n",ERROR_MSG);
        return new_keyboard;
    }
    for(int i = 0; i < key_functions_count; i++){
        new_keyboard.key_functions[i] = key_functions[0];
    }
    return new_keyboard;
}

int keyboard_create_thread(Keyboard_t *keyboard){
    pthread_mutex_lock(&keyboard->mutex);
    keyboard->enabled = true;
    pthread_mutex_unlock(&keyboard->mutex);
    pthread_create(&keyboard->thread, NULL, keyboard_control_thread, (void *)keyboard);
    return SUCCESS;
}

int keyboard_del(Keyboard_t *keyboard){ 
    pthread_mutex_lock(&keyboard->mutex);
    keyboard->enabled = false;
    pthread_mutex_unlock(&keyboard->mutex);
    pthread_join(keyboard->thread, NULL);
    pthread_mutex_destroy(&keyboard->mutex);
    return SUCCESS;
}

uint64_t get_us() {
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

keyeevent_t check_key(keystate_t *kstate, int ch){
    kstate->delta_us = get_us() - kstate->last_us;
    if(ch == -1){
        if((kstate->delta_us > MAX_KEY_DELTA_US) && (kstate->last_ch != 0)){
            kstate->last_ch = 0;
            return eKEY_UP;
        }
        return eKEY_NOKEY;
    }
    kstate->last_us = get_us();
    if((ch >= 0) && (ch != kstate->last_ch)){
        kstate->last_ch = ch;
        return eKEY_DOWN;
    }
    return eKEY_REPEAT;
}

void *keyboard_control_thread(void *arg){
    Keyboard_t *keyboard = (Keyboard_t *) arg;
    keystate_t keystate = { 0,0,0 };
    keyboard->key_state = keystate;
    WINDOW *win = initscr();
    keypad(stdscr, TRUE);
    noecho();
    nodelay(win, TRUE);
    refresh();
    while(keyboard->enabled){
        int ch = getch();
        pthread_mutex_lock(&keyboard->mutex);
        keyeevent_t evt = check_key(&keyboard->key_state,ch);
        pthread_mutex_unlock(&keyboard->mutex);
        switch(evt) {
            case eKEY_DOWN:
                printf("Key[%lu] 0x%02x down\r\n",keyboard->key_state.last_us,(uint8_t)ch);
                pthread_mutex_lock(&keyboard->mutex);
                for(int i = 0; i < keyboard->key_functions_count; i++){
                    if(ch == keyboard->key_functions[i].key){
                        (*keyboard->key_functions[i].func)();
                        break;
                    }
                }
                pthread_mutex_unlock(&keyboard->mutex);
                break;
            case eKEY_UP:
                printf("Key[%lu] 0x%02x up\r\n",keyboard->key_state.delta_us,(uint8_t)ch);
                pthread_mutex_lock(&keyboard->mutex);
                keyboard->default_func();
                pthread_mutex_unlock(&keyboard->mutex);
                break;
            case eKEY_REPEAT:
                printf("Key[%lu] 0x%02x repeat\r\n",keyboard->key_state.last_us,(uint8_t)ch);
                break;
            case eKEY_NOKEY:
                break;
        }
        gpioSleep(PI_TIME_RELATIVE, 0, 3000);
    }
    getch();
    endwin();
}
/*---KEYBOARD_C---*/