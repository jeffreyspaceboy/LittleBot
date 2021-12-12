#include <stdio.h>
#include <ncurses.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>

#define MAX_KEY_DELTA_US 500000ULL
uint64_t get_us() {
    struct timeval tv;
    uint64_t cur_us;
    gettimeofday(&tv,NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}
typedef enum keyeevent_t {
    eKEY_DOWN,
    eKEY_UP,
    eKEY_REPEAT,
    eKEY_NOKEY,
}keyeevent_t;

typedef struct keystate_t {
    uint64_t last_us;
    uint64_t delta_us;
    int last_ch;
}keystate_t;

keyeevent_t check_key(keystate_t *kstate, int ch) {
    kstate->delta_us = get_us() - kstate->last_us;
    if (ch == -1) {
        if ((kstate->delta_us > MAX_KEY_DELTA_US) && (kstate->last_ch != 0)) {
            kstate->last_ch = 0;
            return eKEY_UP;
        }
        return eKEY_NOKEY;
    }
    kstate->last_us = get_us();
    if ((ch >= 0) && (ch != kstate->last_ch)) {
        kstate->last_ch = ch;
        return eKEY_DOWN;
    }
    return eKEY_REPEAT;
}
int main() {
    keystate_t keystate = {0,0,0};
    WINDOW *win = initscr();
    keypad(stdscr, TRUE);
    noecho();
    nodelay(win, TRUE);

    refresh();
    while (1) {
        int ch = getch();
        keyeevent_t evt = check_key(&keystate,ch);
        switch(evt) {
            case eKEY_DOWN:
                printw("Key[%llu] 0x%02x down\n",keystate.last_us,(uint8_t)ch);
                break;
            case eKEY_UP:
                printw("Key[%llu] 0x%02x up\n",keystate.delta_us,(uint8_t)ch);
                break;
            case eKEY_REPEAT:
                printw("Key[%llu] 0x%02x repeat\n",keystate.last_us,(uint8_t)ch);
                break;
            case eKEY_NOKEY:
                break;
        }
        usleep(10000);
    }
    getch();
    endwin();
    return 0;
}
