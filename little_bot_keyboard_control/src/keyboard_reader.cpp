/*---KEYBOARD_READER_CPP---*/
/*----------------------------------------------------------------------------*/
/*    Module:       keyboard_reader.cpp                                       */
/*    Author:       Jeffrey Fisher II                                         */
/*    Modified:     2022-02-23                                                */
/*----------------------------------------------------------------------------*/

/* HEADER INCLUDE */
#include "keyboard_reader.hpp"


KeyboardReader::KeyboardReader()
#ifndef _WIN32
    : kfd(0)
#endif
{
    #ifndef _WIN32
        // get the console in raw mode
        tcgetattr(kfd, &cooked);
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);
        // Setting a new line, then end of file
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
    #endif
}

void KeyboardReader::readOne(char * c){
    #ifndef _WIN32
        int rc = read(kfd, c, 1);
        if (rc < 0) { throw std::runtime_error("read failed"); }
    #else
        for(;;){
            HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
            INPUT_RECORD buffer;
            DWORD events;
            PeekConsoleInput(handle, &buffer, 1, &events);
            if(events > 0){
                ReadConsoleInput(handle, &buffer, 1, &events);
                if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT){
                    *c = KEYCODE_LEFT;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP){
                    *c = KEYCODE_UP;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT){
                    *c = KEYCODE_RIGHT;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN){
                    *c = KEYCODE_DOWN;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x42){
                    *c = KEYCODE_B;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x43){
                    *c = KEYCODE_C;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44){
                    *c = KEYCODE_D;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45){
                    *c = KEYCODE_E;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x46){
                    *c = KEYCODE_F;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x47){
                    *c = KEYCODE_G;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x51){
                    *c = KEYCODE_Q;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52){
                    *c = KEYCODE_R;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54){
                    *c = KEYCODE_T;
                    return;
                }else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x56){
                    *c = KEYCODE_V;
                    return;
                }
            }
        }
    #endif
}

void KeyboardReader::shutdown(){
    #ifndef _WIN32
        tcsetattr(kfd, TCSANOW, &cooked);
    #endif
}

/*---KEYBOARD_READER_CPP---*/