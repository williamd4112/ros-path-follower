#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#define KEYCODE_ESC 27

namespace keyboard
{
	void reset_terminal_mode();
	void set_conio_terminal_mode();
	int getch();
	int getKey();
}

#endif
