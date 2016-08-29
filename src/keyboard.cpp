#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

#include "keyboard.h"

static struct termios orig_termios;

void keyboard::reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

void keyboard::set_conio_terminal_mode()
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

int keyboard::getch()
{
	int r;
	unsigned char c;
	
	return ((r = read(0, &c, sizeof(c))) < 0) ? r : c;
}

int keyboard::getKey()
{
	struct timeval tv = {0L, 0L};
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(0, &fds);
	
	return (select(1, &fds, NULL, NULL, &tv)) ? getch() : 0;
}
