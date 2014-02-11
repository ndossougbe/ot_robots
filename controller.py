#!/usr/bin/env python
import curses


class DirectionalKeyListener:

    def __init__(self, 
                 direction_mappings=(curses.KEY_UP, curses.KEY_LEFT, curses.KEY_DOWN, curses.KEY_RIGHT), 
                 exit_key='\t', callback=None):
        self._exit_key = exit_key
        self._direction_mappings = direction_mappings
        self._callback = callback
        self._enabled = False

    def _run(self, stdscr):
        ''' must be called through curses.wrapper() '''

        stdscr.addstr(0,10,'Hit TAB to quit')
        stdscr.addstr(2, 10, 'Directions:')
        stdscr.addstr(3, 10, 'a z e:')
        stdscr.addstr(4, 10, 'q   d:')
        stdscr.addstr(5, 10, 'w s x:')
        stdscr.addstr(7, 10, 'SPACE to stop.:')
        stdscr.refresh()
        key = ''

        while self._enabled and key != ord(self._exit_key):
            key = stdscr.getch()
            if key in self._direction_mappings:
                self._callback(self._direction_mappings[key], key, self)
            # stdscr.addch(20,25, key)
            stdscr.refresh()

    def activate(self):
        self._enabled = True
        curses.wrapper(self._run)

    def toggle(self):
        self._enabled = not self._enabled
        if self._enabled:
            # creates the screen, initializes it, and restores the terminal when the callback exits or crashes
            curses.wrapper(self._run)


if __name__ == '__main__':
    KEY_BINDINGS = {
        ord('a'): (1, 1),
        ord('z'): (1, 0),
        ord('e'): (1,-1),
        ord('q'): (0, 1),
        ord('s'): (-1, 0),
        ord('d'): (0,-1),
        ord('w'): (-1,-1),
        ord('x'): (-1, 1),
        ord(' '): (0, 0)
    }


    def move(direction, key, listener):
        print direction
        # print key
        # listener.toggle()  # stops the direction listener


    d = DirectionalKeyListener(callback=move, direction_mappings=KEY_BINDINGS)
    d.toggle()