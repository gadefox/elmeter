#ifndef CMD_H
#define CMD_H

//------------------------------------------------------------------------------

// Line protocol on the UART, one reply line per command:
//   SETTIME YYYY-MM-DD HH:MM:SS   set RTC (UTC), keeps T0
//   INIT    YYYY-MM-DD HH:MM:SS   erase history, set RTC and a new T0
//   GETTIME                       RTC time, T0, state, pulses in open window
// Replies: "OK ..." or "ERR <reason>".

// Call from the main loop; handles at most one complete line per call.
void cmd_poll(void);

#endif // CMD_H
