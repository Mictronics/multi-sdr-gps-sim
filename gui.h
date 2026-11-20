/**
 * multi-sdr-gps-sim generates a IQ data stream on-the-fly to simulate a
 * GPS L1 baseband signal using a SDR platform like HackRF or ADLAM-Pluto.
 *
 * This file is part of the Github project at
 * https://github.com/mictronics/multi-sdr-gps-sim.git
 *
 * Copyright © 2026 Mictronics
 * Distributed under the MIT License.
 *
 */

#ifndef GUI_H
#define GUI_H

#include <stdbool.h>

#define ROW_THRD 26
#define COL_THRD 120
#define HEAD_HEIGHT 13
#define HEAD_WIDTH 19
#define HEAD_Y 12
#define HEAD_X 14
#define STATUS_HEIGHT 10

// Layout of the LS_FIX window: the channel table starts at the left, the
// scenario information sits in the column on the right.
#define LS_COL_INFO 40
#define LS_ROW_NAV 3
#define LS_ROW_CHAN 4
#define LS_ROW_XYZ 5
#define LS_ROW_LLH 6
#define LS_ROW_DURATION 7
#define LS_ROW_RINEX 8
#define LS_ROW_ALMANAC 9
#define LS_ROW_START 10
#define LS_ROW_SIMTIME 11
#define LS_COL_SIMTIME 57
#define LS_ROW_ELAPSED 12
#define LS_ROW_IONO 14

// Interactive keys
#define UP_KEY 'w'
#define DOWN_KEY 's'
#define RIGHT_KEY 'd'
#define LEFT_KEY 'a'
#define UPSPEED_KEY 'e'
#define DOWNSPEED_KEY 'q'
#define GAIN_INC_KEY 't'
#define GAIN_DEC_KEY 'g'

typedef enum
{
    TRACK = 0,
    LS_FIX,
    KF_FIX,
    INFO,
    HEADING,
    HEIGHT,
    SPEED,
    TARGET,
    LOCATION,
    TOP,
    STATUS,
    HELP
} window_panel_t;

typedef enum
{
    OFF = 0,
    ON = 1
} attr_status_t;

typedef enum
{
    DEFAULT = 0,
    RED = 1,
    GREEN = 2,
    BLUE = 3,
    CYAN = 4,
    YELLOW = 5
} status_color_t;

void gui_init(bool no_tui);
int gui_getch(void);
void gui_destroy(void);
void gui_resize(void);
void gui_mvwprintw(window_panel_t w, int y, int x, const char *fmt, ...);
void gui_status_wprintw(status_color_t clr, const char *fmt, ...);
void gui_top_panel(window_panel_t p);
void gui_toggle_current_panel(void);
void gui_show_panel(window_panel_t p, attr_status_t onoff);
void gui_show_speed(float speed);
void gui_show_heading(float hdg);
void gui_show_vertical_speed(float vs);
void gui_show_location(void *l);
void gui_show_target(void *t);
void gui_show_status(bool tx_on, int gain, bool has_gain, unsigned fifo_fill, unsigned fifo_size);

#endif /* GUI_H */
