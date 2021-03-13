/**
 * multi-sdr-gps-sim generates a IQ data stream on-the-fly to simulate a
 * GPS L1 baseband signal using a SDR platform like HackRF or ADLAM-Pluto.
 *
 * This file is part of the Github project at
 * https://github.com/mictronics/multi-sdr-gps-sim.git
 *
 * Copyright © 2021 Mictronics
 * Distributed under the MIT License.
 *
 */

#ifndef GUI_H
#define GUI_H

#define ROW_THRD 26
#define COL_THRD 120
#define HEAD_HEIGHT 13
#define HEAD_WIDTH 19
#define HEAD_Y  12
#define HEAD_X  14
#define STATUS_HEIGHT 10

// Interactive keys
#define UP_KEY   'w'
#define DOWN_KEY  's'
#define RIGHT_KEY  'd'
#define LEFT_KEY  'a'
#define UPSPEED_KEY  'e'
#define DOWNSPEED_KEY 'q'
#define GAIN_INC_KEY 't'
#define GAIN_DEC_KEY 'g'

typedef enum {
    TRACK = 0,
    LS_FIX,
    KF_FIX,
    INFO,
    HEADING,
    HEIGHT,
    SPEED,
    TARGET,
    LOCATION,
    EPHEMERIS,
    TOP,
    STATUS,
    HELP
} window_panel_t;

typedef enum {
    OFF = 0,
    ON = 1
} attr_status_t;

typedef enum {
    DEFAULT = 0,
    RED = 1,
    GREEN = 2,
    BLUE = 3,
    CYAN = 4,
    YELLOW = 5
} status_color_t;

void gui_init(void);
int gui_getch(void);
void gui_destroy(void);
void gui_mvwprintw(window_panel_t w, int y, int x, const char * fmt, ...);
void gui_status_wprintw(status_color_t clr, const char * fmt, ...);
void gui_colorpair(window_panel_t w, unsigned clr, attr_status_t onoff);
void gui_top_panel(window_panel_t p);
void gui_toggle_current_panel(void);
void gui_show_panel(window_panel_t p, attr_status_t onoff);
void gui_show_speed(float speed);
void gui_show_heading(float hdg);
void gui_show_vertical_speed(float vs);
void gui_show_location(void *l);
void gui_show_target(void *t);

#endif /* GUI_H */

