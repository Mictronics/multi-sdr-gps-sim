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
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <ncurses.h>
#include <panel.h>
#include <form.h>
#include "gps-sim.h"
#include "gps.h"
#include "gui.h"

static const int info_width = 50;
static const int info_height = 13;
static const int help_width = 50;
static const int help_height = 13;
static int max_x = 0;
static int max_y = 0;

static WINDOW *window[13] = {NULL};
static PANEL *panel[13] = {NULL};

static pthread_mutex_t gui_lock; // Mutex to lock access during GUI updates

static void gui_update() {
    update_panels();
    doupdate();
}

static void show_footer(WINDOW *win) {
    wattron(win, COLOR_PAIR(2));
    mvwprintw(win, max_y - STATUS_HEIGHT - 2, 10, "TAB or F1-F3 switch displays, 'x' Exit, 'i' Info, 'h' Help");
    wattroff(win, COLOR_PAIR(2));
}

/* Show the window with a border and a label */
static void show_window(WINDOW *win, char *label) {
    int startx, half_width, str_leng;

    half_width = max_x / 2;
    str_leng = (int) strlen(label);
    startx = half_width - (str_leng / 2);

    box(win, 0, 0);
    // Header separator
    mvwaddch(win, 2, 0, ACS_LTEE);
    mvwhline(win, 2, 1, ACS_HLINE, max_x - 2);
    mvwaddch(win, 2, max_x - 1, ACS_RTEE);
    // Status separator
    mvwaddch(win, max_y - STATUS_HEIGHT - 2, 0, ACS_LTEE);
    mvwhline(win, max_y - STATUS_HEIGHT - 2, 1, ACS_HLINE, max_x - 2);
    mvwaddch(win, max_y - STATUS_HEIGHT - 2, max_x - 1, ACS_RTEE);
    // Header label
    wattron(win, COLOR_PAIR(1));
    mvwprintw(win, 1, startx, "%s", label);
    wattroff(win, COLOR_PAIR(1));
    show_footer(win);
    doupdate();
}

//print the header info for LS fix result display

static void ls_show_header(void) {
    wattron(window[LS_FIX], COLOR_PAIR(2));
    mvwprintw(window[LS_FIX], 3, 1, "PRN  AZ    ELEV  PRange       dIon");
    wattroff(window[LS_FIX], COLOR_PAIR(2));
    wrefresh(window[TOP]);
}

static void show_heading(float degree) {
    int bias;
    int zero_y = 0; //coordinates of zero
    int zero_x = 9;

    //draw window outline
    wborder(window[HEADING], '.', '.', '.', '.', '.', '.', '.', '.');

    mvwaddch(window[HEADING], 0, 9, ACS_UARROW);
    mvwprintw(window[HEADING], 1, 9, "0");
    mvwprintw(window[HEADING], 6, 0, "<270");
    mvwprintw(window[HEADING], 6, 16, "90>");
    mvwprintw(window[HEADING], 11, 8, "180");
    mvwaddch(window[HEADING], 12, 9, ACS_DARROW);
    //wattron(window[HEADING],COLOR_PAIR(5)|A_BOLD);
    mvwaddch(window[HEADING], 6, 9, ACS_DIAMOND);
    //wattroff(window[HEADING],COLOR_PAIR(5)|A_BOLD);
    //show window title
    wattron(window[HEADING], COLOR_PAIR(11) | A_BOLD);
    mvwprintw(window[HEADING], 4, 6, "DIRECTION");
    wattroff(window[HEADING], COLOR_PAIR(11) | A_BOLD);

    wattron(window[HEADING], COLOR_PAIR(13) | A_BOLD);
    mvwprintw(window[HEADING], 8, 6, "%6.1f", degree);
    wattroff(window[HEADING], COLOR_PAIR(13) | A_BOLD);

    //calculate coordinates
    bias = (int) (degree / 6);
    //data is at top line,0~60
    if ((bias >= 0) && (bias <= 9)) {
        wattron(window[HEADING], COLOR_PAIR(12) | A_BOLD);
        mvwaddch(window[HEADING], 0, 9 + bias, ACS_DIAMOND);
        wattroff(window[HEADING], COLOR_PAIR(12) | A_BOLD);
    }
    //data is at right line 60~126
    if ((bias >= 10) && (bias <= 20)) {
        wattron(window[HEADING], COLOR_PAIR(12) | A_BOLD);
        mvwaddch(window[HEADING], zero_y + bias - 9, zero_x + 9, ACS_DIAMOND);
        wattroff(window[HEADING], COLOR_PAIR(12) | A_BOLD);
    }
    //data is at bottom line 132~180
    if ((bias >= 21) && (bias <= 30)) {
        wattron(window[HEADING], COLOR_PAIR(12) | A_BOLD);
        mvwaddch(window[HEADING], zero_y + 12, 18 - (bias - 20), ACS_DIAMOND);
        wattroff(window[HEADING], COLOR_PAIR(12) | A_BOLD);
    }
    //data is at bottom line 180~234
    if ((bias >= 31) && (bias <= 39)) {
        wattron(window[HEADING], COLOR_PAIR(12) | A_BOLD);
        mvwaddch(window[HEADING], zero_y + 12, 9 + (-bias + 30), ACS_DIAMOND);
        wattroff(window[HEADING], COLOR_PAIR(12) | A_BOLD);
    }
    //data is at left line 240~-300
    if ((bias >= 40) && (bias <= 50)) {
        wattron(window[HEADING], COLOR_PAIR(12) | A_BOLD);
        mvwaddch(window[HEADING], 12 + (-bias + 39), 0, ACS_DIAMOND);
        wattroff(window[HEADING], COLOR_PAIR(12) | A_BOLD);
    }

    //data is at left line 306~-360
    if ((bias >= 51) && (bias <= 60)) {
        wattron(window[HEADING], COLOR_PAIR(12) | A_BOLD);
        mvwaddch(window[HEADING], 0, bias - 51, ACS_DIAMOND);
        wattroff(window[HEADING], COLOR_PAIR(12) | A_BOLD);
    }
    if (window[TOP] == window[KF_FIX]) {
        wrefresh(window[HEADING]);
    }
}

static void show_vertical_speed(float height) {
    wattron(window[HEIGHT], COLOR_PAIR(11) | A_BOLD);
    mvwprintw(window[HEIGHT], 0, 0, "VERT SPEED");
    wattroff(window[HEIGHT], COLOR_PAIR(11) | A_BOLD);

    wattron(window[HEIGHT], COLOR_PAIR(13) | A_BOLD);
    mvwprintw(window[HEIGHT], 1, 0, "%6.1f m/s", height);
    wattroff(window[HEIGHT], COLOR_PAIR(13) | A_BOLD);
    if (window[TOP] == window[KF_FIX]) {
        wrefresh(window[HEIGHT]);
    }
}

static void show_speed(float speed) {
    wattron(window[SPEED], COLOR_PAIR(11) | A_BOLD);
    mvwprintw(window[SPEED], 0, 3, "SPEED");
    wattroff(window[SPEED], COLOR_PAIR(11) | A_BOLD);

    wattron(window[SPEED], COLOR_PAIR(13) | A_BOLD);
    mvwprintw(window[SPEED], 1, 0, "%6.1f km/h", speed);
    wattroff(window[SPEED], COLOR_PAIR(13) | A_BOLD);
    if (window[TOP] == window[KF_FIX]) {
        wrefresh(window[SPEED]);
    }
}

static void show_target(target_t *target) {
    mvwprintw(window[TARGET], 0, 4, "Target:");

    mvwprintw(window[TARGET], 1, 0, "Distance  %9.1f m", target->distance);
    mvwprintw(window[TARGET], 2, 0, "Direction %9.1f deg", target->bearing / 1000);
    mvwprintw(window[TARGET], 3, 0, "Height    %9.1f m", target->height);
    mvwprintw(window[TARGET], 4, 0, "Longitude %9.6f deg", target->lon);
    mvwprintw(window[TARGET], 5, 0, "Latitude  %9.6f deg", target->lat);
    if (window[TOP] == window[KF_FIX]) {
        wnoutrefresh(window[TARGET]);
    }
}

static void show_local(location_t *loc) {
    mvwprintw(window[LOCATION], 0, 4, "Location:");
    mvwprintw(window[LOCATION], 1, 0, "Longitude %9.6f deg", loc->lon);
    mvwprintw(window[LOCATION], 2, 0, "Latitude  %9.6f deg", loc->lat);
    mvwprintw(window[LOCATION], 3, 0, "Height    %9.1f m", loc->height);
    if (window[TOP] == window[KF_FIX]) {
        wnoutrefresh(window[LOCATION]);
    }
}

static void eph_show_header(void) {
    wattron(window[EPHEMERIS], COLOR_PAIR(2));
    mvwprintw(window[EPHEMERIS], 3, 1, "PRN  AZ    ELEV  EPH   SIM        ");
    wattroff(window[EPHEMERIS], COLOR_PAIR(2));
    wrefresh(window[TOP]);
}

static void init_windows(void) {
    int info_x, info_y;

    window[STATUS] = newwin(STATUS_HEIGHT, max_x - 2, max_y - STATUS_HEIGHT - 1, 1);
    window[TRACK] = newwin(max_y, max_x, 0, 0);
    window[LS_FIX] = newwin(max_y, max_x, 0, 0);
    window[EPHEMERIS] = newwin(max_y, max_x, 0, 0);
    window[KF_FIX] = newwin(max_y, max_x, 0, 0);
    window[HEADING] = subwin(window[KF_FIX], HEAD_HEIGHT, HEAD_WIDTH, HEAD_Y, HEAD_X);
    show_heading(0);
    window[HEIGHT] = subwin(window[KF_FIX], 2, 12, HEAD_Y + 6, HEAD_X + 20);
    show_vertical_speed(0);
    window[SPEED] = subwin(window[KF_FIX], 2, 12, HEAD_Y + 6, HEAD_X - 12);
    show_speed(0);
    window[TARGET] = subwin(window[KF_FIX], 7, 26, 4, 30);
    target_t t = {0};
    show_target(&t);
    window[LOCATION] = subwin(window[KF_FIX], 4, 26, 4, 2);
    location_t l = {0};
    show_local(&l);

    /* setup info window */
    info_x = (max_x - info_width) / 2;
    info_y = (max_y - info_height) / 2;
    window[INFO] = newwin(info_height, info_width, info_y, info_x);
    box(window[INFO], 0, 0);
    wattron(window[INFO], COLOR_PAIR(1));
    mvwprintw(window[INFO], 1, 2, "Multi SDR GPS Simulator");
    wattroff(window[INFO], COLOR_PAIR(1));
    wattron(window[INFO], COLOR_PAIR(2));
    mvwprintw(window[INFO], 3, 2, "https://github.com/Mictronics/multi-sdr-gps");
    mvwprintw(window[INFO], 4, 2, "(c) Mictronics 2021");
    mvwprintw(window[INFO], 5, 2, "Distributed under the MIT License");
    mvwprintw(window[INFO], 7, 2, "Based on work from Takuji Ebinuma (gps-sdr-sim)");
    mvwprintw(window[INFO], 8, 2, "and IvanKor.");
    mvwprintw(window[INFO], info_height - 2, 2, "Press any key to return.");
    wattroff(window[INFO], COLOR_PAIR(2));

    /* setup help window */
    info_x = (max_x - help_width) / 2;
    info_y = (max_y - help_height) / 2;
    window[HELP] = newwin(help_height, help_width, info_y, info_x);
    box(window[HELP], 0, 0);
    wattron(window[HELP], COLOR_PAIR(1));
    mvwprintw(window[HELP], 1, 2, "Help");
    wattroff(window[HELP], COLOR_PAIR(1));
    mvwprintw(window[HELP], 2, 2, "w   Increase altitude     i    Info");
    mvwprintw(window[HELP], 3, 2, "s   Decrease altitude     h    Help");
    mvwprintw(window[HELP], 4, 2, "d   Heading right         x    Exit");
    mvwprintw(window[HELP], 5, 2, "a   Heading left          F1   Setup Window");
    mvwprintw(window[HELP], 6, 2, "e   Increase speed        F2   Status Window");
    mvwprintw(window[HELP], 7, 2, "q   Decrease speed        F3   Position Window");
    mvwprintw(window[HELP], 8, 2, "t   Increase TX gain");
    mvwprintw(window[HELP], 9, 2, "g   Decrease TX gain");

    /* Attach a panel to each window
     * Order is bottom up
     */
    panel[TRACK] = new_panel(window[TRACK]);
    panel[LS_FIX] = new_panel(window[LS_FIX]);
    panel[KF_FIX] = new_panel(window[KF_FIX]);
    panel[EPHEMERIS] = new_panel(window[EPHEMERIS]);
    panel[INFO] = new_panel(window[INFO]);
    panel[STATUS] = new_panel(window[STATUS]);
    panel[HELP] = new_panel(window[HELP]);
    hide_panel(panel[INFO]);
    hide_panel(panel[HELP]);

    /* Set up the user pointers to the next panel */

    set_panel_userptr(panel[TRACK], panel[LS_FIX]);
    set_panel_userptr(panel[LS_FIX], panel[KF_FIX]);
    set_panel_userptr(panel[KF_FIX], panel[EPHEMERIS]);
    set_panel_userptr(panel[EPHEMERIS], panel[TRACK]);

    //Print title of each window
    show_window(window[TRACK], "GPS Simulator Setup");
    show_window(window[LS_FIX], "GPS Simulation Status");
    show_window(window[EPHEMERIS], "Test");
    show_window(window[KF_FIX], "Dynamic Position");

    ls_show_header();
    top_panel(panel[TRACK]);
    panel[TOP] = panel[TRACK];

    // Keep status window scrolling when adding lines
    scrollok(window[STATUS], TRUE);

    /* Update the stacking order.tracking_panel will be on the top*/
    update_panels();
    doupdate();
}

void gui_init(void) {
    char ch;
    pthread_mutex_init(&gui_lock, NULL);
    pthread_mutex_lock(&gui_lock);
    /* Initialize curses */
    initscr();
    getmaxyx(stdscr, max_y, max_x);
    endwin();
    /*check if the col and row meet the threshold */
    if (max_y < ROW_THRD || max_x < COL_THRD) {
        printf("Your console window size is %dx%d need 26x120\n", max_y, max_x);
        printf("Do you still want to continue? [Y/N] ");
        ch = getchar();
        if ((ch != 'y') && (ch != 'Y')) {
            exit(0);
        }
    }
    /* redo the initialization */
    initscr();
    getmaxyx(stdscr, max_y, max_x);
    start_color();
    cbreak();
    noecho();
    curs_set(0); //no cursor
    keypad(stdscr, TRUE);
    timeout(100);
    /* Initialize all the colors */
    init_pair(1, COLOR_RED, COLOR_BLACK);
    init_pair(2, COLOR_GREEN, COLOR_BLACK);
    init_pair(3, COLOR_BLUE, COLOR_BLACK);
    init_pair(4, COLOR_CYAN, COLOR_BLACK);
    init_pair(5, COLOR_YELLOW, COLOR_BLACK);

    init_pair(11, COLOR_BLUE, COLOR_BLACK);
    init_pair(12, COLOR_RED, COLOR_BLACK);
    init_pair(13, COLOR_RED, COLOR_WHITE);
    init_pair(14, COLOR_BLACK, COLOR_RED);

    init_windows();
    pthread_mutex_unlock(&gui_lock);
}

void gui_destroy(void) {
    pthread_mutex_unlock(&gui_lock); // Just in case
    pthread_mutex_destroy(&gui_lock);
    delwin(window[TRACK]);
    delwin(window[LS_FIX]);
    delwin(window[KF_FIX]);
    delwin(window[INFO]);
    delwin(window[HEADING]);
    delwin(window[HEIGHT]);
    delwin(window[SPEED]);
    delwin(window[TARGET]);
    delwin(window[LOCATION]);
    delwin(window[EPHEMERIS]);
    delwin(window[STATUS]);
    delwin(window[HELP]);
    endwin();
}

void gui_mvwprintw(window_panel_t w, int y, int x, const char * fmt, ...) {
    pthread_mutex_lock(&gui_lock);
    va_list args;
    if (wmove(window[w], y, x) == ERR) {
        return;
    }
    va_start(args, fmt);
    vw_printw(window[w], fmt, args);
    va_end(args);
    // Refresh only what is printed on top panel, don't care about background
    // updates.
    wrefresh(window[TOP]);
    pthread_mutex_unlock(&gui_lock);
}

void gui_status_wprintw(status_color_t clr, const char * fmt, ...) {
    pthread_mutex_lock(&gui_lock);
    va_list args;
    va_start(args, fmt);
    if (clr > 0) {
        wattron(window[STATUS], COLOR_PAIR(clr));
    }
    vw_printw(window[STATUS], fmt, args);
    if (clr > 0) {
        wattroff(window[STATUS], COLOR_PAIR(clr));
    }
    va_end(args);
    wrefresh(window[STATUS]);
    pthread_mutex_unlock(&gui_lock);
}

void gui_colorpair(window_panel_t w, unsigned clr, attr_status_t onoff) {
    pthread_mutex_lock(&gui_lock);
    if (onoff == ON) {
        wattron(window[w], COLOR_PAIR(clr));
    } else {
        wattroff(window[w], COLOR_PAIR(clr));
    }
    pthread_mutex_unlock(&gui_lock);
}

int gui_getch(void) {
    int ch = getch();
    /*
    if (ch != -1) {
        fprintf(stderr, "%i\n", ch);
    }
     */
    return ch;
}

void gui_top_panel(window_panel_t p) {
    pthread_mutex_lock(&gui_lock);
    top_panel(panel[p]);
    panel[TOP] = panel[p];
    window[TOP] = window[p];
    // Status is alway the top most panel
    top_panel(panel[STATUS]);
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

void gui_toggle_current_panel(void) {
    pthread_mutex_lock(&gui_lock);
    panel[TOP] = (PANEL *) panel_userptr(panel[TOP]);
    top_panel(panel[TOP]);
    window[TOP] = panel_window(panel[TOP]);
    // Status is alway the top most panel
    top_panel(panel[STATUS]);
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

void gui_show_panel(window_panel_t p, attr_status_t onoff) {
    pthread_mutex_lock(&gui_lock);
    if (onoff == ON) {
        show_panel(panel[p]);
    } else {
        hide_panel(panel[p]);
    }
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

void gui_show_speed(float speed) {
    pthread_mutex_lock(&gui_lock);
    show_speed(speed);
    pthread_mutex_unlock(&gui_lock);
}

void gui_show_heading(float hdg) {
    pthread_mutex_lock(&gui_lock);
    show_heading(hdg);
    pthread_mutex_unlock(&gui_lock);
}

void gui_show_vertical_speed(float vs) {
    pthread_mutex_lock(&gui_lock);
    show_vertical_speed(vs);
    pthread_mutex_unlock(&gui_lock);
}

void gui_show_location(void *l) {
    pthread_mutex_lock(&gui_lock);
    show_local((location_t *) (l));
    pthread_mutex_unlock(&gui_lock);
}

void gui_show_target(void *t) {
    pthread_mutex_lock(&gui_lock);
    show_target((target_t *) (t));
    pthread_mutex_unlock(&gui_lock);
}
