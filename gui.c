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
static bool headless = false; // --no-tui: no curses, status lines go to stderr
static bool too_small = false; // Terminal below COL_THRD x ROW_THRD after a resize: draw nothing

static const char *const titles[] = {
    [TRACK] = "GPS Simulator Setup", [LS_FIX] = "GPS Simulation Status", [KF_FIX] = "Dynamic Position"};

static WINDOW *window[HELP + 1] = {NULL};
static PANEL *panel[HELP + 1] = {NULL};

static pthread_mutex_t gui_lock; // Mutex to lock access during GUI updates

static void gui_update() {
    if (too_small)
        return;
    update_panels();
    doupdate();
}

// Status and an open Info/Help box stay above the main panel
static void raise_overlays(void) {
    top_panel(panel[STATUS]);
    if (!panel_hidden(panel[INFO]))
        top_panel(panel[INFO]);
    if (!panel_hidden(panel[HELP]))
        top_panel(panel[HELP]);
}

static void show_footer(WINDOW *win) {
    wattron(win, COLOR_PAIR(2));
    mvwprintw(win, max_y - STATUS_HEIGHT - 2, 10, "TAB or F1-F3 switch displays, 'x' Exit, 'i' Info, 'h' Help");
    wattroff(win, COLOR_PAIR(2));
}

/* Show the window with a border and a label */
static void show_window(WINDOW *win, const char *label) {
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
}

//print the header info for LS fix result display

static void ls_show_header(void) {
    wattron(window[LS_FIX], COLOR_PAIR(2));
    mvwprintw(window[LS_FIX], 3, 1, "PRN  AZ    ELEV  PRange       dIon");
    wattroff(window[LS_FIX], COLOR_PAIR(2));
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
}

static void show_vertical_speed(float height) {
    wattron(window[HEIGHT], COLOR_PAIR(11) | A_BOLD);
    mvwprintw(window[HEIGHT], 0, 0, "VERT SPEED");
    wattroff(window[HEIGHT], COLOR_PAIR(11) | A_BOLD);

    wattron(window[HEIGHT], COLOR_PAIR(13) | A_BOLD);
    mvwprintw(window[HEIGHT], 1, 0, "%6.1f m/s", height);
    wattroff(window[HEIGHT], COLOR_PAIR(13) | A_BOLD);
}

static void show_speed(float speed) {
    wattron(window[SPEED], COLOR_PAIR(11) | A_BOLD);
    mvwprintw(window[SPEED], 0, 3, "SPEED");
    wattroff(window[SPEED], COLOR_PAIR(11) | A_BOLD);

    wattron(window[SPEED], COLOR_PAIR(13) | A_BOLD);
    mvwprintw(window[SPEED], 1, 0, "%6.1f km/h", speed);
    wattroff(window[SPEED], COLOR_PAIR(13) | A_BOLD);
}

static void show_target(target_t *target) {
    mvwprintw(window[TARGET], 0, 4, "Target:");

    mvwprintw(window[TARGET], 1, 0, "Distance  %9.1f m", target->distance);
    mvwprintw(window[TARGET], 2, 0, "Direction %9.1f deg", target->bearing / 1000);
    mvwprintw(window[TARGET], 3, 0, "Height    %9.1f m", target->height);
    mvwprintw(window[TARGET], 4, 0, "Longitude %9.6f deg", target->lon);
    mvwprintw(window[TARGET], 5, 0, "Latitude  %9.6f deg", target->lat);
}

static void show_local(location_t *loc) {
    mvwprintw(window[LOCATION], 0, 4, "Location:");
    mvwprintw(window[LOCATION], 1, 0, "Longitude %9.6f deg", loc->lon);
    mvwprintw(window[LOCATION], 2, 0, "Latitude  %9.6f deg", loc->lat);
    mvwprintw(window[LOCATION], 3, 0, "Height    %9.1f m", loc->height);
}

static void init_windows(void) {
    int info_x, info_y;

    window[STATUS] = newwin(STATUS_HEIGHT, max_x - 2, max_y - STATUS_HEIGHT - 1, 1);
    window[TRACK] = newwin(max_y, max_x, 0, 0);
    window[LS_FIX] = newwin(max_y, max_x, 0, 0);
    window[KF_FIX] = newwin(max_y, max_x, 0, 0);
    window[HEADING] = subwin(window[KF_FIX], HEAD_HEIGHT, HEAD_WIDTH, HEAD_Y, HEAD_X);
    syncok(window[HEADING], TRUE);
    show_heading(0);
    window[HEIGHT] = subwin(window[KF_FIX], 2, 12, HEAD_Y + 6, HEAD_X + 20);
    syncok(window[HEIGHT], TRUE);
    show_vertical_speed(0);
    window[SPEED] = subwin(window[KF_FIX], 2, 12, HEAD_Y + 6, HEAD_X - 12);
    syncok(window[SPEED], TRUE);
    show_speed(0);
    window[TARGET] = subwin(window[KF_FIX], 7, 26, 4, 30);
    syncok(window[TARGET], TRUE);
    target_t t = {0};
    show_target(&t);
    window[LOCATION] = subwin(window[KF_FIX], 4, 26, 4, 2);
    syncok(window[LOCATION], TRUE);
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
    panel[INFO] = new_panel(window[INFO]);
    panel[STATUS] = new_panel(window[STATUS]);
    panel[HELP] = new_panel(window[HELP]);
    hide_panel(panel[INFO]);
    hide_panel(panel[HELP]);

    /* Set up the user pointers to the next panel */

    set_panel_userptr(panel[TRACK], panel[LS_FIX]);
    set_panel_userptr(panel[LS_FIX], panel[KF_FIX]);
    set_panel_userptr(panel[KF_FIX], panel[TRACK]);

    //Print title of each window
    for (window_panel_t p = TRACK; p <= KF_FIX; p++)
        show_window(window[p], titles[p]);

    ls_show_header();
    top_panel(panel[TRACK]);
    panel[TOP] = panel[TRACK];
    top_panel(panel[STATUS]);

    // Keep status window scrolling when adding lines
    scrollok(window[STATUS], TRUE);

    /* Update the stacking order.tracking_panel will be on the top*/
    update_panels();
    doupdate();
}

void gui_init(bool no_tui) {
    char ch;
    pthread_mutex_init(&gui_lock, NULL);
    headless = no_tui;
    if (headless)
        return;
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
    nodelay(stdscr, TRUE); // gui_getch polls, so it never blocks with gui_lock held
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

// Subwindows before their parent KF_FIX, ncurses refuses to delete a parent first
static void delete_windows(WINDOW **w, PANEL **p) {
    static const window_panel_t order[] = {HEADING, HEIGHT, SPEED, TARGET, LOCATION,
        TRACK, LS_FIX, KF_FIX, INFO, STATUS, HELP};
    for (size_t i = 0; i < sizeof order / sizeof order[0]; i++) {
        if (p[order[i]] != NULL)
            del_panel(p[order[i]]);
        delwin(w[order[i]]);
    }
}

void gui_destroy(void) {
    pthread_mutex_unlock(&gui_lock); // Just in case
    pthread_mutex_destroy(&gui_lock);
    if (headless)
        return;
    delete_windows(window, panel);
    endwin();
}

/* Rebuild the windows for a new terminal size and keep their contents; call after KEY_RESIZE. */
void gui_resize(void) {
    if (headless)
        return;
    pthread_mutex_lock(&gui_lock);
    getmaxyx(stdscr, max_y, max_x);
    too_small = max_y < ROW_THRD || max_x < COL_THRD;
    if (too_small) {
        // ponytail: whatever ncurses cut off while shrinking is gone when the terminal grows back
        erase();
        mvprintw(0, 0, "Terminal too small, need %dx%d.", COL_THRD, ROW_THRD);
        refresh();
        pthread_mutex_unlock(&gui_lock);
        return;
    }
    WINDOW *old[HELP + 1];
    PANEL *old_panel[HELP + 1];
    memcpy(old, window, sizeof old);
    memcpy(old_panel, panel, sizeof old_panel);
    window_panel_t top = TRACK;
    for (window_panel_t p = TRACK; p <= KF_FIX; p++) {
        if (panel[TOP] == panel[p])
            top = p;
    }
    init_windows();
    // The whole interior below the title (KF_FIX includes its subwindows), then the frame again
    for (window_panel_t p = TRACK; p <= KF_FIX; p++) {
        int oy, ox;
        getmaxyx(old[p], oy, ox);
        int last_row = (oy < max_y ? oy : max_y) - 2;
        int last_col = (ox < max_x ? ox : max_x) - 2;
        copywin(old[p], window[p], 3, 1, 3, 1, last_row, last_col, FALSE);
        int old_sep = oy - STATUS_HEIGHT - 2; // Separator and footer row of the old size
        if (old_sep <= last_row)
            mvwhline(window[p], old_sep, 1, ' ', max_x - 2);
        show_window(window[p], titles[p]);
    }
    int sy, sx;
    getmaxyx(old[STATUS], sy, sx);
    copywin(old[STATUS], window[STATUS], 0, 0, 0, 0, sy - 1, (sx < max_x - 2 ? sx : max_x - 2) - 1, FALSE);
    delete_windows(old, old_panel);
    top_panel(panel[top]);
    panel[TOP] = panel[top];
    raise_overlays();
    clearok(curscr, TRUE);
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

void gui_mvwprintw(window_panel_t w, int y, int x, const char * fmt, ...) {
    va_list args;
    if (headless) {
        // Only the one-time setup lines; LS_FIX is redrawn every update
        if (w == TRACK) {
            pthread_mutex_lock(&gui_lock);
            va_start(args, fmt);
            vfprintf(stderr, fmt, args);
            va_end(args);
            fputc('\n', stderr);
            pthread_mutex_unlock(&gui_lock);
        }
        return;
    }
    pthread_mutex_lock(&gui_lock);
    if (wmove(window[w], y, x) == ERR) {
        pthread_mutex_unlock(&gui_lock);
        return;
    }
    va_start(args, fmt);
    vw_printw(window[w], fmt, args);
    va_end(args);
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

void gui_status_wprintw(status_color_t clr, const char * fmt, ...) {
    pthread_mutex_lock(&gui_lock);
    va_list args;
    va_start(args, fmt);
    if (headless) {
        char line[512];
        vsnprintf(line, sizeof line, fmt, args);
        va_end(args);
        size_t n = strlen(line);
        if (n > 0 && line[n - 1] == '\r') // In-place TUI line, e.g. "Gain:"
            line[n - 1] = '\n';
        fputs(line, stderr);
        pthread_mutex_unlock(&gui_lock);
        return;
    }
    if (clr > 0) {
        wattron(window[STATUS], COLOR_PAIR(clr));
    }
    vw_printw(window[STATUS], fmt, args);
    if (clr > 0) {
        wattroff(window[STATUS], COLOR_PAIR(clr));
    }
    va_end(args);
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

int gui_getch(void) {
    if (headless) {
        usleep(100000);
        return ERR;
    }
    pthread_mutex_lock(&gui_lock);
    int ch = getch();
    pthread_mutex_unlock(&gui_lock);
    if (ch == ERR)
        usleep(100000); // Poll every 100 ms, outside the lock
    return ch;
}

void gui_top_panel(window_panel_t p) {
    if (headless)
        return;
    pthread_mutex_lock(&gui_lock);
    top_panel(panel[p]);
    panel[TOP] = panel[p];
    raise_overlays();
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

void gui_toggle_current_panel(void) {
    if (headless)
        return;
    pthread_mutex_lock(&gui_lock);
    panel[TOP] = (PANEL *) panel_userptr(panel[TOP]);
    top_panel(panel[TOP]);
    raise_overlays();
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

void gui_show_panel(window_panel_t p, attr_status_t onoff) {
    if (headless)
        return;
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
    if (headless)
        return;
    pthread_mutex_lock(&gui_lock);
    show_speed(speed);
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

void gui_show_heading(float hdg) {
    if (headless)
        return;
    pthread_mutex_lock(&gui_lock);
    show_heading(hdg);
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

void gui_show_vertical_speed(float vs) {
    if (headless)
        return;
    pthread_mutex_lock(&gui_lock);
    show_vertical_speed(vs);
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

void gui_show_location(void *l) {
    if (headless)
        return;
    pthread_mutex_lock(&gui_lock);
    show_local((location_t *) (l));
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

/* One line on every main panel, left of the title. */
void gui_show_status(bool tx_on, int gain, bool has_gain, unsigned fifo_fill, unsigned fifo_size) {
    if (headless)
        return;
    char line[48];
    if (has_gain)
        snprintf(line, sizeof line, "TX %-3s  Gain %3d dB  FIFO %u/%u", tx_on ? "on" : "off", gain, fifo_fill, fifo_size);
    else
        snprintf(line, sizeof line, "TX %-3s  FIFO %u/%u", tx_on ? "on" : "off", fifo_fill, fifo_size);
    pthread_mutex_lock(&gui_lock);
    for (window_panel_t p = TRACK; p <= KF_FIX; p++) {
        wattron(window[p], COLOR_PAIR(5));
        mvwprintw(window[p], 1, 2, "%-40s", line);
        wattroff(window[p], COLOR_PAIR(5));
    }
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}

void gui_show_target(void *t) {
    if (headless)
        return;
    pthread_mutex_lock(&gui_lock);
    show_target((target_t *) (t));
    gui_update();
    pthread_mutex_unlock(&gui_lock);
}
