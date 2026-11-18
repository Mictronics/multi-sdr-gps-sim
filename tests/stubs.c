/* gps.c reports to the GUI and sets thread affinity; these stubs let the tests link without them. */
#include "gps-sim.h"
#include "gui.h"

void gui_status_wprintw(status_color_t c, const char *f, ...) { (void) c; (void) f; }
void gui_mvwprintw(window_panel_t w, int y, int x, const char *f, ...) { (void) w; (void) y; (void) x; (void) f; }
void gui_show_location(void *l) { (void) l; }
void gui_show_target(void *t) { (void) t; }
void set_thread_name(const char *n) { (void) n; }
int thread_to_core(int c) { return c; }
