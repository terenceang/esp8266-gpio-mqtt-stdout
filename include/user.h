#ifndef _USER_H_
#define _USER_H_

static void debounce_timerfunc(void *arg);
static void swtich_intr_handler(int8_t key);
void switch_init();
void led_init();

static os_timer_t debounce_timer;

#endif

