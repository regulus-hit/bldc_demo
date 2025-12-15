#ifndef __OLED_DISPLAY_H_
#include <stdint.h>
#define __OLED_DISPLAY_H_












extern uint8_t usb_open_display_flag;
extern uint8_t data_upload_display_flag;

extern uint8_t motor_run_display_flag;
extern uint8_t display_static_flag;
extern uint8_t init_dispaly_flag;
extern uint8_t display_index_key;
extern uint8_t display_cnt;
extern uint8_t display_flag;
extern uint8_t drv8301_fault_flag;
extern void oled_display_handle(void);
#endif
