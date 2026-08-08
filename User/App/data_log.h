#ifndef __DATA_LOG_H__
#define __DATA_LOG_H__

#include <stdint.h>

#define DATA_LOG_MAX_SAMPLES 500U
#define DATA_LOG_DECIMATE    2U

typedef enum
{
    DLOG_IDLE = 0,
    DLOG_LOGGING,
    DLOG_STOPPED,
    DLOG_FULL,
    DLOG_DUMPING,
    DLOG_DUMPED,
} data_log_state_t;

typedef enum
{
    DLOG_MODE_LINE = 0,
    DLOG_MODE_BALANCE,
} data_log_mode_t;

extern volatile data_log_state_t data_log_state;

void data_log_init(void);
void data_log_start(void);
void data_log_start_balance(void);
void data_log_stop(void);
void data_log_clear(void);
void data_log_record(void);
void data_log_record_balance(void);
void data_log_dump(void);
void data_log_print_status(void);
uint16_t data_log_count(void);

extern volatile data_log_mode_t data_log_mode;

#endif
