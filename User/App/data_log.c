#include "data_log.h"

#include "balance_app.h"
#include "ball_app.h"
#include "encoder_app.h"
#include "gray_app.h"
#include "gyroscope_driver.h"
#include "pendulum_app.h"
#include "pid_app.h"
#include "uart_driver.h"
#include <stdio.h>

#define DLOG_BAL_FLAG_ENABLED    0x01U
#define DLOG_BAL_FLAG_SATURATED  0x02U

typedef struct
{
    uint32_t time_ms;
    float target_left;
    float speed_left;
    float output_left;
    float target_right;
    float speed_right;
    float output_right;
    float line_error;
    int16_t yaw_d10;
    int16_t correction_d100;
    uint8_t center_d16;
    uint8_t line_detected;
    uint8_t track_n;
    uint8_t pid_enabled;
} data_log_line_sample_t;

typedef struct
{
    uint32_t time_ms;
    int16_t target_position_dmm;
    int16_t position_dmm;
    int16_t position_error_dmm;
    int16_t target_speed_dmm_s;
    int16_t velocity_dmm_s;
    int16_t speed_error_dmm_s;
    int16_t position_integral_dmm_s;
    int16_t speed_integral_dmm;
    int16_t motor_request_cdeg;
    int16_t motor_applied_cdeg;
    uint16_t ball_age_ms;
    uint8_t ball_state;
    uint8_t flags;
} data_log_balance_sample_t;

typedef union
{
    data_log_line_sample_t line;
    data_log_balance_sample_t balance;
} data_log_sample_t;

typedef char data_log_line_sample_must_be_40_bytes[
    (sizeof(data_log_line_sample_t) == 40U) ? 1 : -1];
typedef char data_log_balance_sample_must_be_28_bytes[
    (sizeof(data_log_balance_sample_t) == 28U) ? 1 : -1];
typedef char data_log_union_must_be_40_bytes[
    (sizeof(data_log_sample_t) == 40U) ? 1 : -1];

static data_log_sample_t log_buffer[DATA_LOG_MAX_SAMPLES];
static volatile uint16_t log_index = 0U;
static volatile uint8_t log_skip_count = 0U;
static uint32_t log_start_tick = 0U;

volatile data_log_state_t data_log_state = DLOG_IDLE;
volatile data_log_mode_t data_log_mode = DLOG_MODE_LINE;

static void DataLog_UartWrite(const char *data, uint16_t length)
{
    while ((DEBUG_UART)->gState != HAL_UART_STATE_READY)
    {
        __WFI();
    }

    (void)HAL_UART_Transmit(DEBUG_UART,
                            (uint8_t *)data,
                            length,
                            HAL_MAX_DELAY);
}

static int16_t DataLog_Scale(float value, float scale)
{
    float scaled = value * scale;

    if (scaled != scaled)
    {
        return 0;
    }
    if (scaled > 32767.0f)
    {
        return 32767;
    }
    if (scaled < -32768.0f)
    {
        return -32768;
    }

    return (scaled >= 0.0f) ? (int16_t)(scaled + 0.5f)
                            : (int16_t)(scaled - 0.5f);
}

static uint16_t DataLog_ClampAge(uint32_t age_ms)
{
    return (age_ms > 0xFFFFU) ? 0xFFFFU : (uint16_t)age_ms;
}

static const char *DataLog_StateName(data_log_state_t state)
{
    switch (state)
    {
        case DLOG_IDLE:     return "IDLE";
        case DLOG_LOGGING:  return "LOGGING";
        case DLOG_STOPPED:  return "STOPPED";
        case DLOG_FULL:     return "FULL";
        case DLOG_DUMPING:  return "DUMPING";
        case DLOG_DUMPED:   return "DUMPED";
        default:            return "ERROR";
    }
}

static void DataLog_Start(data_log_mode_t mode)
{
    if (data_log_state == DLOG_DUMPING)
    {
        Uart_Printf(DEBUG_UART, "log start refused: dump active\r\n");
        return;
    }

    /* Stop the 10 ms ISR recorder before replacing shared buffer ownership. */
    data_log_state = DLOG_IDLE;
    log_index = 0U;
    log_skip_count = 0U;
    log_start_tick = HAL_GetTick();
    data_log_mode = mode;
    data_log_state = DLOG_LOGGING;
}

void data_log_init(void)
{
    log_index = 0U;
    log_skip_count = 0U;
    log_start_tick = 0U;
    data_log_mode = DLOG_MODE_LINE;
    data_log_state = DLOG_IDLE;
}

void data_log_start(void)
{
    DataLog_Start(DLOG_MODE_LINE);
}

void data_log_start_balance(void)
{
    DataLog_Start(DLOG_MODE_BALANCE);
    Uart_Printf(DEBUG_UART, "blog recording 50Hz, capacity=%u\r\n",
                (unsigned int)DATA_LOG_MAX_SAMPLES);
}

void data_log_stop(void)
{
    if (data_log_state == DLOG_LOGGING)
    {
        data_log_state = DLOG_STOPPED;
    }
    data_log_print_status();
}

void data_log_clear(void)
{
    if (data_log_state == DLOG_DUMPING)
    {
        Uart_Printf(DEBUG_UART, "log clear refused: dump active\r\n");
        return;
    }

    data_log_state = DLOG_IDLE;
    log_index = 0U;
    log_skip_count = 0U;
    log_start_tick = 0U;
    Uart_Printf(DEBUG_UART, "log cleared\r\n");
}

void data_log_record(void)
{
    data_log_line_sample_t *sample;
    uint16_t center;

    if ((data_log_state != DLOG_LOGGING) ||
        (data_log_mode != DLOG_MODE_LINE))
    {
        return;
    }

    if (++log_skip_count < DATA_LOG_DECIMATE)
    {
        return;
    }
    log_skip_count = 0U;

    if (log_index >= DATA_LOG_MAX_SAMPLES)
    {
        data_log_state = DLOG_FULL;
        return;
    }

    sample = &log_buffer[log_index].line;
    center = (Normal[3] < Normal[4]) ? Normal[3] : Normal[4];

    sample->time_ms = HAL_GetTick();
    sample->target_left = pid_speed_left.target;
    sample->speed_left = left_encoder.speed_cm_s;
    sample->output_left = pid_speed_left.out;
    sample->target_right = pid_speed_right.target;
    sample->speed_right = right_encoder.speed_cm_s;
    sample->output_right = pid_speed_right.out;
    sample->line_error = g_line_position_error;
    sample->yaw_d10 = (int16_t)(euler_angles.yaw * 10.0f);
    sample->correction_d100 = (int16_t)(line_speed_correction * 100.0f);
    sample->center_d16 = (uint8_t)(center >> 4);
    sample->line_detected = g_line_detected;
    sample->track_n = TrackN;
    sample->pid_enabled = pid_running;

    ++log_index;
    if (log_index >= DATA_LOG_MAX_SAMPLES)
    {
        data_log_state = DLOG_FULL;
    }
}

void data_log_record_balance(void)
{
    BalanceTelemetry_t telemetry;
    data_log_balance_sample_t *sample;
    uint8_t flags = 0U;

    if ((data_log_state != DLOG_LOGGING) ||
        (data_log_mode != DLOG_MODE_BALANCE))
    {
        return;
    }
    if (log_index >= DATA_LOG_MAX_SAMPLES)
    {
        data_log_state = DLOG_FULL;
        return;
    }

    Balance_GetTelemetry(&telemetry);
    if (telemetry.enabled != 0U)
    {
        flags |= DLOG_BAL_FLAG_ENABLED;
    }
    if (telemetry.saturated != 0U)
    {
        flags |= DLOG_BAL_FLAG_SATURATED;
    }

    sample = &log_buffer[log_index].balance;
    sample->time_ms = HAL_GetTick() - log_start_tick;
    sample->target_position_dmm =
        DataLog_Scale(telemetry.target_position_mm, 10.0f);
    sample->position_dmm = DataLog_Scale(telemetry.position_mm, 10.0f);
    sample->position_error_dmm =
        DataLog_Scale(telemetry.position_error_mm, 10.0f);
    sample->target_speed_dmm_s =
        DataLog_Scale(telemetry.target_speed_mm_s, 10.0f);
    sample->velocity_dmm_s = DataLog_Scale(telemetry.velocity_mm_s, 10.0f);
    sample->speed_error_dmm_s =
        DataLog_Scale(telemetry.speed_error_mm_s, 10.0f);
    sample->position_integral_dmm_s =
        DataLog_Scale(telemetry.position_integral_mm_s, 10.0f);
    sample->speed_integral_dmm =
        DataLog_Scale(telemetry.speed_integral_mm, 10.0f);
    sample->motor_request_cdeg =
        DataLog_Scale(telemetry.motor_request_deg, 100.0f);
    sample->motor_applied_cdeg =
        DataLog_Scale(Pendulum_GetAppliedMotorDeg(), 100.0f);
    sample->ball_age_ms = DataLog_ClampAge(Ball_GetAge());
    sample->ball_state = (uint8_t)Ball_GetState();
    sample->flags = flags;

    ++log_index;
    if (log_index >= DATA_LOG_MAX_SAMPLES)
    {
        data_log_state = DLOG_FULL;
    }
}

uint16_t data_log_count(void)
{
    return log_index;
}

#define DATA_LOG_LINE_CSV_HEADER \
    "time_ms,sp_l_m,cur_l_m,out_l_mpct,sp_r_m,cur_r_m,out_r_mpct," \
    "line_err_m,yaw_d10,correction_d100,line_seen,track_n,center_d16,pid_on\r\n"

#define DATA_LOG_BALANCE_CSV_HEADER \
    "time_ms,state,age_ms,flags,x_tgt_dmm,x_dmm,x_err_dmm," \
    "v_tgt_dmm_s,v_dmm_s,v_err_dmm_s,outer_i_dmm_s," \
    "middle_i_dmm,motor_req_cdeg,motor_applied_cdeg\r\n"

static int DataLog_FormatLine(char *buffer, uint16_t size,
                              const data_log_line_sample_t *sample)
{
    return snprintf(
        buffer, size,
        "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u\r\n",
        (unsigned long)sample->time_ms,
        (int)(sample->target_left * 1000.0f),
        (int)(sample->speed_left * 1000.0f),
        (int)(sample->output_left * 1000.0f),
        (int)(sample->target_right * 1000.0f),
        (int)(sample->speed_right * 1000.0f),
        (int)(sample->output_right * 1000.0f),
        (int)(sample->line_error * 1000.0f),
        (int)sample->yaw_d10,
        (int)sample->correction_d100,
        (unsigned int)sample->line_detected,
        (unsigned int)sample->track_n,
        (unsigned int)sample->center_d16,
        (unsigned int)sample->pid_enabled);
}

static int DataLog_FormatBalance(char *buffer, uint16_t size,
                                 const data_log_balance_sample_t *sample)
{
    return snprintf(
        buffer, size,
        "%lu,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
        (unsigned long)sample->time_ms,
        (unsigned int)sample->ball_state,
        (unsigned int)sample->ball_age_ms,
        (unsigned int)sample->flags,
        (int)sample->target_position_dmm,
        (int)sample->position_dmm,
        (int)sample->position_error_dmm,
        (int)sample->target_speed_dmm_s,
        (int)sample->velocity_dmm_s,
        (int)sample->speed_error_dmm_s,
        (int)sample->position_integral_dmm_s,
        (int)sample->speed_integral_dmm,
        (int)sample->motor_request_cdeg,
        (int)sample->motor_applied_cdeg);
}

void data_log_dump(void)
{
    static char line_buffer[192];
    static const char line_header[] = DATA_LOG_LINE_CSV_HEADER;
    static const char balance_header[] = DATA_LOG_BALANCE_CSV_HEADER;
    const char *header;
    uint16_t header_length;
    uint16_t index;
    uint16_t count;
    int length;

    if (data_log_mode == DLOG_MODE_BALANCE)
    {
        if (Balance_IsEnabled() != 0U)
        {
            Uart_Printf(DEBUG_UART,
                        "blog dump refused: stop balance with bal:0 first\r\n");
            return;
        }
        if (data_log_state == DLOG_LOGGING)
        {
            Uart_Printf(DEBUG_UART,
                        "blog dump refused: use blog:stop first\r\n");
            return;
        }
        header = balance_header;
        header_length = (uint16_t)(sizeof(balance_header) - 1U);
    }
    else
    {
        PID_Set_Enable(0U);
        header = line_header;
        header_length = (uint16_t)(sizeof(line_header) - 1U);
    }

    if (data_log_state == DLOG_DUMPING)
    {
        return;
    }

    data_log_state = DLOG_DUMPING;
    count = log_index;
    DataLog_UartWrite(header, header_length);

    for (index = 0U; index < count; ++index)
    {
        if (data_log_mode == DLOG_MODE_BALANCE)
        {
            length = DataLog_FormatBalance(
                line_buffer, (uint16_t)sizeof(line_buffer),
                &log_buffer[index].balance);
        }
        else
        {
            length = DataLog_FormatLine(
                line_buffer, (uint16_t)sizeof(line_buffer),
                &log_buffer[index].line);
        }

        if (length > 0)
        {
            if (length >= (int)sizeof(line_buffer))
            {
                length = (int)sizeof(line_buffer) - 1;
            }
            DataLog_UartWrite(line_buffer, (uint16_t)length);
        }
    }

    length = snprintf(line_buffer, sizeof(line_buffer),
                      "# END %u samples\r\n", (unsigned int)count);
    if (length > 0)
    {
        DataLog_UartWrite(line_buffer, (uint16_t)length);
    }
    data_log_state = DLOG_DUMPED;
}

void data_log_print_status(void)
{
    uint32_t duration_ms = 0U;

    if ((data_log_mode == DLOG_MODE_BALANCE) && (log_index != 0U))
    {
        duration_ms = log_buffer[log_index - 1U].balance.time_ms;
    }

    Uart_Printf(DEBUG_UART, "log mode=%s state=%s samples=%u/%u duration=%lums\r\n",
                (data_log_mode == DLOG_MODE_BALANCE) ? "BALANCE" : "LINE",
                DataLog_StateName(data_log_state),
                (unsigned int)log_index,
                (unsigned int)DATA_LOG_MAX_SAMPLES,
                (unsigned long)duration_ms);
}
