#include "ball_app.h"
#include "MyDefine.h"
#include <stdio.h>
#include <string.h>

/* Longest legal frame body is about 26 characters. */
#define BALL_LINE_MAX            48U

#if BALL_SOURCE == BALL_SOURCE_UART5
extern struct rt_ringbuffer uart5_ring_buffer;
#define BALL_RING_BUFFER     (&uart5_ring_buffer)
#define BALL_SOURCE_NAME     "UART5 (bring-up, 9600 via Zigbee)"
#else
extern struct rt_ringbuffer uart6_ring_buffer;
#define BALL_RING_BUFFER     (&uart6_ring_buffer)
#define BALL_SOURCE_NAME     "USART6 (PC6/PC7, 115200)"
#endif

typedef struct
{
    uint32_t frames_total;      /* well formed frames that passed the checksum */
    uint32_t frames_ok;         /* accepted into the estimator                 */
    uint32_t frames_invalid;    /* vision reported valid=0                     */
    uint32_t frames_checksum;   /* checksum mismatch                           */
    uint32_t frames_malformed;  /* no '*', bad field count, overlong line      */
    uint32_t frames_jump;       /* rejected by the residual gate               */
    uint32_t frames_range;      /* position outside physical travel            */
    uint32_t seq_gaps;          /* frames the sender emitted that never landed */
} BallStats_t;

static BallStats_t ball_stats;

/* Updated by Ball_Task on the main loop and sampled by Mission_Task in the
 * 10 ms timer ISR. Cortex-M4 word accesses are atomic; volatile prevents stale
 * cached values across those two contexts. */
static volatile float ball_x = 0.0f;      /* estimated position, metres */
static volatile float ball_v = 0.0f;      /* estimated velocity, m/s    */
static volatile BallState_t ball_state = BALL_STATE_LOST;
static volatile uint32_t ball_last_valid_tick = 0U;
static volatile uint32_t ball_last_frame_tick = 0U;
static uint32_t ball_last_accepted_tick = 0U;
static uint8_t ball_last_seq = 0U;
#if BALL_SOURCE == BALL_SOURCE_UART5
static uint8_t ball_last_accepted_seq = 0U;
#endif
static uint8_t ball_last_conf = 0U;
static uint8_t ball_seq_seen = 0U;

/* Sender frame period, measured from seq over a long window. */
static float ball_frame_period_s = BALL_PERIOD_DEFAULT_S;
static uint32_t ball_period_ref_tick = 0U;
static uint32_t ball_period_seq_window = 0U;
static uint8_t ball_period_measured = 0U;

static uint32_t Ball_ScaledTimeoutMs(uint32_t floor_ms, uint32_t frames)
{
    uint32_t scaled = (uint32_t)(ball_frame_period_s * 1000.0f * (float)frames);

    return (scaled > floor_ms) ? scaled : floor_ms;
}

static char ball_line[BALL_LINE_MAX];
static uint8_t ball_line_len = 0U;
static uint8_t ball_collecting = 0U;

static void Ball_Accept(uint8_t seq, int32_t x_dmm, uint8_t valid, uint8_t conf)
{
    uint32_t now = HAL_GetTick();
    uint8_t seq_delta;
    float x_meas;
    float dt;
    float x_pred;
    float residual;

    if (ball_seq_seen == 0U)
    {
        ball_period_ref_tick = now;
        ball_period_seq_window = 0U;
        seq_delta = 1U;
    }
    else
    {
        seq_delta = (uint8_t)(seq - ball_last_seq);
        if (seq_delta == 0U)
        {
            /* Duplicate sequence number. Treat it as one period rather than
             * letting the unsigned wrap report 255 lost frames. */
            seq_delta = 1U;
        }
        else
        {
            ball_stats.seq_gaps += (uint32_t)(seq_delta - 1U);
        }
    }
    ball_last_seq = seq;
    ball_seq_seen = 1U;

    /* Measure the sender's frame period over a window long enough that burst
     * delivery averages out. */
    ball_period_seq_window += seq_delta;
    if (((now - ball_period_ref_tick) >= BALL_PERIOD_WINDOW_MS) ||
        ((ball_period_measured == 0U) &&
         (ball_period_seq_window >= BALL_PERIOD_BOOTSTRAP_FRAMES)))
    {
        if (ball_period_seq_window != 0U)
        {
            float measured = (float)(now - ball_period_ref_tick) /
                             (1000.0f * (float)ball_period_seq_window);

            if (measured < BALL_PERIOD_MIN_S)
            {
                measured = BALL_PERIOD_MIN_S;
            }
            if (measured > BALL_PERIOD_MAX_S)
            {
                measured = BALL_PERIOD_MAX_S;
            }
            ball_frame_period_s = measured;
            ball_period_measured = 1U;
        }
        ball_period_ref_tick = now;
        ball_period_seq_window = 0U;
    }
    ball_last_conf = conf;
    ball_last_frame_tick = now;
    ++ball_stats.frames_total;

    if (valid == 0U)
    {
        ++ball_stats.frames_invalid;
        return;
    }

    /* The only place the wire unit becomes metres. */
    x_meas = (float)x_dmm * 0.0001f;

    if ((x_meas < -BALL_TRAVEL_LIMIT_M) || (x_meas > BALL_TRAVEL_LIMIT_M))
    {
        ++ball_stats.frames_range;
        return;
    }

    if (ball_state == BALL_STATE_LOST)
    {
        /* Reacquisition: adopt the measurement outright. Running it through the
         * residual gate here would reject every recovery, since the stale
         * estimate can be arbitrarily far away. */
        ball_x = x_meas;
        ball_v = 0.0f;
        ball_last_valid_tick = now;
        ball_last_accepted_tick = now;
#if BALL_SOURCE == BALL_SOURCE_UART5
        ball_last_accepted_seq = seq;
#endif
        ball_state = BALL_STATE_TRACKING;
        ++ball_stats.frames_ok;
        return;
    }

    /* On the production point-to-point USART6 wire, arrival time is measurement
     * time. On the optional wireless UART5 bring-up path, frames can arrive in
     * bursts, so sequence count is the more truthful clock. */
#if BALL_SOURCE == BALL_SOURCE_UART5
    dt = (float)((uint8_t)(seq - ball_last_accepted_seq)) * ball_frame_period_s;
#else
    dt = (float)(now - ball_last_accepted_tick) * 0.001f;
#endif
    if (dt < BALL_DT_MIN_S)
    {
        dt = BALL_DT_MIN_S;
    }
    if (dt > BALL_DT_MAX_S)
    {
        dt = BALL_DT_MAX_S;
    }

    x_pred = ball_x + (ball_v * dt);
    residual = x_meas - x_pred;

    /* The gate compares a measurement against a prediction, and the prediction
     * is only as good as dt. Until the sender's rate has been measured there is
     * nothing trustworthy to compare against, so gating would only throw away
     * the data needed to measure it. */
    if ((ball_period_measured != 0U) &&
        ((residual > BALL_MAX_JUMP_M) || (residual < -BALL_MAX_JUMP_M)))
    {
        ++ball_stats.frames_jump;
        return;
    }

    ball_x = x_pred + (BALL_ALPHA * residual);
    ball_v = ball_v + ((BALL_BETA * residual) / dt);

    if (ball_v > BALL_MAX_SPEED_M_S)
    {
        ball_v = BALL_MAX_SPEED_M_S;
    }
    if (ball_v < -BALL_MAX_SPEED_M_S)
    {
        ball_v = -BALL_MAX_SPEED_M_S;
    }

    ball_last_valid_tick = now;
    ball_last_accepted_tick = now;
#if BALL_SOURCE == BALL_SOURCE_UART5
    ball_last_accepted_seq = seq;
#endif
    ball_state = BALL_STATE_TRACKING;
    ++ball_stats.frames_ok;
}

static void Ball_ParseLine(char *line)
{
    char *star = strchr(line, '*');
    char *cursor;
    unsigned int received_checksum = 0U;
    unsigned int seq = 0U;
    unsigned int valid = 0U;
    unsigned int conf = 0U;
    int x_dmm = 0;
    int checksum_chars = 0;
    int body_chars = 0;
    uint8_t checksum = 0U;

    if (star == NULL)
    {
        ++ball_stats.frames_malformed;
        return;
    }
    *star = '\0';

    if ((sscanf(star + 1, "%2x%n", &received_checksum,
                &checksum_chars) != 1) ||
        (checksum_chars != 2) || (star[3] != '\0') ||
        (received_checksum > 0xFFU))
    {
        ++ball_stats.frames_malformed;
        return;
    }

    for (cursor = line; *cursor != '\0'; ++cursor)
    {
        checksum ^= (uint8_t)(*cursor);
    }

    if (checksum != (uint8_t)received_checksum)
    {
        ++ball_stats.frames_checksum;
        return;
    }

    if ((sscanf(line, "B,%u,%d,%u,%u%n", &seq, &x_dmm,
                &valid, &conf, &body_chars) != 4) ||
        (body_chars <= 0) || (line[body_chars] != '\0') ||
        (seq > 255U) || (valid > 1U) || (conf > 100U))
    {
        ++ball_stats.frames_malformed;
        return;
    }

    Ball_Accept((uint8_t)seq, (int32_t)x_dmm,
                (valid != 0U) ? 1U : 0U, (uint8_t)conf);
}

/* Non-frame lines share the wire with '$'-prefixed ball frames. UART5 keeps its
 * bring-up console behavior; production USART6 accepts only the MaixCAM Q6
 * target command and silently ignores all other UART0 startup/log text. */
static char ball_cmd[BALL_LINE_MAX];
static uint8_t ball_cmd_len = 0U;

static void Ball_FeedCommandByte(char byte)
{
    if ((byte == '\r') || (byte == '\n'))
    {
        if (ball_cmd_len != 0U)
        {
            char first = ball_cmd[0];

            ball_cmd[ball_cmd_len] = '\0';
            ball_cmd_len = 0U;

#if BALL_SOURCE == BALL_SOURCE_UART5
            /* Every bring-up console command is lower case. Frame debris is
             * counted instead of echoed because UART5 has only 960 byte/s. */
            if ((first >= 'a') && (first <= 'z'))
            {
                Uart_Parse_Command(ball_cmd);
            }
            else
            {
                ++ball_stats.frames_malformed;
            }
#else
            (void)first;
            if (strncmp(ball_cmd, "balx:", 5U) == 0)
            {
                Uart_Parse_Command(ball_cmd);
            }
#endif
        }
        return;
    }

    if (ball_cmd_len >= (BALL_LINE_MAX - 1U))
    {
        ball_cmd_len = 0U;
        return;
    }

    ball_cmd[ball_cmd_len] = byte;
    ++ball_cmd_len;
}

static void Ball_FeedByte(char byte)
{
    /* '$' always restarts the line. That single rule is the whole resync
     * strategy: a truncated or garbled frame needs no special handling because
     * the next header discards whatever is half assembled. */
    if (byte == '$')
    {
        ball_cmd_len = 0U;
        ball_line_len = 0U;
        ball_collecting = 1U;
        return;
    }

    if (ball_collecting == 0U)
    {
        Ball_FeedCommandByte(byte);
        return;
    }

    if (byte == '\r')
    {
        return;
    }

    if (byte == '\n')
    {
        ball_line[ball_line_len] = '\0';
        ball_collecting = 0U;
        Ball_ParseLine(ball_line);
        return;
    }

    if (ball_line_len >= (BALL_LINE_MAX - 1U))
    {
        ball_collecting = 0U;
        ++ball_stats.frames_malformed;
        return;
    }

    ball_line[ball_line_len] = byte;
    ++ball_line_len;
}

void Ball_Init(void)
{
    /* DMA reception and the ring buffer are already brought up by Uart_Init()
     * for every port; this module only drains the one it was pointed at. */
    ball_x = 0.0f;
    ball_v = 0.0f;
    ball_state = BALL_STATE_LOST;
    ball_last_valid_tick = HAL_GetTick();
    ball_last_frame_tick = ball_last_valid_tick;
    ball_last_seq = 0U;
#if BALL_SOURCE == BALL_SOURCE_UART5
    ball_last_accepted_seq = 0U;
#endif
    ball_last_conf = 0U;
    ball_seq_seen = 0U;
    ball_last_accepted_tick = ball_last_valid_tick;
    ball_frame_period_s = BALL_PERIOD_DEFAULT_S;
    ball_period_ref_tick = ball_last_valid_tick;
    ball_period_seq_window = 0U;
    ball_period_measured = 0U;
    ball_line_len = 0U;
    ball_collecting = 0U;
    ball_cmd_len = 0U;
    Ball_ResetStats();
    Uart_Printf(DEBUG_UART,
                "Ball_Init source=%s waiting for $B frames\r\n",
                BALL_SOURCE_NAME);
#if BALL_SOURCE == BALL_SOURCE_UART5
    Uart_Printf(DEBUG_UART,
                "  WARNING bring-up port: 9600 caps the frame rate near 20Hz "
                "and the wireless hop adds jitter. Switch BALL_SOURCE back to "
                "USART6 before any closed-loop run.\r\n");
#endif
}

void Ball_Task(void)
{
    rt_uint8_t byte;
    uint32_t age;

#if BALL_SOURCE == BALL_SOURCE_USART6
    Uart6_RxEnsureStarted();
#endif

    while (rt_ringbuffer_getchar(BALL_RING_BUFFER, &byte) != 0U)
    {
        Ball_FeedByte((char)byte);
    }

    if (ball_state == BALL_STATE_LOST)
    {
        return;
    }

    age = HAL_GetTick() - ball_last_valid_tick;

    if (age > Ball_ScaledTimeoutMs(BALL_LOST_TIMEOUT_MS, BALL_LOST_FRAMES))
    {
        ball_state = BALL_STATE_LOST;
        ball_v = 0.0f;
    }
    else if (age > Ball_ScaledTimeoutMs(BALL_COAST_TIMEOUT_MS, BALL_COAST_FRAMES))
    {
        ball_state = BALL_STATE_COASTING;
    }
}

float Ball_GetPosition(void)
{
    uint32_t age;

    if (ball_state == BALL_STATE_LOST)
    {
        return ball_x;
    }

    age = HAL_GetTick() - ball_last_valid_tick;
    if (age > BALL_MAX_PREDICT_MS)
    {
        age = BALL_MAX_PREDICT_MS;
    }

    return ball_x + (ball_v * ((float)age * 0.001f));
}

float Ball_GetVelocity(void)
{
    return ball_v;
}

BallState_t Ball_GetState(void)
{
    return ball_state;
}

uint8_t Ball_IsUsable(void)
{
    return (ball_state == BALL_STATE_LOST) ? 0U : 1U;
}

uint32_t Ball_GetAge(void)
{
    return HAL_GetTick() - ball_last_valid_tick;
}

uint32_t Ball_GetLinkAge(void)
{
    return HAL_GetTick() - ball_last_frame_tick;
}

uint8_t Ball_IsLinkAlive(void)
{
    return (Ball_GetLinkAge() <=
            Ball_ScaledTimeoutMs(BALL_COAST_TIMEOUT_MS, BALL_COAST_FRAMES))
           ? 1U : 0U;
}

void Ball_ResetStats(void)
{
    memset(&ball_stats, 0, sizeof(ball_stats));
#if BALL_SOURCE == BALL_SOURCE_USART6
    Uart6_RxResetDiagnostics();
#endif
}

void Ball_PrintStats(void)
{
    static const char *const state_name[] = {"LOST", "COAST", "TRACK"};

    /* Two short lines, not three long ones. Every character costs a millisecond
     * on a 9600 bring-up link, and a stats dump that overruns the transmit
     * queue destroys the statistics it was printing. */
    Uart_Printf(DEBUG_UART,
                "ball %s x=%.1fmm v=%.1fmm/s age=%ums rate=%.1fHz c=%u %s(%ums)\r\n",
                state_name[ball_state],
                Ball_GetPosition() * 1000.0f,
                ball_v * 1000.0f,
                Ball_GetAge(),
                (ball_frame_period_s > 0.0f) ? (1.0f / ball_frame_period_s) : 0.0f,
                ball_last_conf,
                (Ball_IsLinkAlive() != 0U) ? "UP" : "DOWN",
                Ball_GetLinkAge());
    Uart_Printf(DEBUG_UART,
                "ball tot=%u ok=%u inv=%u | crc=%u mal=%u jmp=%u rng=%u gap=%u\r\n",
                ball_stats.frames_total,
                ball_stats.frames_ok,
                ball_stats.frames_invalid,
                ball_stats.frames_checksum,
                ball_stats.frames_malformed,
                ball_stats.frames_jump,
                ball_stats.frames_range,
                ball_stats.seq_gaps);
#if BALL_SOURCE == BALL_SOURCE_USART6
    {
        Uart6RxDiagnostics_t uart6;

        Uart6_RxGetDiagnostics(&uart6);
        Uart_Printf(DEBUG_UART,
                    "u6 evt=%lu bytes=%lu drop=%lu err=%lu rec=%lu fail=%lu last=0x%lX\r\n",
                    (unsigned long)uart6.events,
                    (unsigned long)uart6.bytes,
                    (unsigned long)uart6.dropped,
                    (unsigned long)uart6.errors,
                    (unsigned long)uart6.recoveries,
                    (unsigned long)uart6.start_failures,
                    (unsigned long)uart6.last_error);
    }
#endif
}
