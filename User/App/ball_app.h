#ifndef __BALL_APP_H__
#define __BALL_APP_H__

#include <stdint.h>

/* Receives ball position frames from the MaixCAM2 on USART6 and turns them into
 * a filtered position and velocity estimate.
 *
 * Frame format is frozen in .trellis/spec/backend/ball-vision-uart-contract.md:
 *
 *     $B,<seq>,<x_0.1mm>,<valid>,<conf>*<XOR>\r\n
 *
 * The MaixCAM2 touchscreen may insert one `balx:<mm>\r\n` operator command
 * between complete frames. It does not participate in frame sequence, timing,
 * statistics, or estimation; production USART6 ignores every other non-frame
 * line.
 *
 * The wire unit is 0.1 mm. Everything this module exposes is in metres, and the
 * conversion happens in exactly one place. Do not reintroduce millimetres into
 * the control path. */

/* Which UART carries the ball frames.
 *
 * USART6 (PC6/PC7) is the real wiring. The camera and the F407 are both bolted
 * to the same car, so the link is one short cable at 115200.
 *
 * UART5 is the bring-up detour: it is the port with the Zigbee module on it, so
 * the parser can be exercised from a laptop with a single USB-TTL adapter. It
 * runs at 9600, which caps the usable frame rate near 20 Hz, and a wireless hop
 * adds tens of milliseconds of jitter that no estimator can undo. Verify the
 * parser here, then switch back. Never run the closed loop on it. */
#define BALL_SOURCE_USART6       0
#define BALL_SOURCE_UART5        1

#define BALL_SOURCE              BALL_SOURCE_USART6

typedef enum
{
    /* No usable measurement. The outer loop must disengage and level the rod. */
    BALL_STATE_LOST = 0,
    /* Short dropout. The estimate is still worth using but the integral term
     * must be frozen. */
    BALL_STATE_COASTING = 1,
    /* Fresh measurements arriving. */
    BALL_STATE_TRACKING = 2,
} BallState_t;

/* Alpha-beta tracker. beta = alpha^2 / (2 - alpha) is the critically damped
 * choice; these values sit slightly under it so the velocity estimate stays
 * calm on a noisy detection. */
#define BALL_ALPHA               0.40f
#define BALL_BETA                0.10f

/* Largest residual between prediction and measurement that is still believable.
 * Anything past this is a detection glitch, not ball motion. */
#define BALL_MAX_JUMP_M          0.030f

/* Physical travel of the ball centre. Frames outside this are nonsense. */
#define BALL_TRAVEL_LIMIT_M      0.125f

/* Dropout thresholds.
 *
 * The physically meaningful quantity is "how many frames did I miss", not "how
 * many milliseconds passed". The contract's 100 ms and 500 ms are 3 and 15
 * frames at the 30 Hz it assumes, so both forms are used and whichever is
 * longer wins. That keeps production behaviour identical while stopping a slow
 * bring-up link from reporting a dropout every time a frame is one period late,
 * which is what happens when the threshold equals the frame interval. */
#define BALL_COAST_TIMEOUT_MS    100U
#define BALL_LOST_TIMEOUT_MS     500U
#define BALL_COAST_FRAMES        3U
#define BALL_LOST_FRAMES         15U
#define BALL_TASK_PERIOD_MS      5U

/* Ball_GetPosition extrapolates from the last measurement to now, which removes
 * the one-frame sampling lag. The horizon is capped so a stalled vision link
 * cannot produce a runaway position. */
#define BALL_MAX_PREDICT_MS      60U

/* Guards the alpha-beta divide against a zero or absurd frame interval. */
#define BALL_DT_MIN_S            0.005f
#define BALL_DT_MAX_S            0.100f

/* On the production USART6 link, the frame interval is derived from F407
 * arrival ticks. The direct wire does not buffer frames, so this measures the
 * interval that the estimator actually experienced without assuming the
 * MaixCAM's configured frame rate.
 *
 * The optional UART5 wireless bring-up path can buffer and burst frames. That
 * build uses seq_delta times a long-window sender-period estimate instead. */
#define BALL_PERIOD_WINDOW_MS    1000U
#define BALL_PERIOD_DEFAULT_S    0.020f
#define BALL_PERIOD_MIN_S        0.004f
#define BALL_PERIOD_MAX_S        0.200f

/* Frames needed for the first, provisional period estimate. Waiting a whole
 * window would mean running the gate against BALL_PERIOD_DEFAULT_S, and any
 * default is wrong for some sender: guessing 50 Hz against a 10 Hz sender makes
 * dt five times too small and the gate throws away the first second of good
 * data. Bootstrap from the data instead, then refine on the long window. */
#define BALL_PERIOD_BOOTSTRAP_FRAMES  4U

/* A 1 cm ball in a 25 cm tube cannot usefully exceed this. Clamping stops a bad
 * update from cascading into the prediction. */
#define BALL_MAX_SPEED_M_S       1.0f

void Ball_Init(void);
void Ball_Task(void);

/* Estimated position in metres, extrapolated to the current tick.
 * Positive is toward the front of the car, zero is the rod centre.
 * When the state is BALL_STATE_LOST this returns the last known estimate rather
 * than zero: zero means "centred", which is exactly the wrong thing to tell a
 * controller that has lost the ball. Always gate on Ball_IsUsable(). */
float Ball_GetPosition(void);

/* Estimated velocity in m/s. Use this for the derivative term; never difference
 * the raw measurement. */
float Ball_GetVelocity(void);

BallState_t Ball_GetState(void);
uint8_t Ball_IsUsable(void);

/* Milliseconds since the last accepted measurement. */
uint32_t Ball_GetAge(void);

/* Milliseconds since any well formed frame arrived, valid or not.
 *
 * This is a different question from Ball_GetAge(). "Vision is running and
 * honestly reporting valid=0" and "the vision link is dead" both stop the
 * measurement clock, but only the second one is a fault: the contract requires
 * the sender to keep emitting valid=0 frames when it loses the ball, so silence
 * means the program crashed or the cable came off. */
uint32_t Ball_GetLinkAge(void);
uint8_t Ball_IsLinkAlive(void);

void Ball_PrintStats(void);
void Ball_ResetStats(void);

#endif
