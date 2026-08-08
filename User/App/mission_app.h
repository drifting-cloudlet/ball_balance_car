#ifndef __MISSION_APP_H__
#define __MISSION_APP_H__

#include <stdint.h>

/* Lap and parking logic for requirement 2 of the H problem:
 *
 *   "小车置于A点，按键启动后沿黑线顺时针行驶一圈并停到A点，计时停止并显示
 *    行驶总时间，要求行驶总时间≤20s，停车偏差≤2cm"
 *
 * Track geometry from the problem statement: AB and CD are 1.5 m straights,
 * BC and DA are half circles of radius 0.5 m, so one lap is
 *
 *     1.5 * 2 + pi * 0.5 * 2 = 614.2 cm
 *
 * and 20 s allows an average of 30.7 cm/s.
 *
 * ---- Where the car stops ----
 *
 * The rules let the team nominate the measured point, requiring only that it
 * sits on the car's centre axis. Nominate the centre of the grayscale array.
 * Any other choice introduces a fixed offset between what the sensor measures
 * and what the judge measures, and that offset has to be carried through every
 * calculation and re-measured whenever the array moves. Putting the two at the
 * same place deletes the offset instead of compensating for it.
 *
 * ---- Why odometry alone cannot stop the car ----
 *
 * 2 cm out of 614 cm is 0.33%. No wheel-encoder odometry survives a lap with
 * that accuracy, especially through two half circles where any wheel slip
 * accumulates. So odometry is used only to decide WHEN TO SLOW DOWN, and the
 * grayscale array decides WHERE TO STOP. An odometry error moves the start of
 * the approach, which costs a little time; it does not move the stop point.
 *
 * ---- Why the car slows down first ----
 *
 * The stop is triggered by seeing the line, which means the car is already on
 * it when braking begins. Everything travelled during detection and braking is
 * overshoot. At 45 cm/s that is several centimetres and the 2 cm limit is
 * already lost; at 30 cm/s one control tick is 0.3 cm and the two-frame finish
 * confirmation contributes 0.6 cm before the H-bridge brake begins.
 */

/* Which requirement is armed.
 *
 * The two are not variations of one another: requirement 2 drives the car and
 * never touches the ball, requirement 3 drives the ball and never touches the
 * car. They share the clock and display, but use separate start keys and state
 * machines rather than one state machine with "if driving" sprinkled through
 * it. */
typedef enum
{
    MISSION_MODE_LAP = 0, /* requirement 2: lap the track and park           */
    MISSION_MODE_HOLD,    /* bring-up and requirements 4/5/6: hold a target  */
    MISSION_MODE_BALL,    /* requirement 3: static O -> +5 cm -> -5 cm       */
    MISSION_MODE_Q4,      /* requirement 4: drive A -> B while holding O     */
    MISSION_MODE_Q5,      /* requirement 5: pass A after one balanced lap    */
    MISSION_MODE_Q6,      /* requirement 6: one lap at a UART-selected target */
} MissionMode_t;

/* ============================================================================
 *  INITIAL DISPLAY / UART RUN MODE
 * ============================================================================
 *
 *   MISSION_MODE_LAP   requirement 2.
 *   MISSION_MODE_HOLD  hold the ball on one target.
 *   MISSION_MODE_BALL  requirement 3, O -> +5 cm -> -5 cm. Actuator only.
 *   MISSION_MODE_Q4    requirement 4, A -> B with the ball held at O.
 *   MISSION_MODE_Q5    requirement 5, one lap with the ball held at O.
 *   MISSION_MODE_Q6    requirement 6, one lap at the balx-selected target.
 *
 * Mission_Init resets this mode every power cycle. It controls the initial OLED
 * page and the UART `run` command only. External PC0/PC2/PC3/PC13 presses select
 * and immediately start Q3/Q4/Q5/Q6; KEY2 always selects and toggles Q2.
 */
#define MISSION_BOOT_MODE     MISSION_MODE_LAP

/* New states are appended, not inserted. The lap states keep the numeric values
 * they already had so nothing that stored or compared one changes meaning. */
typedef enum
{
    MISSION_IDLE = 0,     /* both modes: waiting for the start button        */
    MISSION_RUN,          /* lap: line following at cruise speed             */
    MISSION_APPROACH,     /* lap: slowed down, close to A                    */
    MISSION_BRAKE,        /* lap: finish line seen, braking                  */
    MISSION_DONE,         /* both: finished, elapsed time frozen             */
    MISSION_BALL_POS,     /* ball: driving to +5 cm                          */
    MISSION_BALL_NEG,     /* ball: driving to -5 cm                          */
    MISSION_HOLD,         /* hold: parked on a target, runs until stopped    */
    MISSION_Q5_PASSED,    /* Q5/Q6: clock frozen, still line following       */
} MissionState_t;

/* One lap, from the problem statement. */
#define MISSION_LAP_CM              614.2f

/* Below this the finish-line detector is disabled. The car starts parked ON the
 * start line, so without this gate it would finish the instant it started. */
#define MISSION_ARM_CM              500.0f

/* Slow down from here. 580 cm leaves about 34 cm before the nominal A line.
 * At the default -50 cm/s cruise and shared 30 cm/s^2 ramp, reaching the
 * -30 cm/s approach target takes about 26.7 cm and leaves roughly 7.5 cm at the
 * final approach speed. This setting favors lap time over braking distance. */
#define MISSION_APPROACH_CM         580.0f

/* Odometry says a full lap and then some, and the line was never seen. Stop
 * anyway: a missed detection should not turn into a second lap. */
#define MISSION_MAX_CM              700.0f

/* Forward is negative on this chassis. */
#define MISSION_APPROACH_SPEED_CM_S (-30)

/* Channels that must be dark at once to call it the finish line. The track line
 * covers two at a 1.2 cm pitch; the 5 cm perpendicular line covers about four.
 * Tunable at runtime with "xline:<n>" because it depends on how high the array
 * rides and how well it is calibrated. */
#define MISSION_CROSS_MIN_CHANNELS  4U

/* Consecutive control ticks the pattern must hold. Two ticks is 20 ms, which at
 * the approach speed is 0.6 cm of extra travel: cheap insurance against a
 * single noisy frame ending the run early. */
#define MISSION_CROSS_CONFIRM_TICKS 2U

/* Wheels are considered stopped below this, for this many ticks. */
#define MISSION_STOP_SPEED_CM_S     1.5f
#define MISSION_STOP_CONFIRM_TICKS  10U

/* ---- Requirement 3: static ball sequence ----------------------------------
 *
 *   "小车静止，钢球从中心点O运行到+5cm，再折返运行到-5cm并稳定,
 *    总时间不超过5s, ±5cm处最大误差绝对值不超过1cm"
 *
 * ---- Why both waypoints are settled on, not just touched ----
 *
 * The wording attaches "并稳定" only to -5 cm, so the obvious reading is that
 * +5 cm merely has to be reached. But the error limit is stated for ±5 cm, both
 * of them: a ball that sails through +5 cm to +6.5 cm before turning around has
 * already exceeded 1 cm AT the +5 cm point. Requiring it to be stationary there
 * is what makes the overshoot impossible instead of merely unlikely.
 *
 * It is affordable. At zeta*omega = 1.8/s the 0 -> +50 mm step settles inside
 * 1 cm in about 1.0 s and +50 -> -50 mm in about 1.3 s, so the sequence costs
 * roughly 1.0 + 0.3 + 1.3 = 2.6 s against a 5 s limit. The dwell is the cheapest
 * insurance on the board.
 */
#define MISSION_BALL_TARGET_M       0.050f

/* Deliberately tighter than the 10 mm being scored. The judge measures the ball;
 * this measures the camera's opinion of the ball. The gap between those two is
 * where the vision static error lives, so the internal gate has to leave room
 * for it. */
#define MISSION_BALL_TOL_M          0.008f
#define MISSION_BALL_VEL_M_S        0.025f

/* 10 ms ticks the waypoint must hold before moving on. */
#define MISSION_BALL_DWELL_TICKS    30U

/* ---- Hold mode -------------------------------------------------------------
 *
 * Park the ball on one target and stay there. It exists for two reasons and
 * both matter:
 *
 *   1. It is the step before requirement 3. A three-waypoint sequence gives you
 *      no way to tell an unstable loop from a wrong dwell condition, and gains
 *      cannot be tuned against a moving target. Get the ball to sit still on one
 *      point first; everything after that is sequencing.
 *
 *   2. It is what requirements 4, 5 and 6 actually are - hold a target while the
 *      car drives, with requirement 6 choosing the target. Driving gets layered
 *      on top of this mode rather than beside it.
 *
 * The mode's own parameter, so aborting requirement 3 at -50 mm does not leave
 * the next hold run starting from there. */
#define MISSION_HOLD_DEFAULT_M      0.000f

/* Requirement 4 uses its own speed so Q2/bench tuning cannot silently change a
 * competition run. Forward is negative on this chassis. It scores passage
 * through B, 150 cm from A; stop with 10 cm of odometry margin so wheel-scale
 * error cannot leave the judged point short. */
#define MISSION_Q4_CRUISE_SPEED_CM_S (-35)
#define MISSION_Q4_FINISH_CM         160.0f

/* At 35 cm/s and the shared 30 cm/s^2 command ramp, an ideal stop takes
 * v^2/(2a) = 20.4 cm. Begin close to 140 cm so the car crosses B while slowing
 * and reaches the 160 cm parking target without an abrupt H-bridge brake. */
#define MISSION_Q4_DECEL_START_CM \
    (MISSION_Q4_FINISH_CM - 20.0f)

/* Q5 rewards ball stability and allows 30 s for 614.2 cm. At 30 cm/s the
 * nominal lap including the shared startup ramp is about 21.0 s, leaving useful
 * margin without carrying Q4's higher cruise speed around the full track. */
#define MISSION_Q5_CRUISE_SPEED_CM_S (-30)

void Mission_Init(void);

/* Target for hold mode and Q6, metres, positive toward the front of the car.
 * The clamped value is stored for the next Q6 run and applied immediately to
 * any balance loop that is already enabled. */
void Mission_SetHoldTarget(float x_m);
float Mission_GetHoldTarget(void);

/* Selecting a mode stops whatever was running. Switching mid-run would leave
 * the other mode's actuator wherever it happened to be. */
void Mission_SetMode(MissionMode_t mode);
MissionMode_t Mission_GetMode(void);

/* Runs in the 10 ms control interrupt, after Encoder_Task and before
 * PID_Speed_Task: it reads the distance the encoder just updated and writes the
 * speed the PID is about to use. */
void Mission_Task(void);

void Mission_Start(void);
void Mission_Abort(void);
void Mission_Toggle(void);
void Mission_Q2Toggle(void);

/* External mission keys call these start helpers directly. Pressing the same
 * key again aborts; PC1 immediately holds the most recently stored Q6 target
 * without starting the wheels. */
void Mission_Q3Start(void);
void Mission_Q6HoldTarget(void);
void Mission_Q3Stop(void);

void Mission_Q4Start(void);
void Mission_Q5Start(void);

void Mission_Q6Start(void);
void Mission_Q6Toggle(void);

MissionState_t Mission_GetState(void);
uint32_t Mission_GetElapsedMs(void);
float Mission_GetDistanceCm(void);
uint8_t Mission_LineWasMissed(void);
void Mission_PrintStatus(void);

/* 0 is an explicit bench-only override that hands KEY2 to the plain PID toggle;
 * normal competition operation leaves this enabled, making KEY2 Q2-only. */
extern volatile uint8_t mission_enabled;
extern volatile uint8_t mission_cross_min_channels;

#endif
