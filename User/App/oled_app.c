#include "oled_app.h"

static uint8_t oled_result_valid = 0U;
static MissionMode_t oled_result_mode = MISSION_MODE_LAP;
static MissionState_t oled_result_state = MISSION_IDLE;

static const char *Oled_ModeName(MissionMode_t mode)
{
  switch (mode)
  {
    case MISSION_MODE_LAP:  return "Q2";
    case MISSION_MODE_HOLD: return "HOLD";
    case MISSION_MODE_BALL: return "Q3";
    case MISSION_MODE_Q4:   return "Q4";
    case MISSION_MODE_Q5:   return "Q5";
    case MISSION_MODE_Q6:   return "Q6";
    default:                return "ERR";
  }
}

static const char *Oled_MissionName(MissionState_t state)
{
  switch (state)
  {
    case MISSION_IDLE:     return "RDY";
    case MISSION_RUN:      return "RUN";
    case MISSION_APPROACH: return "APPR";
    case MISSION_BRAKE:    return "BRK";
    case MISSION_DONE:     return "END";
    case MISSION_BALL_POS: return "B+5";
    case MISSION_BALL_NEG: return "B-5";
    case MISSION_HOLD:     return "HOLD";
    case MISSION_Q5_PASSED:return "PASS";
    default:               return "ERR";
  }
}

static const char *Oled_LineName(void)
{
  if (Gray_Is_Online() == 0U)
  {
    return "ERR";
  }
  return (g_line_detected != 0U) ? "LOCK" : "LOST";
}

static uint8_t Oled_MissionFinished(MissionMode_t mode, MissionState_t state)
{
  if ((mode == MISSION_MODE_LAP) ||
      (mode == MISSION_MODE_BALL) ||
      (mode == MISSION_MODE_Q4))
  {
    return (state == MISSION_DONE) ? 1U : 0U;
  }

  if ((mode == MISSION_MODE_Q5) || (mode == MISSION_MODE_Q6))
  {
    return (state == MISSION_Q5_PASSED) ? 1U : 0U;
  }

  return 0U;
}

void Oled_Init(void)
{
  Uart_Printf(DEBUG_UART, "Oled_Init ......\r\n");
  OLED_Init();
  OLED_Clear();
}

void Oled_Task(void)
{
  uint8_t track = TrackN;
  int error10 = (int)(g_line_position_error * 10.0f);
  uint32_t elapsed_ms = Mission_GetElapsedMs();
  char calibration =
      (Gray_GetCalibrationStage() == GRAY_CALIBRATION_WAIT_BLACK) ? 'B' : 'W';

  oled_result_valid = 0U;
  Oled_Printf(0, 0, "SEL:%-4s %-4s        ",
              Oled_ModeName(Mission_GetMode()),
              Oled_MissionName(Mission_GetState()));
  Oled_Printf(0, 1, "TIME:%2lu.%02lus          ",
              (unsigned long)(elapsed_ms / 1000U),
              (unsigned long)((elapsed_ms % 1000U) / 10U));
  Oled_Printf(0, 2, "LINE:%-4s E:%+4d     ",
              Oled_LineName(), error10);
  Oled_Printf(0, 3, "ADC:%-3s T:%02X C:%c N:%u ",
              (Gray_Is_Online() != 0U) ? "OK" : "ERR",
              (unsigned int)track, calibration,
              (unsigned int)g_line_dark_count);
}

void Oled_RuntimeTask(void)
{
  uint32_t elapsed_ms = Mission_GetElapsedMs();
  MissionMode_t mission_mode = Mission_GetMode();
  MissionState_t mission_state = Mission_GetState();

  /* Every OLED byte is a blocking 100 kHz I2C transaction. During a mission,
   * do no display I/O at all so Ball -> Balance -> Pendulum keeps its 20 ms
   * cadence. Q3/Q4 finish in DONE; Q5/Q6 finish when the lap passes A. */
  if (Oled_MissionFinished(mission_mode, mission_state) == 0U)
  {
    oled_result_valid = 0U;
    return;
  }

  if ((oled_result_valid != 0U) &&
      (oled_result_mode == mission_mode) &&
      (oled_result_state == mission_state))
  {
    return;
  }

  oled_result_mode = mission_mode;
  oled_result_state = mission_state;
  oled_result_valid = 1U;

  /* External keys now select and start in one edge, so no ready-page refresh is
   * guaranteed before the run. Write the final mode/state label and time once. */
  Oled_Printf(0U, 0U, "SEL:%-4s %-4s",
              Oled_ModeName(mission_mode),
              Oled_MissionName(mission_state));
  Oled_Printf(30U, 1U, "%2lu.%02lus",
              (unsigned long)(elapsed_ms / 1000U),
              (unsigned long)((elapsed_ms % 1000U) / 10U));
}
