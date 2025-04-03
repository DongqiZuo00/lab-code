/* ESC driver for crazyflie
 * Super non-portable!
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <poll.h>

#include <nuttx/sched.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>

#include <pthread.h>

#include "drivers/drv_pwm_output.h"

#include <uORB/uORB.h>
#include <uORB/topics/actuator_direct.h>

#include <parameters/param.h>

#include "Common/Time/HardwareTimer.hpp"
#include "Common/Time/Timer.hpp"
#include "Components/Logic/QuadcopterConstants.hpp"

#include "../boards/bitcraze/crazyflie21/src/board_config.h"

static volatile bool thread_running = false; /**< Daemon status flag */
static int daemon_task; /**< Handle of daemon task / thread */

extern "C" __EXPORT int esc_motors_main(int argc, char *argv[]);
int testESCMotors(float input[4], bool usingPWM);
int esc_motors_thread(int argc, char *argv[]);
int pwm_value_from_speed(const float speed);

const unsigned LOOP_FREQUENCY = 1000;  //Hz
const uint32_t updatePeriod_us = 1000000 / LOOP_FREQUENCY;

volatile unsigned numESCTimeouts = 0;
volatile unsigned numESCCmdsReceived = 0;

static int orb_sub_actuator_direct = 0;

static HardwareTimer hwtimer;

// Min and Max PPM widths in us
const int ESC_PERIOD_MIN = 950;
const int ESC_PERIOD_MAX = 2000;
const float ESC_ARM_TIME = 5.0f;  // Time to wait for ESCs to arm when running motor test

using namespace Onboard;
int currentQuadType = QuadcopterConstants::QC_TYPE_INVALID;

int PWM_RATE = 396;  // Used for ESC control only (Hz)
float period = 1000000 / PWM_RATE;  // PWM period in us

float ESCspeedToPWMConsts[2];

perf_counter_t perf_timer_loop_esc;  //Times how long loop takes to run
perf_counter_t perf_timer_run_esc;  //Times how long it takes esc_motors logic to run

// timer_index_of_motor[i] returns the timer index of ith motor
// timers generate PWM and are defined in timer_io_channels in /PX4-firmware/src/drivers/boards/crazyflie/crazyflie_timer_config.c
static int timer_index_of_motor[4] = { 0, 1, 2, 3 };

static void usage() {
  printf("usage:\n");
  printf(
      "Normal operation: esc_motors {start|stop|status|testSpeed|testPWM}\n");
  printf(
      "\tSpeed test operation (replace <Ni> with desired speed for the four motors in rad/s):\n");
  printf("\t\tesc_motors testSpeed N1 N2 N3 N4\n");
  printf(
      "\t\tIf you provide only one speed, all motors spin at same speed: esc_motors testSpeed N\n");
  printf(
      "\tPWM test operation (replace <Ni> with desired PWM for the four motors between 1000 and 2000 microseconds):\n");
  printf(
      "\tesc_motors testPWM N1 N2 N3 N4 or esc_motors testPWM N (spins all at same PWM)\n\n");
  return;
}

int pwm_value_from_speed(const float speed) {
  // Formulate signal to send to ESC

  // For closed loop low range (in ESC firmware), electrical RPM = 0 to 50,000
  // For mid-range eRPM = 0 to 100,000
  // For high-range eRPM = 0 to 200,000
  // p = # of poles of motor (12 for EMAX motors)
  // Speed of motor (in rad/s) = eRPM*(2/p)*(2*pi/60)
  // Mapping should be desPWM = ESC_PERIOD_MIN + (ESC_PERIOD_MAX - ESC_PERIOD_MIN)/(eRPM_max*(2/p)*(2*pi/60))*desSpeed
  float desPWM = ESCspeedToPWMConsts[0] + ESCspeedToPWMConsts[1] * speed;

  if (desPWM < ESC_PERIOD_MIN) {
    desPWM = ESC_PERIOD_MIN;
  } else if (desPWM > ESC_PERIOD_MAX) {
    desPWM = ESC_PERIOD_MAX;
  }

  // When speed is zero sets PWM to ESC_PERIOD_MIN instead of ESCspeedToPWMConsts[0]
  // Allows ESCs with a ESCspeedToPWMConsts[0] > 975 to properly arm
  if (speed == 0) {
    return (int) (period - ESC_PERIOD_MIN) * TIM_SCALE;
  }

  // We must scale the value sent to ioctl due to the fact that the clock frequency is scale
  // Because a high motor signal pulls the line low, invert the signal
  // We need to do this because of the pullup resistor added in place of the CF motors
  return (int) (period - desPWM) * TIM_SCALE;
}

int testESCMotors(float input[4], bool usingPWM) {
  printf("sleep  = %d\n", int(updatePeriod_us));

  /* open for ioctl only */
  const char *dev = PWM_OUTPUT0_DEVICE_PATH;
  int fd = open(dev, 0);

  if (fd < 0) {
    printf("can't open %s, err = %d\n", dev, fd);
    return -1;
  }

  int ret1 = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
  int ret2 = ioctl(fd, PWM_SERVO_ARM, 0);
  printf("Arming = %d, %d\n", ret1, ret2);

  Timer t(&hwtimer);
  const float RUN_TIME = 5.0f;

  // Wait a couple seconds to arm ESCs
  while (t.GetSeconds<float>() < ESC_ARM_TIME) {
    // Send idle commands for 5 seconds
    for (int i = 0; i < 4; i++) {
      ioctl(fd, PWM_SERVO_SET(timer_index_of_motor[i]),
            pwm_value_from_speed(0));
    }
    usleep(updatePeriod_us);
  }
  t.AdjustTimeBySeconds(-ESC_ARM_TIME);

  unsigned const printPeriod_us = 500000;
  unsigned nextPrintTime = 0;

  while (t.GetSeconds<float>() < RUN_TIME) {
    if (t.GetMicroSeconds() > nextPrintTime) {
      nextPrintTime += printPeriod_us;
      printf("Current time [s] = %.2f\n", double(t.GetSeconds<float>()));
    }

    for (int i = 0; i < 4; i++) {
      int ret;
      if (usingPWM) {
        ret = ioctl(fd, PWM_SERVO_SET(timer_index_of_motor[i]),
                    (int) (period - input[i]) * TIM_SCALE);
      } else {
        ret = ioctl(fd, PWM_SERVO_SET(timer_index_of_motor[i]),
                    pwm_value_from_speed(input[i]));
      }

      if (ret != OK) {
        if (usingPWM) {
          err(1,
              "PWM_SERVO_SET(timer_index_of_motor[%d]) = %d (%.3f PWM command)",
              i, (int) (period - input[i]) * TIM_SCALE, double(input[i]));
        } else {
          err(1,
              "PWM_SERVO_SET(timer_index_of_motor[%d]) = %d (%.3f rad/s command)",
              i, pwm_value_from_speed(input[i]), double(input[i]));
        }
      }
    }

    usleep(updatePeriod_us);
  }

  //Make sure we exit with motors off:
  for (int i = 0; i < 4; i++) {
    int ret = ioctl(fd, PWM_SERVO_SET(timer_index_of_motor[i]),
                    pwm_value_from_speed(0));

    if (ret != OK) {
      err(1, "PWM_SERVO_SET(timer_index_of_motor[%d])", i);
    }
  }

  ioctl(fd, PWM_SERVO_DISARM, 0);

  return 0;
}

int esc_motors_thread(int argc, char *argv[]) {
  orb_sub_actuator_direct = orb_subscribe(ORB_ID(actuator_direct));

  perf_timer_loop_esc = perf_alloc(PC_ELAPSED, "esc_motors_loop");
  perf_timer_run_esc = perf_alloc(PC_ELAPSED, "esc_motors_run");

  if (orb_sub_actuator_direct <= 0) {
    printf("Error in orb_sub_actuator_direct  = %d\n", orb_sub_actuator_direct);
    return -1;
  }

  printf("sleep  = %d\n", int(updatePeriod_us));

  /* open for ioctl only */
  const char *dev = PWM_OUTPUT0_DEVICE_PATH;
  int fd = open(dev, 0);

  if (fd < 0) {
    printf("can't open %s, err = %d\n", dev, fd);
    return -1;
  }

  int ret1 = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
  int ret2 = ioctl(fd, PWM_SERVO_ARM, 0);
  printf("Arming = %d, %d\n", ret1, ret2);

  px4_pollfd_struct_t fds[2];
  fds[0].fd = orb_sub_actuator_direct;
  fds[0].events = POLLIN;

  struct actuator_direct_s actuatorCmds;

  bool motorsOff = true;
  const unsigned MOTOR_TIMEOUT_ms = 50;  //if we don't receive a command every this often, kill motors.

  while (thread_running) {
    perf_begin(perf_timer_loop_esc);
    int poll_ret = px4_poll(fds, 1, MOTOR_TIMEOUT_ms);
    perf_begin(perf_timer_run_esc);
    if (poll_ret == 0) {
      numESCTimeouts++;
      if (motorsOff) {
        //nothing to be done.
        continue;
      }
      //time-out, but motors' last command wasn't zero!
      //PANIC!
      thread_running = false;
      motorsOff = true;
      printf("Timeout panic! Killing motors\n");
      break;
    }

    if (fds[0].revents & POLLIN) {
      /* obtained data for the first file descriptor */
      /* copy sensors raw data into local buffer */

      numESCCmdsReceived++;
      orb_copy(ORB_ID(actuator_direct), orb_sub_actuator_direct, &actuatorCmds);
      if (actuatorCmds.nvalues == 0 && motorsOff) {
        //empty command, most likely the first we get. Skip it.
        continue;
      }
      if (actuatorCmds.nvalues != 4) {
        //Incorrect number of motors!
        err(1, "PWM: incorrect number of motors! %d", actuatorCmds.nvalues);
        thread_running = false;
        break;
      }

      for (int i = 0; i < 4; i++) {
        if (actuatorCmds.values[i] > 0) {
          motorsOff = false;
        }

        int const pwmVal = pwm_value_from_speed(actuatorCmds.values[i]);
        int ret = ioctl(fd, PWM_SERVO_SET(timer_index_of_motor[i]), pwmVal);

        if (ret != OK) {
          err(1, "PWM_SERVO_SET(timer_index_of_motor[%d]) = %d (%.3f)", i,
              pwmVal, double(actuatorCmds.values[i]));
        }
      }
    }
    perf_end(perf_timer_run_esc);
    perf_end(perf_timer_loop_esc);
  }

  //Make sure we exit with motors off:
  for (int i = 0; i < 4; i++) {
    int ret = ioctl(fd, PWM_SERVO_SET(timer_index_of_motor[i]),
                    pwm_value_from_speed(0));

    if (ret != OK) {
      err(1, "PWM_SERVO_SET(timer_index_of_motor[%d])", i);
    }
  }

  ioctl(fd, PWM_SERVO_DISARM, 0);

  printf("Exiting: esc_motors_thread\n");

  return 0;
}

int esc_motors_main(int argc, char *argv[]) {

  if (argc < 1) {
    usage();
    return -1;
  }

  int vehId = 0;
  if (0 != param_get(param_find("VEHICLE_ID"), &vehId)) {
    printf("Failed to get param <VEHICLE_ID>\n");
    vehId = 0;
  }
  currentQuadType = Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehId);
  QuadcopterConstants consts(
      static_cast<QuadcopterConstants::QuadcopterType>(currentQuadType));
  ESCspeedToPWMConsts[0] = consts.ESCspeedToPWMConsts[0];
  ESCspeedToPWMConsts[1] = consts.ESCspeedToPWMConsts[1];

  if (!strcmp(argv[1], "testSpeed") || !strcmp(argv[1], "testPWM")) {
    if (thread_running) {
      printf("A thread already running\n");
      return -1;
    }

    float input[4];
    if (argc == 3) {
      for (int i = 0; i < 4; i++) {
        input[i] = atof(argv[2]);
      }
    } else if (argc == 6) {
      for (int i = 0; i < 4; i++) {
        input[i] = atof(argv[2 + i]);
      }
    } else {
      usage();
      return -1;
    }

    bool usingPWM;
    if (!strcmp(argv[1], "testSpeed")) {
      usingPWM = false;
      testESCMotors(input, usingPWM);
    } else if (!strcmp(argv[1], "testPWM")) {
      usingPWM = true;
      testESCMotors(input, usingPWM);
    }

    usleep(100 * 1000);

    return 0;
  }

  if (!strcmp(argv[1], "start")) {
    if (thread_running) {
      printf("Thread already running\n");
      return 0;
    }

    usleep(100000);

    thread_running = true;

    daemon_task = px4_task_spawn_cmd(
        "esc_motors", SCHED_DEFAULT, SCHED_PRIORITY_MAX - 10, 2000,
        esc_motors_thread,
        (argv) ? (char * const *) &argv[2] : (char * const *) NULL);

    printf("Started\n");
    return 0;
  }

  if (!strcmp(argv[1], "stop")) {
    if (!thread_running) {
      printf("Nothing to do...\n");
      return 0;
    }

    thread_running = false;

    printf("Stopped!\n");
    return 0;
  }

  if (!strcmp(argv[1], "test") || !strcmp(argv[1], "t")) {
    if (argc < 2) {
      usage();
      return -1;
    }

    //TODO: test modes
  }

  if (!strcmp(argv[1], "status")) {

    printf("\tmotors is running.\n");

    printf("Quad type = <%s>\n",
        QuadcopterConstants::GetNameString(
            QuadcopterConstants::QuadcopterType(currentQuadType)));
    printf("\t\tPWM mapping consts c0=%.3f, c1=%.3f\n", double(ESCspeedToPWMConsts[0]),
           double(ESCspeedToPWMConsts[1]));

    if (thread_running) {
      printf("\tmotors is running.\n");
      printf("\tNum timeouts = %d, num cmds = %d\n", numESCTimeouts,
             numESCCmdsReceived);
    } else {
      printf("\tmotors not started\n");
    }
    return 0;
  }

  usage();
  return -1;
}
