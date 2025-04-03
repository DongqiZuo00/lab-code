/* Motor driver for crazyflie
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
#include <uORB/topics/battery_status.h>

#include <parameters/param.h>

#include "Common/Time/HardwareTimer.hpp"
#include "Common/Time/Timer.hpp"
#include "Components/Logic/QuadcopterConstants.hpp"

static volatile bool thread_running = false; /**< Daemon status flag */
static int daemon_task; /**< Handle of daemon task / thread */

extern "C" __EXPORT int cf_motors_main(int argc, char *argv[]);
int testMotors(float input[4], bool usingPWM);
int cf_motors_thread(int argc, char *argv[]);
int pwm_value_from_speed(const float speed, float battV);

const unsigned LOOP_FREQUENCY = 1000;  //Hz

volatile unsigned numTimeouts = 0;
volatile unsigned numCmdsReceived = 0;

static int orb_sub_actuator_direct = 0;
static int orb_sub_battery = 0;

static HardwareTimer hwtimer;

volatile float batt_v = 4.1;
float const battFiltConst = 0.01;

using namespace Onboard;

float CFspeedToPWMConsts[3][2];

perf_counter_t perf_timer_loop;  //Times how long loop takes to run
perf_counter_t perf_timer_run;  //Times how long it takes cf_motors logic to run

// timer_index_of_motor[i] returns the timer index of ith motor
// timers are defined in timer_io_channels in /PX4-firmware/src/drivers/boards/crazyflie/crazyflie_timer_config.c
static int timer_index_of_motor[4] = { 0, 3, 1, 2 };

static void usage() {
  printf("usage:\n");
  printf("Normal operation: cf_motors {start|stop|status|testSpeed|testPWM}\n");
  printf(
      "\tSpeed test operation (replace <Ni> with desired speed for the four motors in rad/s):\n");
  printf("\t\tcf_motors testSpeed N1 N2 N3 N4\n");
  printf(
      "\t\tIf you provide only one speed, all motors spin at same speed: cf_motors testSpeed N\n");
  printf(
      "\tPWM test operation (replace <Ni> with desired PWM for the four motors between 0 and 255):\n");
  printf(
      "\tcf_motors testPWM N1 N2 N3 N4 or esc_motors testPWM N (spins all at same PWM)");
  return;
}

int pwm_value_from_speed(const float speed, float battV) {
  float const MIN_BATT_V = 3.1f;
  float const MAX_BATT_V = 4.1f;

  if (battV < MIN_BATT_V) {
    battV = MIN_BATT_V;
  } else if (battV > MAX_BATT_V) {
    battV = MAX_BATT_V;
  }

  float const k_1 = CFspeedToPWMConsts[0][0] + CFspeedToPWMConsts[0][1] * battV;
  float const k_2 = CFspeedToPWMConsts[1][0] + CFspeedToPWMConsts[1][1] * battV;
  float const k_3 = CFspeedToPWMConsts[2][0] + CFspeedToPWMConsts[2][1] * battV;

  int outpwm = int(k_1 + (k_2 + k_3 * speed) * speed);

  int const MIN_PWM = 0;
  int const MAX_PWM = 255;
  if (outpwm < MIN_PWM) {
    return MIN_PWM;
  }
  if (outpwm > MAX_PWM) {
    return MAX_PWM;
  }
  return outpwm;
}

int testMotors(float input[4], bool usingPWM) {
  orb_sub_battery = orb_subscribe(ORB_ID(battery_status));
  printf("orb_sub_battery  = %d\n", orb_sub_battery);

  const uint32_t updatePeriod_us = 1000000 / LOOP_FREQUENCY;
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
  struct battery_status_s batt;

  unsigned const printPeriod_us = 500000;
  unsigned nextPrintTime = 0;
  unsigned numBattMeas = 0;

  while (t.GetSeconds<float>() < RUN_TIME) {
    bool updated = false;
    orb_check(orb_sub_battery, &updated);
    if (updated) {
      numBattMeas++;
      orb_copy(ORB_ID(battery_status), orb_sub_battery, &batt);
      //update battery voltage estimate:
      batt_v = (1 - battFiltConst) * batt_v + battFiltConst * batt.voltage_v;
    }

    if (t.GetMicroSeconds() > nextPrintTime) {
      nextPrintTime += printPeriod_us;
      printf("batt_v = %.3f (over %d meas)\n", double(batt_v),
             int(numBattMeas));
    }

    for (int i = 0; i < 4; i++) {
      int ret;
      if (usingPWM) {
        ret = ioctl(fd, PWM_SERVO_SET(timer_index_of_motor[i]), double(input[i]));
      } else {
        ret = ioctl(fd, PWM_SERVO_SET(timer_index_of_motor[i]),
                    pwm_value_from_speed(input[i], batt_v));
      }

      if (ret != OK) {
        if (usingPWM) {
          err(1, "PWM_SERVO_SET(timer_index_of_motor[%d]) = %d (%.3f PWM)", i,
              int(input[i]), double(input[i]));
        } else {
          err(1, "PWM_SERVO_SET(timer_index_of_motor[%d]) = %d (%.3f rad/s)", i,
              pwm_value_from_speed(input[i], batt_v), double(input[i]));
        }
      }
    }

    usleep(updatePeriod_us);
  }

  //Make sure we exit with motors off:
  for (int i = 0; i < 4; i++) {
    int ret = ioctl(fd, PWM_SERVO_SET(timer_index_of_motor[i]), 0);

    if (ret != OK) {
      err(1, "PWM_SERVO_SET(timer_index_of_motor[%d])", i);
    }
  }

  ioctl(fd, PWM_SERVO_DISARM, 0);

  return 0;
}

int cf_motors_thread(int argc, char *argv[]) {
  orb_sub_actuator_direct = orb_subscribe(ORB_ID(actuator_direct));
  orb_sub_battery = orb_subscribe(ORB_ID(battery_status));

  perf_timer_loop = perf_alloc(PC_ELAPSED, "cf_motors_loop");
  perf_timer_run = perf_alloc(PC_ELAPSED, "cf_motors_run");

  if (orb_sub_actuator_direct <= 0) {
    printf("Error in orb_sub_actuator_direct  = %d\n", orb_sub_actuator_direct);
    return -1;
  }

  const uint32_t updatePeriod_us = 1000000 / LOOP_FREQUENCY;
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

  struct battery_status_s batt;
  struct actuator_direct_s actuatorCmds;

  bool motorsOff = true;
  const unsigned MOTOR_TIMEOUT_ms = 50;  //if we don't receive a command every this often, kill motors.

  while (thread_running) {
    perf_begin(perf_timer_loop);
    int poll_ret = px4_poll(fds, 1, MOTOR_TIMEOUT_ms);
    perf_begin(perf_timer_run);
    if (poll_ret == 0) {
      numTimeouts++;
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

      bool updated = false;
      orb_check(orb_sub_battery, &updated);
      if (updated) {
        /* obtained data for the battery*/
        orb_copy(ORB_ID(battery_status), orb_sub_battery, &batt);
        //update battery voltage estimate:
        batt_v = (1 - battFiltConst) * batt_v + battFiltConst * batt.voltage_v;
      }

      numCmdsReceived++;
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

        int const pwmVal = pwm_value_from_speed(actuatorCmds.values[i], batt_v);
        int ret = ioctl(fd, PWM_SERVO_SET(timer_index_of_motor[i]), pwmVal);

        if (ret != OK) {
          err(1, "PWM_SERVO_SET(timer_index_of_motor[%d]) = %d (%.3f)", i,
              pwmVal, double(actuatorCmds.values[i]));
        }
      }
    }
    perf_end(perf_timer_run);
    perf_end(perf_timer_loop);
  }

  //Make sure we exit with motors off:
  for (int i = 0; i < 4; i++) {
    int ret = ioctl(fd, PWM_SERVO_SET(timer_index_of_motor[i]), 0);

    if (ret != OK) {
      err(1, "PWM_SERVO_SET(timer_index_of_motor[%d])", i);
    }
  }

  ioctl(fd, PWM_SERVO_DISARM, 0);

  printf("Exiting: cf_motors_thread\n");

  return 0;
}

int cf_motors_main(int argc, char *argv[]) {

  if (argc < 1) {
    usage();
    return -1;
  }

  int vehId = 0;
  if (0 != param_get(param_find("VEHICLE_ID"), &vehId)) {
    printf("Failed to get param <VEHICLE_ID>\n");
    vehId = 0;
  }
  int currentQuadType = Onboard::QuadcopterConstants::GetVehicleTypeFromID(
      vehId);
  QuadcopterConstants consts(
      static_cast<QuadcopterConstants::QuadcopterType>(currentQuadType));
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      CFspeedToPWMConsts[i][j] = consts.CFspeedToPWMConsts[i][j];
    }
  }

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
      testMotors(input, usingPWM);
    } else if (!strcmp(argv[1], "testPWM")) {
      usingPWM = true;
      testMotors(input, usingPWM);
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
        "cf_motors", SCHED_DEFAULT, SCHED_PRIORITY_MAX - 10, 2000,
        cf_motors_thread,
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
    if (thread_running) {
      printf("\tmotors is running.\n");
      printf("\tNum timeouts = %d, num cmds = %d\n", numTimeouts,
             numCmdsReceived);
    } else {
      printf("\tmotors not started\n");
    }
    printf("\tVoltage = %.3fV\n", double(batt_v));
    return 0;
  }

  usage();
  return -1;
}
