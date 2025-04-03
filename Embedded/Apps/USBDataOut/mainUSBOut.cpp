/* Pipe data out over USB. Corresponds to script in general code
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include <nuttx/sched.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <parameters/param.h>

#include <uORB/uORB.h>
#include <uORB/topics/hiperlab_quad_log.h>

#include "Common/Time/HardwareTimer.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"
#include "Common/Math/Vec3.hpp"

static volatile bool thread_running = false; /**< Daemon status flag */
static int daemon_task; /**< Handle of daemon task / thread */

extern "C" __EXPORT int usbDataOut_main(int argc, char *argv[]);
int usbOut_thread(int argc, char *argv[]);

static HardwareTimer hwtimer;

static void usage() {
  printf("usage:\n");
  printf(
      "This program only makes sense when used with the corresponding script in general code!\n");
  printf("\n");
  return;
}

//UORB STUFF
static int orb_sub_quadLogger = 0;

int usbOut_thread(int argc, char *argv[]) {

  if (!orb_sub_quadLogger) {
    orb_sub_quadLogger = orb_subscribe(ORB_ID(hiperlab_quad_log));
  }

  //For now, we just tie this to the IMU, log w/ each IMU:
  px4_pollfd_struct_t fds[1];
  fds[0].fd = orb_sub_quadLogger;
  fds[0].events = POLLIN;

  const unsigned POLL_TIMEOUT_ms = 50;  //We don't really use this

  uint8_t id_responder;
  int32_t range_mm;

  hiperlab_quad_log_s qLog;

  thread_running = true;
  while (thread_running) {
    int poll_ret = px4_poll(fds, 1, POLL_TIMEOUT_ms);

    if (poll_ret == 0) {
      //time out!
      continue;
    }

    bool updated = false;
    orb_check(orb_sub_quadLogger, &updated);
    if (!updated) {
      continue;
    }
    orb_copy(ORB_ID(hiperlab_quad_log), orb_sub_quadLogger, &qLog);

    if (!qLog.uwb_responder_id) {
      //no UWB info here.
      continue;
    }

    id_responder = qLog.uwb_responder_id;
    if (qLog.uwb_responder_range > 0) {
      range_mm = uint16_t(qLog.uwb_responder_range * 1000 + 0.5f);
    } else {
      //failed, for some reason.
      range_mm = 0;
    }

    //todo: this is very inefficient.
    // better would be to pipe this out as chars, can get everything out using 3bytes, currently use way
    // more than that. But, this is good enough for now.
    printf("%d:%d\n", int(id_responder), int(range_mm));
  }

  return 0;
}

int usbDataOut_main(int argc, char *argv[]) {
  if (argc < 2) {
    usage();
    return -1;
  }

  if (!strcmp(argv[1], "start")) {
    if (thread_running) {
      printf("Thread already running\n");
      return 0;
    }

    // Set priority to 180 so that logic is higher priority than other processes
    // This allows the logic to run more consistently at 0.002s per loop
    daemon_task = px4_task_spawn_cmd("usbOut", SCHED_DEFAULT,
    SCHED_PRIORITY_MAX - 20,
                                     4096, usbOut_thread,
                                     (char * const *) NULL);

    return 0;
  }

  if (!strcmp(argv[1], "stop")) {
    if (!thread_running) {
      printf("thread not running\n");
      return 0;
    }
    thread_running = false;

    usleep(1000);

    printf("done!\n");
    return 0;
  }

  printf("Unknown command\n");
  usage();
  return -1;
}

