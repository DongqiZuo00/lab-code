/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include <nuttx/sched.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include <nuttx/sched.h>

//#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>
#include <parameters/param.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/ranging_report.h>
#include <uORB/topics/ranging_request.h>

#include "P2PRanging.h"

extern "C" __EXPORT int ranging_main(int argc, char *argv[]);

static bool volatile thread_should_exit = false;
static bool volatile thread_running = false;
static int volatile daemon_task;

//UORB STUFF
struct ranging_report_s rangingReport;
orb_advert_t pub_fd_rangingReport = 0;
int orb_sub_rangingRequest = 0;

int ranging_thread_main(int argc, char *argv[]);

void usage();
void OnTimerP2PR(void* p);

int getParameters(int32_t &id, float &posx, float &posy, float &posz);

void OnTimerP2PR(void* p) {
  sem_t* _sem = (sem_t*) p;
  int svalue;
  sem_getvalue(_sem, &svalue);
  if (svalue < 0)
    sem_post(_sem);
}

static DW1000NS::P2PRanging p2pRanging;

void usage() {
  printf("UWB Ranging app, USAGE: range {start|stop|status|set|get}\n");
  printf(
      "\t Start the app as follows, where [VERBOSE] is an optional argument (0/1) to set whether to print info to the screen");
  printf("\t nsh> ranging start [VERBOSE]\n");
  printf("Type `ranging set` to see info for setting params.\n");
}

int getParameters(int32_t &id, float &posx, float &posy, float &posz) {
  bool success = true;
  if (param_get(param_find("RANGRAD_ID"), &id)) {
    printf("Failed to get <RANGRAD_ID>\n");
    success = false;
  }

  if (param_get(param_find("RANGRAD_POS_X"), &posx)) {
    printf("Failed to get <RANGRAD_POS_X>\n");
    success = false;
  }

  if (param_get(param_find("RANGRAD_POS_Y"), &posy)) {
    printf("Failed to get <RANGRAD_POS_Y>\n");
    success = false;
  }

  if (param_get(param_find("RANGRAD_POS_Z"), &posz)) {
    printf("Failed to get <RANGRAD_POS_Z>\n");
    success = false;
  }

  if (!success) {
    return -1;
  }

  return 0;
}

int ranging_main(int argc, char *argv[]) {
  if (argc <= 1) {
    usage();
    return 0;
  }

  if (!strcmp(argv[1], "stop")) {
    thread_should_exit = true;
    return 0;
  }

  if (!strcmp(argv[1], "status")) {
    if (thread_running) {
      p2pRanging.printStatus();
    } else {
      warnx("\tnot started\n");
    }

    return 0;
  }

  if (!strcmp(argv[1], "set")) {
    //we're defining parameters:
    if (argc < 6) {
      printf("Insufficient # arguments:\n", argc);
      printf("Use as \n\tnsh> ranging set ID posx posy posz");
      printf(
          "\twhere ID is the id, and posx,posy,posz are the position of the radio [m]\n\n");
      return -1;
    }

    int32_t newId = atoi(argv[2]);
    float posx = atof(argv[3]);
    float posy = atof(argv[4]);
    float posz = atof(argv[5]);

    bool success = true;

    if (param_set(param_find("RANGRAD_ID"), &newId)) {
      printf("Failed to set <RANGRAD_ID>\n");
      success = false;
    }

    if (param_set(param_find("RANGRAD_POS_X"), &posx)) {
      printf("Failed to set <RANGRAD_POS_X>\n");
      success = false;
    }

    if (param_set(param_find("RANGRAD_POS_Y"), &posy)) {
      printf("Failed to set <RANGRAD_POS_Y>\n");
      success = false;
    }

    if (param_set(param_find("RANGRAD_POS_Z"), &posz)) {
      printf("Failed to set <RANGRAD_POS_Z>\n");
      success = false;
    }

    if (success) {
      printf("Set ID = %d, pos = <%.3f, %.3f, %.3f>m\n", newId, double(posx),
             double(posy), double(posz));

      printf("Now you *MUST* run `param save` to store the params\n");
    } else {
      printf("Failed to set params\n");
    }

    return 0;
  }

  if (!strcmp(argv[1], "get")) {
    //we're defining parameters:

    int32_t id;
    float posx;
    float posy;
    float posz;
    if (getParameters(id, posx, posy, posz)) {
      printf("Failed to get params\n");
      return -1;
    }

    printf("Loaded: ID = %d, pos = <%.3f, %.3f, %.3f>m\n", id, double(posx),
           double(posy), double(posz));

    return 0;
  }

  if (!strcmp(argv[1], "start")) {

    if (thread_running) {
      warnx("daemon already running\n");
      /* this is not an error */
      return 0;
    }

    thread_should_exit = false;
    daemon_task = px4_task_spawn_cmd(
        "dw1000p2pranging", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2000,
        ranging_thread_main,
        (argv) ? (char * const *) &argv[2] : (char * const *) NULL);
    return 0;
  }

  usage();
  return -1;
}

int ranging_thread_main(int argc, char *argv[]) {
  //make sure the passed arguments make sense:
  //nsh> ranging start MY_ID VERBOSE

  int32_t id;
  float posx;
  float posy;
  float posz;
  if (getParameters(id, posx, posy, posz)) {
    printf("Failed to get params\n");
    return -1;
  }
  uint8_t myId = uint8_t(id);

  bool verbose = false;
  if (argc > 2) {
    verbose = !strcmp(argv[2], "1");
  }

  if (!pub_fd_rangingReport) {
    memset(&rangingReport, 0, sizeof(rangingReport));
    pub_fd_rangingReport = orb_advertise(ORB_ID(ranging_report),
                                         &rangingReport);
    //printf("Advertising uorb = %d\n", int(pub_fd_rangingReport));
  }

  if (!orb_sub_rangingRequest) {
    orb_sub_rangingRequest = orb_subscribe(ORB_ID(ranging_request));
  }

  printf("### P2P-ranging ###\n");
  if (verbose) {
    printf("Verbose = true\n");
    printf("My ID = %d, pos = <%.3f, %.3f, %.3f>m\n", int(myId), double(posx),
           double(posy), double(posz));
  }

  p2pRanging.SetVerbose(verbose);

  if (p2pRanging.Initialize(myId, 10)) {
    printf("Init failed, returning.");
    return -1;
  }

  thread_running = true;

  static struct hrt_call ol_tick_call;
  memset(&ol_tick_call, 0, sizeof(ol_tick_call));

  sem_t _sem;
  sem_init(&_sem, 0, 0);

  unsigned const APP_FREQ = 1000;  //Hz
  const hrt_abstime updatePeriod = 1000000 / APP_FREQ;
  hrt_call_every(&ol_tick_call, updatePeriod, updatePeriod, &OnTimerP2PR,
                 &_sem);

  bool updated;
  ranging_request_s rangingRequest;
  for (;;) {
    sem_wait(&_sem);

    updated = false;
    orb_check(orb_sub_rangingRequest, &updated);
    if (updated) {
      orb_copy(ORB_ID(ranging_request), orb_sub_rangingRequest,
               &rangingRequest);
      p2pRanging.SetNewRangingRequest(rangingRequest.desired_responder_id);
    }

    p2pRanging.runLoop();

    if (p2pRanging.HaveNewResult()) {
      rangingReport = p2pRanging.GetLatestResult();
      int res = orb_publish(ORB_ID(ranging_report), pub_fd_rangingReport,
                            &rangingReport);
      if (res < 0) {
        thread_running = false;
        printf("Error publishing to uorb = %d\n", res);
        continue;
      }
    }

    if (thread_should_exit) {
      break;
    }
  }
  thread_running = false;
  hrt_cancel(&ol_tick_call);
  usleep(100);
  sem_destroy(&_sem);

  return 0;
}
