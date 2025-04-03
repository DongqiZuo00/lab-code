/* HiPeRLab flight recorder
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

//For the current sensor. This is ugly, should probably be moved elsewhere.
#include <fcntl.h>
#include <stdio.h>
#include <arch/board/board.h>
#include <drivers/drv_adc.h>

static volatile bool thread_running = false; /**< Daemon status flag */
static int daemon_task; /**< Handle of daemon task / thread */

extern "C" __EXPORT int flightrecorder_main(int argc, char *argv[]);
int flightrecorder_thread(int argc, char *argv[]);
void print_status();

static HardwareTimer hwtimer;

int32_t vehId = 0;

//set this to >1 to write many lines at once. Testing shows that 1 is best though (fewest drop-outs)
#define LOG_BUFF_LENGTH 1

int16_t ConvertToInt(float const in, float const range);

int16_t ConvertToInt(float const in, float const range) {
  if (fabsf(in) > range) {
    return 32767;
  }
  return int16_t(in / range * 32768);
}

#pragma pack(push, 1)
struct LogStruct {
  uint8_t time_ms;
  int16_t sensor_acc_x;
  int16_t sensor_acc_y;
  int16_t sensor_acc_z;
  int16_t sensor_gyro_x;
  int16_t sensor_gyro_y;
  int16_t sensor_gyro_z;

  uint8_t uwb_request_id;

  uint8_t uwb_responder_id;
  uint16_t uwb_responder_range_mm;
};
#pragma pack(pop)

unsigned numMsgs(0);
unsigned numWriteToFile(0);
unsigned numFileOpenClose(0);

unsigned flightRec_numTimeouts = 0;

unsigned flight_logger_file_no = 0;

uint16_t logRandomIdentifier = 0;  //we use this to mark all log file bits as belonging to the same batch

///fs/microsd/flightrecorder%03d.log
char filename[36];

perf_counter_t pt_loop = perf_alloc(PC_ELAPSED, "flightrecorder_loop");
perf_counter_t pt_write = perf_alloc(PC_ELAPSED, "flightrecorder_write");
perf_counter_t pt_open_close = perf_alloc(PC_ELAPSED,
                                          "flightrecorder_open_close");

//LOG FILES:
FILE *fp_log = 0;

static void usage() {
  printf("usage:\n");
  printf("Normal operation: flightrecorder {start|stop|status}\n");
  printf("\n");
  return;
}

//UORB STUFF
static int orb_sub_quadLogger = 0;

int flightrecorder_thread(int argc, char *argv[]) {

  if (!orb_sub_quadLogger) {
    orb_sub_quadLogger = orb_subscribe(ORB_ID(hiperlab_quad_log));
  }

  //use the current microseconds as proxy for RNG:
  logRandomIdentifier = hrt_absolute_time() % 0xFFFF;

  flight_logger_file_no = 0;
  sprintf(filename, "/fs/microsd/flightrecorder%03d.log",
          flight_logger_file_no);
  fp_log = fopen(filename, "wb");
  if (!fp_log) {
    printf(
        "Failed to open flightrecorder log file. Probably no SD? Exiting.\n");
    return -1;
  }

  irqstate_t flags = enter_critical_section();
  //Write vehicle ID and some test characters, to get going
  fwrite(&vehId, sizeof(vehId), 1, fp_log);
  {
    //write test characters:
    float test_float = float(M_PI);  //make sure we can read floats
    uint64_t test_uint64 = 3141592653589793238;  //test endedness
    fwrite(&test_uint64, sizeof(test_uint64), 1, fp_log);
    fwrite(&test_float, sizeof(test_float), 1, fp_log);
  }
  //write the log identifier
  fwrite(&logRandomIdentifier, sizeof(logRandomIdentifier), 1, fp_log);
  leave_critical_section(flags);

  hiperlab_quad_log_s qLog;
  bool updated;

  //For now, we just tie this to the IMU, log w/ each IMU:
  px4_pollfd_struct_t fds[1];
  fds[0].fd = orb_sub_quadLogger;
  fds[0].events = POLLIN;

  const unsigned POLL_TIMEOUT_ms = 50;  //We don't really use this

  static LogStruct lBuff[LOG_BUFF_LENGTH];
  unsigned logStructIndex = 0;
  unsigned writesSinceLastOpenClose = 0;

  thread_running = true;
  while (thread_running) {
    int poll_ret = px4_poll(fds, 1, POLL_TIMEOUT_ms);

    if (poll_ret == 0) {
      flightRec_numTimeouts++;
      //but continue to check other subscriptions
    }

    perf_begin(pt_loop);

    updated = false;
    orb_check(orb_sub_quadLogger, &updated);
    if (!updated) {
      continue;
    }
    orb_copy(ORB_ID(hiperlab_quad_log), orb_sub_quadLogger, &qLog);

    LogStruct* l = &lBuff[logStructIndex];

    l->time_ms = qLog.time_ms;
    float const ACC_RANGE = 16 * 9.81f;
    float const GYRO_RANGE = 36.0f;
    l->sensor_acc_x = ConvertToInt(qLog.sensor_acc_x, ACC_RANGE);
    l->sensor_acc_y = ConvertToInt(qLog.sensor_acc_y, ACC_RANGE);
    l->sensor_acc_z = ConvertToInt(qLog.sensor_acc_z, ACC_RANGE);
    l->sensor_gyro_x = ConvertToInt(qLog.sensor_gyro_x, GYRO_RANGE);
    l->sensor_gyro_y = ConvertToInt(qLog.sensor_gyro_y, GYRO_RANGE);
    l->sensor_gyro_z = ConvertToInt(qLog.sensor_gyro_z, GYRO_RANGE);

    l->uwb_request_id = qLog.uwb_request_id;

    l->uwb_responder_id = qLog.uwb_responder_id;
    l->uwb_responder_range_mm = int(qLog.uwb_responder_range * 1e3f + 0.5f);
    numMsgs++;

    logStructIndex++;

    if (logStructIndex >= LOG_BUFF_LENGTH) {
      perf_begin(pt_write);
      logStructIndex = 0;
      numWriteToFile++;
      //hiperlab hack, unclear why we need this, but otherwise we get preempted by UWB(?) //suggests sd card not set up correctly.
      size_t sizeWritten = 0;
      flags = enter_critical_section();
      sizeWritten += fwrite(lBuff, LOG_BUFF_LENGTH * sizeof(LogStruct), 1,
                            fp_log);
      //done writing
      leave_critical_section(flags);

      if (sizeWritten != 1) {
        thread_running = false;
        printf("[flightrecorder] Write failed on a packet [%d/%d]\n",
               int(sizeWritten), int(1));
        print_status();
        return -1;
      }
      perf_end(pt_write);

      writesSinceLastOpenClose++;
      //force file open/close every ~20 sec. VERY SLOW on uC!
      if (qLog.is_idle) {
        if (writesSinceLastOpenClose > 10000) {
          writesSinceLastOpenClose = 0;
          numFileOpenClose++;
          perf_begin(pt_open_close);
#if 1
          //force file open/close, so that there's something written in case the vehicle turns off unexpectedly
          // (this is a bit of a hack)
          flags = enter_critical_section();
          fclose(fp_log);
          flight_logger_file_no++;
          sprintf(filename, "/fs/microsd/flightrecorder%03d.log",
                  flight_logger_file_no);
          fp_log = fopen(filename, "wb");
          if (!fp_log) {
            printf("Failed to open %s\n", filename);
            return -1;
          }
          fwrite(&logRandomIdentifier, sizeof(logRandomIdentifier), 1, fp_log);
#else
          int syncResult = fsync(int(fp_log));
          leave_critical_section(flags);
          if (!syncResult) {
            printf("Failed sync flightrecorder.log\n");
            return -1;
          }
#endif
          perf_end(pt_open_close);
        }
      }
    }
    perf_end(pt_loop);
  }

  fclose(fp_log);

  return 0;
}

void print_status() {
  static unsigned lastTime = 0;
  float dt = (hwtimer.GetMicroSeconds() - lastTime) * 1e-6f;
  lastTime = hwtimer.GetMicroSeconds();

  printf("Flight recorder \n");
  printf("----------------\n");
  printf("Num poll timeouts (total) = %d\n", int(flightRec_numTimeouts));
  printf(
      "Num packets recorded since last call: (total), over dt=%.3fs, each size=%dx%dbytes\n",
      double(dt), int(LOG_BUFF_LENGTH), int(sizeof(LogStruct)));
  printf("uORB msgs seen  %d @ %.1fHz\n", numMsgs, double(numMsgs / dt));
  numMsgs = 0;
  printf("Written to file %d @ %.1fHz\n", numWriteToFile,
         double(numWriteToFile / dt));
  numWriteToFile = 0;
  printf("File open/close %d @ %.3fHz\n", numFileOpenClose,
         double(numFileOpenClose / dt));
  numFileOpenClose = 0;
  printf("Current log random number identifier: %d\n",
         int(logRandomIdentifier));
  printf("\n\n");

}

int flightrecorder_main(int argc, char *argv[]) {
  if (argc < 2) {
    usage();
    return -1;
  }

  if (!strcmp(argv[1], "start")) {
    if (thread_running) {
      printf("Thread already running\n");
      return 0;
    }

    if (0 != param_get(param_find("VEHICLE_ID"), &vehId)) {
      printf("Failed to get param <VEHICLE_ID>\n");
      vehId = 0;
    }

    // Set priority to 180 so that logic is higher priority than other processes
    // This allows the logic to run more consistently at 0.002s per loop
    daemon_task = px4_task_spawn_cmd("flightrecorder", SCHED_DEFAULT,
    SCHED_PRIORITY_MAX - 20,
                                     4096, flightrecorder_thread,
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

  if (!strcmp(argv[1], "status")) {
    if (thread_running) {
      //TODO FIXME more detailed commands here.
      printf("Thread running\n");
      print_status();
    } else {
      printf("\tflightrecorder not started\n");
    }
    return 0;
  }

  printf("Unknown command\n");
  usage();
  return -1;
}
