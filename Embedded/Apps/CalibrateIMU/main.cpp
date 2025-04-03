/* Quadcopter app
 *
 * TODO FIXME: We're only receiving IMU updates at 250Hz. Why?
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_accel.h>
#include <parameters/param.h>

#include "Common/Math/Vec3.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Common/Time/Timer.hpp"

#include "../matrix/matrix/math.hpp"

extern "C" __EXPORT int calibrateIMU_main(int argc, char *argv[]);
int getAccMeasAvg(unsigned const numPoints, Vec3f &measOut);

//UORB STUFF
static int orb_sub_imuAccel = 0;

int getAccMeasAvg(unsigned const numPoints, Vec3f &measOut) {
  HardwareTimer hwtimer;
  Timer t(&hwtimer);

  measOut = Vec3f(0, 0, 0);
  unsigned numAccumulated = 0;

  sensor_accel_s acc;
  bool updated;
  t.Reset();

  const float TIMEOUT_SEC = 1;
  for (;;) {
    if (t.GetSeconds<float>() > TIMEOUT_SEC) {
      return -1;
    }
    //TODO: set inputs
    updated = false;
    orb_check(orb_sub_imuAccel, &updated);
    if (!updated) {
      usleep(500);
      continue;
    }

    orb_copy(ORB_ID(sensor_accel), orb_sub_imuAccel, &acc);

    measOut += Vec3f(acc.x, acc.y, acc.z);
    numAccumulated++;

    if (numAccumulated >= numPoints) {
      break;
    }
  }

  measOut = measOut / numAccumulated;
  return 0;
}

template<int M, int N>
void PrintMatrix(matrix::Matrix<float, M, N> m);

template<int M, int N>
void PrintMatrix(matrix::Matrix<float, M, N> m) {
  printf("=====================\n");
  for (int i = 0; i < M; i++) {
    for (int j = 0; j < N; j++) {
      printf("\t%.3f,", double(m(i, j)));
    }
    printf("\n");
  }
}

enum {
  NUM_ORIENTATIONS = 6,
  NUM_POINTS_PER_ORIENTATION = 200,
  NUM_VARS = 6,
  MAX_NUM_ITERS = 5,
};

//horrible hack to place everything on the heap
struct MyData {
  Vec3f meas[NUM_ORIENTATIONS];
  matrix::Matrix<float, NUM_ORIENTATIONS, NUM_VARS> A;
  matrix::Matrix<float, NUM_VARS, NUM_ORIENTATIONS> AT;
  matrix::Matrix<float, NUM_VARS, 1> x;  //bias, scale
  matrix::Matrix<float, NUM_ORIENTATIONS, 1> e;  //errors

  matrix::SquareMatrix<float, NUM_VARS> ATA;
  matrix::SquareMatrix<float, NUM_VARS> ATAinv;
  matrix::Matrix<float, NUM_VARS, 1> ATe;
  matrix::Matrix<float, NUM_VARS, 1> dx;

};

int calibrateIMU_main(int argc, char *argv[]) {
  if (!orb_sub_imuAccel) {
    orb_sub_imuAccel = orb_subscribe(ORB_ID(sensor_accel));
  }

  printf("\n============\nCalibrate accelerometer\n============\n");
  printf(
      "Hold the vehicle in %d different orientations, and when told to do so, hold it steady.\n",
      int(NUM_ORIENTATIONS));

  MyData* d = new MyData;

  //first we gather the measurements:
  for (int measNo = 0; measNo < NUM_ORIENTATIONS; measNo++) {
    printf("Hit <enter> to continue. ");
    fflush(stdout);
    getchar();

    printf("Orientation #%d:\t", measNo);
    if (0 != getAccMeasAvg(NUM_POINTS_PER_ORIENTATION, d->meas[measNo])) {
      printf("FAILED");
      delete d;
      return -1;
    }
    printf("<%.6f,%.6f,%.6f>\n", double(d->meas[measNo].x),
           double(d->meas[measNo].y), double(d->meas[measNo].z));
  }

  //now we solve for the offsets:
  printf("Solving least squares:\n");
  fflush(stdout);

  d->x(0, 0) = 0;
  d->x(1, 0) = 0;
  d->x(2, 0) = 0;
  d->x(3, 0) = 1.0;
  d->x(4, 0) = 1.0;
  d->x(5, 0) = 1.0;

  for (int iter = 0; iter < MAX_NUM_ITERS; iter++) {
    printf("Iteration #%d with ", iter);
    Vec3f b(d->x(0, 0), d->x(1, 0), d->x(2, 0));
    Vec3f k(d->x(3, 0), d->x(4, 0), d->x(5, 0));

    for (int orientation = 0; orientation < NUM_ORIENTATIONS; orientation++) {
      for (int ax = 0; ax < 3; ax++) {
        d->A(orientation, 0 + ax) = 2 * (b[ax] - d->meas[orientation][ax])
            / (k[ax] * k[ax]);
        d->A(orientation, 3 + ax) = -2 * (d->meas[orientation][ax] - b[ax])
            * (d->meas[orientation][ax] - b[ax]) / (k[ax] * k[ax] * k[ax]);
      }
      Vec3f corr = d->meas[orientation] - b;
      for (int ax = 0; ax < 3; ax++) {
        corr[ax] /= k[ax];
      }
      d->e(orientation, 0) = float(9.81 * 9.81) - corr.GetNorm2Squared();
    }

    d->AT = d->A.transpose();
    d->ATA = (d->AT * d->A);
    d->ATAinv = d->ATA.I();
    d->ATe = (d->A.transpose() * d->e);
    d->dx = d->ATAinv * d->ATe;

    float totErr = 0;
    for (int orientation = 0; orientation < NUM_ORIENTATIONS; orientation++) {
      totErr += d->e(orientation, 0) * d->e(orientation, 0);
    }

    d->x += d->dx;
    printf("||d->e|| = %.3f, ", double(totErr));
    printf("bias = <%.3f, %.3f, %.3f>, scale =<%.3f, %.3f, %.3f>",
           double(d->x(0, 0)), double(d->x(1, 0)), double(d->x(2, 0)),
           double(d->x(3, 0)), double(d->x(4, 0)), double(d->x(5, 0)));
    printf("\n");
  }

  bool success = true;

  if (param_set(param_find("CALIBACC_BX"), &d->x(0, 0))) {
    printf("Failed to set <CALIBACC_BX>\n");
    success = false;
  }

  if (param_set(param_find("CALIBACC_BY"), &d->x(1, 0))) {
    printf("Failed to set <CALIBACC_BY>\n");
    success = false;
  }

  if (param_set(param_find("CALIBACC_BZ"), &d->x(2, 0))) {
    printf("Failed to set <CALIBACC_BZ>\n");
    success = false;
  }

  if (param_set(param_find("CALIBACC_KX"), &d->x(3, 0))) {
    printf("Failed to set <CALIBACC_KX>\n");
    success = false;
  }

  if (param_set(param_find("CALIBACC_KY"), &d->x(4, 0))) {
    printf("Failed to set <CALIBACC_KY>\n");
    success = false;
  }

  if (param_set(param_find("CALIBACC_KZ"), &d->x(5, 0))) {
    printf("Failed to set <CALIBACC_KZ>\n");
    success = false;
  }

  if (success) {
    printf("Now you *MUST* run `param save` to store the params\n");
  } else {
    printf("Failed to write parameters!\n");
    delete d;
    return -1;
  }

  delete d;
  return 0;
}
