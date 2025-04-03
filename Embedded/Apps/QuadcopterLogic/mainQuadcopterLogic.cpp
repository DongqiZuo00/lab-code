/* Quadcopter app
 *
 */

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/sched.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_direct.h>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/hiperlab_quad_log.h>
#include <uORB/topics/radio_received.h>
#include <uORB/topics/radio_send.h>
#include <uORB/topics/radio_send_esp.h>
#include <uORB/topics/radio_send_ready.h>
#include <uORB/topics/ranging_report.h>
#include <uORB/topics/ranging_request.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/uORB.h>
/* PX4-ESP topics */
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/lobot_command.h>
#include <uORB/topics/lobot_feedback.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>

#include <drivers/drv_board_led.h> // to turn on red LED for flight...
#include <drivers/drv_led.h>       // to turn on red LED for flight...
__BEGIN_DECLS
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

#include "Common/DataTypes/TelemetryPacket.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Logic/QuadcopterLogic.hpp"

// For the current sensor. This is ugly, should probably be moved elsewhere.
#include <arch/board/board.h>
#include <drivers/drv_adc.h>
#include <fcntl.h>
#include <stdio.h>

int fd_adc = -1;
int const currentSensorChannel = 2; // Pin 1 on crazyflie right header
int const voltageSensorChannel = 3; // Pin 2 on crazyflie right header
float voltage_calibration_m = 0.0f; // voltage calibration slope
float voltage_calibration_c = 0.0f; // voltage calibration intercept
float current_calibration_m = 0.0f; // current calibration slope
float current_calibration_c = 0.0f; // current calibration intercept
unsigned numPacketsOverTelemetry = TelemetryPacket::NUM_PACKETS_OVER_TELEMETRY;

int FCType = 0;
adc_report_s adcReport; // uORB method for ADC
/* PX4-ESP topics */
esc_status_s esc_status;
distance_sensor_s distance_sensor;
vehicle_attitude_s vehicle_attitude;
vehicle_gps_position_s vehicle_gps_position;
vehicle_local_position_s vehicle_local_position;
vehicle_odometry_s vehicle_odometry;
lobot_feedback_s lobot_feedback;
uORB::Subscription esc_status_sub{ORB_ID(esc_status)};
uORB::Subscription distance_sensor_sub{ORB_ID(distance_sensor)};
uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
uORB::Subscription vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
uORB::Subscription vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
uORB::Subscription vehicle_odometry_sub{ORB_ID(vehicle_odometry)};
uORB::Subscription lobot_feedback_sub{ORB_ID(lobot_feedback)};

// UORB STUFF
static orb_advert_t orb_pub_actuatorCmds = 0;
static orb_advert_t orb_pub_rangingRequest = 0;
static orb_advert_t orb_pub_radioSend = 0;
static orb_advert_t orb_pub_quadLog = 0;
static orb_advert_t orb_pub_lobot_command = 0;
static struct actuator_direct_s actuatorCmds;
static struct ranging_request_s rangingRequest;
static struct radio_send_s radio_send;
static struct radio_send_esp_s radio_send_esp;
static struct lobot_command_s lobot_command;
static struct hiperlab_quad_log_s qLog;
static int orb_sub_battery = 0;
static int orb_sub_imuAccel = 0;
static int orb_sub_imuGyro = 0;
static int orb_sub_rangingReport = 0;
static int orb_sub_radioReceived = 0;
static int orb_sub_radioSendReady = 0;
static int orb_sub_adcReport = 0;

float currentMeas(0.0f), voltageMeas(0.0f);
int MeasureVoltageAndCurrent(void);
int MeasureVoltageAndCurrent(void)
{
    unsigned nChannels = 12; // Hard coding total number of channels
    for (unsigned j = 0; j < nChannels; j++)
    {
        if (currentSensorChannel == adcReport.channel_id[j])
        {
            currentMeas = float(adcReport.raw_data[j]) * current_calibration_m + current_calibration_c;
        }
        if (voltageSensorChannel == adcReport.channel_id[j])
        {
            voltageMeas = float(adcReport.raw_data[j]) * voltage_calibration_m + voltage_calibration_c;
        }
    }
    // didn't find the right channel!
    return 0;
}

static volatile bool thread_running = false; /**< Daemon status flag */
static int daemon_task;                      /**< Handle of daemon task / thread */
static volatile bool syslink_ready = false;

static volatile float rangingTests_latestMeas = -1.0f;
static volatile bool rangingTests_haveNewResult = false;
static volatile bool rangingTests_shouldStartNewUWBConversation = false;

extern "C" __EXPORT int quad_main(int argc, char *argv[]);
void OnTimer(void *p);
int logicThread(int argc, char *argv[]);
int RunTestRanging();

const unsigned ONBOARD_FREQUENCY = 500; // Hz

static bool verbose = false;

static HardwareTimer hwtimer;

Onboard::QuadcopterLogic logic(&hwtimer, 1.0f / float(ONBOARD_FREQUENCY));
// float telemetryPeriod = 5 * (1.0f / float(ONBOARD_FREQUENCY));  // Send telemetry at 1/5 the rate of the logic
float telemetryPeriod = 1.0f / float(100); // Faster telemetry for testing

// Measures loop time (should be 1/ONBOARD_FREQUENCY)
perf_counter_t perf_timer_obLoop = perf_alloc(PC_ELAPSED, "quad_logic_loop");
// Measures time it takes to run logic (should be significantly less than
// 1/ONBOARD_FREQUENCY)
perf_counter_t perf_timer_obLogic = perf_alloc(PC_ELAPSED, "quad_logic_run");

/* PX4-ESP variables */
bool send_on_receive = false;       // send telemetry when command is received
bool send_on_receive_state = false; // true when command is received to send telemetry

void OnTimer(void *p)
{
    sem_t *_sem = (sem_t *)p;
    int svalue;
    sem_getvalue(_sem, &svalue);
    if (svalue < 0)
        sem_post(_sem);
}

static void usage()
{
    printf("usage:\n");
    printf("Normal operation: quad "
           "{start|stop|status|test|setVehicleId|resetPropCalib}\n");
    printf("To print ranging results, use `quad start v`\n");
    printf("\tTest modes:\n");
    printf("\t`quad test thrust 0.5` -> Test thrust = 0.5 * weight (at 5 "
           "second default)\n");
    printf("\t`quad test thrust 0.1 1` -> Test thrust = 0.1 * weight at 1 "
           "integer second\n");
    printf("\t`quad test current` -> current sensor\n");
    printf("\t`quad test ranging` -> UWB ranging (make sure to only define one "
           "anchor though!)\n");
    printf("\n");
    printf("To change the quad type, change GetVehicleTypeFromID() in "
           "QuadcopterConstants.hpp\n");
    return;
}

perf_counter_t pc_count_rangingRadioTimeouts(perf_alloc(PC_COUNT, "quad_num_ranging_radio_timeouts"));

int logicThread(int argc, char *argv[])
{
    if (!orb_pub_actuatorCmds)
    {
        memset(&actuatorCmds, 0, sizeof(actuatorCmds));
        orb_pub_actuatorCmds = orb_advertise(ORB_ID(actuator_direct), &actuatorCmds);
    }

    if (!orb_pub_rangingRequest)
    {
        memset(&rangingRequest, 0, sizeof(rangingRequest));
        orb_pub_rangingRequest = orb_advertise(ORB_ID(ranging_request), &rangingRequest);
    }

    if (!orb_pub_radioSend)
    {
        if (FCType == Onboard::QuadcopterConstants::FCType::FC_TYPE_PX4)
        {
            memset(&radio_send_esp, 0, sizeof(radio_send_esp));
            orb_pub_radioSend = orb_advertise(ORB_ID(radio_send_esp), &radio_send_esp);
        }
        else
        {
            memset(&radio_send, 0, sizeof(radio_send));
            orb_pub_radioSend = orb_advertise(ORB_ID(radio_send), &radio_send);
        }
    }

    if (!orb_pub_quadLog)
    {
        memset(&qLog, 0, sizeof(qLog));
        orb_pub_quadLog = orb_advertise(ORB_ID(hiperlab_quad_log), &qLog);
    }

    if (!orb_pub_lobot_command)
    {
        memset(&lobot_command, 0, sizeof(lobot_command));
        orb_pub_lobot_command = orb_advertise(ORB_ID(lobot_command), &lobot_command);
    }

    if (!orb_sub_battery)
    {
        orb_sub_battery = orb_subscribe(ORB_ID(battery_status));
    }

    if (!orb_sub_imuAccel)
    {
        orb_sub_imuAccel = orb_subscribe(ORB_ID(sensor_accel));
    }

    if (!orb_sub_imuGyro)
    {
        orb_sub_imuGyro = orb_subscribe(ORB_ID(sensor_gyro));
    }

    if (!orb_sub_rangingReport)
    {
        orb_sub_rangingReport = orb_subscribe(ORB_ID(ranging_report));
    }

    if (!orb_sub_radioReceived)
    {
        orb_sub_radioReceived = orb_subscribe(ORB_ID(radio_received));
    }

    if (!orb_sub_radioSendReady)
    {
        orb_sub_radioSendReady = orb_subscribe(ORB_ID(radio_send_ready));
    }

    if (!orb_sub_adcReport)
    {
        orb_sub_adcReport = orb_subscribe(ORB_ID(adc_report));
    }

    static struct hrt_call ol_tick_call;
    memset(&ol_tick_call, 0, sizeof(ol_tick_call));

    printf("Started: fast_loop...\n");

    sem_t _sem;
    sem_init(&_sem, 0, 0);

    const hrt_abstime updatePeriod = 1000000 / ONBOARD_FREQUENCY;
    hrt_call_every(&ol_tick_call, updatePeriod, updatePeriod, &OnTimer, &_sem);

    // Load the accelerometer corrections:
    struct
    {
        Vec3f b;
        Vec3f k;
    } accCorr;

    bool loadSuccess = true;
    if (0 != param_get(param_find("CALIBACC_BX"), &accCorr.b.x))
    {
        loadSuccess = false;
        printf("Failed to get param <CALIBACC_BX>\n");
    }
    if (0 != param_get(param_find("CALIBACC_BY"), &accCorr.b.y))
    {
        loadSuccess = false;
        printf("Failed to get param <CALIBACC_BY>\n");
    }
    if (0 != param_get(param_find("CALIBACC_BZ"), &accCorr.b.z))
    {
        loadSuccess = false;
        printf("Failed to get param <CALIBACC_BZ>\n");
    }
    if (0 != param_get(param_find("CALIBACC_KX"), &accCorr.k.x))
    {
        loadSuccess = false;
        printf("Failed to get param <CALIBACC_KX>\n");
    }
    if (0 != param_get(param_find("CALIBACC_KY"), &accCorr.k.y))
    {
        loadSuccess = false;
        printf("Failed to get param <CALIBACC_KY>\n");
    }
    if (0 != param_get(param_find("CALIBACC_KZ"), &accCorr.k.z))
    {
        loadSuccess = false;
        printf("Failed to get param <CALIBACC_KZ>\n");
    }

    if (!loadSuccess)
    {
        printf("Failed to load params!\n");
        return -1;
    }

    // TODO, FIXME: we hardcode the positions here!
    logic.ClearRangingTargets();
#if 1
    logic.AddRangingTargetId(101, Vec3f(0, 0, 0));
#elif 0
    // mwm office:
    logic.AddRangingTargetId(101, Vec3f(-1, +1, 0));
    logic.AddRangingTargetId(102, Vec3f(+1, +1, 0));
    logic.AddRangingTargetId(103, Vec3f(-1, -1, 0));
    logic.AddRangingTargetId(104, Vec3f(-1.190f, -0.630f, 1.760f));
#else
    // Lab set-up
    logic.AddRangingTargetId(101, Vec3f(-1.906, +2.980, +0.220));
    logic.AddRangingTargetId(103, Vec3f(+1.354, +3.001, +0.220));
    logic.AddRangingTargetId(104, Vec3f(+1.124, -2.710, +0.220));
    logic.AddRangingTargetId(107, Vec3f(-1.884, -2.875, +0.220));
    logic.AddRangingTargetId(105, Vec3f(-0.936, -2.979, +1.728));
#endif

    // We have a problem where the P2P ranging can sometimes
    // "forget" about a ranging request, and return to an
    // idle state without sending a ranging report. We will
    // implement an ugly hack here, to work around this is
    const unsigned RANGING_RADIO_TIMEOUT_us = 30 * 1000; //[us] should be much longer than a typical ranging coversatio
    Timer rangingRadioTimeoutDetection_timeSinceRequest(&hwtimer);
    bool rangingRadioTimeoutDetection_areWaiting(false);

    battery_status_s batt;
    sensor_accel_s acc;
    sensor_gyro_s gyro;
    ranging_report_s rangingReport;
    radio_send_ready_s rsReady;
    bool updated;

    // Telemetry
    Timer telemetryTimer(&hwtimer); // Governs how often we send back telemetry packets
    TelemetryPacket::data_packet_t dataPacket1, dataPacket2, dataPacket3, dataPacket4;
    TelemetryPacket::esp_data_packet_t esp_data_packet;

    /* Check syslink_ready here since syslink publishes ready status on startup,
     but checking if the topic has been updated below will yield false */
    if (!syslink_ready)
    {
        memset(&rsReady, 0, sizeof(rsReady));
        orb_copy(ORB_ID(radio_send_ready), orb_sub_radioSendReady, &rsReady);
        syslink_ready = rsReady.is_ready;
    }

    const unsigned GYRO_CALIB_TIME = 2; //[s]
    // // ESP DShot driver section, instantiate
    // ESPDShot::DShot dshot = ESPDShot::DShot();

    unsigned loopCounter = 0; // for LEDs
    thread_running = true;
    while (thread_running)
    {
        // /* Trying */
        // if (orb_sub_vehicle_gps_position == -1) {
        //     orb_sub_vehicle_gps_position = orb_subscribe(ORB_ID(vehicle_gps_position));
        //     logic.subscription_id[3] = orb_sub_vehicle_gps_position;
        // }

        perf_begin(perf_timer_obLoop);
        sem_wait(&_sem);
        perf_begin(perf_timer_obLogic);

        loopCounter++;

        /* Battery voltage and current measurement update */
        updated = false;
        orb_check(orb_sub_battery, &updated);
        if (updated)
        {
            orb_copy(ORB_ID(battery_status), orb_sub_battery, &batt);
            if (FCType == Onboard::QuadcopterConstants::FCType::FC_TYPE_CF)
            {
                logic.SetBatteryMeasurement(batt.voltage_v, batt.current_a);
            }
            /* PX4-ESP voltage and current measurement update */
            if (FCType == Onboard::QuadcopterConstants::FCType::FC_TYPE_PX4)
            {
                logic.SetBatteryMeasurement(logic.esc_telemetry.voltage[0], batt.current_a);
            }
        }

        updated = false;
        orb_check(orb_sub_adcReport, &updated);
        if (updated)
        {
            orb_copy(ORB_ID(adc_report), orb_sub_adcReport, &adcReport);
            if (FCType == Onboard::QuadcopterConstants::FCType::FC_TYPE_CF)
            {
                MeasureVoltageAndCurrent();
                logic.SetBatteryMeasurement(voltageMeas, currentMeas);
            }
        }

        updated = false;
        orb_check(orb_sub_imuAccel, &updated);
        if (updated)
        {
            orb_copy(ORB_ID(sensor_accel), orb_sub_imuAccel, &acc);
            logic.SetIMUMeasurementAccelerometer((1 / accCorr.k.x) * (acc.x - accCorr.b.x), (1 / accCorr.k.y) * (acc.y - accCorr.b.y),
                                                 (1 / accCorr.k.z) * (acc.z - accCorr.b.z));
            logic.SetIMUMeasurementTemperature(acc.temperature);
        }

        updated = false;
        orb_check(orb_sub_imuGyro, &updated);
        if (updated)
        {
            orb_copy(ORB_ID(sensor_gyro), orb_sub_imuGyro, &gyro);
            logic.SetIMUMeasurementRateGyro(gyro.x, gyro.y, gyro.z);
        }

        updated = false;
        orb_check(orb_sub_rangingReport, &updated);
        if (updated)
        {
            rangingRadioTimeoutDetection_areWaiting = false;
            orb_copy(ORB_ID(ranging_report), orb_sub_rangingReport, &rangingReport);
            logic.SetUWBMeasurement(rangingReport.range, rangingReport.responder_id, rangingReport.failure);

            qLog.uwb_responder_id = rangingReport.responder_id;
            qLog.uwb_responder_range = rangingReport.failure ? -2 : rangingReport.range;

            rangingTests_latestMeas = rangingReport.range;
            if (rangingReport.failure)
            {
                rangingTests_latestMeas = -1.23f;
            }
            rangingTests_haveNewResult = true;

            if (verbose)
            {
                if (!rangingReport.failure)
                {
                    printf("%d, %.4f, ", rangingReport.responder_id, double(rangingReport.range));
                }
            }
        }
        else
        {
            qLog.uwb_responder_id = 0; // nothing to log now.
        }

        // P2P ranging bug workaround:
        if (rangingRadioTimeoutDetection_areWaiting)
        {
            if (rangingRadioTimeoutDetection_timeSinceRequest.GetMicroSeconds() > RANGING_RADIO_TIMEOUT_us)
            {
                perf_count(pc_count_rangingRadioTimeouts);
                rangingRadioTimeoutDetection_areWaiting = false;
                logic.SetUWBMeasurement(0, 0, true); // failure!
                rangingTests_latestMeas = -2.34f;
                rangingTests_haveNewResult = true;
                if (verbose)
                {
                    printf("RANGING RADIO TIMEOUT!\n");
                }
            }
        }

        updated = false;
        orb_check(orb_sub_radioReceived, &updated);
        if (updated)
        {
            struct radio_received_s raw;
            memset(&raw, 0, sizeof(raw));
            orb_copy(ORB_ID(radio_received), orb_sub_radioReceived, &raw);

            RadioTypes::RadioMessageDecoded msgDecoded(raw.data);
            logic.SetRadioMessage(msgDecoded);
            if (send_on_receive)
            {
                send_on_receive_state = true;
            }
        }

        logic.SetGyroCalibration(loopCounter < (GYRO_CALIB_TIME * ONBOARD_FREQUENCY));

        logic.Run();

        /* Radio telemetry */
        bool orbErr = false;
        updated = false;
        orb_check(orb_sub_radioSendReady, &updated);
        if (updated)
        {
            memset(&rsReady, 0, sizeof(rsReady));
            orb_copy(ORB_ID(radio_send_ready), orb_sub_radioSendReady, &rsReady);
            syslink_ready = rsReady.is_ready;
        }
        if (send_on_receive)
        {
            if (send_on_receive_state)
            {
                logic.GetESPTelemetryDataPacket(esp_data_packet);
                memset(radio_send_esp.data, 0, sizeof(radio_send_esp.data));
                memcpy(radio_send_esp.data, &esp_data_packet, sizeof(TelemetryPacket::esp_data_packet_t));
                orbErr |= orb_publish(ORB_ID(radio_send_esp), orb_pub_radioSend, &radio_send_esp);
                send_on_receive_state = false;
            }
        }
        else
        {
            if (syslink_ready and telemetryTimer.GetSeconds_f() > telemetryPeriod)
            {
                telemetryTimer.AdjustTimeBySeconds(-telemetryPeriod);
                /* PX4-ESP telemetry update */
                if (FCType == Onboard::QuadcopterConstants::FCType::FC_TYPE_PX4)
                {
                    logic.GetESPTelemetryDataPacket(esp_data_packet);
                    memset(radio_send_esp.data, 0, sizeof(radio_send_esp.data));
                    memcpy(radio_send_esp.data, &esp_data_packet, sizeof(TelemetryPacket::esp_data_packet_t));
                    orbErr |= orb_publish(ORB_ID(radio_send_esp), orb_pub_radioSend, &radio_send_esp);
                    /* PX4-ESP lobot servo update */
                    for (int i = 0; i < 2; i++)
                    {
                        lobot_command.angle[i] = logic.lobot_servo.angle_command[i];
                    }
                    orbErr |= orb_publish(ORB_ID(lobot_command), orb_pub_lobot_command, &lobot_command);
                }
                /* Crazyflie telemetry update */
                else
                {
                    // assemble the telemetry
                    logic.GetTelemetryDataPackets(dataPacket1, dataPacket2, dataPacket3, dataPacket4);

                    /* num_packets = number of packets to be grouped together in a
                     single transmission For the telemetry, we send two packets. The
                     first contains imu data (accel and gyro) and the second contains
                     estimator states. Packets are grouped together to ensure that the
                     uwb radio ATTEMPTS to send the whole group one after the other.
                     Note: this does not ensure receival of the entire group, as
                     individual packets can still get dropped. */
                    radio_send.num_packets = numPacketsOverTelemetry;
                    memset(radio_send.data1, 0, sizeof(radio_send.data1));
                    memcpy(radio_send.data1, &dataPacket1, sizeof(TelemetryPacket::data_packet_t));
                    memset(radio_send.data2, 0, sizeof(radio_send.data2));
                    memcpy(radio_send.data2, &dataPacket2, sizeof(TelemetryPacket::data_packet_t));
                    if (numPacketsOverTelemetry >= 3)
                    {
                        memset(radio_send.data3, 0, sizeof(radio_send.data3));
                        memcpy(radio_send.data3, &dataPacket3, sizeof(TelemetryPacket::data_packet_t));
                    }
                    if (numPacketsOverTelemetry >= 4)
                    {
                        memset(radio_send.data4, 0, sizeof(radio_send.data4));
                        memcpy(radio_send.data4, &dataPacket4, sizeof(TelemetryPacket::data_packet_t));
                    }
                    orbErr |= orb_publish(ORB_ID(radio_send), orb_pub_radioSend, &radio_send);
                }
            }
        }

        /* PX4-ESP uorb update */
        // Telemetry update
        if (esc_status_sub.update(&esc_status))
        {
            logic.status_bits[0] = true;
            for (int i = 0; i < 4; i++)
            {
                logic.esc_telemetry.temperature[i] = esc_status.esc[i].esc_temperature;
                logic.esc_telemetry.voltage[i] = esc_status.esc[i].esc_voltage;
                logic.esc_telemetry.current[i] = esc_status.esc[i].esc_current;
                logic.esc_telemetry.rpm[i] = esc_status.esc[i].esc_rpm;
            }
        }
        // Distance sensor update
        if (distance_sensor_sub.update(&distance_sensor))
        {
            logic.status_bits[1] = true;
            logic.distance_sensor.distance = distance_sensor.current_distance;
        }
        // Attitude update
        if (vehicle_attitude_sub.update(&vehicle_attitude))
        {
            // Get PX4 attitude
            Rotationf q(vehicle_attitude.q[0], vehicle_attitude.q[1], vehicle_attitude.q[2], vehicle_attitude.q[3]);
            q.Normalise();
            // Convert to lab frame
            // Rotationf const R_Lab_EKF = Rotationf::FromRotationVector(Vec3f(M_PI, 0, 0));
            q = logic._R * q * logic._R.Inverse();
            // Assign to variables in quadcopter logic
            logic.vehicle_attitude.q = q;
            q.ToEulerYPR(logic.vehicle_attitude.ypr[0], logic.vehicle_attitude.ypr[1], logic.vehicle_attitude.ypr[2]);
        }
        // GPS update
        if (vehicle_gps_position_sub.update(&vehicle_gps_position))
        {
            logic.status_bits[2] = true;
            logic.vehicle_gps_position.eph = vehicle_gps_position.eph;
            logic.vehicle_gps_position.epv = vehicle_gps_position.epv;
            logic.vehicle_gps_position.satellites_used = vehicle_gps_position.satellites_used;
        }
        // Local position update
        if (vehicle_local_position_sub.update(&vehicle_local_position))
        {
            logic.status_bits[3] = true;
            // Get position and velocity in NED frame
            Vec3f position(vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z);
            Vec3f velocity(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);
            // Convert to lab frame
            position = logic._R * position;
            velocity = logic._R * velocity;
            // Assign to variables in quadcopter logic
            logic.vehicle_local_position.position = position;
            logic.vehicle_local_position.velocity = velocity;
        }
        // Vehicle odometry update
        if (vehicle_odometry_sub.update(&vehicle_odometry))
        {
            logic.status_bits[4] = true;
            // Get position and velocity in NED frame
            Vec3f position(vehicle_odometry.x, vehicle_odometry.y, vehicle_odometry.z);
            Vec3f velocity(vehicle_odometry.vx, vehicle_odometry.vy, vehicle_odometry.vz);
            Rotationf q(vehicle_odometry.q[0], vehicle_odometry.q[1], vehicle_odometry.q[2], vehicle_odometry.q[3]);
            q.Normalise();
            Vec3f omega(vehicle_odometry.rollspeed, vehicle_odometry.pitchspeed, vehicle_odometry.yawspeed);
            // Convert to lab frame
            position = logic._R * position;
            velocity = logic._R * velocity;
            q = logic._R * q * logic._R.Inverse();
            omega = logic._R * omega;
            // Assign to variables in quadcopter logic
            logic.vehicle_odometry.position = position;
            logic.vehicle_odometry.velocity = velocity;
            logic.vehicle_odometry.q = q;
            logic.vehicle_odometry.omega = omega;
            q.ToEulerYPR(logic.vehicle_odometry.ypr[0], logic.vehicle_odometry.ypr[1], logic.vehicle_odometry.ypr[2]);
        }

        /* Update motor commands */
        actuatorCmds.nvalues = 4;
        actuatorCmds.dshot_direct = logic.dshot_direct;
        for (int i = 0; i < 4; i++)
        {
            actuatorCmds.values[i] = logic.GetMotorSpeedCmd(i); // uncomment to enable motor commands
            actuatorCmds.dshot_values[i] = logic.dshot_values[i];
            actuatorCmds.bidirectional = logic.bidirectional;
            // actuatorCmds.values[i] = 0.0f; // uncomment to disable motor commands
        }
        actuatorCmds.voltage = (logic.esc_telemetry.voltage[0] + logic.esc_telemetry.voltage[1] + logic.esc_telemetry.voltage[2] + logic.esc_telemetry.voltage[3])/4.0f;
        /* Publish motor commands */
        orbErr |= orb_publish(ORB_ID(actuator_direct), orb_pub_actuatorCmds, &actuatorCmds);

        /* Legacy? */
        rangingRequest.desired_responder_id = logic.GetNextUWBRangingTarget();

        if (logic.ShouldStartNewUWBConversation())
        {
            rangingRadioTimeoutDetection_areWaiting = true;
            rangingRadioTimeoutDetection_timeSinceRequest.Reset();
            orbErr |= orb_publish(ORB_ID(ranging_request), orb_pub_rangingRequest, &rangingRequest);
            qLog.uwb_request_id = rangingRequest.desired_responder_id;
        }
        else
        {
            qLog.uwb_request_id = 0;
        }

        if (logic.GetFlightState() == Onboard::QuadcopterLogic::FS_IDLE)
        {
            if (logic.GetAndResetShouldWriteParameters())
            {
                float f0 = logic.GetPropellerCorrectionFactor(0);
                float f1 = logic.GetPropellerCorrectionFactor(1);
                float f2 = logic.GetPropellerCorrectionFactor(2);
                float f3 = logic.GetPropellerCorrectionFactor(3);

                param_set(param_find("PROPCORR_0"), &f0);
                param_set(param_find("PROPCORR_1"), &f1);
                param_set(param_find("PROPCORR_2"), &f2);
                param_set(param_find("PROPCORR_3"), &f3);
                param_save_default();
            }
        }

        if (!logic.GetAreMotorsRunning())
        {
            qLog.is_idle = true;
        }
        else
        {
            qLog.is_idle = false;
        }

        // Run the LEDs:
        if (logic.GetGyroCalibration())
        {
            // Turn on LED when we're calibrating
            led_on(LED_RED);
        }
        else
        {
            // tell the user about panics (as appropriate):
            switch (logic.GetFlightState())
            {
            case Onboard::QuadcopterLogic::FS_KILLED:
            {
                unsigned const KILLED_BLINK_FREQ = 10;      // Hz
                float const KILLED_BLINK_DUTY_CYCLE = 0.8f; // between 0 and 1

                if ((KILLED_BLINK_FREQ * loopCounter) % ONBOARD_FREQUENCY < unsigned(KILLED_BLINK_DUTY_CYCLE * ONBOARD_FREQUENCY))
                {
                    led_on(LED_RED);
                }
                else
                {
                    led_off(LED_RED);
                }
            }
            break;

            case Onboard::QuadcopterLogic::FS_PANIC:
            {
                // blink out the panic code
                int const BLINK_FREQ = 2; //[Hz]
                int const PAUSE = 2;      // periods, not sec
                int blinkPeriod = int(logic.GetFirstPanicReason()) + PAUSE;
                int curCycle = ((BLINK_FREQ * loopCounter) / ONBOARD_FREQUENCY) % blinkPeriod; // in (0,1,2,3...blinkPeriod-1)
                if (curCycle < int(logic.GetFirstPanicReason()))
                {
                    if ((BLINK_FREQ * loopCounter) % ONBOARD_FREQUENCY < (ONBOARD_FREQUENCY / 2))
                    {
                        led_on(LED_RED);
                    }
                    else
                    {
                        led_off(LED_RED);
                    }
                }
                else
                {
                    led_off(LED_RED);
                }
            }
            break;
            default:
                led_off(LED_RED);
                break;
            }
        }

        // populate the logger:
        {
            uint64_t curTime = hwtimer.GetMicroSeconds() / 1000;
            qLog.time_ms = curTime % (1 << 16);
            qLog.sensor_acc_x = logic.GetAccelerometer().x;
            qLog.sensor_acc_y = logic.GetAccelerometer().y;
            qLog.sensor_acc_z = logic.GetAccelerometer().z;
            qLog.sensor_gyro_x = logic.GetRateGyroBiasCorrected().x;
            qLog.sensor_gyro_y = logic.GetRateGyroBiasCorrected().y;
            qLog.sensor_gyro_z = logic.GetRateGyroBiasCorrected().z;
            orbErr |= orb_publish(ORB_ID(hiperlab_quad_log), orb_pub_quadLog, &qLog);
        }

        if (orbErr)
        {
            thread_running = false;
            printf("Error publishing to uorb!\n");
            continue;
        }

        perf_end(perf_timer_obLogic);
        perf_end(perf_timer_obLoop);
    }

    hrt_cancel(&ol_tick_call);
    usleep(100);
    sem_destroy(&_sem);

    printf("Exiting: fast_loop\n");

    return 0;
}

int quad_main(int argc, char *argv[])
{

    if (argc < 2)
    {
        usage();
        return -1;
    }

    if (!strcmp(argv[1], "start"))
    {
        if (thread_running)
        {
            printf("Thread already running\n");
            return 0;
        }

        if (argc > 2)
        {
            if (!strcmp(argv[2], "v"))
            {
                verbose = true;
                printf("Setting to verbose mode\n");
            }
        }

        int vehId = 0;
        int motorType = 0;
        if (0 != param_get(param_find("VEHICLE_ID"), &vehId))
        {
            printf("Failed to get param <VEHICLE_ID>\n");
            vehId = 0;
        }
        if (0 != param_get(param_find("MOTOR_TYPE"), &motorType))
        {
            printf("Failed to get param <MOTOR_TYPE>\n");
            vehId = 0;
        }
        if (0 != param_get(param_find("FC_TYPE"), &FCType))
        {
            printf("Failed to get param <FC_TYPE>\n");
            vehId = 0;
        }
        Onboard::QuadcopterConstants::QuadcopterType quadType = Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehId);
        Onboard::QuadcopterConstants consts(static_cast<Onboard::QuadcopterConstants::QuadcopterType>(quadType));
        // Set the voltage and current calibration coefficients
        voltage_calibration_m = consts.voltageCurrentCalibrationConsts[0][0];
        voltage_calibration_c = consts.voltageCurrentCalibrationConsts[0][1];
        current_calibration_m = consts.voltageCurrentCalibrationConsts[1][0];
        current_calibration_c = consts.voltageCurrentCalibrationConsts[1][1];

        if (quadType == Onboard::QuadcopterConstants::QC_TYPE_INVALID)
        {
            printf("Invalid quad type (change vehicle ID to fix). Exiting.\n");
            return 0;
        }

        // Read the propeller correction factors:
        float propCorr0, propCorr1, propCorr2, propCorr3;
        if (0 != param_get(param_find("PROPCORR_0"), &propCorr0))
        {
            printf("Failed to get param <PROPCORR_0>\n");
            vehId = 0;
        }

        if (0 != param_get(param_find("PROPCORR_1"), &propCorr1))
        {
            printf("Failed to get param <PROPCORR_1>\n");
            vehId = 0;
        }

        if (0 != param_get(param_find("PROPCORR_2"), &propCorr2))
        {
            printf("Failed to get param <PROPCORR_2>\n");
            vehId = 0;
        }

        if (0 != param_get(param_find("PROPCORR_3"), &propCorr3))
        {
            printf("Failed to get param <PROPCORR_3>\n");
            vehId = 0;
        }
        logic.SetPropellerCorrectionFactor(propCorr0, propCorr1, propCorr2, propCorr3);

        logic.Initialise(Onboard::QuadcopterConstants::QuadcopterType(quadType), vehId);

        // Set priority to 180 so that logic is higher priority than other
        // processes This allows the logic to run more consistently at 0.002s
        // per loop
        daemon_task = px4_task_spawn_cmd("logic_quad", SCHED_DEFAULT, SCHED_PRIORITY_MAX - 20, 4096, logicThread, (char *const *)NULL);

        printf("Started\n");
        return 0;
    }

    if (!strcmp(argv[1], "go"))
    {
        if (!thread_running)
        {
            printf("Thread not running\n");
            return 0;
        }

        int takeoffDelay = 3;
        if (argc > 2)
        {
            takeoffDelay = atoi(argv[2]);
        }
        printf("Will take off in %d seconds:\n", takeoffDelay);
        for (int i = 0; i < takeoffDelay; i++)
        {
            printf("%d\n", int(takeoffDelay - i));
            usleep(1000 * 1000);
        }
        printf("Decolage!\n");
        logic.SetGoAutonomous();
        return 0;
    }
    if (!strcmp(argv[1], "stop"))
    {
        if (!thread_running)
        {
            printf("thread not running\n");
            return 0;
        }
        thread_running = false;

        usleep(1000);
        OnTimer(nullptr);

        printf("done!\n");
        return 0;
    }

    if (!strcmp(argv[1], "test") || !strcmp(argv[1], "t"))
    {
        if (argc < 3)
        {
            usage();
            return -1;
        }

        if (!strcmp(argv[2], "thrust") || !strcmp(argv[2], "t"))
        {
            if (argc < 4)
            {
                usage();
                return -1;
            }

            // If the 5th argument was provided, use that value for running
            // motors for x integer seconds; if not, default to 5 seconds
            unsigned TEST_TIME = 5;
            if (argc > 4)
            {
                int temp = atoi(argv[4]);
                TEST_TIME = (temp >= 1 && temp <= 30) ? temp : 1;
            }

            float thrustFrac = atof(argv[3]);
            printf("Running thrust test for %fs, @ %.3f*weight\n", double(TEST_TIME), double(thrustFrac));
            // TODO: test modes
            logic.TestMotors(true, thrustFrac);
            usleep(TEST_TIME * 1000 * 1000 / 2);
            if (verbose)
            {
                // Print status in the middle of test to show
                // stats of the vehicle while in action
                logic.PrintStatus();
            }
            usleep(TEST_TIME * 1000 * 1000 / 2);
            logic.TestMotors(false, 0);
            return 0;
        }

        if (!strcmp(argv[2], "current"))
        {
            const unsigned TEST_TIME = 5;
            printf("Running current test for %ds\n", TEST_TIME);
            printf("Note: measurements are from the ADC ports and NOT for use "
                   "with a standard crazyflie!\n");
            for (unsigned i = 0; i < 100 * TEST_TIME; i++)
            {
                orb_copy(ORB_ID(adc_report), orb_sub_adcReport, &adcReport);
                MeasureVoltageAndCurrent();
                printf("i = %6.3fA\n", double(currentMeas));
                printf("v = %6.3fV\n", double(voltageMeas));
                usleep(10 * 1000);
            }
            return 0;
        }

        if (!strcmp(argv[2], "adc"))
        {
            const unsigned TEST_TIME = 1;
            printf("Printing raw ADC values for %ds\n", TEST_TIME);
            for (unsigned i = 0; i < 100 * TEST_TIME; i++)
            {
                orb_copy(ORB_ID(adc_report), orb_sub_adcReport, &adcReport);
                unsigned nChannels = 5; // 12 channels max. Have as many as you want to print
                printf("Channel:");
                for (unsigned j = 0; j < nChannels; j++)
                {
                    printf("\t%d", adcReport.channel_id[j]);
                }
                printf("\nValue:");
                for (unsigned j = 0; j < nChannels; j++)
                {
                    printf("\t%.3f", double(adcReport.raw_data[j]));
                }
                printf("\n");
                usleep(10 * 1000);
            }
            return 0;
        }

        if (!strcmp(argv[2], "ranging"))
        {
            return RunTestRanging();
        }
    }

    if (!strcmp(argv[1], "setVehicleId"))
    {
        if (argc >= 3)
        {
            int vehIdIn = atoi(argv[2]);
            if ((vehIdIn >= 1) && (vehIdIn <= 255))
            {
                // valid argument
                printf("setting vehicle ID to %d\n", vehIdIn);

                if (param_set(param_find("VEHICLE_ID"), &vehIdIn))
                {
                    printf("Failed to set <VEHICLE_ID>\n");
                    return -1;
                }

                // Get the quadType from the vehicle ID
                Onboard::QuadcopterConstants::QuadcopterType quadTypeIn = Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehIdIn);
                // Get the motor type based on quadType
                int motorType = Onboard::QuadcopterConstants(quadTypeIn).motorType;
                // Get the FC type based on quadType
                FCType = Onboard::QuadcopterConstants(quadTypeIn).FCType;

                if (param_set(param_find("MOTOR_TYPE"), &motorType))
                {
                    printf("Failed to set <MOTOR_TYPE>\n");
                    return -1;
                }
                if (param_set(param_find("FC_TYPE"), &FCType))
                {
                    printf("Failed to set <FC_TYPE>\n");
                    return -1;
                }

                printf("Param set: VEHICLE_ID to %d\n", vehIdIn);
                printf("Param set: MOTOR_TYPE to %d\n", motorType);
                printf("Param set: FC_TYPE to %d\n", FCType);
                printf("QuadcopterType = %s\n", Onboard::QuadcopterConstants::GetNameString(quadTypeIn));
                printf("Now you *MUST* run `param save` to store the params\n");

                return 0;
            }
            else
            {
                printf("Invalid type\n");
            }
        }
        else
        {
            printf("Too few arguments\n");
            printf("e.g. quad vehIdIn 7\n");
            return -1;
        }
    }

    if (!strcmp(argv[1], "resetPropCalib"))
    {
        printf("resetting all propeller calibration factors to 1.0\n");
        float newVal = 1.0f;
        if (param_set(param_find("PROPCORR_0"), &newVal))
        {
            printf("Failed to set <PROPCORR_0>\n");
        }
        if (param_set(param_find("PROPCORR_1"), &newVal))
        {
            printf("Failed to set <PROPCORR_1>\n");
        }
        if (param_set(param_find("PROPCORR_2"), &newVal))
        {
            printf("Failed to set <PROPCORR_2>\n");
        }
        if (param_set(param_find("PROPCORR_3"), &newVal))
        {
            printf("Failed to set <PROPCORR_3>\n");
        }
        printf("Now you *MUST* run `param save` to store the params\n");
        return 0;
    }

    if (!strcmp(argv[1], "status"))
    {
        if (thread_running)
        {
            logic.PrintStatus();
        }
        else
        {
            printf("\tquad not started\n");
        }
        return 0;
    }

    printf("Unknown command\n");
    usage();
    return -1;
}

int RunTestRanging()
{
    printf("---------------------\n");
    printf("  Ranging test \n");
    printf("---------------------\n");
    printf("Ranges are: \n\n");

    float tFull = 10.0f; // run test this long

    Timer testTimer(&hwtimer);
    testTimer.Reset();
    unsigned measCounter_all = 0;
    unsigned measCounter_valid = 0;
    rangingTests_shouldStartNewUWBConversation = true;
    for (;;)
    {
        if (testTimer.GetSeconds_f() > tFull)
        {
            break;
        }

        if (rangingTests_haveNewResult)
        {
            rangingTests_haveNewResult = false;
            printf("%6.6f,", double(rangingTests_latestMeas));
            measCounter_all++;
            if (rangingTests_latestMeas > 0)
            {
                measCounter_valid++;
            }
        }

        usleep(2000);
    }
    printf("\n");
    printf("===============\n");
    printf("Test complete\n");
    printf("===============\n");
    printf("\tin %3.3fsec, we got %d meas (of which %d were valid)\n", double(testTimer.GetSeconds_f()), int(measCounter_all), int(measCounter_valid));
    return 0;
}
