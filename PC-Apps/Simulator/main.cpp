/* A simple simulator for a single quadcopter.
 *
 */

#include <memory>
#include <Eigen/Dense>

#include "Common/Time/ManualTimer.hpp"
#include "Components/Simulation/Quadcopter_T.hpp"
#include "Components/Simulation/UWBNetwork.hpp"
#include "Components/Simulation/CommunicationsDelay.hpp"

#include "Components/Offboard/MocapStateEstimator.hpp"
#include "Components/Offboard/QuadcopterController.hpp"
#include "Components/Offboard/SafetyNet.hpp"

#include "Components/Logic/QuadcopterLogic.hpp"

#include <fstream>

using namespace std;
using namespace Offboard;

enum {
  CTRL_ONBOARD_UWB,
  CTRL_OFFBOARD_RATES,
  CTRL_OFFBOARD_ACCELERATION,
} controllerType;

template<typename Real>
string toCSV(const Vec3<Real> v) {
  stringstream ss;
  ss << v.x << "," << v.y << "," << v.z << ",";
  return ss.str();
}

int main(void) {
  ////////////////////////////////////////////
  //Basic timing:
  const double dt = 1.0 / 500.0;  //run the simulation at this rate

  const double endTime = 10.0f;  //[s]
  ManualTimer simTimer;

  //Create the quadcopter:
  uint8_t vehicleId = 1;  //For UWB network, commands, etc.
  Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
      Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehicleId);

  Onboard::QuadcopterConstants vehConsts(quadcopterType);
  //Create the quadcopter:
  double const mass = vehConsts.mass;  //[kg]
  double const inertia_xx = vehConsts.Ixx;  //[kg.m**2]
  double const inertia_yy = inertia_xx;  //[kg.m**2]
  double const inertia_zz = vehConsts.Izz;  //[kg.m**2]
  double armLength = vehConsts.armLength;  //[m]
  double propThrustFromSpeedSqr = vehConsts.propellerThrustFromSpeedSqr;  //[N/(rad/s)**2]
  double propTorqueFromSpeedSqr = vehConsts.propellerTorqueFromThrust
      * vehConsts.propellerThrustFromSpeedSqr;

  double motorTimeConst = vehConsts.motorTimeConst;  // [s]
  double motorInertia = vehConsts.motorInertia;  // [kg.m**2]
  double motorMinSpeed = vehConsts.motorMinSpeed;  // [rad/s]
  double motorMaxSpeed = vehConsts.motorMaxSpeed;  // [rad/s]

  Vec3d centreOfMassError = Vec3d(0, 0, 0);

  //Mocap system:
  double const periodMocapSystem = 1.0 / 200.0;  //[s] time between mocap measurments
  double const periodOffboardMainLoop = 1.0 / 50.0;  //[s] time between offboard main loop runs
  double const periodOnboardLogic = 1.0 / 500.0;  //[s] time between onboard logic runs

  double const timeDelayOffboardControlLoopTrue = 0.03;  //[s] TODO: we should measure this!
  double const timeDelayOffboardControlLoopEstimate = 0.03;  //[s]

  //noise in the UWB system:
  double uwbNoiseStdDev = 50e-3;  //[m]
  double uwbOutlierProb = 0.01;
  double uwbOutlierStdDev = 10;  //[m]

  Eigen::Matrix<double, 3, 3> inertiaMatrix;
  inertiaMatrix << inertia_xx, 0, 0, 0, inertia_yy, 0, 0, 0, inertia_zz;

  //drag coefficients
  Vec3d linDragCoeffB = Vec3d(vehConsts.linDragCoeffBx,
                              vehConsts.linDragCoeffBy,
                              vehConsts.linDragCoeffBz);

  std::shared_ptr<Simulation::Quadcopter> quad;
  quad.reset(
      new Simulation::Quadcopter(&simTimer, mass, inertiaMatrix, armLength,
                                 centreOfMassError, motorMinSpeed,
                                 motorMaxSpeed, propThrustFromSpeedSqr,
                                 propTorqueFromSpeedSqr, motorTimeConst,
                                 motorInertia, linDragCoeffB, vehicleId,
                                 quadcopterType, periodOnboardLogic));

  //Create other UWB objects:
  double const uwbNetworkRangingPeriod = 1.0 / 100;  //[s], runs at approx. 100Hz.

  //Create some static UWB radios (Ids = 2,3,4)
  std::vector<std::shared_ptr<Simulation::UWBRadio> > staticRadios;

  std::shared_ptr<Simulation::UWBRadio> r2, r3, r4;
  r2.reset(new Simulation::UWBRadio(&simTimer, 2));
  r2->SetPosition(Vec3d(10, 0, 0));
  r3.reset(new Simulation::UWBRadio(&simTimer, 3));
  r3->SetPosition(Vec3d(0, 10, 0));
  r4.reset(new Simulation::UWBRadio(&simTimer, 4));
  r4->SetPosition(Vec3d(0, 0, 10));

  //add all radios to the network
  std::shared_ptr<Simulation::UWBNetwork> uwbNetwork;
  uwbNetwork.reset(
      new Simulation::UWBNetwork(&simTimer, uwbNetworkRangingPeriod));
  uwbNetwork->SetNoiseProperties(uwbNoiseStdDev, uwbOutlierProb,
                                 uwbOutlierStdDev);
  uwbNetwork->AddRadio(r2);
  uwbNetwork->AddRadio(r3);
  uwbNetwork->AddRadio(r4);
  uwbNetwork->AddRadio(quad->GetRadio());

  //Tell the quadcopter's firmware about the radios:
  quad->AddUWBRadioTarget(r2->GetId(), r2->GetPosition());
  quad->AddUWBRadioTarget(r3->GetId(), r3->GetPosition());
  quad->AddUWBRadioTarget(r4->GetId(), r4->GetPosition());

  //Offboard estimation / control code:
  shared_ptr<MocapStateEstimator> est;
  est.reset(
      new MocapStateEstimator(&simTimer, vehicleId,
                              timeDelayOffboardControlLoopEstimate));
  QuadcopterController ctrl;
  Onboard::QuadcopterPositionController ctrlPos;

  SafetyNet safetyNet;
  const float timeConstYaw = 1.0f;
  const float posControl_natFreq = 1.0f;
  const float posControl_damping = 0.7f;
  ctrlPos.SetParameters(posControl_natFreq, posControl_damping);
  ctrl.SetParameters(vehConsts.posControl_natFreq, vehConsts.posControl_damping,
                     vehConsts.attControl_timeConst_xy,
                     vehConsts.attControl_timeConst_z);

  controllerType = CTRL_OFFBOARD_RATES;
//  controllerType = CTRL_OFFBOARD_ACCELERATION;

  //create an initial error:
  Vec3d const initErrPos = Vec3d(2, 0, 0);
  Rotationd const initErrAtt = Rotationd::FromEulerYPR(20 * M_PI / 180.0,
                                                       0 * M_PI / 180.0, 0);

  //where we want the quadcopter to fly to:
  Vec3d desiredPosition(0, 0, 1.5);
  double desYawAngleDeg = 0;  //[deg]

  //file for logging:
  ofstream logfile;
  logfile.open("Logs/Simulation.csv");

  printf("Starting simulation\n");
  Timer t(&simTimer);
  quad->SetPosition(initErrPos);
  quad->SetAttitude(initErrAtt);

  Simulation::CommunicationsDelay<RadioTypes::RadioMessageDecoded::RawMessage> cmdRadioChannel(
      &simTimer, timeDelayOffboardControlLoopTrue);

  Timer timerPrint(&simTimer);
  Timer timerMocap(&simTimer);
  Timer timerOffboardMainLoop(&simTimer);

  float lastRadioCommand[4];
  for (int i = 0; i < 4; i++) {
    lastRadioCommand[i] = 0;
  }
  while (t.GetSeconds<double>() < endTime) {
    quad->Run();
    uwbNetwork->Run();
    simTimer.AdvanceMicroSeconds(uint64_t(dt * 1e6));

    if (timerPrint.GetSeconds<double>() > 1) {
      timerPrint.AdjustTimeBySeconds(-1);
      printf("Current sim time = %.1fs\n", t.GetSeconds<double>());
    }

    if (timerMocap.GetSeconds<double>() > periodMocapSystem) {
      //simulate mocap packets over the network
      timerMocap.AdjustTimeBySeconds(-periodMocapSystem);
      Vec3d measPos(quad->GetPosition());
      Rotationd measAtt(quad->GetAttitude());
      est->UpdateWithMeasurement(measPos, measAtt);
    }

    if (timerOffboardMainLoop.GetSeconds<double>() > periodOffboardMainLoop) {
      //Simulate the offboard main loop
      timerOffboardMainLoop.AdjustTimeBySeconds(-periodOffboardMainLoop);

      //Publish the commands:
      RadioTypes::RadioMessageDecoded::RawMessage rawMsg;
      if (controllerType == CTRL_OFFBOARD_RATES) {
        MocapStateEstimator::MocapEstimatedState estState = est->GetPrediction(
            timeDelayOffboardControlLoopEstimate);

        safetyNet.UpdateWithEstimator(estState,
                                      est->GetTimeSinceLastGoodMeasurement());

        Vec3d cmdAngVel;
        double cmdThrust;
        uint8_t flags = 0;
        ctrl.Run(estState.pos, estState.vel, estState.att, desiredPosition,
                 Vec3d(0, 0, 0), Vec3d(0, 0, 0), desYawAngleDeg * M_PI / 180.0,
                 cmdAngVel, cmdThrust);

        RadioTypes::RadioMessageDecoded::CreateRatesCommand(flags,
                                                            float(cmdThrust),
                                                            Vec3f(cmdAngVel),
                                                            rawMsg.raw);

        lastRadioCommand[0] = cmdThrust;
        lastRadioCommand[1] = cmdAngVel.x;
        lastRadioCommand[2] = cmdAngVel.y;
        lastRadioCommand[3] = cmdAngVel.z;

        est->SetPredictedValues(
            cmdAngVel,
            (estState.att * Vec3d(0, 0, 1) * cmdThrust - Vec3d(0, 0, 9.81)));

        //We disable this for now, since it triggers on the first run.
//        if (!safetyNet.GetIsSafe()) {
//          RadioTypes::RadioMessageDecoded::CreateKillOneCommand(vehicleId, 0,
//                                                                rawMsg.raw);
//        }
      } else if (controllerType == CTRL_OFFBOARD_ACCELERATION) {
        MocapStateEstimator::MocapEstimatedState estState = est->GetPrediction(
            timeDelayOffboardControlLoopEstimate);
        safetyNet.UpdateWithEstimator(estState,
                                      est->GetTimeSinceLastGoodMeasurement());

        Vec3f const cmdAcc = ctrlPos.GetDesAcceleration(Vec3f(estState.pos),
                                                        Vec3f(estState.vel),
                                                        Vec3f(desiredPosition),
                                                        Vec3f(0, 0, 0),
                                                        Vec3f(0, 0, 0));

        //ugly, lazy hack -- there will be a nicer, geometric trick to do this too, no euler angles
        double yaw = estState.att.ToEulerYPR()[0];
        Rotationf yawCorrection = Rotationd::FromEulerYPR(float(yaw), 0, 0);
        Vec3f cmdAccYawed = yawCorrection.Inverse() * cmdAcc;

        double yawError = desYawAngleDeg * M_PI / 180 - yaw;
        if (yawError > M_PI) {
          yawError -= 2 * M_PI;
        } else if (yawError < -M_PI) {
          yawError += 2 * M_PI;
        }

        float const desYawRate = yawError / timeConstYaw;
        uint8_t flags = 0;

        //Publish the commands:
        RadioTypes::RadioMessageDecoded::CreateAccelerationCommand(flags,
                                                                   cmdAccYawed,
                                                                   desYawRate,
                                                                   rawMsg.raw);
      } else if (controllerType == CTRL_ONBOARD_UWB) {
        uint8_t flags = 0;
        RadioTypes::RadioMessageDecoded::CreatePositionCommand(flags,
                                                               desiredPosition,
                                                               Vec3d(0, 0, 0),  //zero velocity
                                                               Vec3d(0, 0, 0),  //zero acceleration
                                                               0,  //yaw
                                                               rawMsg.raw);
      }

      TelemetryPacket::data_packet_t dataPacketRaw1, dataPacketRaw2, dataPacketRaw3, dataPacketRaw4;
      quad->GetTelemetryDataPackets(dataPacketRaw1, dataPacketRaw2, dataPacketRaw3, dataPacketRaw4);

      TelemetryPacket::TelemetryPacket dataPacket;
      TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw1, dataPacket);
      TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw2, dataPacket);

      //add message to command radio queue
      cmdRadioChannel.AddMessage(rawMsg);

      //Write the simulation state to a file:
      logfile << t.GetSeconds<double>() << ",";
      logfile << toCSV(quad->GetPosition());
      logfile << toCSV(quad->GetVelocity());
      logfile << toCSV(quad->GetAttitude().ToEulerYPR());
      logfile << toCSV(quad->GetAngularVelocity());
      logfile << dataPacket.motorForces[0] << ",";
      logfile << dataPacket.motorForces[1] << ",";
      logfile << dataPacket.motorForces[2] << ",";
      logfile << dataPacket.motorForces[3] << ",";

      //estimator state:
      Vec3f pos;
      Vec3f vel;
      Rotationf att;
      Vec3f angVel;
      if (controllerType != CTRL_ONBOARD_UWB) {
        MocapStateEstimator::MocapEstimatedState estState = est->GetPrediction(
            0);
        pos = estState.pos;
        vel = estState.vel;
        att = estState.att;
        angVel = estState.angVel;
      } else {
        pos = Vec3d(dataPacket.position);
        vel = Vec3d(dataPacket.velocity);
        att = Rotationd::FromVectorPartOfQuaternion(Vec3d(dataPacket.attitude));
        angVel = Vec3d(dataPacket.gyro);
      }
      logfile << toCSV(pos);
      logfile << toCSV(vel);
      logfile << toCSV(att.ToEulerYPR());
      logfile << toCSV(angVel);

      //desired position:
      logfile << toCSV(desiredPosition);

      logfile << int(dataPacket.panicReason) << ",";

      for (int i = 0; i < 4; i++) {
        logfile << double(lastRadioCommand[i]) << ",";
      }

      logfile << "\n";
    }  //main loop

    //Check if we have a new message to transmit:
    if (cmdRadioChannel.HaveNewMessage()) {
      quad->SetCommandRadioMsg(cmdRadioChannel.GetMessage());
    }

  }  //sim loop
  printf("Done.\n");
  logfile.close();
}

