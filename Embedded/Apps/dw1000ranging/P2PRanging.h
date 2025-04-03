#pragma once
#include <stdint.h>

#include "DW1000Constants.h"
#include "DW1000.h"

#include <uORB/topics/ranging_report.h>

namespace DW1000NS {
class P2PRanging {
 public:
  enum MessageTypes {
    MSG_RANGING_INIT = 0,  //initiate the ranging exchange A->B.
    MSG_RANGING_REPLY1 = 1,  //acknowledge receipt of "MSG_RANGING_INIT", B->A
    MSG_RANGING_REPLY2 = 2,  //Third message in exchange, A->B
    MSG_RANGING_REPORT = 3,  //Computed range, B->A
    MSG_RANGING_FAILED = 255,  //exchange failed.
  };

  enum MessageFields {
    MSG_FIELD_TYPE = 0,  //what type of message is this?
    MSG_FIELD_SENDER_ID = MSG_FIELD_TYPE + 1,
    MSG_FIELD_TARGET_ID = MSG_FIELD_SENDER_ID + 1,
    MSG_FIELD_INTERACTION_COUNTER = MSG_FIELD_TARGET_ID + 1,
    MSG_FIELD_DATA_START = MSG_FIELD_INTERACTION_COUNTER + 1,
  };

  enum {  //some constants
    LEN_DATA = 19,
    //Tested as working:
    // Preamble | Delay
    // Length	|
	//----------+--------
    // 128      |  300
    // 1024     | 1200
	// 2048		| 2400
    //----------+--------
    MSG_TX_DELAY_TIME_us = 2400,
    PROTOCOL_TIMEOUT_us = (MSG_TX_DELAY_TIME_us * 2 + MSG_TX_DELAY_TIME_us / 2),
  };

  P2PRanging();

  static int Initialize(uint8_t deviceId, uint16_t networkId);

  static void SetVerbose(bool v) {
    _verbose = v;
  }

  static void LoopFunction(void);

  static bool HaveNewResult() {
    return _haveNewResult;
  }

  static ranging_report_s GetLatestResult() {
    _haveNewResult = false;
    return _rangingRept;
  }

  static void SetNewRangingRequest(uint8_t targetId) {
    _newRangingRequestId = targetId;
  }

  void printStatus();
  void clearSysStatus();

  static void runLoop();

  inline static float CalibratedRange(uint32_t raw_range_um) {
    // Converts raw range measurements in [um] to calibrated range measurements in [m].
    return _rangingScale * (float(raw_range_um) * 1e-6f) + _rangingBias;
  }

 private:
  static void noteActivity() {
    _lastActivityTime_ms = DW1000Class::getCPUTimeMillis();
  }

  static bool isRxMessageConsistent();

  static void resetInactive();

  static void handleSent();
  static void handleReceived();
  static void handleError();
  static void handleReceiveFailed();
  static void handleReceiveTimeout();
  static void handleReceiveTimestampAvailable();

  //the four message types
  static void transmitRangingInit();
  static void transmitRangingReply1();
  static void transmitRangingReply2();
  static void transmitRangeReport(const uint32_t curRange_um);
  static void transmitRangeFailed();

  static void computeRangeAsymmetric();
  static void computeRangeSymmetric();

  static void rangingReceiver();  //->setUpAsReceiver();

  static void printMessageStr(MessageTypes msg);

  static volatile MessageTypes _expectedMsg;
  static volatile MessageTypes _lastTxMsg;
  static volatile MessageTypes _lastRxMsg;

  // message sent/received state
  static volatile bool _haveUnhandledSentMsg;
  static volatile bool _haveUnhandledReceivedMsg;
  static volatile bool _haveUnhandledError;
  static volatile bool _haveUnhandledReceiveFailed;
  static volatile bool _haveUnhandledReceiveTimeout;
  static volatile bool _haveUnhandledReceiveTimestampAvailable;
  // protocol error state
  static volatile bool _protocolFailed;

  // timestamps to remember
  static DW1000Time _timeRangingInitSent;
  static DW1000Time _timeRangingInitReceived;
  static DW1000Time _timeRangingReply1Sent;
  static DW1000Time _timeRangingReply1Received;
  static DW1000Time _timeRangingReply2Sent;
  static DW1000Time _timeRangingReply2Received;
  // last computed range/time
  static DW1000Time _timeComputedRange;
  // data buffer
  static uint8_t _rxData[LEN_DATA];
  static uint8_t _txData[LEN_DATA];
  // watchdog and reset period
  static volatile uint32_t _lastActivityTime_ms;

  static uint8_t _myId;
  static uint8_t _commPartnerId;  //the Id of the other party we're talking to
  static uint16_t _networkId;

  static unsigned _numRangingsInitiationsSent;
  static unsigned _numRangingsInitiationsReceived;
  static unsigned _numRangingsCompletedSent;
  static unsigned _numRangingsCompletedReceived;
  static unsigned _numTimeouts;
  static unsigned _numResets;
  static unsigned _numMsgReceived;
  static unsigned _numMsgSent;
  static unsigned _numMsgSentInit;

  static uint8_t _newRangingRequestId;

  static bool _verbose;

  static bool _haveNewResult;
  static ranging_report_s _rangingRept;

  static float _rangingBias;  // in [m]
  static float _rangingScale;  // dimensionless
};

}  // namespace DW1000NS
