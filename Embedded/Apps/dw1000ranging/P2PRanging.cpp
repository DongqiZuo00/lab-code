#include "P2PRanging.h"

#include <perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include "DW1000.h"

using namespace DW1000NS;

//definitions of static members:
volatile P2PRanging::MessageTypes P2PRanging::_expectedMsg;
volatile P2PRanging::MessageTypes P2PRanging::_lastTxMsg;
volatile P2PRanging::MessageTypes P2PRanging::_lastRxMsg;

volatile bool P2PRanging::_haveUnhandledSentMsg;
volatile bool P2PRanging::_haveUnhandledReceivedMsg;
volatile bool P2PRanging::_protocolFailed;
DW1000Time P2PRanging::_timeRangingInitSent;
DW1000Time P2PRanging::_timeRangingInitReceived;
DW1000Time P2PRanging::_timeRangingReply1Sent;
DW1000Time P2PRanging::_timeRangingReply1Received;
DW1000Time P2PRanging::_timeRangingReply2Sent;
DW1000Time P2PRanging::_timeRangingReply2Received;
DW1000Time P2PRanging::_timeComputedRange;
uint8_t P2PRanging::_rxData[P2PRanging::LEN_DATA];
uint8_t P2PRanging::_txData[P2PRanging::LEN_DATA];
volatile uint32_t P2PRanging::_lastActivityTime_ms;
uint8_t P2PRanging::_myId;
uint8_t P2PRanging::_commPartnerId;
uint16_t P2PRanging::_networkId;
unsigned P2PRanging::_numRangingsInitiationsSent;
unsigned P2PRanging::_numRangingsInitiationsReceived;
unsigned P2PRanging::_numRangingsCompletedSent;
unsigned P2PRanging::_numRangingsCompletedReceived;
unsigned P2PRanging::_numTimeouts;
unsigned P2PRanging::_numResets;
unsigned P2PRanging::_numMsgReceived;
unsigned P2PRanging::_numMsgSent;
unsigned P2PRanging::_numMsgSentInit;
unsigned _numUnexpectedMsg = 0;
uint8_t P2PRanging::_newRangingRequestId;

bool P2PRanging::_verbose;
bool P2PRanging::_haveNewResult;
ranging_report_s P2PRanging::_rangingRept;

float P2PRanging::_rangingBias;
float P2PRanging::_rangingScale;

unsigned volatile countMsgRx_MSG_RANGING_INIT = 0;
unsigned volatile countMsgRx_MSG_RANGING_REPLY1 = 0;
unsigned volatile countMsgRx_MSG_RANGING_REPLY2 = 0;
unsigned volatile countMsgRx_MSG_RANGING_REPORT = 0;
unsigned volatile countMsgRx_MSG_RANGING_FAILED = 0;

unsigned volatile countMsgTx_MSG_RANGING_INIT = 0;
unsigned volatile countMsgTx_MSG_RANGING_REPLY1 = 0;
unsigned volatile countMsgTx_MSG_RANGING_REPLY2 = 0;
unsigned volatile countMsgTx_MSG_RANGING_REPORT = 0;
unsigned volatile countMsgTx_MSG_RANGING_FAILED = 0;

volatile unsigned numIRQReceived = 0;
volatile unsigned numIRQSent = 0;
volatile unsigned numIRQErrors = 0;
volatile unsigned numIRQReceiveFailed = 0;
volatile unsigned numIRQReceiveTimeout = 0;
volatile unsigned numIRQRxTimestmpAvail = 0;

unsigned volatile countMsgRx_UNEXPECTED_REPLY1 = 0;
unsigned volatile countMsgRx_UNEXPECTED_REPLY2 = 0;
unsigned volatile countMsgRx_UNEXPECTED_REPORT = 0;
unsigned volatile countMsgRx_UNEXPECTED_FAILED = 0;
unsigned volatile countMsgRx_UNHANDLED_TYPE = 0;

unsigned volatile expected_init = 0;
unsigned volatile expected_reply1 = 0;
unsigned volatile expected_reply2 = 0;
unsigned volatile expected_report = 0;
unsigned volatile expected_UNHANDLED_TYPE = 0;

perf_counter_t pc_timeout(perf_alloc(PC_COUNT, "dw1000_timeout"));

perf_counter_t pc_time_loopRxSec(perf_alloc(PC_ELAPSED, "dw1000_loopRx"));
perf_counter_t pc_time_loopTxSec(perf_alloc(PC_ELAPSED, "dw1000_loopTx"));

//requester:
perf_counter_t pc_time_SInit_RReply1(
    perf_alloc(PC_ELAPSED, "dw1000_timeSendInitReceiveReply1"));
perf_counter_t pc_time_RReply1_SReply2(
    perf_alloc(PC_ELAPSED, "dw1000_timeReceiveReply1SendReply2"));
perf_counter_t pc_time_SReply2_RRange(
    perf_alloc(PC_ELAPSED, "dw1000_timeSendReply2ReceiveRangeRept"));
perf_counter_t pc_time_RRange_SInit(
    perf_alloc(PC_ELAPSED, "dw1000_timeReceiveRangeSendInit"));
perf_counter_t pc_time_LoopRequester(
    perf_alloc(PC_ELAPSED, "dw1000_rangingLoopTimeRequester"));

//responder
perf_counter_t pc_time_RInit_SReply1(
    perf_alloc(PC_ELAPSED, "dw1000_timeReceiveInitSendReply1"));
perf_counter_t pc_time_SReply1_RReply2(
    perf_alloc(PC_ELAPSED, "dw1000_timeSendReply1ReceiveReply2"));
perf_counter_t pc_time_RReply2_SRange(
    perf_alloc(PC_ELAPSED, "dw1000_timeReceiveReply2SendRange"));
perf_counter_t pc_time_SRange_RInit(
    perf_alloc(PC_ELAPSED, "dw1000_timeSendRangeReceiveInit"));
perf_counter_t pc_time_LoopResponder(
    perf_alloc(PC_ELAPSED, "dw1000_rangingLoopTimeResponder"));

// found these (bias, scale) based on visually inspecting the plot of true vs. measurement of range to anchors.
// TODO: can be changed later on with more accurate method.
float const RANGING_BIAS = -0.193f;  // in [m]
float const RANGING_SCALE = 1.000f;  // dimensionless

P2PRanging::P2PRanging() {
  // message sent/received state
  _haveUnhandledSentMsg = false;
  _haveUnhandledReceivedMsg = false;
  _protocolFailed = false;

  // ranging counter (per second)
  _lastActivityTime_ms = 0;

  _numRangingsInitiationsSent = 0;
  _numRangingsInitiationsReceived = 0;
  _numRangingsCompletedSent = 0;
  _numRangingsCompletedReceived = 0;
  _numTimeouts = 0;
  _numResets = 0;
  _numMsgReceived = 0;
  _numMsgSent = 0;
  _numMsgSentInit = 0;

  _verbose = false;

  _haveNewResult = false;
  _rangingRept.range = 0;
  _rangingRept.responder_id = 0;
  _rangingRept.failure = false;

  _newRangingRequestId = 0;  //Zero is a magic ID: it means no one (i.e. don't start a new conversation)

  _rangingBias = RANGING_BIAS;
  _rangingScale = RANGING_SCALE;
}

int P2PRanging::Initialize(uint8_t deviceId, uint16_t networkId) {
  _myId = deviceId;
  _networkId = networkId;
  // initialize the driver
  if (DW1000.begin()) {
    return -2;
  }
  if (DW1000.configure()) {
    return -1;
  }
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(uint16_t(_myId));
  DW1000.setNetworkId(_networkId);
//  DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_ACCURACY); just change default value in driver
  DW1000.commitConfiguration();

  DW1000.enableAllLeds();

  // attach callback for (successfully) sent and received messages
  DW1000.attachSentHandler(P2PRanging::handleSent);
  DW1000.attachReceivedHandler(P2PRanging::handleReceived);
  DW1000.attachErrorHandler(P2PRanging::handleError);
  DW1000.attachReceiveFailedHandler(P2PRanging::handleReceiveFailed);
  DW1000.attachReceiveTimeoutHandler(P2PRanging::handleReceiveTimeout);
  DW1000.attachReceiveTimestampAvailableHandler(
      P2PRanging::handleReceiveTimestampAvailable);

  DW1000.interruptOnReceived(true);
  DW1000.interruptOnSent(true);
  DW1000.interruptOnReceiveFailed(true);
  DW1000.interruptOnAutomaticAcknowledgeTrigger(true);
  DW1000.setReceiverAutoReenable(true);

//	DW1000.interruptOnReceiveFailed(true);
//	DW1000.interruptOnReceiveTimeout(true);
//  DW1000.interruptOnReceiveTimestampAvailable(false);

  DW1000.writeSystemEventMaskRegister();
  resetInactive();

  _rangingBias = RANGING_BIAS;
  _rangingScale = RANGING_SCALE;

  return 0;
}

void P2PRanging::resetInactive() {
  _numResets++;
  DW1000.idle();

  //TODO: unclear why we need the below, but this helps with the resets...
  DW1000.clearReceiveStatus();
  DW1000.clearReceiveTimestampAvailableStatus();
  DW1000.clearTransmitStatus();
  DW1000.writeSystemEventMaskRegister();

  _expectedMsg = MSG_RANGING_INIT;  //listen for this, if we don't know what else is going on.
  noteActivity();
  rangingReceiver();

}


void P2PRanging::clearSysStatus() {
  DW1000.printStatus();

  DW1000.clearReceiveStatus();
  DW1000.clearReceiveTimestampAvailableStatus();
  DW1000.clearTransmitStatus();
  resetInactive();
  DW1000.printStatus();
}

void P2PRanging::handleSent() {
  // status change on sent success
  _haveUnhandledSentMsg = true;
  numIRQSent++;
}

void P2PRanging::handleReceived() {
  // status change on received success
  _haveUnhandledReceivedMsg = true;
  numIRQReceived++;
}

void P2PRanging::handleError() {
  //TODO
  numIRQErrors++;
}

void P2PRanging::handleReceiveFailed() {
  //TODO
  numIRQReceiveFailed++;
}

void P2PRanging::handleReceiveTimeout() {
  //TODO
  numIRQReceiveTimeout++;
}

void P2PRanging::handleReceiveTimestampAvailable() {
  //TODO
  numIRQRxTimestmpAvail++;
}

void P2PRanging::transmitRangingInit() {
  //automatically cycle through the ranging partners:

  _commPartnerId = _newRangingRequestId;
  _newRangingRequestId = 0;

  DW1000.newTransmit();
  DW1000.setDefaults();
  _lastTxMsg = MSG_RANGING_INIT;
  _txData[MSG_FIELD_TYPE] = _lastTxMsg;
  _txData[MSG_FIELD_SENDER_ID] = _myId;
  _txData[MSG_FIELD_TARGET_ID] = _commPartnerId;
  // delay the same amount as ranging tag
  DW1000Time deltaTime = DW1000Time(MSG_TX_DELAY_TIME_us,
                                    DW1000Time::MICROSECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.setData(_txData, LEN_DATA);
  DW1000.startTransmit();
  _numMsgSentInit++;
  _expectedMsg = MSG_RANGING_REPLY1;
}

void P2PRanging::transmitRangingReply1() {
  /*
   mwm: why does commenting out this part make it fail more? Why does not sending reply cause the Rx to fail????
   Check if the IRQ pin is high in main loop, maybe?
   */
  DW1000.newTransmit();
  DW1000.setDefaults();
  _lastTxMsg = MSG_RANGING_REPLY1;
  _txData[MSG_FIELD_TYPE] = _lastTxMsg;
  _txData[MSG_FIELD_SENDER_ID] = _myId;
  _txData[MSG_FIELD_TARGET_ID] = _commPartnerId;
  // delay the same amount as ranging tag
  DW1000Time deltaTime = DW1000Time(MSG_TX_DELAY_TIME_us,
                                    DW1000Time::MICROSECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.setData(_txData, LEN_DATA);
  DW1000.startTransmit();
  _numMsgSentInit++;
  _expectedMsg = MSG_RANGING_REPLY2;
}

void P2PRanging::transmitRangingReply2() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  _lastTxMsg = MSG_RANGING_REPLY2;
  _txData[MSG_FIELD_TYPE] = _lastTxMsg;
  _txData[MSG_FIELD_SENDER_ID] = _myId;
  _txData[MSG_FIELD_TARGET_ID] = _commPartnerId;
  // delay sending the message and remember expected future sent timestamp
  DW1000Time deltaTime = DW1000Time(MSG_TX_DELAY_TIME_us,
                                    DW1000Time::MICROSECONDS);
  _timeRangingReply2Sent = DW1000.setDelay(deltaTime);
  _timeRangingInitSent.getTimestamp(&_txData[MSG_FIELD_DATA_START]);
  _timeRangingReply1Received.getTimestamp(&_txData[MSG_FIELD_DATA_START] + 5);
  _timeRangingReply2Sent.getTimestamp(&_txData[MSG_FIELD_DATA_START] + 10);
  DW1000.setData(_txData, LEN_DATA);
  DW1000.startTransmit();
  _numMsgSentInit++;
  _expectedMsg = MSG_RANGING_REPORT;
}

void P2PRanging::transmitRangeReport(const uint32_t curRange_um) {
  DW1000.newTransmit();
  DW1000.setDefaults();
  _lastTxMsg = MSG_RANGING_REPORT;
  _txData[MSG_FIELD_TYPE] = _lastTxMsg;
  _txData[MSG_FIELD_SENDER_ID] = _myId;
  _txData[MSG_FIELD_TARGET_ID] = _commPartnerId;
  // write final ranging result
  memcpy(&_txData[MSG_FIELD_DATA_START], &curRange_um, 4);
  // delay the same amount as ranging tag
  DW1000Time deltaTime = DW1000Time(MSG_TX_DELAY_TIME_us,
                                    DW1000Time::MICROSECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.setData(_txData, LEN_DATA);
  DW1000.startTransmit();
  _numMsgSentInit++;
  _expectedMsg = MSG_RANGING_INIT;
}

void P2PRanging::transmitRangeFailed() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  _lastTxMsg = MSG_RANGING_FAILED;
  _txData[MSG_FIELD_TYPE] = _lastTxMsg;
  _txData[MSG_FIELD_SENDER_ID] = _myId;
  _txData[MSG_FIELD_TARGET_ID] = _commPartnerId;
  DW1000.setData(_txData, LEN_DATA);
  DW1000.startTransmit();
  _numMsgSentInit++;
  //NOTE: cannot reset here, would cancel transmission resetInactive();
}

void P2PRanging::rangingReceiver() {
  DW1000.newReceive();
  DW1000.setDefaults();

  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

void P2PRanging::printMessageStr(P2PRanging::MessageTypes msg) {
  switch (msg) {
    case P2PRanging::MSG_RANGING_INIT:
      printf("MSG_RANGING_INIT");
      return;
    case P2PRanging::MSG_RANGING_REPLY1:
      printf("MSG_RANGING_REPLY1");
      return;

    case P2PRanging::MSG_RANGING_REPLY2:
      printf("MSG_RANGING_REPLY2");
      return;

    case P2PRanging::MSG_RANGING_REPORT:
      printf("MSG_RANGING_REPORT");
      return;

    case P2PRanging::MSG_RANGING_FAILED:
      printf("MSG_RANGING_FAILED");
      return;

    default:
      printf("UNKNOWN CODE = <%d>", int(msg));
  }
}

void P2PRanging::runLoop() {
  if (_haveUnhandledSentMsg) {
    //This means that the radio has successfully sent out a message
    _numMsgSent++;
    perf_begin(pc_time_loopTxSec);
    _haveUnhandledSentMsg = false;
    switch (_lastTxMsg)  //message ID
    {
      case MSG_RANGING_INIT:
        countMsgTx_MSG_RANGING_INIT++;
        DW1000.getTransmitTimestamp(_timeRangingInitSent);
        _numRangingsInitiationsSent++;
        noteActivity();
        perf_begin(pc_time_SInit_RReply1);
        perf_end(pc_time_RRange_SInit);
        perf_end(pc_time_LoopRequester);
        perf_begin(pc_time_LoopRequester);
        break;

      case MSG_RANGING_REPLY1:
        countMsgTx_MSG_RANGING_REPLY1++;
        DW1000.getTransmitTimestamp(_timeRangingReply1Sent);
        noteActivity();
        perf_end(pc_time_RInit_SReply1);
        perf_begin(pc_time_SReply1_RReply2);
        break;

      case MSG_RANGING_REPLY2:
        countMsgTx_MSG_RANGING_REPLY2++;
        DW1000.getTransmitTimestamp(_timeRangingReply2Sent);
        noteActivity();
        perf_end(pc_time_RReply1_SReply2);
        perf_begin(pc_time_SReply2_RRange);
        break;

      case MSG_RANGING_REPORT:
        countMsgTx_MSG_RANGING_REPORT++;
        _numRangingsCompletedSent++;
        perf_end(pc_time_RReply2_SRange);
        perf_begin(pc_time_SRange_RInit);
        resetInactive();
        break;

        // Don't check for MSG_RANGING_FAILED case here because never occurs

      default:
        //???
        //TODO this shouldn't happen. Should log this somehow.
        break;
    }
    perf_end(pc_time_loopTxSec);
    return;
  }

  if (_haveUnhandledReceivedMsg) {
    //This means that the radio has successfully received a message
    _numMsgReceived++;
    perf_begin(pc_time_loopRxSec);
    _haveUnhandledReceivedMsg = false;
    // get message and parse
    DW1000.getData(_rxData, LEN_DATA);
    P2PRanging::MessageTypes msgId = P2PRanging::MessageTypes(
        _rxData[MSG_FIELD_TYPE]);

    uint8_t senderId = _rxData[MSG_FIELD_SENDER_ID];
    uint8_t targetId = _rxData[MSG_FIELD_TARGET_ID];

    _lastRxMsg = msgId;
    if (targetId != _myId) {
      //not for us.
      //TODO: We should record this message anyway
      return;
    }

    if (msgId != _expectedMsg && msgId != MSG_RANGING_INIT) {
      // Received unexpected message. Starting protocol over
      // causes permanent drop in ranging rate that cannot be
      // recovered until a hard reset on the CF is done. Ignoring
      // and proceeding as if this were the expected message
      // seems to avoid this issue. So we just log here.
      if (_verbose) {
        printf("Rx unexpected message! expected <");
        printMessageStr(_expectedMsg);
        printf(">, received <");
        printMessageStr(msgId);
        printf(">\n");
      }

      switch (msgId) {
        case MSG_RANGING_REPLY1:
          countMsgRx_UNEXPECTED_REPLY1++;
          break;
        case MSG_RANGING_REPLY2:
          countMsgRx_UNEXPECTED_REPLY2++;
          break;
        case MSG_RANGING_REPORT:
          countMsgRx_UNEXPECTED_REPORT++;
          break;
        case MSG_RANGING_FAILED:
          countMsgRx_UNEXPECTED_FAILED++;
          break;
        default:
          countMsgRx_UNHANDLED_TYPE++;
          break;
      }

      switch (_expectedMsg) {
        case MSG_RANGING_INIT:
          expected_init++;
          break;
        case MSG_RANGING_REPLY1:
          expected_reply1++;
          break;
        case MSG_RANGING_REPLY2:
          expected_reply2++;
          break;
        case MSG_RANGING_REPORT:
          expected_report++;
          break;
        default:
          expected_UNHANDLED_TYPE++;
          break;
      }

      _numUnexpectedMsg++;
    }

    switch (msgId) {
      case MSG_RANGING_INIT:
        countMsgRx_MSG_RANGING_INIT++;

        // on MSG_RANGING_INIT we (re-)start, so no protocol failure
        _commPartnerId = senderId;
        _protocolFailed = false;
        DW1000.getReceiveTimestamp(_timeRangingInitReceived);
        _expectedMsg = MSG_RANGING_REPLY2;
        transmitRangingReply1();

        noteActivity();
        perf_begin(pc_time_RInit_SReply1);
        perf_end(pc_time_SRange_RInit);

        perf_end(pc_time_LoopResponder);
        perf_begin(pc_time_LoopResponder);
        _numRangingsInitiationsReceived++;
        break;
      case MSG_RANGING_REPLY1:
        countMsgRx_MSG_RANGING_REPLY1++;
        if (_commPartnerId != senderId) {
          //Something's weird, this is not from the person we were talking to.
          _protocolFailed = true;
          break;
        }

        DW1000.getReceiveTimestamp(_timeRangingReply1Received);
        _expectedMsg = MSG_RANGING_REPORT;
        transmitRangingReply2();

        noteActivity();
        perf_end(pc_time_SInit_RReply1);
        perf_begin(pc_time_RReply1_SReply2);
        break;
      case MSG_RANGING_REPLY2:
        countMsgRx_MSG_RANGING_REPLY2++;
        if (_commPartnerId != senderId) {
          //Something's weird, this is not from the person we were talking to.
          _protocolFailed = true;
          break;
        }
        DW1000.getReceiveTimestamp(_timeRangingReply2Received);
        _expectedMsg = MSG_RANGING_INIT;
        if (!_protocolFailed) {
          _timeRangingInitSent.setTimestamp(&_rxData[MSG_FIELD_DATA_START]);
          _timeRangingReply1Received.setTimestamp(
              &_rxData[MSG_FIELD_DATA_START] + 5);
          _timeRangingReply2Sent.setTimestamp(
              &_rxData[MSG_FIELD_DATA_START] + 10);
          // (re-)compute range as two-way ranging is done
          computeRangeAsymmetric();  // CHOSEN RANGING ALGORITHM
          float distance = _timeComputedRange.getAsMeters();
          uint32_t range_um = uint32_t(distance * 1000000 + 0.5f);
          transmitRangeReport(range_um);  //_timeComputedRange.getAsMicroSeconds());

          if (_verbose) {
            static float lpfDistance = 0;
            const float lpfConstant = 0.01f;
            if ((distance > 10e-3f) && (distance < 5.0f)) {  //somewhat arbitrary limits for reasonableness...
              lpfDistance = (1.0f - lpfConstant) * lpfDistance
                  + (lpfConstant) * distance;
            }

            static float avgDistance = 0;
            static int avgDistanceCount = 0;
            static int32_t rangingCountPeriod = 0;
            avgDistance += distance;
            avgDistanceCount++;
            // update sampling rate (each second)
            uint16_t successRangingCount = 0;
            successRangingCount++;
            int32_t curTime_ms = DW1000Class::getCPUTimeMillis();
            if (curTime_ms - rangingCountPeriod > 1000) {
              float samplingRate = (1000.0f * successRangingCount)
                  / (curTime_ms - rangingCountPeriod);
              rangingCountPeriod = curTime_ms;
              successRangingCount = 0;
              printf("Range: %dmm\n", int(1000.f * distance + 0.5f));
              printf("Avg. range: %dmm, over %d\n",
                     int(1000.f * avgDistance + 0.5f) / avgDistanceCount,
                     avgDistanceCount);
              printf("LPF distance: %.3fm\n", double(lpfDistance));
              printf("\t RX power: %f dDm\n", double(DW1000.getReceivePower()));
              printf("\t Sampling: %d.%03d Hz\n", int(0.5f + samplingRate),
                     int(0.5f + 1000 * samplingRate) % 1000);
            }
            avgDistance = 0;
            avgDistanceCount = 0;
          }
        } else {
          //mwm: do we ever execute this? Doesn't look like it.
          transmitRangeFailed();
        }

        noteActivity();
        perf_begin(pc_time_RReply2_SRange);
        perf_end(pc_time_SReply1_RReply2);
        break;
      case MSG_RANGING_REPORT:
        countMsgRx_MSG_RANGING_REPORT++;
        if (_commPartnerId != senderId) {
          //Something's weird, this is not from the person we were talking to.
          _protocolFailed = true;
          break;
        }
        _expectedMsg = MSG_RANGING_INIT;
        uint32_t curRange_um;
        memcpy(&curRange_um, &_rxData[MSG_FIELD_DATA_START], 4);
        if (_verbose) {
          //printf("Received MSG_RANGING_REPORT, dist = %dmm\n", int(curRange_um/1000));
          printf("%d,", int(curRange_um / 1000));
        }
        // calibrate the range:
        float calRange;
        calRange = CalibratedRange(curRange_um);
        //Register the result:
        _rangingRept.range = calRange;
        _rangingRept.responder_id = senderId;
        _rangingRept.failure = false;
        _haveNewResult = true;
        //TODO: Implement target position & covariance!

        if (_newRangingRequestId) {
          transmitRangingInit();
        }

        noteActivity();
        perf_end(pc_time_SReply2_RRange);
        perf_begin(pc_time_RRange_SInit);
        _numRangingsCompletedReceived++;
        break;

      case MSG_RANGING_FAILED:
        break;
        //Don't check for MSG_RANGING_FAILED case here because never occurs

    }

    perf_end(pc_time_loopRxSec);
    return;
  }

  if (_expectedMsg == MSG_RANGING_INIT) {
    // We are waiting for someone to talk to us.
    if (_newRangingRequestId) {
      transmitRangingInit();
      return;
    }
  }

  //communication in progress, because we're expecting a message other than init
  if (((DW1000Class::getCPUTimeMillis() - _lastActivityTime_ms) * 1000
      > PROTOCOL_TIMEOUT_us) || _protocolFailed) {
    //timeout
    //mean that comms failed!

    //can ask whether it's a failure, or nothing is happening

    /*if (ranging failed detection){
     send out uorb message that says failure happened.
     }*/

    _rangingRept.failure = true;
    _haveNewResult = true;
    _rangingRept.range = 0;
    _rangingRept.responder_id = _commPartnerId;

    perf_count(pc_timeout);
    resetInactive();
    _numTimeouts++;
    if (_newRangingRequestId) {
      transmitRangingInit();
    }
    return;
  }
}

/*
 * RANGING ALGORITHMS
 * ------------------
 * Either of the below functions can be used for range computation (see line "CHOSEN
 * RANGING ALGORITHM" in the code).
 * - Asymmetric is more computation intense but least error prone
 * - Symmetric is less computation intense but more error prone to clock drifts
 *
 * The anchors and tags of this reference example use the same reply delay times, hence
 * are capable of symmetric ranging (and of asymmetric ranging anyway).
 */

void P2PRanging::computeRangeAsymmetric() {
  // asymmetric two-way ranging (more computation intense, less error prone)
  DW1000Time round1 =
      (_timeRangingReply1Received - _timeRangingInitSent).wrap();
  DW1000Time reply1 =
      (_timeRangingReply1Sent - _timeRangingInitReceived).wrap();
  DW1000Time round2 =
      (_timeRangingReply2Received - _timeRangingReply1Sent).wrap();
  DW1000Time reply2 =
      (_timeRangingReply2Sent - _timeRangingReply1Received).wrap();
  DW1000Time tof = (round1 * round2 - reply1 * reply2)
      / (round1 + round2 + reply1 + reply2);
  // set tof timestamp
  _timeComputedRange.setTimestamp(tof);
}

void P2PRanging::computeRangeSymmetric() {
  // symmetric two-way ranging (less computation intense, more error prone on clock drift)
  DW1000Time tof = ((_timeRangingReply1Received - _timeRangingInitSent)
      - (_timeRangingReply1Sent - _timeRangingInitReceived)
      + (_timeRangingReply2Received - _timeRangingReply1Sent)
      - (_timeRangingReply2Sent - _timeRangingReply1Received)) * 0.25f;
  // set tof timestamp
  _timeComputedRange.setTimestamp(tof);
}

/*
 * END RANGING ALGORITHMS
 * ----------------------
 */

void P2PRanging::printStatus() {
  printf("\n== P2PRanging status: ==\n");
  printf("\tMy ID = %d, comm partner = %d, next ranging target = %d\n",
         int(_myId), int(_commPartnerId), int(_newRangingRequestId));
  printf("\tLast Tx message type = ");
  printMessageStr(_lastTxMsg);
  printf("\n\tLast Rx message type = ");
  printMessageStr(_lastRxMsg);
  printf("\n\t expected next message = ");
  printMessageStr(_expectedMsg);
  printf("\n");
  printf("\tNum ranging initations:  sent = %d, received = %d\n",
         int(_numRangingsInitiationsSent),
         int(_numRangingsInitiationsReceived));
  printf("\tNum ranging completions: sent = %d, received = %d\n",
         int(_numRangingsCompletedSent), int(_numRangingsCompletedReceived));
  printf("\tNum resets = %d, timeouts = %d\n", int(_numResets),
         int(_numTimeouts));
  printf("\tTotal num msg rx = %d, tx init = %d, tx complete = %d\n",
         int(_numMsgReceived), int(_numMsgSentInit), int(_numMsgSent));
  printf("\tTotal num IRQ:\n");
  printf("\t\trx              = %d\n", int(numIRQReceived));
  printf("\t\ttx              = %d\n", int(numIRQSent));
  printf("\t\terrors          = %d\n", int(numIRQErrors));
  printf("\t\treceiveFailed   = %d\n", int(numIRQReceiveFailed));
  printf("\t\treceiveTimeout  = %d\n", int(numIRQReceiveTimeout));
  printf("\t\trxTimestmpAvail = %d\n", int(numIRQRxTimestmpAvail));
  static int lastNumRangingsCompletedReceived = 0;
  static int lastNumRangingsInitiationSent = 0;
  static hrt_abstime lastTime = 0;
  int numR = _numRangingsCompletedReceived - lastNumRangingsCompletedReceived;
  int numS = _numRangingsInitiationsSent - lastNumRangingsInitiationSent;
  float dt = (hrt_absolute_time() - lastTime) * 1e-6f;
  lastNumRangingsCompletedReceived = _numRangingsCompletedReceived;
  lastNumRangingsInitiationSent = _numRangingsInitiationsSent;
  lastTime = hrt_absolute_time();
  printf(
      "\tSuccessful ranging rate since last call = %6.3fHz, over %6.3fs (inits @ %6.3fHz)\n",
      double(numR / dt), double(dt), double(numS / dt));
  printf("\tMessaging count: Tx || Rx\n");
  printf("\t\t MSG_RANGING_INIT:   %d, %d\n", countMsgTx_MSG_RANGING_INIT,
         countMsgRx_MSG_RANGING_INIT);
  printf("\t\t MSG_RANGING_REPLY1: %d, %d\n", countMsgTx_MSG_RANGING_REPLY1,
         countMsgRx_MSG_RANGING_REPLY1);
  printf("\t\t MSG_RANGING_REPLY2: %d, %d\n", countMsgTx_MSG_RANGING_REPLY2,
         countMsgRx_MSG_RANGING_REPLY2);
  printf("\t\t MSG_RANGING_REPORT: %d, %d\n", countMsgTx_MSG_RANGING_REPORT,
         countMsgRx_MSG_RANGING_REPORT);
  printf("\t\t MSG_RANGING_FAILED: %d, %d\n", countMsgTx_MSG_RANGING_FAILED,
         countMsgRx_MSG_RANGING_FAILED);
  printf("\t\t countMsgRx_UNEXPECTED_REPLY1: %d\n",
         countMsgRx_UNEXPECTED_REPLY1);
  printf("\t\t countMsgRx_UNEXPECTED_REPLY2: %d\n",
         countMsgRx_UNEXPECTED_REPLY2);
  printf("\t\t countMsgRx_UNEXPECTED_REPORT: %d\n",
         countMsgRx_UNEXPECTED_REPORT);
  printf("\t\t countMsgRx_UNEXPECTED_FAILED: %d\n",
         countMsgRx_UNEXPECTED_FAILED);
  printf("\t\t expected_init: %d\n", expected_init);
  printf("\t\t expected_reply1: %d\n", expected_reply1);
  printf("\t\t expected_reply2: %d\n", expected_reply2);
  printf("\t\t expected_report: %d\n", expected_report);

  printf("\tNum unexpected messages = %d\n", _numUnexpectedMsg);

  printf("\tTime since last activity %dms\n",
         int(DW1000Class::getCPUTimeMillis() - _lastActivityTime_ms));

  printf("----DW1000 driver status----\n");
  DW1000.printStatus();
  printf("\n\n");

}
