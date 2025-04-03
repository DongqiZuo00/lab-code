/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file DW1000.cpp
 * Arduino driver library (source file) for the Decawave DW1000 UWB transceiver IC.
 */

#include "DW1000.h"

#include <nuttx/clock.h>
#include <nuttx/spi/spi.h>

static struct spi_dev_s *spi1;

DW1000NS::DW1000Class DW1000NS::DW1000;

using namespace DW1000NS;
using namespace DW1000Constants;

/* ###########################################################################
 * #### Static member variables ##############################################
 * ######################################################################### */
// IRQ callbacks
void (*DW1000Class::_handleSent)(void) = 0;
void (*DW1000Class::_handleError)(void) = 0;
void (*DW1000Class::_handleReceived)(void) = 0;
void (*DW1000Class::_handleReceiveFailed)(void) = 0;
void (*DW1000Class::_handleReceiveTimeout)(void) = 0;
void (*DW1000Class::_handleReceiveTimestampAvailable)(void) = 0;

// registers
uint8_t DW1000Class::_syscfg[DW1000Constants::LEN_SYS_CFG];
uint8_t DW1000Class::_sysctrl[DW1000Constants::LEN_SYS_CTRL];
uint8_t DW1000Class::_sysstatus[DW1000Constants::LEN_SYS_STATUS];
uint8_t DW1000Class::_txfctrl[DW1000Constants::LEN_TX_FCTRL];
uint8_t DW1000Class::_sysmask[DW1000Constants::LEN_SYS_MASK];
uint8_t DW1000Class::_chanctrl[DW1000Constants::LEN_CHAN_CTRL];
uint8_t DW1000Class::_networkAndAddress[DW1000Constants::LEN_PANADR];

// monitoring
uint8_t DW1000Class::_vmeas3v3 = 0;
uint8_t DW1000Class::_tmeas23C = 0;

// driver internal state
uint8_t DW1000Class::_extendedFrameLength = FRAME_LENGTH_NORMAL;
uint8_t DW1000Class::_pacSize = PAC_SIZE_8;
uint8_t DW1000Class::_pulseFrequency = TX_PULSE_FREQ_16MHZ;
uint8_t DW1000Class::_dataRate = TRX_RATE_6800KBPS;
uint8_t DW1000Class::_preambleLength = TX_PREAMBLE_LEN_128;
uint8_t DW1000Class::_preambleCode = PREAMBLE_CODE_16MHZ_4;
uint8_t DW1000Class::_channel = CHANNEL_5;
DW1000Time DW1000Class::_antennaDelay;
bool DW1000Class::_smartPower = false;

bool DW1000Class::_frameCheck = true;
bool DW1000Class::_permanentReceive = false;
uint8_t DW1000Class::_deviceMode = IDLE_MODE;  // TODO replace by enum

bool DW1000Class::_debounceClockEnabled = false;

// modes of operation
// TODO use enum external, not config array
// this declaration is needed to make variables accessible while runtime from external code
constexpr uint8_t DW1000Class::MODE_LONGDATA_RANGE_LOWPOWER[];
constexpr uint8_t DW1000Class::MODE_SHORTDATA_FAST_LOWPOWER[];
constexpr uint8_t DW1000Class::MODE_LONGDATA_FAST_LOWPOWER[];
constexpr uint8_t DW1000Class::MODE_SHORTDATA_FAST_ACCURACY[];
constexpr uint8_t DW1000Class::MODE_LONGDATA_FAST_ACCURACY[];
constexpr uint8_t DW1000Class::MODE_LONGDATA_RANGE_ACCURACY[];
constexpr uint8_t DW1000Class::MODE_SHORTDATA_MID_ACCURACY[];
constexpr uint8_t DW1000Class::MODE_LONGDATA_MID_ACCURACY[];
/*
 const uint8_t DW1000Class::MODE_LONGDATA_RANGE_LOWPOWER[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_2048};
 const uint8_t DW1000Class::MODE_SHORTDATA_FAST_LOWPOWER[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_128};
 const uint8_t DW1000Class::MODE_LONGDATA_FAST_LOWPOWER[]  = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_16MHZ, TX_PREAMBLE_LEN_1024};
 const uint8_t DW1000Class::MODE_SHORTDATA_FAST_ACCURACY[] = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_128};
 const uint8_t DW1000Class::MODE_LONGDATA_FAST_ACCURACY[]  = {TRX_RATE_6800KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_1024};
 const uint8_t DW1000Class::MODE_LONGDATA_RANGE_ACCURACY[] = {TRX_RATE_110KBPS, TX_PULSE_FREQ_64MHZ, TX_PREAMBLE_LEN_2048};
 */
// range bias tables (500 MHz in [mm] and 900 MHz in [2mm] - to fit into bytes)
constexpr uint8_t DW1000Class::BIAS_500_16[];
constexpr uint8_t DW1000Class::BIAS_500_64[];
constexpr uint8_t DW1000Class::BIAS_900_16[];
constexpr uint8_t DW1000Class::BIAS_900_64[];
/*
 const uint8_t DW1000Class::BIAS_500_16[] = {198, 187, 179, 163, 143, 127, 109, 84, 59, 31, 0, 36, 65, 84, 97, 106, 110, 112};
 const uint8_t DW1000Class::BIAS_500_64[] = {110, 105, 100, 93, 82, 69, 51, 27, 0, 21, 35, 42, 49, 62, 71, 76, 81, 86};
 const uint8_t DW1000Class::BIAS_900_16[] = {137, 122, 105, 88, 69, 47, 25, 0, 21, 48, 79, 105, 127, 147, 160, 169, 178, 197};
 const uint8_t DW1000Class::BIAS_900_64[] = {147, 133, 117, 99, 75, 50, 29, 0, 24, 45, 63, 76, 87, 98, 116, 122, 132, 142};
 */
// SPI settings
//const SPISettings DW1000Class::_fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);
//const SPISettings DW1000Class::_slowSPI = SPISettings(2000000L, MSBFIRST, SPI_MODE0);
//const SPISettings* DW1000Class::_currentSPI = &_fastSPI;
#define FAST_SPI_FREQ (16000000L)
#define SLOW_SPI_FREQ (2000000L)

volatile unsigned DW1000Class::_numInterrupts = 0;

/* ###########################################################################
 * #### Init and end #######################################################
 * ######################################################################### */

////////////////////////////////////////////////////////
int DW1000Class::configure() {
  enableClock (AUTO_CLOCK);
  usleep(5000);  // delay(5);

  // reset chip (either soft or hard)
  stm32_configgpio (GPIO_EXPANSION_LPSDECK_RESET_IN);
  reset();

  // default network and node id
  writeValueToBytes(_networkAndAddress, 0xFF, LEN_PANADR);
  writeNetworkIdAndDeviceAddress();

  // default system configuration
  memset(_syscfg, 0, LEN_SYS_CFG);
  setDoubleBuffering(false);
  setInterruptPolarity(true);
  writeSystemConfigurationRegister();
  // default interrupt mask, i.e. no interrupts
  clearInterrupts();
  writeSystemEventMaskRegister();
  // load LDE micro-code
  enableClock (XTI_CLOCK);
  usleep(5000);  // delay(5);
  manageLDE();
  usleep(5000);  // delay(5);
  enableClock (PLL_CLOCK);  //mwm as done by bitcraze
  usleep(5000);  // delay(5);

  // read the temp and vbat readings from OTP that were recorded during production test
  // see 6.3.1 OTP memory map
  uint8_t buf_otp[4];
  readBytesOTP(0x008, buf_otp);  // the stored 3.3 V reading
  _vmeas3v3 = buf_otp[0];
  readBytesOTP(0x009, buf_otp);  // the stored 23C reading
  _tmeas23C = buf_otp[0];

  //test that we can communicate:
  uint8_t data[LEN_DEV_ID];
  readBytes(DEV_ID, NO_SUB, data, LEN_DEV_ID);
  if (*((uint32_t*) data) != 0xDECA0130) {
//			data[3] != 0xDE || data[2] != 0xCA){
    printf(
        "[DW1000Class::configure] Cannot read chip correctly, aborting init.\n");
    return -1;
  }

  return 0;
}

/*
 void DW1000Class::reselect(uint8_t ss) {
 _ss = ss;
 pinMode(_ss, OUTPUT);
 digitalWrite(_ss, HIGH);
 } */

int DW1000Class::begin() {  //uint32_t irq, uint32_t rst) {
  // generous initial init/wake-up-idle delay
  usleep(5000);  // delay(5);
  //------------------------------------------//
  // start SPI
  //SPI.begin();

  sched_lock();
  spi1 = stm32_spibus_initialize(PX4_SPI_BUS_EXPANSION);
  if (!spi1) {
    printf("[boot] FAILED to initialize SPI port 1\r\n");
    return -1;
  }

  spi1->ops->select(spi1, PX4_SPIDEV_EXPANSION_DW1000_DEVID, false);
  usleep(5000);
  spi1->ops->select(spi1, PX4_SPIDEV_EXPANSION_DW1000_DEVID, true);
  usleep(5000);

  // Default SPI1 to 1MHz and de-assert the known chip selects.
  spi1->ops->setfrequency(spi1, SLOW_SPI_FREQ);
  spi1->ops->setbits(spi1, 8);
  spi1->ops->setmode(spi1, SPIDEV_MODE0);
  sched_unlock();

  //------------------------------------------//
#ifndef ESP8266
  //mwm: this is to prevent conflicts, see here: https://www.arduino.cc/en/Reference/SPIusingInterrupt
  //mwmspi SPI.usingInterrupt(digitalPinToInterrupt(irq)); // not every board support this, e.g. ESP8266
#endif
  // pin and basic member setup
  _deviceMode = IDLE_MODE;
  // attach interrupt // TODO throw error if pin is not a interrupt pin
  //mwmspi attachInterrupt(digitalPinToInterrupt(_irq), DW1000Class::handleInterrupt, RISING); // todo interrupt for ESP8266

  //int stm32_gpiosetevent(, , , , , void *arg)
  stm32_gpiosetevent(GPIO_EXPANSION_LPSDECK_IRQ,  //uint32_t pinset
      true,  //bool risingedge
      false,  //bool fallingedge
      false,  //bool event
      DW1000Class::handleInterrupt,  //xcpt_t func
      NULL);  //,

  return 0;
}

void DW1000Class::manageLDE() {
  // transfer any ldo tune values
  uint8_t ldoTune[LEN_OTP_RDAT];
  readBytesOTP(0x04, ldoTune);  // TODO #define
  if (ldoTune[0] != 0) {
    // TODO tuning available, copy over to RAM: use OTP_LDO bit
  }
  // tell the chip to load the LDE microcode
  // TODO remove clock-related code (PMSC_CTRL) as handled separately
  uint8_t pmscctrl0[LEN_PMSC_CTRL0];
  uint8_t otpctrl[LEN_OTP_CTRL];
  memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
  memset(otpctrl, 0, LEN_OTP_CTRL);
  readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  readBytes(OTP_IF, OTP_CTRL_SUB, otpctrl, LEN_OTP_CTRL);
  pmscctrl0[0] = 0x01;
  pmscctrl0[1] = 0x03;
  otpctrl[0] = 0x00;
  otpctrl[1] = 0x80;
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
  writeBytes(OTP_IF, OTP_CTRL_SUB, otpctrl, 2);
  usleep(5000);  // delay(5);
  pmscctrl0[0] = 0x00;
  pmscctrl0[1] &= 0x02;
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
}

void DW1000Class::enableClock(uint8_t clock) {
  uint8_t pmscctrl0[LEN_PMSC_CTRL0];
  memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
  readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  sched_lock();
  if (clock == AUTO_CLOCK) {
    spi1->ops->setfrequency(spi1, FAST_SPI_FREQ);
    pmscctrl0[0] = AUTO_CLOCK;
    pmscctrl0[1] &= 0xFE;
  } else if (clock == XTI_CLOCK) {
    spi1->ops->setfrequency(spi1, SLOW_SPI_FREQ);
    pmscctrl0[0] &= 0xFC;
    pmscctrl0[0] |= XTI_CLOCK;
  } else if (clock == PLL_CLOCK) {
    spi1->ops->setfrequency(spi1, FAST_SPI_FREQ);
    pmscctrl0[0] &= 0xFC;
    pmscctrl0[0] |= PLL_CLOCK;
  } else {
    // TODO deliver proper warning
  }
  sched_unlock();
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 1);
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
}

void DW1000Class::enableDebounceClock() {
  uint8_t pmscctrl0[LEN_PMSC_CTRL0];
  memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
  readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  setBit(pmscctrl0, LEN_PMSC_CTRL0, GPDCE_BIT, 1);
  setBit(pmscctrl0, LEN_PMSC_CTRL0, KHZCLKEN_BIT, 1);
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  _debounceClockEnabled = true;
}

void DW1000Class::enableLedBlinking() {
  uint8_t pmscledc[LEN_PMSC_LEDC];
  memset(pmscledc, 0, LEN_PMSC_LEDC);
  readBytes(PMSC, PMSC_LEDC_SUB, pmscledc, LEN_PMSC_LEDC);
  setBit(pmscledc, LEN_PMSC_LEDC, BLNKEN, 1);
  writeBytes(PMSC, PMSC_LEDC_SUB, pmscledc, LEN_PMSC_LEDC);
}

void DW1000Class::setGPIOMode(uint8_t msgp, uint8_t mode) {
  uint8_t gpiomode[LEN_GPIO_MODE];
  memset(gpiomode, 0, LEN_GPIO_MODE);
  readBytes(GPIO_CTRL, GPIO_MODE_SUB, gpiomode, LEN_GPIO_MODE);
  for (char i = 0; i < 2; i++) {
    setBit(gpiomode, LEN_GPIO_MODE, msgp + i, (mode >> i) & 1);
  }
  writeBytes(GPIO_CTRL, GPIO_MODE_SUB, gpiomode, LEN_GPIO_MODE);
}

void DW1000Class::deepSleep() {
  uint8_t aon_wcfg[LEN_AON_WCFG];
  memset(aon_wcfg, 0, LEN_AON_WCFG);
  readBytes(AON, AON_WCFG_SUB, aon_wcfg, LEN_AON_WCFG);
  setBit(aon_wcfg, LEN_AON_WCFG, ONW_LDC_BIT, true);
  setBit(aon_wcfg, LEN_AON_WCFG, ONW_LDD0_BIT, true);
  writeBytes(AON, AON_WCFG_SUB, aon_wcfg, LEN_AON_WCFG);

  uint8_t pmsc_ctrl1[LEN_PMSC_CTRL1];
  memset(pmsc_ctrl1, 0, LEN_PMSC_CTRL1);
  readBytes(PMSC, PMSC_CTRL1_SUB, pmsc_ctrl1, LEN_PMSC_CTRL1);
  setBit(pmsc_ctrl1, LEN_PMSC_CTRL1, ATXSLP_BIT, false);
  setBit(pmsc_ctrl1, LEN_PMSC_CTRL1, ARXSLP_BIT, false);
  writeBytes(PMSC, PMSC_CTRL1_SUB, pmsc_ctrl1, LEN_PMSC_CTRL1);

  uint8_t aon_cfg0[LEN_AON_CFG0];
  memset(aon_cfg0, 0, LEN_AON_CFG0);
  readBytes(AON, AON_CFG0_SUB, aon_cfg0, LEN_AON_CFG0);
  setBit(aon_cfg0, LEN_AON_CFG0, WAKE_SPI_BIT, true);
  setBit(aon_cfg0, LEN_AON_CFG0, WAKE_PIN_BIT, true);
  setBit(aon_cfg0, LEN_AON_CFG0, WAKE_CNT_BIT, false);
  setBit(aon_cfg0, LEN_AON_CFG0, SLEEP_EN_BIT, true);
  writeBytes(AON, AON_CFG0_SUB, aon_cfg0, LEN_AON_CFG0);

  uint8_t aon_ctrl[LEN_AON_CTRL];
  memset(aon_ctrl, 0, LEN_AON_CTRL);
  readBytes(AON, AON_CTRL_SUB, aon_ctrl, LEN_AON_CTRL);
  setBit(aon_ctrl, LEN_AON_CTRL, UPL_CFG_BIT, true);
  setBit(aon_ctrl, LEN_AON_CTRL, SAVE_BIT, true);
  writeBytes(AON, AON_CTRL_SUB, aon_ctrl, LEN_AON_CTRL);
}

/*
 void DW1000Class::spiWakeup(){
 digitalWrite(_ss, LOW);
 usleep(2000);
 digitalWrite(_ss, HIGH);
 if (_debounceClockEnabled){
 DW1000Class::enableDebounceClock();
 }
 }
 */

void DW1000Class::reset() {
  if (stm32_gpioread (GPIO_EXPANSION_LPSDECK_RESET_IN)) {  //	if(_rst == 0xff) {
    softReset();
  } else {
    // dw1000 data sheet v2.08 §5.6.1 page 20, the RSTn pin should not be driven high but left floating.
    stm32_configgpio (GPIO_EXPANSION_LPSDECK_RESET_OUT);	//pinMode(_rst, OUTPUT);
    stm32_gpiowrite(GPIO_EXPANSION_LPSDECK_RESET_OUT, 0);  //digitalWrite(_rst, LOW);
    usleep(2000);  // delay(2);  // dw1000 data sheet v2.08 §5.6.1 page 20: nominal 50ns, to be safe take more time
    stm32_configgpio(GPIO_EXPANSION_LPSDECK_RESET_IN);  //pinMode(_rst, INPUT);
    usleep(10000);  // delay(10); // dwm1000 data sheet v1.2 page 5: nominal 3 ms, to be safe take more time
    // force into idle mode (although it should be already after reset)
    idle();
  }
}

void DW1000Class::softReset() {
  uint8_t pmscctrl0[LEN_PMSC_CTRL0];
  readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  pmscctrl0[0] = 0x01;
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  pmscctrl0[3] = 0x00;
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  usleep(10000);		//delay(10);
  pmscctrl0[0] = 0x00;
  pmscctrl0[3] = 0xF0;
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  // force into idle mode
  idle();
}

void DW1000Class::enableMode(const uint8_t mode[]) {
  setDataRate(mode[0]);
  setPulseFrequency(mode[1]);
  setPreambleLength(mode[2]);
  // TODO add channel and code to mode tuples
  // TODO add channel and code settings with checks (see Table 58)
//  uint8_t channel = CHANNEL_1;
  uint8_t channel = CHANNEL_5;
  setChannel(channel);  //MWM change channel

  //Table 61
  if (channel == 5) {
    if (mode[1] == TX_PULSE_FREQ_16MHZ) {
      setPreambleCode (PREAMBLE_CODE_16MHZ_4);
    } else {
      setPreambleCode (PREAMBLE_CODE_64MHZ_10);
    }
  } else if (channel == 1) {
    if (mode[1] == TX_PULSE_FREQ_16MHZ) {
      setPreambleCode (PREAMBLE_CODE_16MHZ_1);
    } else {
      setPreambleCode (PREAMBLE_CODE_64MHZ_10);
    }
  }
}

void DW1000Class::tune() {
  // these registers are going to be tuned/configured
  uint8_t agctune1[LEN_AGC_TUNE1];
  uint8_t agctune2[LEN_AGC_TUNE2];
  uint8_t agctune3[LEN_AGC_TUNE3];
  uint8_t drxtune0b[LEN_DRX_TUNE0b];
  uint8_t drxtune1a[LEN_DRX_TUNE1a];
  uint8_t drxtune1b[LEN_DRX_TUNE1b];
  uint8_t drxtune2[LEN_DRX_TUNE2];
  uint8_t drxtune4H[LEN_DRX_TUNE4H];
  uint8_t ldecfg1[LEN_LDE_CFG1];
  uint8_t ldecfg2[LEN_LDE_CFG2];
  uint8_t lderepc[LEN_LDE_REPC];
  uint8_t txpower[LEN_TX_POWER];
  uint8_t rfrxctrlh[LEN_RF_RXCTRLH];
  uint8_t rftxctrl[LEN_RF_TXCTRL];
  uint8_t tcpgdelay[LEN_TC_PGDELAY];
  uint8_t fspllcfg[LEN_FS_PLLCFG];
  uint8_t fsplltune[LEN_FS_PLLTUNE];
  uint8_t fsxtalt[LEN_FS_XTALT];
  // AGC_TUNE1
  if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
    writeValueToBytes(agctune1, 0x8870, LEN_AGC_TUNE1);
  } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
    writeValueToBytes(agctune1, 0x889B, LEN_AGC_TUNE1);
  } else {
    // TODO proper error/warning handling
  }
  // AGC_TUNE2
  writeValueToBytes(agctune2, 0x2502A907L, LEN_AGC_TUNE2);
  // AGC_TUNE3
  writeValueToBytes(agctune3, 0x0035, LEN_AGC_TUNE3);
  // DRX_TUNE0b (already optimized according to Table 20 of user manual)
  if (_dataRate == TRX_RATE_110KBPS) {
    writeValueToBytes(drxtune0b, 0x0016, LEN_DRX_TUNE0b);
  } else if (_dataRate == TRX_RATE_850KBPS) {
    writeValueToBytes(drxtune0b, 0x0006, LEN_DRX_TUNE0b);
  } else if (_dataRate == TRX_RATE_6800KBPS) {
    writeValueToBytes(drxtune0b, 0x0001, LEN_DRX_TUNE0b);
  } else {
    // TODO proper error/warning handling
  }
  // DRX_TUNE1a
  if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
    writeValueToBytes(drxtune1a, 0x0087, LEN_DRX_TUNE1a);
  } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
    writeValueToBytes(drxtune1a, 0x008D, LEN_DRX_TUNE1a);
  } else {
    // TODO proper error/warning handling
  }
  // DRX_TUNE1b
  if (_preambleLength == TX_PREAMBLE_LEN_1536
      || _preambleLength == TX_PREAMBLE_LEN_2048
      || _preambleLength == TX_PREAMBLE_LEN_4096) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(drxtune1b, 0x0064, LEN_DRX_TUNE1b);
    } else {
      // TODO proper error/warning handling
    }
  } else if (_preambleLength != TX_PREAMBLE_LEN_64) {
    if (_dataRate == TRX_RATE_850KBPS || _dataRate == TRX_RATE_6800KBPS) {
      writeValueToBytes(drxtune1b, 0x0020, LEN_DRX_TUNE1b);
    } else {
      // TODO proper error/warning handling
    }
  } else {
    if (_dataRate == TRX_RATE_6800KBPS) {
      writeValueToBytes(drxtune1b, 0x0010, LEN_DRX_TUNE1b);
    } else {
      // TODO proper error/warning handling
    }
  }
  // DRX_TUNE2
  if (_pacSize == PAC_SIZE_8) {
    if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
      writeValueToBytes(drxtune2, 0x311A002DL, LEN_DRX_TUNE2);
    } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
      writeValueToBytes(drxtune2, 0x313B006BL, LEN_DRX_TUNE2);
    } else {
      // TODO proper error/warning handling
    }
  } else if (_pacSize == PAC_SIZE_16) {
    if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
      writeValueToBytes(drxtune2, 0x331A0052L, LEN_DRX_TUNE2);
    } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
      writeValueToBytes(drxtune2, 0x333B00BEL, LEN_DRX_TUNE2);
    } else {
      // TODO proper error/warning handling
    }
  } else if (_pacSize == PAC_SIZE_32) {
    if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
      writeValueToBytes(drxtune2, 0x351A009AL, LEN_DRX_TUNE2);
    } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
      writeValueToBytes(drxtune2, 0x353B015EL, LEN_DRX_TUNE2);
    } else {
      // TODO proper error/warning handling
    }
  } else if (_pacSize == PAC_SIZE_64) {
    if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
      writeValueToBytes(drxtune2, 0x371A011DL, LEN_DRX_TUNE2);
    } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
      writeValueToBytes(drxtune2, 0x373B0296L, LEN_DRX_TUNE2);
    } else {
      // TODO proper error/warning handling
    }
  } else {
    // TODO proper error/warning handling
  }
  // DRX_TUNE4H
  if (_preambleLength == TX_PREAMBLE_LEN_64) {
    writeValueToBytes(drxtune4H, 0x0010, LEN_DRX_TUNE4H);
  } else {
    writeValueToBytes(drxtune4H, 0x0028, LEN_DRX_TUNE4H);
  }
  // RF_RXCTRLH
  if (_channel != CHANNEL_4 && _channel != CHANNEL_7) {
    writeValueToBytes(rfrxctrlh, 0xD8, LEN_RF_RXCTRLH);
  } else {
    writeValueToBytes(rfrxctrlh, 0xBC, LEN_RF_RXCTRLH);
  }
  // RX_TXCTRL
  if (_channel == CHANNEL_1) {
    writeValueToBytes(rftxctrl, 0x00005C40L, LEN_RF_TXCTRL);
  } else if (_channel == CHANNEL_2) {
    writeValueToBytes(rftxctrl, 0x00045CA0L, LEN_RF_TXCTRL);
  } else if (_channel == CHANNEL_3) {
    writeValueToBytes(rftxctrl, 0x00086CC0L, LEN_RF_TXCTRL);
  } else if (_channel == CHANNEL_4) {
    writeValueToBytes(rftxctrl, 0x00045C80L, LEN_RF_TXCTRL);
  } else if (_channel == CHANNEL_5) {
    writeValueToBytes(rftxctrl, 0x001E3FE0L, LEN_RF_TXCTRL);
  } else if (_channel == CHANNEL_7) {
    writeValueToBytes(rftxctrl, 0x001E7DE0L, LEN_RF_TXCTRL);
  } else {
    // TODO proper error/warning handling
  }
  // TC_PGDELAY
  if (_channel == CHANNEL_1) {
    writeValueToBytes(tcpgdelay, 0xC9, LEN_TC_PGDELAY);
  } else if (_channel == CHANNEL_2) {
    writeValueToBytes(tcpgdelay, 0xC2, LEN_TC_PGDELAY);
  } else if (_channel == CHANNEL_3) {
    writeValueToBytes(tcpgdelay, 0xC5, LEN_TC_PGDELAY);
  } else if (_channel == CHANNEL_4) {
    writeValueToBytes(tcpgdelay, 0x95, LEN_TC_PGDELAY);
  } else if (_channel == CHANNEL_5) {
    writeValueToBytes(tcpgdelay, 0xC0, LEN_TC_PGDELAY);
  } else if (_channel == CHANNEL_7) {
    writeValueToBytes(tcpgdelay, 0x93, LEN_TC_PGDELAY);
  } else {
    // TODO proper error/warning handling
  }
  // FS_PLLCFG and FS_PLLTUNE
  if (_channel == CHANNEL_1) {
    writeValueToBytes(fspllcfg, 0x09000407L, LEN_FS_PLLCFG);
    writeValueToBytes(fsplltune, 0x1E, LEN_FS_PLLTUNE);
  } else if (_channel == CHANNEL_2 || _channel == CHANNEL_4) {
    writeValueToBytes(fspllcfg, 0x08400508L, LEN_FS_PLLCFG);
    writeValueToBytes(fsplltune, 0x26, LEN_FS_PLLTUNE);
  } else if (_channel == CHANNEL_3) {
    writeValueToBytes(fspllcfg, 0x08401009L, LEN_FS_PLLCFG);
    writeValueToBytes(fsplltune, 0x56, LEN_FS_PLLTUNE);
  } else if (_channel == CHANNEL_5 || _channel == CHANNEL_7) {
    writeValueToBytes(fspllcfg, 0x0800041DL, LEN_FS_PLLCFG);
    writeValueToBytes(fsplltune, 0xBE, LEN_FS_PLLTUNE);
  } else {
    // TODO proper error/warning handling
  }
  // LDE_CFG1
  writeValueToBytes(ldecfg1, 0xD, LEN_LDE_CFG1);
  // LDE_CFG2
  if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
    writeValueToBytes(ldecfg2, 0x1607, LEN_LDE_CFG2);
  } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
    writeValueToBytes(ldecfg2, 0x0607, LEN_LDE_CFG2);
  } else {
    // TODO proper error/warning handling
  }
  // LDE_REPC
  if (_preambleCode == PREAMBLE_CODE_16MHZ_1
      || _preambleCode == PREAMBLE_CODE_16MHZ_2) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x5998 >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x5998, LEN_LDE_REPC);
    }
  } else if (_preambleCode == PREAMBLE_CODE_16MHZ_3
      || _preambleCode == PREAMBLE_CODE_16MHZ_8) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x51EA >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x51EA, LEN_LDE_REPC);
    }
  } else if (_preambleCode == PREAMBLE_CODE_16MHZ_4) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x428E >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x428E, LEN_LDE_REPC);
    }
  } else if (_preambleCode == PREAMBLE_CODE_16MHZ_5) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x451E >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x451E, LEN_LDE_REPC);
    }
  } else if (_preambleCode == PREAMBLE_CODE_16MHZ_6) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x2E14 >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x2E14, LEN_LDE_REPC);
    }
  } else if (_preambleCode == PREAMBLE_CODE_16MHZ_7) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x8000 >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x8000, LEN_LDE_REPC);
    }
  } else if (_preambleCode == PREAMBLE_CODE_64MHZ_9) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x28F4 >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x28F4, LEN_LDE_REPC);
    }
  } else if (_preambleCode == PREAMBLE_CODE_64MHZ_10
      || _preambleCode == PREAMBLE_CODE_64MHZ_17) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x3332 >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x3332, LEN_LDE_REPC);
    }
  } else if (_preambleCode == PREAMBLE_CODE_64MHZ_11) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x3AE0 >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x3AE0, LEN_LDE_REPC);
    }
  } else if (_preambleCode == PREAMBLE_CODE_64MHZ_12) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x3D70 >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x3D70, LEN_LDE_REPC);
    }
  } else if (_preambleCode == PREAMBLE_CODE_64MHZ_18
      || _preambleCode == PREAMBLE_CODE_64MHZ_19) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x35C2 >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x35C2, LEN_LDE_REPC);
    }
  } else if (_preambleCode == PREAMBLE_CODE_64MHZ_20) {
    if (_dataRate == TRX_RATE_110KBPS) {
      writeValueToBytes(lderepc, ((0x47AE >> 3) & 0xFFFF), LEN_LDE_REPC);
    } else {
      writeValueToBytes(lderepc, 0x47AE, LEN_LDE_REPC);
    }
  } else {
    // TODO proper error/warning handling
  }
  // TX_POWER (enabled smart transmit power control)
  if (_channel == CHANNEL_1 || _channel == CHANNEL_2) {
    if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
      if (_smartPower) {
        writeValueToBytes(txpower, 0x15355575L, LEN_TX_POWER);
      } else {
        writeValueToBytes(txpower, 0x75757575L, LEN_TX_POWER);
      }
    } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
      if (_smartPower) {
        writeValueToBytes(txpower, 0x07274767L, LEN_TX_POWER);
      } else {
        writeValueToBytes(txpower, 0x67676767L, LEN_TX_POWER);
      }
    } else {
      // TODO proper error/warning handling
    }
  } else if (_channel == CHANNEL_3) {
    if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
      if (_smartPower) {
        writeValueToBytes(txpower, 0x0F2F4F6FL, LEN_TX_POWER);
      } else {
        writeValueToBytes(txpower, 0x6F6F6F6FL, LEN_TX_POWER);
      }
    } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
      if (_smartPower) {
        writeValueToBytes(txpower, 0x2B4B6B8BL, LEN_TX_POWER);
      } else {
        writeValueToBytes(txpower, 0x8B8B8B8BL, LEN_TX_POWER);
      }
    } else {
      // TODO proper error/warning handling
    }
  } else if (_channel == CHANNEL_4) {
    if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
      if (_smartPower) {
        writeValueToBytes(txpower, 0x1F1F3F5FL, LEN_TX_POWER);
      } else {
        writeValueToBytes(txpower, 0x5F5F5F5FL, LEN_TX_POWER);
      }
    } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
      if (_smartPower) {
        writeValueToBytes(txpower, 0x3A5A7A9AL, LEN_TX_POWER);
      } else {
        writeValueToBytes(txpower, 0x9A9A9A9AL, LEN_TX_POWER);
      }
    } else {
      // TODO proper error/warning handling
    }
  } else if (_channel == CHANNEL_5) {
    if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
      if (_smartPower) {
        writeValueToBytes(txpower, 0x0E082848L, LEN_TX_POWER);
      } else {
        writeValueToBytes(txpower, 0x48484848L, LEN_TX_POWER);
      }
    } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
      if (_smartPower) {
        writeValueToBytes(txpower, 0x25456585L, LEN_TX_POWER);
      } else {
        writeValueToBytes(txpower, 0x85858585L, LEN_TX_POWER);
      }
    } else {
      // TODO proper error/warning handling
    }
  } else if (_channel == CHANNEL_7) {
    if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
      if (_smartPower) {
        writeValueToBytes(txpower, 0x32527292L, LEN_TX_POWER);
      } else {
        writeValueToBytes(txpower, 0x92929292L, LEN_TX_POWER);
      }
    } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
      if (_smartPower) {
        writeValueToBytes(txpower, 0x5171B1D1L, LEN_TX_POWER);
      } else {
        writeValueToBytes(txpower, 0xD1D1D1D1L, LEN_TX_POWER);
      }
    } else {
      // TODO proper error/warning handling
    }
  } else {
    // TODO proper error/warning handling
  }
  // Crystal calibration from OTP (if available)
  uint8_t buf_otp[4];
  readBytesOTP(0x01E, buf_otp);
  if (buf_otp[0] == 0) {
    // No trim value available from OTP, use midrange value of 0x10
    writeValueToBytes(fsxtalt, ((0x10 & 0x1F) | 0x60), LEN_FS_XTALT);
  } else {
    writeValueToBytes(fsxtalt, ((buf_otp[0] & 0x1F) | 0x60), LEN_FS_XTALT);
  }
  // write configuration back to chip
  writeBytes(AGC_TUNE, AGC_TUNE1_SUB, agctune1, LEN_AGC_TUNE1);
  writeBytes(AGC_TUNE, AGC_TUNE2_SUB, agctune2, LEN_AGC_TUNE2);
  writeBytes(AGC_TUNE, AGC_TUNE3_SUB, agctune3, LEN_AGC_TUNE3);
  writeBytes(DRX_TUNE, DRX_TUNE0b_SUB, drxtune0b, LEN_DRX_TUNE0b);
  writeBytes(DRX_TUNE, DRX_TUNE1a_SUB, drxtune1a, LEN_DRX_TUNE1a);
  writeBytes(DRX_TUNE, DRX_TUNE1b_SUB, drxtune1b, LEN_DRX_TUNE1b);
  writeBytes(DRX_TUNE, DRX_TUNE2_SUB, drxtune2, LEN_DRX_TUNE2);
  writeBytes(DRX_TUNE, DRX_TUNE4H_SUB, drxtune4H, LEN_DRX_TUNE4H);
  writeBytes(LDE_IF, LDE_CFG1_SUB, ldecfg1, LEN_LDE_CFG1);
  writeBytes(LDE_IF, LDE_CFG2_SUB, ldecfg2, LEN_LDE_CFG2);
  writeBytes(LDE_IF, LDE_REPC_SUB, lderepc, LEN_LDE_REPC);
  writeBytes(TX_POWER, NO_SUB, txpower, LEN_TX_POWER);
  writeBytes(RF_CONF, RF_RXCTRLH_SUB, rfrxctrlh, LEN_RF_RXCTRLH);
  writeBytes(RF_CONF, RF_TXCTRL_SUB, rftxctrl, LEN_RF_TXCTRL);
  writeBytes(TX_CAL, TC_PGDELAY_SUB, tcpgdelay, LEN_TC_PGDELAY);
  writeBytes(FS_CTRL, FS_PLLTUNE_SUB, fsplltune, LEN_FS_PLLTUNE);
  writeBytes(FS_CTRL, FS_PLLCFG_SUB, fspllcfg, LEN_FS_PLLCFG);
  writeBytes(FS_CTRL, FS_XTALT_SUB, fsxtalt, LEN_FS_XTALT);
}

/* ###########################################################################
 * #### Interrupt handling ###################################################
 * ######################################################################### */

int DW1000Class::handleInterrupt(int irq, FAR void *context,
    void* args) {
  // read current status and handle via callbacks
  _numInterrupts++;
  readSystemEventStatusRegister();
  if (isClockProblem() /* TODO and others */&& _handleError != 0) {
    (*_handleError)();
  }
  if (isTransmitDone() && _handleSent != 0) {
    (*_handleSent)();
    clearTransmitStatus();
  }
  if (isReceiveTimestampAvailable() && _handleReceiveTimestampAvailable != 0) {
    (*_handleReceiveTimestampAvailable)();
    clearReceiveTimestampAvailableStatus();
  }
  if (isReceiveFailed() && _handleReceiveFailed != 0) {
    (*_handleReceiveFailed)();
    clearReceiveStatus();
    if (_permanentReceive) {
      newReceive();
      startReceive();
    }
  } else if (isReceiveTimeout() && _handleReceiveTimeout != 0) {
    (*_handleReceiveTimeout)();
    clearReceiveStatus();
    if (_permanentReceive) {
      newReceive();
      startReceive();
    }
  } else if (isReceiveDone() && _handleReceived != 0) {
    (*_handleReceived)();
    clearReceiveStatus();
    if (_permanentReceive) {
      newReceive();
      startReceive();
    }
  }
  // clear all status that is left unhandled
  clearAllStatus();
  return 0;
}

uint32_t DW1000Class::getDeviceIdentifier() {
  uint32_t data;
  readBytes(DEV_ID, NO_SUB, (uint8_t*) &data, LEN_DEV_ID);
  return data;
}
/* ###########################################################################
 * #### Pretty printed device information ####################################
 * ######################################################################### */

void DW1000Class::getPrintableDeviceIdentifier(char msgBuffer[]) {
  uint8_t data[LEN_DEV_ID];
  readBytes(DEV_ID, NO_SUB, data, LEN_DEV_ID);
  sprintf(msgBuffer, "%02X - model: %d, version: %d, revision: %d",
          (uint16_t)((data[3] << 8) | data[2]), data[1], (data[0] >> 4) & 0x0F,
          data[0] & 0x0F);
}

void DW1000Class::getPrintableExtendedUniqueIdentifier(char msgBuffer[]) {
  uint8_t data[LEN_EUI];
  readBytes(EUI, NO_SUB, data, LEN_EUI);
  sprintf(msgBuffer, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X", data[7],
          data[6], data[5], data[4], data[3], data[2], data[1], data[0]);
}

void DW1000Class::getPrintableNetworkIdAndShortAddress(char msgBuffer[]) {
  uint8_t data[LEN_PANADR];
  readBytes(PANADR, NO_SUB, data, LEN_PANADR);
  sprintf(msgBuffer, "PAN: %02X, Short Address: %02X",
          (uint16_t)((data[3] << 8) | data[2]),
          (uint16_t)((data[1] << 8) | data[0]));
}

void DW1000Class::getPrintableDeviceMode(char msgBuffer[]) {
  // data not read from device! data is from class
  // TODO
  uint8_t prf;
  uint16_t plen;
  uint16_t dr;
  uint8_t ch;
  uint8_t pcode;
  if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
    prf = 16;
  } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
    prf = 64;
  } else {
    prf = 0;  // error
  }
  if (_preambleLength == TX_PREAMBLE_LEN_64) {
    plen = 64;
  } else if (_preambleLength == TX_PREAMBLE_LEN_128) {
    plen = 128;
  } else if (_preambleLength == TX_PREAMBLE_LEN_256) {
    plen = 256;
  } else if (_preambleLength == TX_PREAMBLE_LEN_512) {
    plen = 512;
  } else if (_preambleLength == TX_PREAMBLE_LEN_1024) {
    plen = 1024;
  } else if (_preambleLength == TX_PREAMBLE_LEN_1536) {
    plen = 1536;
  } else if (_preambleLength == TX_PREAMBLE_LEN_2048) {
    plen = 2048;
  } else if (_preambleLength == TX_PREAMBLE_LEN_4096) {
    plen = 4096;
  } else {
    plen = 0;  // error
  }
  if (_dataRate == TRX_RATE_110KBPS) {
    dr = 110;
  } else if (_dataRate == TRX_RATE_850KBPS) {
    dr = 850;
  } else if (_dataRate == TRX_RATE_6800KBPS) {
    dr = 6800;
  } else {
    dr = 0;  // error
  }
  ch = (uint8_t) _channel;
  pcode = (uint8_t) _preambleCode;
  sprintf(
      msgBuffer,
      "Data rate: %u kb/s, PRF: %u MHz, Preamble: %u symbols (code #%u), Channel: #%u",
      dr, prf, plen, pcode, ch);
}

/* ###########################################################################
 * #### DW1000 register read/write ###########################################
 * ######################################################################### */

void DW1000Class::readSystemConfigurationRegister() {
  readBytes(SYS_CFG, NO_SUB, _syscfg, LEN_SYS_CFG);
}

void DW1000Class::writeSystemConfigurationRegister() {
  writeBytes(SYS_CFG, NO_SUB, _syscfg, LEN_SYS_CFG);
}

void DW1000Class::readSystemEventStatusRegister() {
  readBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

void DW1000Class::readNetworkIdAndDeviceAddress() {
  readBytes(PANADR, NO_SUB, _networkAndAddress, LEN_PANADR);
}

void DW1000Class::writeNetworkIdAndDeviceAddress() {
  writeBytes(PANADR, NO_SUB, _networkAndAddress, LEN_PANADR);
}

void DW1000Class::readSystemEventMaskRegister() {
  readBytes(SYS_MASK, NO_SUB, _sysmask, LEN_SYS_MASK);
}

void DW1000Class::writeSystemEventMaskRegister() {
  writeBytes(SYS_MASK, NO_SUB, _sysmask, LEN_SYS_MASK);
}

void DW1000Class::readChannelControlRegister() {
  readBytes(CHAN_CTRL, NO_SUB, _chanctrl, LEN_CHAN_CTRL);
}

void DW1000Class::writeChannelControlRegister() {
  writeBytes(CHAN_CTRL, NO_SUB, _chanctrl, LEN_CHAN_CTRL);
}

void DW1000Class::readTransmitFrameControlRegister() {
  readBytes(TX_FCTRL, NO_SUB, _txfctrl, LEN_TX_FCTRL);
}

void DW1000Class::writeTransmitFrameControlRegister() {
  writeBytes(TX_FCTRL, NO_SUB, _txfctrl, LEN_TX_FCTRL);
}

/* ###########################################################################
 * #### DW1000 operation functions ###########################################
 * ######################################################################### */

void DW1000Class::setNetworkId(uint16_t val) {
  _networkAndAddress[2] = (uint8_t)(val & 0xFF);
  _networkAndAddress[3] = (uint8_t)((val >> 8) & 0xFF);
}

void DW1000Class::setDeviceAddress(uint16_t val) {
  _networkAndAddress[0] = (uint8_t)(val & 0xFF);
  _networkAndAddress[1] = (uint8_t)((val >> 8) & 0xFF);
}

uint8_t DW1000Class::nibbleFromChar(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  return 255;
}

void DW1000Class::convertToByte(char string[], uint8_t* bytes) {
  uint8_t eui_byte[LEN_EUI];
  // we fill it with the char array under the form of "AA:FF:1C:...."
  for (uint16_t i = 0; i < LEN_EUI; i++) {
    eui_byte[i] = (nibbleFromChar(string[i * 3]) << 4)
        + nibbleFromChar(string[i * 3 + 1]);
  }
  memcpy(bytes, eui_byte, LEN_EUI);
}

void DW1000Class::getTempAndVbat(float& temp, float& vbat) {
  // follow the procedure from section 6.4 of the User Manual
  uint8_t step1 = 0x80;
  writeBytes(RF_CONF, 0x11, &step1, 1);
  uint8_t step2 = 0x0A;
  writeBytes(RF_CONF, 0x12, &step2, 1);
  uint8_t step3 = 0x0F;
  writeBytes(RF_CONF, 0x12, &step3, 1);
  uint8_t step4 = 0x01;
  writeBytes(TX_CAL, NO_SUB, &step4, 1);
  uint8_t step5 = 0x00;
  writeBytes(TX_CAL, NO_SUB, &step5, 1);
  uint8_t sar_lvbat = 0;
  readBytes(TX_CAL, 0x03, &sar_lvbat, 1);
  uint8_t sar_ltemp = 0;
  readBytes(TX_CAL, 0x04, &sar_ltemp, 1);

  // calculate voltage and temperature
  vbat = (sar_lvbat - _vmeas3v3) / 173.0f + 3.3f;
  temp = (sar_ltemp - _tmeas23C) * 1.14f + 23.0f;
}

void DW1000Class::setEUI(char eui[]) {
  uint8_t eui_byte[LEN_EUI];
  convertToByte(eui, eui_byte);
  setEUI(eui_byte);
}

void DW1000Class::setEUI(uint8_t eui[]) {
  //we reverse the address->
  uint8_t reverseEUI[8];
  uint8_t size = 8;
  for (uint8_t i = 0; i < size; i++) {
    *(reverseEUI + i) = *(eui + size - i - 1);
  }
  writeBytes(EUI, NO_SUB, reverseEUI, LEN_EUI);
}

//Frame Filtering BIT in the SYS_CFG register
void DW1000Class::setFrameFilter(bool val) {
  setBit(_syscfg, LEN_SYS_CFG, FFEN_BIT, val);
}

void DW1000Class::setFrameFilterBehaveCoordinator(bool val) {
  setBit(_syscfg, LEN_SYS_CFG, FFBC_BIT, val);
}

void DW1000Class::setFrameFilterAllowBeacon(bool val) {
  setBit(_syscfg, LEN_SYS_CFG, FFAB_BIT, val);
}

void DW1000Class::setFrameFilterAllowData(bool val) {
  setBit(_syscfg, LEN_SYS_CFG, FFAD_BIT, val);
}

void DW1000Class::setFrameFilterAllowAcknowledgement(bool val) {
  setBit(_syscfg, LEN_SYS_CFG, FFAA_BIT, val);
}

void DW1000Class::setFrameFilterAllowMAC(bool val) {
  setBit(_syscfg, LEN_SYS_CFG, FFAM_BIT, val);
}

void DW1000Class::setFrameFilterAllowReserved(bool val) {
  setBit(_syscfg, LEN_SYS_CFG, FFAR_BIT, val);
}

void DW1000Class::setDoubleBuffering(bool val) {
  setBit(_syscfg, LEN_SYS_CFG, DIS_DRXB_BIT, !val);
}

void DW1000Class::setInterruptPolarity(bool val) {
  setBit(_syscfg, LEN_SYS_CFG, HIRQ_POL_BIT, val);
}

void DW1000Class::setReceiverAutoReenable(bool val) {
  setBit(_syscfg, LEN_SYS_CFG, RXAUTR_BIT, val);
}

void DW1000Class::interruptOnSent(bool val) {
  setBit(_sysmask, LEN_SYS_MASK, TXFRS_BIT, val);
}

void DW1000Class::interruptOnReceived(bool val) {
  setBit(_sysmask, LEN_SYS_MASK, RXDFR_BIT, val);
  setBit(_sysmask, LEN_SYS_MASK, RXFCG_BIT, val);
}

void DW1000Class::interruptOnReceiveFailed(bool val) {
  setBit(_sysmask, LEN_SYS_STATUS, LDEERR_BIT, val);
  setBit(_sysmask, LEN_SYS_STATUS, RXFCE_BIT, val);
  setBit(_sysmask, LEN_SYS_STATUS, RXPHE_BIT, val);
  setBit(_sysmask, LEN_SYS_STATUS, RXRFSL_BIT, val);
}

void DW1000Class::interruptOnReceiveTimeout(bool val) {
  setBit(_sysmask, LEN_SYS_MASK, RXRFTO_BIT, val);
}

void DW1000Class::interruptOnReceiveTimestampAvailable(bool val) {
  setBit(_sysmask, LEN_SYS_MASK, LDEDONE_BIT, val);
}

void DW1000Class::interruptOnAutomaticAcknowledgeTrigger(bool val) {
  setBit(_sysmask, LEN_SYS_MASK, AAT_BIT, val);
}

void DW1000Class::clearInterrupts() {
  memset(_sysmask, 0, LEN_SYS_MASK);
}

void DW1000Class::idle() {
  memset(_sysctrl, 0, LEN_SYS_CTRL);
  setBit(_sysctrl, LEN_SYS_CTRL, TRXOFF_BIT, true);
  _deviceMode = IDLE_MODE;
  writeBytes(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
}

void DW1000Class::newReceive() {
  idle();
  memset(_sysctrl, 0, LEN_SYS_CTRL);
  clearReceiveStatus();
  _deviceMode = RX_MODE;
}

void DW1000Class::startReceive() {
  setBit(_sysctrl, LEN_SYS_CTRL, SFCST_BIT, !_frameCheck);
  setBit(_sysctrl, LEN_SYS_CTRL, RXENAB_BIT, true);
  writeBytes(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
}

void DW1000Class::newTransmit() {
  idle();
  memset(_sysctrl, 0, LEN_SYS_CTRL);
  clearTransmitStatus();
  _deviceMode = TX_MODE;
}

void DW1000Class::startTransmit() {
  writeTransmitFrameControlRegister();
  setBit(_sysctrl, LEN_SYS_CTRL, SFCST_BIT, !_frameCheck);
  setBit(_sysctrl, LEN_SYS_CTRL, TXSTRT_BIT, true);
  writeBytes(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
  if (_permanentReceive) {
    memset(_sysctrl, 0, LEN_SYS_CTRL);
    _deviceMode = RX_MODE;
    startReceive();
  } else {
    _deviceMode = IDLE_MODE;
  }
}

void DW1000Class::newConfiguration() {
  idle();
  readNetworkIdAndDeviceAddress();
  readSystemConfigurationRegister();
  readChannelControlRegister();
  readTransmitFrameControlRegister();
  readSystemEventMaskRegister();
}

void DW1000Class::commitConfiguration() {
  // write all configurations back to device
  writeNetworkIdAndDeviceAddress();
  writeSystemConfigurationRegister();
  writeChannelControlRegister();
  writeTransmitFrameControlRegister();
  writeSystemEventMaskRegister();
  // tune according to configuration
  tune();
  // TODO clean up code + antenna delay/calibration API
  // TODO setter + check not larger two bytes integer
  uint8_t antennaDelayBytes[LEN_STAMP];
  static const float ANTENNA_DELAY_DIST_CORR = 26e-3f;  //offset in measurements [m]

  _antennaDelay.setTimestampFromDistance(
      (154.445f - ANTENNA_DELAY_DIST_CORR) / 2.0f);
  _antennaDelay.getTimestamp(antennaDelayBytes);
  writeBytes(TX_ANTD, NO_SUB, antennaDelayBytes, LEN_TX_ANTD);
  writeBytes(LDE_IF, LDE_RXANTD_SUB, antennaDelayBytes, LEN_LDE_RXANTD);
}

void DW1000Class::waitForResponse(bool val) {
  setBit(_sysctrl, LEN_SYS_CTRL, WAIT4RESP_BIT, val);
}

void DW1000Class::suppressFrameCheck(bool val) {
  _frameCheck = !val;
}

void DW1000Class::useSmartPower(bool smartPower) {
  _smartPower = smartPower;
  setBit(_syscfg, LEN_SYS_CFG, DIS_STXP_BIT, !smartPower);
}

DW1000Time DW1000Class::setDelay(const DW1000Time& delay) {
  if (_deviceMode == TX_MODE) {
    setBit(_sysctrl, LEN_SYS_CTRL, TXDLYS_BIT, true);
  } else if (_deviceMode == RX_MODE) {
    setBit(_sysctrl, LEN_SYS_CTRL, RXDLYS_BIT, true);
  } else {
    // in idle, ignore
    return DW1000Time();
  }
  uint8_t delayBytes[5];
  DW1000Time futureTime;
  getSystemTimestamp(futureTime);
  futureTime += delay;
  futureTime.getTimestamp(delayBytes);
  delayBytes[0] = 0;
  delayBytes[1] &= 0xFE;
  writeBytes(DX_TIME, NO_SUB, delayBytes, LEN_DX_TIME);
  // adjust expected time with configured antenna delay
  futureTime.setTimestamp(delayBytes);
  futureTime += _antennaDelay;
  return futureTime;
}

void DW1000Class::setDataRate(uint8_t rate) {
  rate &= 0x03;
  _txfctrl[1] &= 0x83;
  _txfctrl[1] |= (uint8_t)((rate << 5) & 0xFF);
  // special 110kbps flag
  if (rate == TRX_RATE_110KBPS) {
    setBit(_syscfg, LEN_SYS_CFG, RXM110K_BIT, true);
  } else {
    setBit(_syscfg, LEN_SYS_CFG, RXM110K_BIT, false);
  }
  // SFD mode and type (non-configurable, as in Table 21)
  if (rate == TRX_RATE_6800KBPS) {
    setBit(_chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, false);
    setBit(_chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, false);
    setBit(_chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, false);
  } else if (rate == TRX_RATE_850KBPS) {
    setBit(_chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, true);
    setBit(_chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, true);
    setBit(_chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, true);
  } else {
    setBit(_chanctrl, LEN_CHAN_CTRL, DWSFD_BIT, true);
    setBit(_chanctrl, LEN_CHAN_CTRL, TNSSFD_BIT, false);
    setBit(_chanctrl, LEN_CHAN_CTRL, RNSSFD_BIT, false);
  }
  uint8_t sfdLength;
  if (rate == TRX_RATE_6800KBPS) {
    sfdLength = 0x08;
  } else if (rate == TRX_RATE_850KBPS) {
    sfdLength = 0x10;
  } else {
    sfdLength = 0x40;
  }
  writeBytes(USR_SFD, SFD_LENGTH_SUB, &sfdLength, LEN_SFD_LENGTH);
  _dataRate = rate;
}

void DW1000Class::setPulseFrequency(uint8_t freq) {
  freq &= 0x03;
  _txfctrl[2] &= 0xFC;
  _txfctrl[2] |= (uint8_t)(freq & 0xFF);
  _chanctrl[2] &= 0xF3;
  _chanctrl[2] |= (uint8_t)((freq << 2) & 0xFF);
  _pulseFrequency = freq;
}

uint8_t DW1000Class::getPulseFrequency() {
  return _pulseFrequency;
}

void DW1000Class::setPreambleLength(uint8_t prealen) {
  prealen &= 0x0F;
  _txfctrl[2] &= 0xC3;
  _txfctrl[2] |= (uint8_t)((prealen << 2) & 0xFF);
  if (prealen == TX_PREAMBLE_LEN_64 || prealen == TX_PREAMBLE_LEN_128) {
    _pacSize = PAC_SIZE_8;
  } else if (prealen == TX_PREAMBLE_LEN_256 || prealen == TX_PREAMBLE_LEN_512) {
    _pacSize = PAC_SIZE_16;
  } else if (prealen == TX_PREAMBLE_LEN_1024) {
    _pacSize = PAC_SIZE_32;
  } else {
    _pacSize = PAC_SIZE_64;
  }
  _preambleLength = prealen;
}

void DW1000Class::useExtendedFrameLength(bool val) {
  _extendedFrameLength = (val ? FRAME_LENGTH_EXTENDED : FRAME_LENGTH_NORMAL);
  _syscfg[2] &= 0xFC;
  _syscfg[2] |= _extendedFrameLength;
}

void DW1000Class::receivePermanently(bool val) {
  _permanentReceive = val;
  if (val) {
    // in case permanent, also reenable receiver once failed
    setReceiverAutoReenable(true);
    writeSystemConfigurationRegister();
  }
}

void DW1000Class::setChannel(uint8_t channel) {
  channel &= 0xF;
  _chanctrl[0] = ((channel | (channel << 4)) & 0xFF);
  _channel = channel;
}

void DW1000Class::setPreambleCode(uint8_t preacode) {
  preacode &= 0x1F;
  _chanctrl[2] &= 0x3F;
  _chanctrl[2] |= ((preacode << 6) & 0xFF);
  _chanctrl[3] = 0x00;
  _chanctrl[3] = ((((preacode >> 2) & 0x07) | (preacode << 3)) & 0xFF);
  _preambleCode = preacode;
}

void DW1000Class::setDefaults() {
  if (_deviceMode == TX_MODE) {

  } else if (_deviceMode == RX_MODE) {

  } else if (_deviceMode == IDLE_MODE) {
    useExtendedFrameLength(false);
    useSmartPower(false);
    suppressFrameCheck(false);
    //for global frame filtering
    setFrameFilter(false);
    /* old defaults with active frame filter - better set filter in every script where you really need it
     setFrameFilter(true);
     //for data frame (poll, poll_ack, range, range report, range failed) filtering
     setFrameFilterAllowData(true);
     //for reserved (blink) frame filtering
     setFrameFilterAllowReserved(true);
     //setFrameFilterAllowMAC(true);
     //setFrameFilterAllowBeacon(true);
     //setFrameFilterAllowAcknowledgement(true);
     */
    interruptOnSent(true);
    interruptOnReceived(true);
    interruptOnReceiveFailed(true);
    interruptOnReceiveTimestampAvailable(false);
    interruptOnAutomaticAcknowledgeTrigger(true);
    setReceiverAutoReenable(true);
    // default mode when powering up the chip
    // still explicitly selected for later tuning
    enableMode (MODE_LONGDATA_RANGE_ACCURACY);
  }
}

void DW1000Class::setData(uint8_t data[], uint16_t n) {
  if (_frameCheck) {
    n += 2;  // two bytes CRC-16
  }
  if (n > LEN_EXT_UWB_FRAMES) {
    return;  // TODO proper error handling: frame/buffer size
  }
  if (n > LEN_UWB_FRAMES && !_extendedFrameLength) {
    return;  // TODO proper error handling: frame/buffer size
  }
  // transmit data and length
  writeBytes(TX_BUFFER, NO_SUB, data, n);
  _txfctrl[0] = (uint8_t)(n & 0xFF);  // 1 uint8_t (regular length + 1 bit)
  _txfctrl[1] &= 0xE0;
  _txfctrl[1] |= (uint8_t)((n >> 8) & 0x03);  // 2 added bits if extended length
}

/*mwm no string
 void DW1000Class::setData(const String& data) {
 uint16_t n = data.length()+1;
 uint8_t* dataBytes = (uint8_t*)malloc(n);
 data.getBytes(dataBytes, n);
 setData(dataBytes, n);
 free(dataBytes);
 }
 */

// TODO reorder
uint16_t DW1000Class::getDataLength() {
  uint16_t len = 0;
  if (_deviceMode == TX_MODE) {
    // 10 bits of TX frame control register
    len = ((((uint16_t) _txfctrl[1] << 8) | (uint16_t) _txfctrl[0]) & 0x03FF);
  } else if (_deviceMode == RX_MODE) {
    // 10 bits of RX frame control register
    uint8_t rxFrameInfo[LEN_RX_FINFO];
    readBytes(RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
    len = ((((uint16_t) rxFrameInfo[1] << 8) | (uint16_t) rxFrameInfo[0])
        & 0x03FF);
  }
  if (_frameCheck && len > 2) {
    return len - 2;
  }
  return len;
}

void DW1000Class::getData(uint8_t data[], uint16_t n) {
  if (n <= 0) {
    return;
  }
  readBytes(RX_BUFFER, NO_SUB, data, n);
}

/*mwm no string void DW1000Class::getData(String& data) {
 uint16_t i;
 uint16_t n = getDataLength(); // number of bytes w/o the two FCS ones
 if(n <= 0) { // TODO
 return;
 }
 uint8_t* dataBytes = (uint8_t*)malloc(n);
 getData(dataBytes, n);
 // clear string
 data.remove(0);
 data  = "";
 // append to string
 for(i = 0; i < n; i++) {
 data += (char)dataBytes[i];
 }
 free(dataBytes);
 }
 */

void DW1000Class::getTransmitTimestamp(DW1000Time& time) {
  uint8_t txTimeBytes[LEN_TX_STAMP];
  readBytes(TX_TIME, TX_STAMP_SUB, txTimeBytes, LEN_TX_STAMP);
  time.setTimestamp(txTimeBytes);
}

void DW1000Class::getReceiveTimestamp(DW1000Time& time) {
  uint8_t rxTimeBytes[LEN_RX_STAMP];
  readBytes(RX_TIME, RX_STAMP_SUB, rxTimeBytes, LEN_RX_STAMP);
  time.setTimestamp(rxTimeBytes);
  // correct timestamp (i.e. consider range bias)
  correctTimestamp(time);
}

// TODO check function, different type violations between uint8_t and int
void DW1000Class::correctTimestamp(DW1000Time& timestamp) {
  // base line dBm, which is -61, 2 dBm steps, total 18 data points (down to -95 dBm)
  float rxPowerBase = -(getReceivePower() + 61.0f) * 0.5f;
  int16_t rxPowerBaseLow = (int16_t) rxPowerBase;  // TODO check type
  int16_t rxPowerBaseHigh = rxPowerBaseLow + 1;  // TODO check type
  if (rxPowerBaseLow <= 0) {
    rxPowerBaseLow = 0;
    rxPowerBaseHigh = 0;
  } else if (rxPowerBaseHigh >= 17) {
    rxPowerBaseLow = 17;
    rxPowerBaseHigh = 17;
  }
  // select range low/high values from corresponding table
  int16_t rangeBiasHigh;
  int16_t rangeBiasLow;
  if (_channel == CHANNEL_4 || _channel == CHANNEL_7) {
    // 900 MHz receiver bandwidth
    if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
      rangeBiasHigh = (
          rxPowerBaseHigh < BIAS_900_16_ZERO ?
              -BIAS_900_16[rxPowerBaseHigh] : BIAS_900_16[rxPowerBaseHigh]);
      rangeBiasHigh <<= 1;
      rangeBiasLow = (
          rxPowerBaseLow < BIAS_900_16_ZERO ?
              -BIAS_900_16[rxPowerBaseLow] : BIAS_900_16[rxPowerBaseLow]);
      rangeBiasLow <<= 1;
    } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
      rangeBiasHigh = (
          rxPowerBaseHigh < BIAS_900_64_ZERO ?
              -BIAS_900_64[rxPowerBaseHigh] : BIAS_900_64[rxPowerBaseHigh]);
      rangeBiasHigh <<= 1;
      rangeBiasLow = (
          rxPowerBaseLow < BIAS_900_64_ZERO ?
              -BIAS_900_64[rxPowerBaseLow] : BIAS_900_64[rxPowerBaseLow]);
      rangeBiasLow <<= 1;
    } else {
      // TODO proper error handling
      return;
    }
  } else {
    // 500 MHz receiver bandwidth
    if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
      rangeBiasHigh = (
          rxPowerBaseHigh < BIAS_500_16_ZERO ?
              -BIAS_500_16[rxPowerBaseHigh] : BIAS_500_16[rxPowerBaseHigh]);
      rangeBiasLow = (
          rxPowerBaseLow < BIAS_500_16_ZERO ?
              -BIAS_500_16[rxPowerBaseLow] : BIAS_500_16[rxPowerBaseLow]);
    } else if (_pulseFrequency == TX_PULSE_FREQ_64MHZ) {
      rangeBiasHigh = (
          rxPowerBaseHigh < BIAS_500_64_ZERO ?
              -BIAS_500_64[rxPowerBaseHigh] : BIAS_500_64[rxPowerBaseHigh]);
      rangeBiasLow = (
          rxPowerBaseLow < BIAS_500_64_ZERO ?
              -BIAS_500_64[rxPowerBaseLow] : BIAS_500_64[rxPowerBaseLow]);
    } else {
      // TODO proper error handling
      return;
    }
  }
  // linear interpolation of bias values
  float rangeBias = rangeBiasLow
      + (rxPowerBase - rxPowerBaseLow) * (rangeBiasHigh - rangeBiasLow);
  // range bias [mm] to timestamp modification value conversion
  DW1000Time adjustmentTime;
  adjustmentTime.setTimestamp(
      (int16_t)(rangeBias * DW1000Time::DISTANCE_OF_RADIO_INV * 0.001f));
  // apply correction
  timestamp += adjustmentTime;
}

void DW1000Class::getSystemTimestamp(DW1000Time& time) {
  uint8_t sysTimeBytes[LEN_SYS_TIME];
  readBytes(SYS_TIME, NO_SUB, sysTimeBytes, LEN_SYS_TIME);
  time.setTimestamp(sysTimeBytes);
}

void DW1000Class::getTransmitTimestamp(uint8_t data[]) {
  readBytes(TX_TIME, TX_STAMP_SUB, data, LEN_TX_STAMP);
}

void DW1000Class::getReceiveTimestamp(uint8_t data[]) {
  readBytes(RX_TIME, RX_STAMP_SUB, data, LEN_RX_STAMP);
}

void DW1000Class::getSystemTimestamp(uint8_t data[]) {
  readBytes(SYS_TIME, NO_SUB, data, LEN_SYS_TIME);
}

bool DW1000Class::isTransmitDone() {
  return getBit(_sysstatus, LEN_SYS_STATUS, TXFRS_BIT);
}

bool DW1000Class::isReceiveTimestampAvailable() {
  return getBit(_sysstatus, LEN_SYS_STATUS, LDEDONE_BIT);
}

bool DW1000Class::isReceiveDone() {
  if (_frameCheck) {
    return getBit(_sysstatus, LEN_SYS_STATUS, RXFCG_BIT);
  }
  return getBit(_sysstatus, LEN_SYS_STATUS, RXDFR_BIT);
}

bool DW1000Class::isReceiveFailed() {
  bool ldeErr, rxCRCErr, rxHeaderErr, rxDecodeErr;
  ldeErr = getBit(_sysstatus, LEN_SYS_STATUS, LDEERR_BIT);
  rxCRCErr = getBit(_sysstatus, LEN_SYS_STATUS, RXFCE_BIT);
  rxHeaderErr = getBit(_sysstatus, LEN_SYS_STATUS, RXPHE_BIT);
  rxDecodeErr = getBit(_sysstatus, LEN_SYS_STATUS, RXRFSL_BIT);
  if (ldeErr || rxCRCErr || rxHeaderErr || rxDecodeErr) {
    return true;
  }
  return false;
}

//Checks to see any of the three timeout bits in sysstatus are high (RXRFTO (Frame Wait timeout), RXPTO (Preamble timeout), RXSFDTO (Start frame delimiter(?) timeout).
bool DW1000Class::isReceiveTimeout() {
  return (getBit(_sysstatus, LEN_SYS_STATUS, RXRFTO_BIT)
      | getBit(_sysstatus, LEN_SYS_STATUS, RXPTO_BIT)
      | getBit(_sysstatus, LEN_SYS_STATUS, RXSFDTO_BIT));
}

bool DW1000Class::isClockProblem() {
  bool clkllErr, rfllErr;
  clkllErr = getBit(_sysstatus, LEN_SYS_STATUS, CLKPLL_LL_BIT);
  rfllErr = getBit(_sysstatus, LEN_SYS_STATUS, RFPLL_LL_BIT);
  if (clkllErr || rfllErr) {
    return true;
  }
  return false;
}

void DW1000Class::clearAllStatus() {
  //Latched bits in status register are reset by writing 1 to them
  memset(_sysstatus, 0xff, LEN_SYS_STATUS);
  writeBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

void DW1000Class::clearReceiveTimestampAvailableStatus() {
  setBit(_sysstatus, LEN_SYS_STATUS, LDEDONE_BIT, true);
  writeBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

void DW1000Class::clearReceiveStatus() {
  // clear latched RX bits (i.e. write 1 to clear)
  setBit(_sysstatus, LEN_SYS_STATUS, RXDFR_BIT, true);
  setBit(_sysstatus, LEN_SYS_STATUS, LDEDONE_BIT, true);
  setBit(_sysstatus, LEN_SYS_STATUS, LDEERR_BIT, true);
  setBit(_sysstatus, LEN_SYS_STATUS, RXPHE_BIT, true);
  setBit(_sysstatus, LEN_SYS_STATUS, RXFCE_BIT, true);
  setBit(_sysstatus, LEN_SYS_STATUS, RXFCG_BIT, true);
  setBit(_sysstatus, LEN_SYS_STATUS, RXRFSL_BIT, true);
  writeBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

void DW1000Class::clearTransmitStatus() {
  // clear latched TX bits
  setBit(_sysstatus, LEN_SYS_STATUS, TXFRB_BIT, true);
  setBit(_sysstatus, LEN_SYS_STATUS, TXPRS_BIT, true);
  setBit(_sysstatus, LEN_SYS_STATUS, TXPHS_BIT, true);
  setBit(_sysstatus, LEN_SYS_STATUS, TXFRS_BIT, true);
  writeBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

float DW1000Class::getReceiveQuality() {
  uint8_t noiseBytes[LEN_STD_NOISE];
  uint8_t fpAmpl2Bytes[LEN_FP_AMPL2];
  uint16_t noise, f2;
  readBytes(RX_FQUAL, STD_NOISE_SUB, noiseBytes, LEN_STD_NOISE);
  readBytes(RX_FQUAL, FP_AMPL2_SUB, fpAmpl2Bytes, LEN_FP_AMPL2);
  noise = (uint16_t) noiseBytes[0] | ((uint16_t) noiseBytes[1] << 8);
  f2 = (uint16_t) fpAmpl2Bytes[0] | ((uint16_t) fpAmpl2Bytes[1] << 8);
  return (float) f2 / noise;
}

float DW1000Class::getFirstPathPower() {
  uint8_t fpAmpl1Bytes[LEN_FP_AMPL1];
  uint8_t fpAmpl2Bytes[LEN_FP_AMPL2];
  uint8_t fpAmpl3Bytes[LEN_FP_AMPL3];
  uint8_t rxFrameInfo[LEN_RX_FINFO];
  uint16_t f1, f2, f3, N;
  float A, corrFac;
  readBytes(RX_TIME, FP_AMPL1_SUB, fpAmpl1Bytes, LEN_FP_AMPL1);
  readBytes(RX_FQUAL, FP_AMPL2_SUB, fpAmpl2Bytes, LEN_FP_AMPL2);
  readBytes(RX_FQUAL, FP_AMPL3_SUB, fpAmpl3Bytes, LEN_FP_AMPL3);
  readBytes(RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
  f1 = (uint16_t) fpAmpl1Bytes[0] | ((uint16_t) fpAmpl1Bytes[1] << 8);
  f2 = (uint16_t) fpAmpl2Bytes[0] | ((uint16_t) fpAmpl2Bytes[1] << 8);
  f3 = (uint16_t) fpAmpl3Bytes[0] | ((uint16_t) fpAmpl3Bytes[1] << 8);
  N = (((uint16_t) rxFrameInfo[2] >> 4) & 0xFF)
      | ((uint16_t) rxFrameInfo[3] << 4);
  if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
    A = 113.77;
    corrFac = 2.3334;
  } else {
    A = 121.74;
    corrFac = 1.1667;
  }
  float estFpPwr = 10.0f
      * log10f(
          ((float) f1 * (float) f1 + (float) f2 * (float) f2
              + (float) f3 * (float) f3) / ((float) N * (float) N)) - A;
  if (estFpPwr <= -88) {
    return estFpPwr;
  } else {
    // approximation of Fig. 22 in user manual for dbm correction
    estFpPwr += (estFpPwr + 88) * corrFac;
  }
  return estFpPwr;
}

float DW1000Class::getReceivePower() {
  uint8_t cirPwrBytes[LEN_CIR_PWR];
  uint8_t rxFrameInfo[LEN_RX_FINFO];
  uint32_t twoPower17 = 131072;
  uint16_t C, N;
  float A, corrFac;
  readBytes(RX_FQUAL, CIR_PWR_SUB, cirPwrBytes, LEN_CIR_PWR);
  readBytes(RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
  C = (uint16_t) cirPwrBytes[0] | ((uint16_t) cirPwrBytes[1] << 8);
  N = (((uint16_t) rxFrameInfo[2] >> 4) & 0xFF)
      | ((uint16_t) rxFrameInfo[3] << 4);
  if (_pulseFrequency == TX_PULSE_FREQ_16MHZ) {
    A = 113.77;
    corrFac = 2.3334;
  } else {
    A = 121.74;
    corrFac = 1.1667;
  }
  float estRxPwr = 10.0f
      * log10f(((float) C * (float) twoPower17) / ((float) N * (float) N)) - A;
  if (estRxPwr <= -88) {
    return estRxPwr;
  } else {
    // approximation of Fig. 22 in user manual for dbm correction
    estRxPwr += (estRxPwr + 88) * corrFac;
  }
  return estRxPwr;
}

/* ###########################################################################
 * #### Helper functions #####################################################
 * ######################################################################### */

/*
 * Set the value of a bit in an array of bytes that are considered
 * consecutive and stored from MSB to LSB.
 * @param data
 * 		The number as uint8_t array.
 * @param n
 * 		The number of bytes in the array.
 * @param bit
 * 		The position of the bit to be set.
 * @param val
 *		The bool value to be set to the given bit position.
 */
void DW1000Class::setBit(uint8_t data[], uint16_t n, uint16_t bit, bool val) {
  uint16_t idx;
  uint8_t shift;

  idx = bit / 8;
  if (idx >= n) {
    return;  // TODO proper error handling: out of bounds
  }
  uint8_t* targetByte = &data[idx];
  shift = bit % 8;
  if (val) {
    *targetByte |= 1 << shift;  //bitSet(*targetByte, shift);
  } else {
    *targetByte &= ~(1 << shift);  //bitClear(*targetByte, shift);
  }
}

/*
 * Check the value of a bit in an array of bytes that are considered
 * consecutive and stored from MSB to LSB.
 * @param data
 * 		The number as uint8_t array.
 * @param n
 * 		The number of bytes in the array.
 * @param bit
 * 		The position of the bit to be checked.
 */
bool DW1000Class::getBit(uint8_t data[], uint16_t n, uint16_t bit) {
  uint16_t idx;
  uint8_t shift;

  idx = bit / 8;
  if (idx >= n) {
    return false;  // TODO proper error handling: out of bounds
  }
  uint8_t targetByte = data[idx];
  shift = bit % 8;

  return (targetByte >> shift) & 1;  //bitRead(targetByte, shift); // TODO wrong type returned uint8_t instead of bool
}

void DW1000Class::writeValueToBytes(uint8_t data[], int32_t val, uint16_t n) {
  uint16_t i;
  for (i = 0; i < n; i++) {
    data[i] = ((val >> (i * 8)) & 0xFF);  // TODO bad types - signed unsigned problem
  }
}
/*
 * Read bytes from the DW1000. Number of bytes depend on register length.
 * @param cmd
 * 		The register address (see Chapter 7 in the DW1000 user manual).
 * @param data
 *		The data array to be read into.
 * @param n
 *		The number of bytes expected to be received.
 */
// TODO incomplete doc
void DW1000Class::readBytes(uint8_t cmd, uint16_t offset, uint8_t data[],
                            uint16_t n) {
  uint8_t header[3];
  uint8_t headerLen = 1;

  // build SPI header
  if (offset == NO_SUB) {
    header[0] = READ | cmd;
  } else {
    header[0] = READ_SUB | cmd;
    if (offset < 128) {
      header[1] = (uint8_t) offset;
      headerLen++;
    } else {
      header[1] = RW_SUB_EXT | (uint8_t) offset;
      header[2] = (uint8_t)(offset >> 7);
      headerLen += 2;
    }
  }

  static uint8_t spiTxBuffer[196];
  static uint8_t spiRxBuffer[196];

  sched_lock();
  memcpy(spiTxBuffer, header, headerLen);
  memset(spiTxBuffer + headerLen, 0, n);
  spi1->ops->select(spi1, PX4_SPIDEV_EXPANSION_DW1000_DEVID, true);
  spi1->ops->exchange(spi1, spiTxBuffer, spiRxBuffer, headerLen + n);
  spi1->ops->select(spi1, PX4_SPIDEV_EXPANSION_DW1000_DEVID, false);
  memcpy(data, spiRxBuffer + headerLen, n);
  sched_unlock();
}

// always 4 bytes
// TODO why always 4 bytes? can be different, see p. 58 table 10 otp memory map
void DW1000Class::readBytesOTP(uint16_t address, uint8_t data[]) {
  uint8_t addressBytes[LEN_OTP_ADDR];

  // p60 - 6.3.3 Reading a value from OTP memory
  // bytes of address
  addressBytes[0] = (address & 0xFF);
  addressBytes[1] = ((address >> 8) & 0xFF);
  // set address
  writeBytes(OTP_IF, OTP_ADDR_SUB, addressBytes, LEN_OTP_ADDR);
  // switch into read mode
  writeByte(OTP_IF, OTP_CTRL_SUB, 0x03);  // OTPRDEN | OTPREAD
  writeByte(OTP_IF, OTP_CTRL_SUB, 0x01);  // OTPRDEN
  // read value/block - 4 bytes
  readBytes(OTP_IF, OTP_RDAT_SUB, data, LEN_OTP_RDAT);
  // end read mode
  writeByte(OTP_IF, OTP_CTRL_SUB, 0x00);
}

// Helper to set a single register
void DW1000Class::writeByte(uint8_t cmd, uint16_t offset, uint8_t data) {
  writeBytes(cmd, offset, &data, 1);
}

/*
 * Write bytes to the DW1000. Single bytes can be written to registers via sub-addressing.
 * @param cmd
 * 		The register address (see Chapter 7 in the DW1000 user manual).
 * @param offset
 *		The offset to select register sub-parts for writing, or 0x00 to disable
 * 		sub-adressing.
 * @param data
 *		The data array to be written.
 * @param data_size
 *		The number of bytes to be written (take care not to go out of bounds of
 * 		the register).
 */
// TODO offset really bigger than uint8_t?
void DW1000Class::writeBytes(uint8_t cmd, uint16_t offset, uint8_t data[],
                             uint16_t data_size) {
  uint8_t header[3];
  uint8_t headerLen = 1;

  // TODO proper error handling: address out of bounds
  // build SPI header
  if (offset == NO_SUB) {
    header[0] = WRITE | cmd;
  } else {
    header[0] = WRITE_SUB | cmd;
    if (offset < 128) {
      header[1] = (uint8_t) offset;
      headerLen++;
    } else {
      header[1] = RW_SUB_EXT | (uint8_t) offset;
      header[2] = (uint8_t)(offset >> 7);
      headerLen += 2;
    }
  }

  static uint8_t spiTxBuffer[196];
  static uint8_t spiRxBuffer[196];

  sched_lock();
  memcpy(spiTxBuffer, header, headerLen);
  memcpy(spiTxBuffer + headerLen, data, data_size);
  spi1->ops->select(spi1, PX4_SPIDEV_EXPANSION_DW1000_DEVID, true);
  spi1->ops->exchange(spi1, spiTxBuffer, spiRxBuffer, headerLen + data_size);
  spi1->ops->select(spi1, PX4_SPIDEV_EXPANSION_DW1000_DEVID, false);
  sched_unlock();
}

void DW1000Class::getPrettyBytes(uint8_t data[], char msgBuffer[], uint16_t n) {
  uint16_t i, j, b;
  b = sprintf(msgBuffer, "Data, bytes: %d\nB: 7 6 5 4 3 2 1 0\n", n);  // TODO - type
  for (i = 0; i < n; i++) {
    uint8_t curByte = data[i];
    snprintf(&msgBuffer[b++], 2, "%d", (i + 1));
    msgBuffer[b++] = (char) ((i + 1) & 0xFF);
    msgBuffer[b++] = ':';
    msgBuffer[b++] = ' ';
    for (j = 0; j < 8; j++) {
      msgBuffer[b++] = ((curByte >> (7 - j)) & 0x01) ? '1' : '0';
      if (j < 7) {
        msgBuffer[b++] = ' ';
      } else if (i < n - 1) {
        msgBuffer[b++] = '\n';
      } else {
        msgBuffer[b++] = '\0';
      }
    }
  }
  msgBuffer[b++] = '\0';
}

void DW1000Class::getPrettyBytes(uint8_t cmd, uint16_t offset, char msgBuffer[],
                                 uint16_t n) {
  uint16_t i, j, b;
  uint8_t* readBuf = (uint8_t*) malloc(n);
  readBytes(cmd, offset, readBuf, n);
  b = sprintf(msgBuffer, "Reg: 0x%02x, bytes: %d\nB: 7 6 5 4 3 2 1 0\n", cmd,
              n);  // TODO - tpye
  for (i = 0; i < n; i++) {
    uint8_t curByte = readBuf[i];
    snprintf(&msgBuffer[b++], 2, "%d", (i + 1));
    msgBuffer[b++] = (char) ((i + 1) & 0xFF);
    msgBuffer[b++] = ':';
    msgBuffer[b++] = ' ';
    for (j = 0; j < 8; j++) {
      msgBuffer[b++] = ((curByte >> (7 - j)) & 0x01) ? '1' : '0';
      if (j < 7) {
        msgBuffer[b++] = ' ';
      } else if (i < n - 1) {
        msgBuffer[b++] = '\n';
      } else {
        msgBuffer[b++] = '\0';
      }
    }
  }
  msgBuffer[b++] = '\0';
  free(readBuf);
}

void DW1000Class::enableAllLeds(void) {
  //copied from Bticraze implementation

  // Set all 4 GPIO in LED mode
  uint8_t dat[4];
  readBytes(GPIO_CTRL, GPIO_MODE_SUB, dat, 4);
//  reg = dwSpiRead32(dev, GPIO_CTRL, GPIO_MODE_SUB);
  dat[3] &= ~0x00;
  dat[2] &= ~0x00;
  dat[1] &= ~0x3F;
  dat[0] &= ~0xC0;
//  reg &= ~0x00 00 3F C0ul;
  dat[3] |= 0x00;
  dat[2] |= 0x00;
  dat[1] |= 0x15;
  dat[0] |= 0x40;
//  reg |= 0x00 00 15 40ul;
  writeBytes(GPIO_CTRL, GPIO_MODE_SUB, dat, 4);
//  dwSpiWrite32(dev, GPIO_CTRL, GPIO_MODE_SUB, reg);

  // Enable debounce clock (used to clock the LED blinking)
  readBytes(PMSC, PMSC_CTRL0_SUB, dat, 4);
//  reg = dwSpiRead32(dev, PMSC, PMSC_CTRL0_SUB);
  dat[3] |= 0x00;
  dat[2] |= 0x84;
  dat[1] |= 0x00;
  dat[0] |= 0x00;
  //reg |= 0x00 84 00 00ul;
  writeBytes(PMSC, PMSC_CTRL0_SUB, dat, 4);
//  dwSpiWrite32(dev, PMSC, PMSC_CTRL0_SUB, reg);

  // Enable LED blinking and set the rate
  dat[3] = 0x00;
  dat[2] = 0x00;
  dat[1] = 0x01;  //blink enable
  dat[0] = 0x10;  //blink time count
//  reg = 0x00 00 01 10ul;
  writeBytes(PMSC, PMSC_LEDC_SUB, dat, 4);
//  dwSpiWrite32(dev, PMSC, PMSC_LEDC_SUB, reg);

  // Trigger a manual blink of the LEDs for test
  dat[3] |= 0x00;
  dat[2] |= 0x0f;
  dat[1] |= 0x00;
  dat[0] |= 0x00;
//  reg |= 0x00 0f 00 00ul;
  writeBytes(PMSC, PMSC_LEDC_SUB, dat, 4);
//  dwSpiWrite32(dev, PMSC, PMSC_LEDC_SUB, reg);
  dat[3] &= ~0x00;
  dat[2] &= ~0x0f;
  dat[1] &= ~0x00;
  dat[0] &= ~0x00;
//  reg &= ~0x00 0f 00 00ul;
//  dwSpiWrite32(dev, PMSC, PMSC_LEDC_SUB, reg);
  writeBytes(PMSC, PMSC_LEDC_SUB, dat, 4);
}

void DW1000Class::printStatus() {
  printf("\t_permanentReceive = %d\n", int(_permanentReceive));
  printf("\t_frameCheck = %d\n", int(_frameCheck));
  printf("\t_deviceMode = %d\n", int(_deviceMode));
  printf("\tnum interrupts = %d\n", int(_numInterrupts));
  printf("\tdevice ID = %0X\n", int(getDeviceIdentifier()));
  printf("\tdevice sysstatus = 0x");
  for (int i = 0; i < DW1000Constants::LEN_SYS_STATUS; i++) {
    printf("%02x", _sysstatus[i]);
  }
  printf("\n");
  printf("\t\tSys status bits set (DW counting) = ");
  for (int bitNo = 0; bitNo < (8 * DW1000Constants::LEN_SYS_STATUS); bitNo++) {
    if (getBit(_sysstatus, LEN_SYS_STATUS, bitNo)) {
      printf("%d, ", int(bitNo));
    }
  }
  printf("\n");
  printf("\tIRQ GPIO pin = %d\n",
         int(stm32_gpioread(GPIO_EXPANSION_LPSDECK_IRQ)));

  char text[192];
  getPrintableDeviceIdentifier(text);
  printf("\tDevice identifier: \n\t\t%s\n", text);
  getPrintableExtendedUniqueIdentifier(text);
  printf("\tExtended Unique Identifier: \n\t\t%s\n", text);
  getPrintableNetworkIdAndShortAddress(text);
  printf("\tNetworkId and Short Address: \n\t\t%s\n", text);
  getPrintableDeviceMode(text);
  printf("\tDevice mode: \n\t\t%s\n", text);

  printf("===========\n");
  readChannelControlRegister();
  printf("\t\tChannel control register bits:");
  for (int bitNo = 0; bitNo < (8 * DW1000Constants::LEN_CHAN_CTRL); bitNo++) {
    if (getBit(_chanctrl, LEN_CHAN_CTRL, bitNo)) {
      printf("%d, ", int(bitNo));
    }
  }
  printf("\n");

}

int32_t DW1000Class::getCPUTimeMillis() {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

