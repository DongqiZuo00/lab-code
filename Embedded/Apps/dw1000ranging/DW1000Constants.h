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
 * @file DW1000Constants.h
 * Arduino driver library (header file) for the Decawave DW1000 UWB transceiver IC.
 */

#pragma once

namespace DW1000NS {
namespace DW1000Constants {

enum {

// time stamp byte length
  LEN_STAMP = 5,

// enum to determine RX or TX mode of device
  IDLE_MODE = 0x00,
  RX_MODE = 0x01,
  TX_MODE = 0x02,

// used for SPI ready w/o actual writes
  JUNK = 0x00,

// no sub-address for register write
  NO_SUB = 0xFF,

// device id register
  DEV_ID = 0x00,
  LEN_DEV_ID = 4,

// extended unique identifier register
  EUI = 0x01,
  LEN_EUI = 8,

// PAN identifier, short address register
  PANADR = 0x03,
  LEN_PANADR = 4,

// device configuration register
  SYS_CFG = 0x04,
  LEN_SYS_CFG = 4,
  FFEN_BIT = 0,
  FFBC_BIT = 1,
  FFAB_BIT = 2,
  FFAD_BIT = 3,
  FFAA_BIT = 4,
  FFAM_BIT = 5,
  FFAR_BIT = 6,
  DIS_DRXB_BIT = 12,
  DIS_STXP_BIT = 18,
  HIRQ_POL_BIT = 9,
  RXAUTR_BIT = 29,
  PHR_MODE_SUB = 16,
  LEN_PHR_MODE_SUB = 2,
  RXM110K_BIT = 22,

// device control register
  SYS_CTRL = 0x0D,
  LEN_SYS_CTRL = 4,
  SFCST_BIT = 0,
  TXSTRT_BIT = 1,
  TXDLYS_BIT = 2,
  TRXOFF_BIT = 6,
  WAIT4RESP_BIT = 7,
  RXENAB_BIT = 8,
  RXDLYS_BIT = 9,

// system event status register
  SYS_STATUS = 0x0F,
  LEN_SYS_STATUS = 5,
  CPLOCK_BIT = 1,
  AAT_BIT = 3,
  TXFRB_BIT = 4,
  TXPRS_BIT = 5,
  TXPHS_BIT = 6,
  TXFRS_BIT = 7,
  LDEDONE_BIT = 10,
  RXPHE_BIT = 12,
  RXDFR_BIT = 13,
  RXFCG_BIT = 14,
  RXFCE_BIT = 15,
  RXRFSL_BIT = 16,
  RXRFTO_BIT = 17,
  RXPTO_BIT = 21,
  RXSFDTO_BIT = 26,
  LDEERR_BIT = 18,
  RFPLL_LL_BIT = 24,
  CLKPLL_LL_BIT = 25,

// system event mask register
// NOTE: uses the bit definitions of SYS_STATUS (below 32)
  SYS_MASK = 0x0E,
  LEN_SYS_MASK = 4,

// system time counter
  SYS_TIME = 0x06,
  LEN_SYS_TIME = LEN_STAMP,

// RX timestamp register
  RX_TIME = 0x15,
  LEN_RX_TIME = 14,
  RX_STAMP_SUB = 0x00,
  FP_AMPL1_SUB = 0x07,
  LEN_RX_STAMP = LEN_STAMP,
  LEN_FP_AMPL1 = 2,

// RX frame quality
  RX_FQUAL = 0x12,
  LEN_RX_FQUAL = 8,
  STD_NOISE_SUB = 0x00,
  FP_AMPL2_SUB = 0x02,
  FP_AMPL3_SUB = 0x04,
  CIR_PWR_SUB = 0x06,
  LEN_STD_NOISE = 2,
  LEN_FP_AMPL2 = 2,
  LEN_FP_AMPL3 = 2,
  LEN_CIR_PWR = 2,

// TX timestamp register
  TX_TIME = 0x17,
  LEN_TX_TIME = 10,
  TX_STAMP_SUB = 0,
  LEN_TX_STAMP = LEN_STAMP,

// timing register (for delayed RX/TX)
  DX_TIME = 0x0A,
  LEN_DX_TIME = LEN_STAMP,

// transmit data buffer
  TX_BUFFER = 0x09,
  LEN_TX_BUFFER = 1024,
  LEN_UWB_FRAMES = 127,
  LEN_EXT_UWB_FRAMES = 1023,

// RX frame info
  RX_FINFO = 0x10,
  LEN_RX_FINFO = 4,

// receive data buffer
  RX_BUFFER = 0x11,
  LEN_RX_BUFFER = 1024,

// transmit control
  TX_FCTRL = 0x08,
  LEN_TX_FCTRL = 5,

// channel control
  CHAN_CTRL = 0x1F,
  LEN_CHAN_CTRL = 4,
  DWSFD_BIT = 17,
  TNSSFD_BIT = 20,
  RNSSFD_BIT = 21,

// user-defined SFD
  USR_SFD = 0x21,
  LEN_USR_SFD = 41,
  SFD_LENGTH_SUB = 0x00,
  LEN_SFD_LENGTH = 1,

// OTP control (for LDE micro code loading only)
  OTP_IF = 0x2D,
  OTP_ADDR_SUB = 0x04,
  OTP_CTRL_SUB = 0x06,
  OTP_RDAT_SUB = 0x0A,
  LEN_OTP_ADDR = 2,
  LEN_OTP_CTRL = 2,
  LEN_OTP_RDAT = 4,

// AGC_TUNE1/2 (for re-tuning only)
  AGC_TUNE = 0x23,
  AGC_TUNE1_SUB = 0x04,
  AGC_TUNE2_SUB = 0x0C,
  AGC_TUNE3_SUB = 0x12,
  LEN_AGC_TUNE1 = 2,
  LEN_AGC_TUNE2 = 4,
  LEN_AGC_TUNE3 = 2,

// DRX_TUNE2 (for re-tuning only)
  DRX_TUNE = 0x27,
  DRX_TUNE0b_SUB = 0x02,
  DRX_TUNE1a_SUB = 0x04,
  DRX_TUNE1b_SUB = 0x06,
  DRX_TUNE2_SUB = 0x08,
  DRX_TUNE4H_SUB = 0x26,
  LEN_DRX_TUNE0b = 2,
  LEN_DRX_TUNE1a = 2,
  LEN_DRX_TUNE1b = 2,
  LEN_DRX_TUNE2 = 4,
  LEN_DRX_TUNE4H = 2,

// LDE_CFG1 (for re-tuning only)
  LDE_IF = 0x2E,
  LDE_CFG1_SUB = 0x0806,
  LDE_RXANTD_SUB = 0x1804,
  LDE_CFG2_SUB = 0x1806,
  LDE_REPC_SUB = 0x2804,
  LEN_LDE_CFG1 = 1,
  LEN_LDE_CFG2 = 2,
  LEN_LDE_REPC = 2,
  LEN_LDE_RXANTD = 2,

// TX_POWER (for re-tuning only)
  TX_POWER = 0x1E,
  LEN_TX_POWER = 4,

// RF_CONF (for re-tuning only)
  RF_CONF = 0x28,
  RF_RXCTRLH_SUB = 0x0B,
  RF_TXCTRL_SUB = 0x0C,
  LEN_RF_RXCTRLH = 1,
  LEN_RF_TXCTRL = 4,

// TX_CAL (for re-tuning only)
  TX_CAL = 0x2A,
  TC_PGDELAY_SUB = 0x0B,
  LEN_TC_PGDELAY = 1,
  TC_SARC = 0x00,
  TC_SARL = 0x03,

// FS_CTRL (for re-tuning only)
  FS_CTRL = 0x2B,
  FS_PLLCFG_SUB = 0x07,
  FS_PLLTUNE_SUB = 0x0B,
  FS_XTALT_SUB = 0x0E,
  LEN_FS_PLLCFG = 4,
  LEN_FS_PLLTUNE = 1,
  LEN_FS_XTALT = 1,

// AON
  AON = 0x2C,
  AON_WCFG_SUB = 0x00,
  LEN_AON_WCFG = 2,
  ONW_LDC_BIT = 6,
  ONW_LDD0_BIT = 12,
  AON_CTRL_SUB = 0x02,
  LEN_AON_CTRL = 1,
  RESTORE_BIT = 0,
  SAVE_BIT = 1,
  UPL_CFG_BIT = 2,

  AON_CFG0_SUB = 0x06,
  LEN_AON_CFG0 = 4,
  SLEEP_EN_BIT = 0,
  WAKE_PIN_BIT = 1,
  WAKE_SPI_BIT = 2,
  WAKE_CNT_BIT = 3,

// PMSC
  PMSC = 0x36,
  PMSC_CTRL0_SUB = 0x00,
  PMSC_CTRL1_SUB = 0x04,
  PMSC_LEDC_SUB = 0x28,
  LEN_PMSC_CTRL0 = 4,
  LEN_PMSC_CTRL1 = 4,
  LEN_PMSC_LEDC = 4,
  GPDCE_BIT = 18,
  KHZCLKEN_BIT = 23,
  BLNKEN = 8,

  ATXSLP_BIT = 11,
  ARXSLP_BIT = 12,

// TX_ANTD Antenna delays
  TX_ANTD = 0x18,
  LEN_TX_ANTD = 2,

// GPIO control register
  GPIO_CTRL = 0x26,
  GPIO_MODE_SUB = 0x00,
  LEN_GPIO_MODE = 4,

  MSGP0 = 6,
  MSGP1 = 8,
  MSGP2 = 10,
  MSGP3 = 12,
  MSGP4 = 14,
  MSGP5 = 16,
  MSGP6 = 18,
  MSGP7 = 20,

};
}
}

