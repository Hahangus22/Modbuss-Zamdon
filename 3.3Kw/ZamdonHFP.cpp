/*
 * ZamdonHFP.cpp - Arduino Library Implementation for HFP Zamdon Inverter Modbus RTU
 */

#include "ZamdonHFP.h"

ZamdonHFP::ZamdonHFP(uint8_t slaveId) {
  _slaveId = slaveId;
  _serial = NULL;
  _timeoutMs = 1000;
  _hasRealtime = false;
  _hasSetting = false;
  memset(&_realtimeData, 0, sizeof(_realtimeData));
  memset(&_settingData, 0, sizeof(_settingData));
  memset(_realtimeRegs, 0, sizeof(_realtimeRegs));
  memset(_settingRegs, 0, sizeof(_settingRegs));
}

void ZamdonHFP::begin(Stream &serialPort, uint32_t timeoutMs) {
  _serial = &serialPort;
  _timeoutMs = timeoutMs;
}

uint16_t ZamdonHFP::calculateCRC(const uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void ZamdonHFP::sendModbusRequest(uint8_t funcCode, uint16_t startAddr, uint16_t count) {
  if (!_serial) return;

  uint8_t frame[8];
  frame[0] = _slaveId;
  frame[1] = funcCode;
  frame[2] = (startAddr >> 8) & 0xFF;
  frame[3] = startAddr & 0xFF;
  frame[4] = (count >> 8) & 0xFF;
  frame[5] = count & 0xFF;

  uint16_t crc = calculateCRC(frame, 6);
  frame[6] = crc & 0xFF;        // CRC Low
  frame[7] = (crc >> 8) & 0xFF; // CRC High

  while (_serial->available()) _serial->read();

  _serial->write(frame, 8);
  _serial->flush();
}

bool ZamdonHFP::readRegisters(uint8_t funcCode, uint16_t startAddr, uint16_t regCount, uint16_t *destBuffer) {
  if (!_serial) return false;

  sendModbusRequest(funcCode, startAddr, regCount);

  uint16_t expectedBytes = 5 + (regCount * 2);
  uint8_t rxBuf[256];
  uint16_t bytesRead = 0;
  unsigned long startWait = millis();

  while ((millis() - startWait) < _timeoutMs) {
    if (_serial->available()) {
      rxBuf[bytesRead++] = _serial->read();
      if (bytesRead >= expectedBytes) break;
      if (bytesRead >= sizeof(rxBuf)) break;
    }
  }

  if (bytesRead < expectedBytes) {
    return false;
  }

  uint16_t crcReceived = rxBuf[bytesRead - 2] | (rxBuf[bytesRead - 1] << 8);
  uint16_t crcCalc = calculateCRC(rxBuf, bytesRead - 2);

  if (crcReceived != crcCalc) {
    return false;
  }

  if (rxBuf[0] != _slaveId || rxBuf[1] != funcCode) {
    return false;
  }

  for (uint16_t i = 0; i < regCount; i++) {
    destBuffer[i] = (rxBuf[3 + i * 2] << 8) | rxBuf[4 + i * 2];
  }

  return true;
}

bool ZamdonHFP::writeSingleRegister(uint16_t regAddr, uint16_t value) {
  if (!_serial) return false;

  uint8_t frame[8];
  frame[0] = _slaveId;
  frame[1] = 0x06;
  frame[2] = (regAddr >> 8) & 0xFF;
  frame[3] = regAddr & 0xFF;
  frame[4] = (value >> 8) & 0xFF;
  frame[5] = value & 0xFF;

  uint16_t crc = calculateCRC(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;

  while (_serial->available()) _serial->read();
  _serial->write(frame, 8);
  _serial->flush();

  uint8_t rxBuf[8];
  uint8_t bytesRead = 0;
  unsigned long startWait = millis();

  while ((millis() - startWait) < _timeoutMs) {
    if (_serial->available()) {
      rxBuf[bytesRead++] = _serial->read();
      if (bytesRead >= 8) break;
    }
  }

  if (bytesRead < 8) return false;
  uint16_t crcCalc = calculateCRC(rxBuf, 6);
  uint16_t crcReceived = rxBuf[6] | (rxBuf[7] << 8);
  return (crcCalc == crcReceived && rxBuf[0] == _slaveId && rxBuf[1] == 0x06);
}

void ZamdonHFP::parseRealtimeData() {
  _realtimeData.protocolId            = _realtimeRegs[0];
  _realtimeData.inverterCurrent       = _realtimeRegs[1] * 0.01f;
  _realtimeData.inverterVoltage       = _realtimeRegs[2];
  _realtimeData.busVoltage            = _realtimeRegs[4] * 0.1f;
  _realtimeData.gridVoltage           = _realtimeRegs[5];
  _realtimeData.loadCurrent           = _realtimeRegs[6] * 0.1f;
  _realtimeData.gridCurrent           = _realtimeRegs[7] * 0.1f;
  _realtimeData.batteryVoltage        = _realtimeRegs[8] * 0.1f;
  _realtimeData.busCurrent            = ((int16_t)_realtimeRegs[9]) * 0.01f;
  _realtimeData.tempCharging          = _realtimeRegs[10];
  _realtimeData.pv1Current            = _realtimeRegs[11] * 0.1f;
  _realtimeData.pv1Voltage            = _realtimeRegs[12] * 0.1f;
  _realtimeData.pv2Current            = _realtimeRegs[13] * 0.1f;
  _realtimeData.pv2Voltage            = _realtimeRegs[14] * 0.1f;
  _realtimeData.tempDc                = _realtimeRegs[15];
  _realtimeData.tempInverter          = _realtimeRegs[16];
  _realtimeData.batteryCurrent        = ((int16_t)_realtimeRegs[17]) * 0.1f;
  _realtimeData.gridFreq              = _realtimeRegs[18] * 0.1f;
  _realtimeData.loadPercentage        = _realtimeRegs[19];
  _realtimeData.deviceMode            = _realtimeRegs[20] & 0xFF;
  _realtimeData.inverterMode          = _realtimeRegs[21] & 0xFF;
  _realtimeData.dcdcMode              = _realtimeRegs[22] & 0xFF;

  _realtimeData.systemFlags           = _realtimeRegs[23];
  _realtimeData.inverterFlags         = _realtimeRegs[24];
  _realtimeData.dcFlags               = _realtimeRegs[25];
  _realtimeData.mpptFlags             = _realtimeRegs[26];
  _realtimeData.loadPower             = _realtimeRegs[32];
  _realtimeData.ongridFlags           = _realtimeRegs[40];

  _realtimeData.lithiumVoltage        = _realtimeRegs[48] * 0.01f;
  _realtimeData.lithiumCurrent        = ((int16_t)_realtimeRegs[49]) * 0.01f;
  _realtimeData.lithiumTemp           = _realtimeRegs[50];
  _realtimeData.lithiumSoc            = _realtimeRegs[51];
  _realtimeData.lithiumRemainingCap   = _realtimeRegs[52] * 0.01f;
  _realtimeData.lithiumRatedCap       = _realtimeRegs[53] * 0.01f;
  _realtimeData.lithiumAlarmBits      = _realtimeRegs[54];

  _realtimeData.pv1Power              = _realtimeRegs[55];
  _realtimeData.pv2Power              = _realtimeRegs[56];
  _realtimeData.pv1EnergyKwh          = _realtimeRegs[57];
  _realtimeData.pv2EnergyKwh          = _realtimeRegs[58];
  _realtimeData.ongridRectifyingPower = _realtimeRegs[59];
  _realtimeData.ongridEnergyKwh       = _realtimeRegs[60];
  _realtimeData.rectifiedEnergyKwh    = _realtimeRegs[61];
  _realtimeData.battPower             = _realtimeRegs[62];
  _realtimeData.battChargeEnergyKwh   = _realtimeRegs[63];
  _realtimeData.battDischargeEnergyKwh= _realtimeRegs[64];
  _realtimeData.loadEnergyKwh         = _realtimeRegs[65];
  _realtimeData.mainsPower            = _realtimeRegs[66];
  _realtimeData.mainsEnergyKwh        = _realtimeRegs[67];
}

void ZamdonHFP::parseSettingData() {
  _settingData.workMode               = (_settingRegs[0] >> 8) & 0xFF;
  _settingData.devId                  = _settingRegs[0] & 0xFF;
  _settingData.devicePower            = _settingRegs[1];
  _settingData.outVoltage             = (_settingRegs[2] >> 8) & 0xFF;
  _settingData.outFreq                = _settingRegs[2] & 0xFF;
  _settingData.chgPriority            = (_settingRegs[3] >> 8) & 0xFF;
  _settingData.battConnected          = _settingRegs[3] & 0xFF;
  _settingData.maxChgCurrent          = (_settingRegs[4] >> 8) & 0xFF;
  _settingData.maxAcChgCurrent        = _settingRegs[4] & 0xFF;
  _settingData.maxRectifiedCurrent    = (_settingRegs[5] >> 8) & 0xFF;
  _settingData.maxOngridCurrent       = _settingRegs[5] & 0xFF;
  _settingData.mainsRange             = (_settingRegs[6] >> 8) & 0xFF;
  _settingData.battNominalVoltage     = _settingRegs[6] & 0xFF;
  _settingData.equalizingVoltage      = _settingRegs[7] * 0.1f;
  _settingData.floatVoltage           = _settingRegs[8] * 0.1f;
  _settingData.battHighVoltProt       = _settingRegs[9] * 0.1f;
  _settingData.battHighVoltAlarm      = _settingRegs[10] * 0.1f;
  _settingData.battHighVoltRecov      = _settingRegs[11] * 0.1f;
  _settingData.battLowVoltProt        = _settingRegs[12] * 0.1f;
  _settingData.battLowVoltAlarm       = _settingRegs[13] * 0.1f;
  _settingData.battLowVoltRecov       = _settingRegs[14] * 0.1f;
  _settingData.loadOverloadRecov      = (_settingRegs[15] >> 8) & 0xFF;
  _settingData.battLowRecovEn         = _settingRegs[15] & 0xFF;
  _settingData.invPrioBattVolt        = _settingRegs[16] * 0.1f;
  _settingData.invPrioMainsVolt       = _settingRegs[17] * 0.1f;
  _settingData.overtempRestart       = (_settingRegs[18] >> 8) & 0xFF;
  _settingData.ongridSelection        = _settingRegs[18] & 0xFF;
  _settingData.antiReverseMeterCurrent= ((int16_t)_settingRegs[19]) * 0.01f;
  _settingData.dcComponentCalib       = (int16_t)_settingRegs[20];
  _settingData.hfpOngridCurrSet       = (_settingRegs[21] >> 8) & 0xFF;
  _settingData.neutralGrounding       = _settingRegs[21] & 0xFF;

  _settingData.bmsBattAddr            = (_settingRegs[39] >> 8) & 0xFF;
  _settingData.remotePower            = _settingRegs[39] & 0xFF;
  _settingData.lithiumShutdownSoc     = (_settingRegs[40] >> 8) & 0xFF;
  _settingData.lithiumProtocol        = _settingRegs[40] & 0xFF;
  _settingData.lithiumCutoffSoc       = (_settingRegs[41] >> 8) & 0xFF;
  _settingData.lithiumLowSocRec       = _settingRegs[41] & 0xFF;
  _settingData.invPrioSwitchMainsSoc  = (_settingRegs[42] >> 8) & 0xFF;
  _settingData.invPrioSwitchBattSoc   = _settingRegs[42] & 0xFF;
}

bool ZamdonHFP::readRealtimeData() {
  if (readRegisters(0x03, 0x0000, 68, _realtimeRegs)) {
    _hasRealtime = true;
    parseRealtimeData();
    return true;
  }
  _hasRealtime = false;
  return false;
}

bool ZamdonHFP::readSettingData() {
  if (readRegisters(0x03, 0x0060, 43, _settingRegs)) {
    _hasSetting = true;
    parseSettingData();
    return true;
  }
  _hasSetting = false;
  return false;
}

const char* ZamdonHFP::getDeviceModeStr(uint8_t mode) {
  switch (mode) {
    case 0x33: return "Mains Power Priority Mode (PLN Priority)";
    case 0x66: return "PV Priority Mode";
    case 0x99: return "Battery Priority Mode";
    default:   return "Unknown Mode";
  }
}

const char* ZamdonHFP::getInverterModeStr(uint8_t mode) {
  switch (mode) {
    case 0x44: return "Mains Power Supply Mode";
    case 0x55: return "On Grid Mode";
    case 0x88: return "Inversion Mode";
    case 0xAA: return "Rectification Mode";
    case 0x00: return "Stop Working";
    default:   return "Unknown Mode";
  }
}

const char* ZamdonHFP::getDcdcModeStr(uint8_t mode) {
  switch (mode) {
    case 0x55: return "Charging Mode";
    case 0xAA: return "Discharging Mode";
    case 0x00: return "Stop Working";
    default:   return "Unknown Mode";
  }
}

const char* ZamdonHFP::getLithiumTypeStr(uint8_t battType) {
  switch (battType) {
    case 0: return "Lithium Iron Phosphate (LiFePO4)";
    case 1: return "Ternary Lithium";
    case 2: return "Lithium Titanate (LTO)";
    default: return "Reserve";
  }
}

const char* ZamdonHFP::getLithiumProtocolStr(uint8_t proto) {
  switch (proto) {
    case 1: return "Growatt Lithium Protocol";
    case 2: return "Voltronic Lithium Protocol";
    case 3: return "Plyn Lithium Protocol";
    case 4: return "Pace Lithium Protocol";
    case 5: return "Pace-A / PYL-A Lithium Protocol";
    default: return "Lead-Acid Battery";
  }
}

const char* ZamdonHFP::getWorkModeStr(uint8_t mode) {
  switch (mode) {
    case 1: return "Mains Priority Mode";
    case 2: return "PV Priority Mode";
    case 3: return "Battery Priority Mode";
    default: return "Unknown Mode";
  }
}

const char* ZamdonHFP::getChgPriorityStr(uint8_t prio) {
  switch (prio) {
    case 0: return "PV & PLN Bersamaan";
    case 1: return "Hanya Charging PV";
    case 2: return "Prioritas Charging PV";
    default: return "Unknown Priority";
  }
}

const char* ZamdonHFP::getOngridSelectionStr(uint8_t sel) {
  switch (sel) {
    case 0: return "AC Power Complementary Mode";
    case 1: return "On-Grid Mode";
    case 2: return "Off-Grid Mode";
    case 3: return "On Grid Anti-Reverse Flow Mode";
    default: return "Unknown Selection";
  }
}

void ZamdonHFP::printDecodedFlags(Print &output) {
  if (!_hasRealtime) return;

  uint16_t sf = _realtimeData.systemFlags;
  output.printf("  [0x0017] System Flags (0x%04X):\n", sf);
  output.printf("    - Status Sakelar Power : %s\n", (sf & (1 << 0)) ? "ON" : "OFF");
  output.printf("    - Buffer Start         : %s\n", (sf & (1 << 1)) ? "Selesai" : "Belum Selesai");
  output.printf("    - Lock Fasa Listrik/Inv: %s\n", (sf & (1 << 2)) ? "Normal" : "Abnormal");
  output.printf("    - Bus Soft Start       : %s\n", (sf & (1 << 4)) ? "Abnormal" : "Normal");
  output.printf("    - Output Short Circuit : %s\n", (sf & (1 << 7)) ? "HUBUNG SINGKAT!" : "Normal");
  output.printf("    - Tegangan Input PLN   : %s\n", (sf & (1 << 8)) ? "Dibawah 50VAC" : "Normal");
  output.printf("    - Frekuensi Input      : %s\n", (sf & (1 << 9)) ? "Abnormal" : "Normal");
  output.printf("    - Range Voltage Input  : %s\n", (sf & (1 << 10)) ? "120VAC-280VAC" : "165VAC-280VAC");
  output.printf("    - Sampling Base        : %s\n", (sf & (1 << 11)) ? "Abnormal" : "Normal");
  output.printf("    - Batt Low Volt Restart: %s\n", (sf & (1 << 13)) ? "Restarted" : "Belum");
  output.printf("    - Overload Restart     : %s\n", (sf & (1 << 14)) ? "Restarted" : "Belum");
  output.printf("    - Overtemp Restart     : %s\n", (sf & (1 << 15)) ? "Restarted" : "Belum");

  uint16_t inf = _realtimeData.inverterFlags;
  output.printf("  [0x0018] Inverter Flags (0x%04X):\n", inf);
  output.printf("    - Inv Soft Start       : %s\n", (inf & (1 << 0)) ? "Belum Selesai" : "Selesai");
  output.printf("    - Tegangan PLN Tinggi  : %s\n", (inf & (1 << 1)) ? "OVER VOLTAGE!" : "Normal");
  output.printf("    - Tegangan PLN Rendah  : %s\n", (inf & (1 << 2)) ? "UNDER VOLTAGE!" : "Normal");
  output.printf("    - Inverter Phase Lock  : %s\n", (inf & (1 << 4)) ? "Normal" : "Abnormal");
  output.printf("    - Status Input PLN     : %s\n", (inf & (1 << 5)) ? "Abnormal" : "Normal");
  output.printf("    - Rectifier SoftStart  : %s\n", (inf & (1 << 7)) ? "Belum Selesai" : "Selesai");
  output.printf("    - Grid Soft Start      : %s\n", (inf & (1 << 8)) ? "Belum Selesai" : "Selesai");
  output.printf("    - Suhu Inverter        : %s\n", (inf & (1 << 9)) ? "OVERTEMP/Abnormal" : "Normal");
  output.printf("    - Tegangan Inverter    : %s\n", (inf & (1 << 10)) ? "Abnormal" : "Normal");
  output.printf("    - Proteksi Beban Output: %s\n", (inf & (1 << 11)) ? "PROTEKSI AKTIF!" : "Normal");
  output.printf("    - Alarm Beban Output   : %s\n", (inf & (1 << 12)) ? "ALARM OVERLOAD!" : "Normal");
  output.printf("    - Off-grid Current Limit: %s\n", (inf & (1 << 15)) ? "Limit Aktif" : "Normal");

  uint16_t dcf = _realtimeData.dcFlags;
  output.printf("  [0x0019] DC Flags (0x%04X):\n", dcf);
  output.printf("    - Soft Start Pengisian : %s\n", (dcf & (1 << 0)) ? "Selesai" : "Proses Soft Start");
  output.printf("    - Tegangan Baterai High: %s\n", (dcf & (1 << 1)) ? "HIGH VOLTAGE!" : "Normal");
  output.printf("    - Peringatan Batt High : %s\n", (dcf & (1 << 2)) ? "ALARM!" : "Normal");
  output.printf("    - Proteksi Batt Low    : %s\n", (dcf & (1 << 3)) ? "PROTEKSI LOW BATT!" : "Normal");
  output.printf("    - Peringatan Batt Low  : %s\n", (dcf & (1 << 4)) ? "ALARM LOW BATT!" : "Normal");
  output.printf("    - Proteksi BUS High    : %s\n", (dcf & (1 << 5)) ? "PROTEKSI BUS HIGH!" : "Normal");
  output.printf("    - Proteksi BUS Low     : %s\n", (dcf & (1 << 6)) ? "PROTEKSI BUS LOW!" : "Normal");
  output.printf("    - Status Koneksi Batt  : %s\n", (dcf & (1 << 8)) ? "TIDAK Terhubung" : "Terhubung");
  output.printf("    - Status Tegangan BUS  : %s\n", (dcf & (1 << 9)) ? "Abnormal" : "Normal");
  output.printf("    - Suhu DC              : %s\n", (dcf & (1 << 10)) ? "Abnormal" : "Normal");
  output.printf("    - Inv Priority Transfer: %s\n", (dcf & (1 << 12)) ? "Transfer ke PLN" : "Transfer ke Baterai");

  uint16_t mpt = _realtimeData.mpptFlags;
  output.printf("  [0x001A] MPPT Flags (0x%04X):\n", mpt);
  output.printf("    - PV2 High Voltage     : %s\n", (mpt & (1 << 0)) ? "HIGH VOLTAGE!" : "Normal");
  output.printf("    - PV2 Low Voltage      : %s\n", (mpt & (1 << 1)) ? "LOW VOLTAGE" : "Normal");
  output.printf("    - PV1 High Voltage     : %s\n", (mpt & (1 << 2)) ? "HIGH VOLTAGE!" : "Normal");
  output.printf("    - PV1 Low Voltage      : %s\n", (mpt & (1 << 3)) ? "LOW VOLTAGE" : "Normal");
  output.printf("    - MPPT2 Soft Start     : %s\n", (mpt & (1 << 4)) ? "Selesai" : "Belum Selesai");
  output.printf("    - MPPT1 Soft Start     : %s\n", (mpt & (1 << 5)) ? "Selesai" : "Belum Selesai");
  output.printf("    - Status Suhu PV1      : %s\n", (mpt & (1 << 6)) ? "Abnormal" : "Normal");
  output.printf("    - Status Kerja MPPT1   : %s\n", (mpt & (1 << 7)) ? "Normal" : "Abnormal");
  output.printf("    - Status Tegangan PV1  : %s\n", (mpt & (1 << 8)) ? "Normal" : "Abnormal");
  output.printf("    - Status Tegangan PV2  : %s\n", (mpt & (1 << 9)) ? "Normal" : "Abnormal");
  output.printf("    - Status Kerja MPPT2   : %s\n", (mpt & (1 << 10)) ? "Normal" : "Abnormal");

  uint16_t ogf = _realtimeData.ongridFlags;
  output.printf("  [0x0028] On-Grid Flags (0x%04X):\n", ogf);
  output.printf("    - On-Grid Volt Low     : %s\n", (ogf & (1 << 0)) ? "LOW VOLTAGE" : "Normal");
  output.printf("    - On-Grid Volt High    : %s\n", (ogf & (1 << 1)) ? "OVER VOLTAGE" : "Normal");
  output.printf("    - On-Grid Voltage      : %s\n", (ogf & (1 << 2)) ? "Abnormal" : "Normal");
  output.printf("    - On-Grid Frequency    : %s\n", (ogf & (1 << 3)) ? "Abnormal" : "Normal");
  output.printf("    - PV Priority On-Grid  : %s\n", (ogf & (1 << 4)) ? "Dapat On-Grid" : "Tidak Bisa On-Grid");

  uint16_t la = _realtimeData.lithiumAlarmBits;
  output.printf("  [0x0036] Lithium Alarm & Status (0x%04X):\n", la);
  output.printf("    - Single Cell Over Volt : %s\n", (la & (1 << 0)) ? "ALARM" : "Normal");
  output.printf("    - Single Cell Under Volt: %s\n", (la & (1 << 1)) ? "ALARM" : "Normal");
  output.printf("    - Total Over Voltage    : %s\n", (la & (1 << 2)) ? "ALARM" : "Normal");
  output.printf("    - Total Under Voltage   : %s\n", (la & (1 << 3)) ? "ALARM" : "Normal");
  output.printf("    - Discharge Over Current: %s\n", (la & (1 << 4)) ? "ALARM" : "Normal");
  output.printf("    - Charge Over Current   : %s\n", (la & (1 << 5)) ? "ALARM" : "Normal");
  output.printf("    - Discharge High Temp   : %s\n", (la & (1 << 6)) ? "ALARM" : "Normal");
  output.printf("    - Discharge Low Temp    : %s\n", (la & (1 << 7)) ? "ALARM" : "Normal");
  output.printf("    - Charge High Temp      : %s\n", (la & (1 << 8)) ? "ALARM" : "Normal");
  output.printf("    - Charge Low Temp       : %s\n", (la & (1 << 9)) ? "ALARM" : "Normal");
  output.printf("    - MOS / Env Temp Alarm  : %s\n", (la & (1 << 10)) ? "ALARM" : "Normal");
  output.printf("    - System Low Volt Off   : %s\n", (la & (1 << 13)) ? "WARNING SHUTDOWN" : "Normal");
  uint8_t battType = (la >> 14) & 0x03;
  output.printf("    - Tipe Baterai Lithium  : %d (%s)\n", battType, getLithiumTypeStr(battType));
}

void ZamdonHFP::printRealtimeData(Print &output) {
  if (!_hasRealtime) {
    output.println("Data Realtime belum tersedia!");
    return;
  }
  output.println("\n=======================================================");
  output.println("--- REAL-TIME DATA (FC 03, Register 0x0000 - 0x0043) ---");
  output.println("=======================================================");
  output.printf("Protocol ID (0x0000)       : %d (%s)\n", _realtimeData.protocolId, (_realtimeData.protocolId == 1) ? "HFP High Frequency Inverter" : "Unknown");
  output.printf("Inverter Current (0x0001)  : %.2f A\n", _realtimeData.inverterCurrent);
  output.printf("Inverter Voltage (0x0002)  : %d V\n", _realtimeData.inverterVoltage);
  output.printf("Bus Voltage      (0x0004)  : %.1f V\n", _realtimeData.busVoltage);
  output.printf("Input (PLN) Volt (0x0005)  : %d V\n", _realtimeData.gridVoltage);
  output.printf("Load Current     (0x0006)  : %.1f A\n", _realtimeData.loadCurrent);
  output.printf("Input Current    (0x0007)  : %.1f A\n", _realtimeData.gridCurrent);
  output.printf("Battery Voltage  (0x0008)  : %.1f V\n", _realtimeData.batteryVoltage);
  output.printf("Bus Current      (0x0009)  : %.2f A\n", _realtimeData.busCurrent);
  output.printf("Charging Temp    (0x000A)  : %d °C\n", _realtimeData.tempCharging);
  output.printf("PV1 Current      (0x000B)  : %.1f A\n", _realtimeData.pv1Current);
  output.printf("PV1 Voltage      (0x000C)  : %.1f V\n", _realtimeData.pv1Voltage);
  output.printf("PV2 Current      (0x000D)  : %.1f A\n", _realtimeData.pv2Current);
  output.printf("PV2 Voltage      (0x000E)  : %.1f V\n", _realtimeData.pv2Voltage);
  output.printf("DC Temp          (0x000F)  : %d °C\n", _realtimeData.tempDc);
  output.printf("Inverter Temp    (0x0010)  : %d °C\n", _realtimeData.tempInverter);
  output.printf("Batt Charge/Disch(0x0011)  : %.1f A\n", _realtimeData.batteryCurrent);
  output.printf("Input Frequency  (0x0012)  : %.1f Hz\n", _realtimeData.gridFreq);
  output.printf("Load Percentage  (0x0013)  : %d %%\n", _realtimeData.loadPercentage);
  output.printf("Device Working Mode (0x0014): 0x%02X (%s)\n", _realtimeData.deviceMode, getDeviceModeStr(_realtimeData.deviceMode));
  output.printf("Inverter Section Mode(0x0015): 0x%02X (%s)\n", _realtimeData.inverterMode, getInverterModeStr(_realtimeData.inverterMode));
  output.printf("DCDC Section Mode   (0x0016): 0x%02X (%s)\n", _realtimeData.dcdcMode, getDcdcModeStr(_realtimeData.dcdcMode));
  output.printf("Load Power          (0x0020): %d W\n", _realtimeData.loadPower);

  printDecodedFlags(output);

  output.println("\n--- DATA BATERAI LITHIUM ---");
  output.printf("Lithium Batt Voltage (0x0030): %.2f V\n", _realtimeData.lithiumVoltage);
  output.printf("Lithium Batt Curr    (0x0031): %.2f A\n", _realtimeData.lithiumCurrent);
  output.printf("Lithium Batt Temp    (0x0032): %d °C\n", _realtimeData.lithiumTemp);
  output.printf("Lithium Batt SOC     (0x0033): %d %%\n", _realtimeData.lithiumSoc);
  output.printf("Lithium Remaining Cap(0x0034): %.2f AH\n", _realtimeData.lithiumRemainingCap);
  output.printf("Lithium Rated Cap    (0x0035): %.2f AH\n", _realtimeData.lithiumRatedCap);

  output.println("\n--- STATISTIK DAYA & ENERGI ---");
  output.printf("PV1 Real-time Power  (0x0037): %d W\n", _realtimeData.pv1Power);
  output.printf("PV2 Real-time Power  (0x0038): %d W\n", _realtimeData.pv2Power);
  output.printf("PV1 Total Generation (0x0039): %d KWH\n", _realtimeData.pv1EnergyKwh);
  output.printf("PV2 Total Generation (0x003A): %d KWH\n", _realtimeData.pv2EnergyKwh);
  output.printf("On-Grid Rectifying Pwr(0x003B): %d W\n", _realtimeData.ongridRectifyingPower);
  output.printf("On-Grid Electricity  (0x003C): %d KWH\n", _realtimeData.ongridEnergyKwh);
  output.printf("Rectified Quantity   (0x003D): %d KWH\n", _realtimeData.rectifiedEnergyKwh);
  output.printf("Batt Charge/Disch Pwr(0x003E): %d W\n", _realtimeData.battPower);
  output.printf("Batt Charging Cap    (0x003F): %d KWH\n", _realtimeData.battChargeEnergyKwh);
  output.printf("Batt Discharge Cap   (0x0040): %d KWH\n", _realtimeData.battDischargeEnergyKwh);
  output.printf("Load Power Capacity  (0x0041): %d KWH\n", _realtimeData.loadEnergyKwh);
  output.printf("Mains Real-time Pwr  (0x0042): %d W\n", _realtimeData.mainsPower);
  output.printf("Mains Electricity Pwr(0x0043): %d KWH\n", _realtimeData.mainsEnergyKwh);
}

void ZamdonHFP::printSettingData(Print &output) {
  if (!_hasSetting) {
    output.println("Data Setting belum tersedia!");
    return;
  }
  output.println("\n=======================================================");
  output.println("--- SETTING PARAMETERS (FC 03, Register 0x0060 - 0x008A) ---");
  output.println("=======================================================");
  output.printf("Mode Kerja Perangkat (0x0060 High) : %d (%s)\n", _settingData.workMode, getWorkModeStr(_settingData.workMode));
  output.printf("Device ID           (0x0060 Low)  : %d\n", _settingData.devId);
  output.printf("Daya Kapasitas Alat (0x0061)      : %d W\n", _settingData.devicePower);
  output.printf("Setting Tegangan Out(0x0062 High) : %d V\n", _settingData.outVoltage);
  output.printf("Setting Frekuensi Out(0x0062 Low) : %d Hz\n", _settingData.outFreq);
  output.printf("Prioritas Charging  (0x0063 High) : %d (%s)\n", _settingData.chgPriority, getChgPriorityStr(_settingData.chgPriority));
  output.printf("Koneksi Baterai     (0x0063 Low)  : %s\n", (_settingData.battConnected == 0) ? "Dengan Baterai" : "Tanpa Baterai");
  output.printf("Max Charging Curr   (0x0064 High) : %d A\n", _settingData.maxChgCurrent);
  output.printf("Max AC Charging Curr(0x0064 Low)  : %d A\n", _settingData.maxAcChgCurrent);
  output.printf("Max Rectified Curr  (0x0065 High) : %d A\n", _settingData.maxRectifiedCurrent);
  output.printf("Max On-Grid Current (0x0065 Low)  : %d A\n", _settingData.maxOngridCurrent);
  output.printf("Range Volt Input PLN(0x0066 High) : %s\n", (_settingData.mainsRange == 0) ? "165V-280V" : "120V-280V");
  output.printf("Volt Nominal Baterai(0x0066 Low)  : %d V\n", _settingData.battNominalVoltage);
  output.printf("Equalizing Voltage  (0x0067)      : %.1f V\n", _settingData.equalizingVoltage);
  output.printf("Floating Charge Volt(0x0068)      : %.1f V\n", _settingData.floatVoltage);
  output.printf("Batt High Volt Prot (0x0069)      : %.1f V\n", _settingData.battHighVoltProt);
  output.printf("Batt High Volt Alarm(0x006A)      : %.1f V\n", _settingData.battHighVoltAlarm);
  output.printf("Batt High Volt Recov(0x006B)      : %.1f V\n", _settingData.battHighVoltRecov);
  output.printf("Batt Low Volt Prot  (0x006C)      : %.1f V\n", _settingData.battLowVoltProt);
  output.printf("Batt Low Volt Alarm (0x006D)      : %.1f V\n", _settingData.battLowVoltAlarm);
  output.printf("Batt Low Volt Recov (0x006E)      : %.1f V\n", _settingData.battLowVoltRecov);
  output.printf("Overload Recovery   (0x006F High) : %s\n", (_settingData.loadOverloadRecov == 1) ? "Aktif" : "Non-aktif");
  output.printf("Batt Low Recov En   (0x006F Low)  : %s\n", (_settingData.battLowRecovEn == 1) ? "Aktif" : "Non-aktif");
  output.printf("Inv Priority -> Batt Volt (0x0070): %.1f V\n", _settingData.invPrioBattVolt);
  output.printf("Inv Priority -> Mains Volt(0x0071): %.1f V\n", _settingData.invPrioMainsVolt);
  output.printf("Overtemp Restart    (0x0072 High) : %s\n", (_settingData.overtempRestart == 1) ? "Dapat Restart" : "Tidak Restart");
  output.printf("Pilihan On-Grid     (0x0072 Low)  : %d (%s)\n", _settingData.ongridSelection, getOngridSelectionStr(_settingData.ongridSelection));
  output.printf("Anti-Reverse Meter Curr(0x0073)   : %.2f A\n", _settingData.antiReverseMeterCurrent);
  output.printf("DC Component Calibration(0x0074)  : %d V\n", _settingData.dcComponentCalib);
  output.printf("HFP On-Grid Curr Set(0x0075 High) : %d\n", _settingData.hfpOngridCurrSet);
  output.printf("Neutral Grounding   (0x0075 Low)  : %s\n", (_settingData.neutralGrounding == 1) ? "Grounded" : "Tidak Grounded");
  output.printf("Alamat Lithium BMS  (0x0087 High) : %d\n", _settingData.bmsBattAddr);
  output.printf("Remote Power On/Off (0x0087 Low)  : %s\n", (_settingData.remotePower == 1) ? "Power ON" : "Power OFF");
  output.printf("Lithium Shutdown SOC(0x0088 High) : %d %%\n", _settingData.lithiumShutdownSoc);
  output.printf("Protokol Lithium    (0x0088 Low)  : %d (%s)\n", _settingData.lithiumProtocol, getLithiumProtocolStr(_settingData.lithiumProtocol));
  output.printf("Lithium Cutoff SOC  (0x0089 H): %d %%\n", _settingData.lithiumCutoffSoc);
  output.printf("Lithium Low SOC Rec (0x0089 L): %d %%\n", _settingData.lithiumLowSocRec);
  output.printf("Inv Prio Switch Mains SOC(0x008A H): %d %%\n", _settingData.invPrioSwitchMainsSoc);
  output.printf("Inv Prio Switch Batt SOC (0x008A L): %d %%\n", _settingData.invPrioSwitchBattSoc);
}
