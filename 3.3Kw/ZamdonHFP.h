/*
 * ZamdonHFP.h - Arduino Library for HFP High Frequency Inverter/Controller Modbus RTU (V1.23)
 */

#ifndef ZAMDON_HFP_H
#define ZAMDON_HFP_H

#include <Arduino.h>

struct ZamdonRealtimeData {
  uint16_t protocolId;
  float inverterCurrent;
  uint16_t inverterVoltage;
  float busVoltage;
  uint16_t gridVoltage;
  float loadCurrent;
  float gridCurrent;
  float batteryVoltage;
  float busCurrent;
  uint16_t tempCharging;
  float pv1Current;
  float pv1Voltage;
  float pv2Current;
  float pv2Voltage;
  uint16_t tempDc;
  uint16_t tempInverter;
  float batteryCurrent; // Charge / Discharge current (+/-)
  float gridFreq;
  uint16_t loadPercentage;
  uint8_t deviceMode;
  uint8_t inverterMode;
  uint8_t dcdcMode;
  uint16_t loadPower;

  // Flag registers
  uint16_t systemFlags;
  uint16_t inverterFlags;
  uint16_t dcFlags;
  uint16_t mpptFlags;
  uint16_t ongridFlags;

  // Lithium Data
  float lithiumVoltage;
  float lithiumCurrent;
  uint16_t lithiumTemp;
  uint16_t lithiumSoc;
  float lithiumRemainingCap;
  float lithiumRatedCap;
  uint16_t lithiumAlarmBits;

  // Energy & Power Stats
  uint16_t pv1Power;
  uint16_t pv2Power;
  uint16_t pv1EnergyKwh;
  uint16_t pv2EnergyKwh;
  uint16_t ongridRectifyingPower;
  uint16_t ongridEnergyKwh;
  uint16_t rectifiedEnergyKwh;
  uint16_t battPower;
  uint16_t battChargeEnergyKwh;
  uint16_t battDischargeEnergyKwh;
  uint16_t loadEnergyKwh;
  uint16_t mainsPower;
  uint16_t mainsEnergyKwh;
};

struct ZamdonSettingData {
  uint8_t workMode;
  uint8_t devId;
  uint16_t devicePower;
  uint8_t outVoltage;
  uint8_t outFreq;
  uint8_t chgPriority;
  uint8_t battConnected;
  uint8_t maxChgCurrent;
  uint8_t maxAcChgCurrent;
  uint8_t maxRectifiedCurrent;
  uint8_t maxOngridCurrent;
  uint8_t mainsRange;
  uint8_t battNominalVoltage;
  float equalizingVoltage;
  float floatVoltage;
  float battHighVoltProt;
  float battHighVoltAlarm;
  float battHighVoltRecov;
  float battLowVoltProt;
  float battLowVoltAlarm;
  float battLowVoltRecov;
  uint8_t loadOverloadRecov;
  uint8_t battLowRecovEn;
  float invPrioBattVolt;
  float invPrioMainsVolt;
  uint8_t overtempRestart;
  uint8_t ongridSelection;
  float antiReverseMeterCurrent;
  int16_t dcComponentCalib;
  uint8_t hfpOngridCurrSet;
  uint8_t neutralGrounding;
  uint8_t bmsBattAddr;
  uint8_t remotePower;
  uint8_t lithiumShutdownSoc;
  uint8_t lithiumProtocol;
  uint8_t lithiumCutoffSoc;
  uint8_t lithiumLowSocRec;
  uint8_t invPrioSwitchMainsSoc;
  uint8_t invPrioSwitchBattSoc;
};

class ZamdonHFP {
public:
  ZamdonHFP(uint8_t slaveId = 0x01);

  // Initialize stream (e.g. Serial1) and timeout
  void begin(Stream &serialPort, uint32_t timeoutMs = 1000);

  // Read operations
  bool readRealtimeData();
  bool readSettingData();
  
  // Write operation (FC 06)
  bool writeSingleRegister(uint16_t regAddr, uint16_t value);

  // Getters for parsed data structures
  const ZamdonRealtimeData& getRealtimeData() const { return _realtimeData; }
  const ZamdonSettingData& getSettingData() const { return _settingData; }

  // Raw register array access (0x0000-0x0043 : 68 regs, 0x0060-0x008A : 43 regs)
  const uint16_t* getRawRealtimeRegs() const { return _realtimeRegs; }
  const uint16_t* getRawSettingRegs() const { return _settingRegs; }

  bool hasRealtimeData() const { return _hasRealtime; }
  bool hasSettingData() const { return _hasSetting; }

  // Serial print utilities
  void printRealtimeData(Print &output = Serial);
  void printSettingData(Print &output = Serial);
  void printDecodedFlags(Print &output = Serial);

  // Decoder string helpers
  static const char* getDeviceModeStr(uint8_t mode);
  static const char* getInverterModeStr(uint8_t mode);
  static const char* getDcdcModeStr(uint8_t mode);
  static const char* getLithiumTypeStr(uint8_t battType);
  static const char* getLithiumProtocolStr(uint8_t proto);
  static const char* getWorkModeStr(uint8_t mode);
  static const char* getChgPriorityStr(uint8_t prio);
  static const char* getOngridSelectionStr(uint8_t sel);

  // Static CRC calculation helper
  static uint16_t calculateCRC(const uint8_t *buf, int len);

private:
  Stream* _serial;
  uint8_t _slaveId;
  uint32_t _timeoutMs;

  uint16_t _realtimeRegs[68];
  uint16_t _settingRegs[43];
  bool _hasRealtime;
  bool _hasSetting;

  ZamdonRealtimeData _realtimeData;
  ZamdonSettingData _settingData;

  void parseRealtimeData();
  void parseSettingData();
  void sendModbusRequest(uint8_t funcCode, uint16_t startAddr, uint16_t count);
  bool readRegisters(uint8_t funcCode, uint16_t startAddr, uint16_t regCount, uint16_t *destBuffer);
};

#endif // ZAMDON_HFP_H
