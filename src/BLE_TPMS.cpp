#include "BLE_TPMS.h"

// class methods

bool BLEtpms::isManufacturerId(std::string d) {
  uint32_t manu = d[0] << 8 | d[1];
  return (manu == BLETPMS_ManufacturerId);
}

int BLEtpms::tire_id(std::string d) {
  int tire;
  //uint32_t addr_vender = d[2] << 16 | d[3] << 8 | d[4];
  uint32_t addr_tire = d[5] << 16 | d[6] << 8 | d[7];
  switch (addr_tire) {
    case BLETPMS_Tire_FL:
      tire = TIRE_FL;
      break;
    case BLETPMS_Tire_RL:
      tire = TIRE_RL;
      break;
    case BLETPMS_Tire_FR:
      tire = TIRE_FR;
      break;
    case BLETPMS_Tire_RR:
      tire = TIRE_RR;
      break;
    default:
      tire = -1;
  }
  return tire;
}

// member methods

BLEtpms::BLEtpms() {
  _tire_id = -1;
}

void BLEtpms::scan(std::string d) {
  int16_t temp;
  pressure_raw( d[10] << 16 | d[9] << 8 | d[8] );
  temp = d[13] << 8 | d[12];
  temp_raw( temp );
  battery_raw( d[16] );
}

void BLEtpms::tire_id(int id) {
  _tire_id = id;
}

int BLEtpms::tire_id() {
  return _tire_id;
}

void BLEtpms::temp_raw(int t) {
  _temp_c = (float)t ;
}

float BLEtpms::temp() {
  return _temp_c;
}

void BLEtpms::pressure_raw(int p) {
  _pressure_kpa = ((float)p / 1000.0) + PRESSURE_CALIB;   
}

float BLEtpms::pressure() {
  return _pressure_kpa;
}

void BLEtpms::battery_raw(int b) {
  _battery_percent = (float)b;
}

float BLEtpms::battery() {
  return _battery_percent;
}
float BLEtpms::batteryV() {
  return (_battery_percent / 100.0) * 3.0;
}

void BLEtpms::updated(bool b) {
  _updated = b;
}

bool BLEtpms::updated() {
  return _updated;
}