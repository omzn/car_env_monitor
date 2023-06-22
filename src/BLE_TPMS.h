#ifndef BLE_TPMS_H
#define BLE_TPMS_H

#include <Arduino.h>

#define PRESSURE_CALIB (0)

#define BLETPMS_ManufacturerId (0x0001) 

#define BLETPMS_VenderId_FL (0x80eaca)
#define BLETPMS_VenderId_FR (0x81eaca)
#define BLETPMS_VenderId_RL (0x82eaca)
#define BLETPMS_VenderId_RR (0x83eaca)

#define BLETPMS_Tire_FL (0x11acb7) 
#define BLETPMS_Tire_FR (0x21aa09) 
#define BLETPMS_Tire_RL (0x31a6c5) 
#define BLETPMS_Tire_RR (0x41a834) 

#define TIRE_FL (0)
#define TIRE_FR (1)
#define TIRE_RL (2)
#define TIRE_RR (3)

class BLEtpms {
private:
  int _tire_id;
  float _temp_c;
  float _pressure_kpa;
  float _prev_pressure_kpa = 0;
  float _battery_percent;
  bool _updated = false;
public:
  static bool isManufacturerId(std::string d);
  static int tire_id(std::string d);
  BLEtpms();
  void scan(std::string d);
  void tire_id(int id);
  int tire_id();
  void temp_raw(int t);
  float temp();
  void pressure_raw(int p);
  float pressure();
  void battery_raw(int p);
  float battery();
  float batteryV();
  void updated(bool);
  bool updated();
  String macaddress();
};


#endif