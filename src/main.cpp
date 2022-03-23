#include <Arduino.h>


#define BLE_ACTIVE
#define LSM_ACTIVE
#define MAG_ACTIVE
#define FFT_ACTIVE
#define ACCINT_ACTIVE


#ifdef BLE_ACTIVE
  #include "config_BLE.h"
  void BLEAdvInit(){
    Bluefruit.begin();
    Bluefruit.setTxPower(kTxPowerSTD);
    Bluefruit.Advertising.setInterval(kAdvIntSTD, kAdvIntSTD);

    Bluefruit.Advertising.setData(kAdvDataArr, sizeof(kAdvDataArr));
    Bluefruit.Advertising.start();
  }
#endif //BLE_ACTIVE


#ifdef LSM_ACTIVE
  #include "config_LSM.h"
#endif //LSM_ACTIVE


void setup() {
    Serial.begin(115200);
    while ( !Serial ) delay(10);

    #ifdef BLE_ACTIVE
      BLEAdvInit();
    #endif
}

void loop(void) {

}