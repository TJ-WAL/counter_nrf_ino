#include <bluefruit.h>

#define kTxPowerSTD 0
#define kTxPowerLP  -12

#define kAdvIntSTD    1600 //0.625ms steps * 1600 = 1sec.
#define kAdvIntLP     4800

uint8_t kAdvDataArr[] = {0x1c               //Length of Message-1
                        ,0xff               //Type: ff = ManuSpecData
                        ,0x59,0x00          //Company: 0059 = Nordic
                        ,0x69               //ACN ID
                        ,0x20               //Prod ID
                        ,0x00               //Vers. Nr.
                        ,0x01, 0x02, 0x03, 0x04, 0x05, 0x06 //MAC Adr.
                        ,0x01, 0x02, 0x03, 0x04    //UINT32 MachineHours
                        ,0x01, 0x02, 0x03, 0x04    //UINT32 ShaftHours
                        ,0x01, 0x02         //UINT16 RPM
                        ,0x01               //UINT8 Bat%
                        ,0x01               //BOOL isTurning
                        ,0x01, 0x02         //UINT16 BatV
                        ,0x01, 0x02         //SINT16 Temp
                        };