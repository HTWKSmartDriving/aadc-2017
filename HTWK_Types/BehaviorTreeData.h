
#ifndef AADC_USER_TTERNARYSIGNALVALUE_H
#define AADC_USER_TTERNARYSIGNALVALUE_H

#include <a_utils.h>

namespace BT {
    enum moduleState {
        FAIL = 0, SUCCESS = 1, RUNNING = 2
    };

#pragma pack(push, 1)
    struct tStateSignal {
        moduleState state;
        tUInt32 ui32ArduinoTimestamp;
    };
#pragma pack(pop)
}

#define MEDIA_TYPE_LANE_DATA     0x00041678
#define MEDIA_SUBTYPE_LANE_DATA  0x00041678

#endif //AADC_USER_TTERNARYSIGNALVALUE_H
