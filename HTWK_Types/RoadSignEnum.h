#ifndef _ROAD_SIGN_ENUM_H_
#define _ROAD_SIGN_ENUM_H_

#include <string>
#include <map>

enum ROAD_SIGN {
    MISSING = -1,
    UNMARKED = 0,
    STOP = 1,
    PARKING = 2,
    HAVE_WAY = 3,
    THIS_WAY = 4,
    GIVE_WAY = 5,
    PEDESTRIANS_CROSSING = 6,
    ROUNDABOUT = 7,
    NO_PASSING = 8,
    NO_ENTRY = 9,
    A9 = 10,
    ONE_WAY = 11,
    CONSTRUCTION_SITE = 12,
    SPEED_50 = 13,
    SPEED_100 = 14
};

static const std::map<const ROAD_SIGN, const std::string> ROAD_SIGN_TO_NAME_MAP = {
        {MISSING,              "Missing"},
        {UNMARKED,             "Unmarked"},
        {STOP,                 "Stop"},
        {PARKING,              "Parking"},
        {HAVE_WAY,             "Have-Way"},
        {THIS_WAY,             "This-Way"},
        {GIVE_WAY,             "Give-Way"},
        {PEDESTRIANS_CROSSING, "Pedestrian-Crossing"},
        {ROUNDABOUT,           "Roundabout"},
        {NO_PASSING,           "No-Passing"},
        {NO_ENTRY,             "No-Entry"},
        {A9,                   "Test-A9"},
        {ONE_WAY,              "One-Way"},
        {CONSTRUCTION_SITE,    "Construction-Site"},
        {SPEED_50,             "Speed-50"},
        {SPEED_100,            "Speed-100"},
};

inline const std::string mapTypeToString(const ROAD_SIGN &id) {
    auto it = ROAD_SIGN_TO_NAME_MAP.find(id);
    if (it != ROAD_SIGN_TO_NAME_MAP.end()) {
        return it->second;
    } else {
        return std::string("No Valid ID");
    }
};

#endif
