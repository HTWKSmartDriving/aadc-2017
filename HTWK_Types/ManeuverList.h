//
// Created by fabian on 04.09.17.
//

#ifndef AADC_USER_MANEUVERLIST_H
#define AADC_USER_MANEUVERLIST_H

struct tAADC_Maneuver {
    int id;
    cString action;
};

struct tSector {
    int id;
    std::vector<tAADC_Maneuver> maneuverList;
};

#endif //AADC_USER_MANEUVERLIST_H
