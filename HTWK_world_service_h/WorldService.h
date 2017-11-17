#ifndef _WORLD_SERVICE_CLASS_HEADER_
#define _WORLD_SERVICE_CLASS_HEADER_

#include "stdafx.h"
#include <ucom/service.h>

#include <utility>
#include <opencv2/core/mat.hpp>
#include "tWorldData.h"
#include "../HTWK_Debug/EnableLogs.h"
#include "../HTWK_Map/TrackMap.h"

#define SERVICE_NAME "HTWK World Service"
#define OID_WORLD_SERVICE "htwk.world_service"
#define IID_WORLD_INTERFACE "htwk.i_world_service"

#define WORLD_ACTION_ID "actionId"
#define WORLD_LAST_MANEUVER_ENTRY_IN_LIST "lastManeuver"
#define WORLD_NEXT_MANEUVER "nextManeuver"
#define WORLD_NEXT_TURN "nextTurn"

#define WORLD_LANE_LEFT "leftLane"
#define WORLD_LANE_RIGHT "rightLane"
#define WORLD_LANE_CENTER "pictureCenter"

#define WORLD_CURRENT_SPEED "currentSpeed"
#define WORLD_CURRENT_POSITION "currentPosition"
#define WORLD_CURRENT_RADIUS "currentRadius"
#define WORLD_CURRENT_HEADING "currentHeading"

#define WORLD_ROAD_SIGN_EXT "roadSign"
#define WORLD_LAST_ROAD_SIGN "lastRoadSign"

#define WORLD_MAP "map"
#define WORLD_CURRENT_TILE_POINT "tilePoint"
#define WORLD_NEXT_TILE_POINT "nextTilePoint"
#define WORLD_NEXT_NEXT_TILE_POINT "nextNextTilePoint"
#define WORLD_NEXT_NEXT_NEXT_TILE_POINT "nextNextNextTilePoint"

#define WORLD_ULTRASONICS "usStruct"

#define WORLD_INTERSECTION_STATE "intersectionState"
#define WORLD_ZEBRA_STATE "zebraState"

#define WORLD_PARKING_LOTS "parkingLot"
#define WORLD_TARGET_PARKING_LOT "targetParkingLot"

#define WORLD_OBSTACLES "obstacles"
#define THIS_IS_THE_POLICE "police" //Kuer

#define WORLD_IS_OFFROAD "isOffRoad"

//===============================================

#define CAR_STEERING "carSteering"
#define CAR_SPEED "carSpeed"

#define CAR_STATE "carState"
#define CAR_MANEUVER_ENTRY "maneuverEntry"

#define CAR_HEADLIGHTS "carHeadlights"
#define CAR_BREAKLIGHTS "carBreaklights"
#define CAR_HAZARDLIGHTS "carHazards"
#define CAR_TURNSIGNAL_LEFT "carTurnSignalLeft"
#define CAR_TURNSIGNAL_RIGHT "carTurnSignalRight"
#define CAR_REVERSELIGHTS "carReverselights"

#define CAR_AVOID_MOVE "avoidMove"

class WorldService : public ucom::cService {
ADTF_SERVICE_BEGIN(OID_WORLD_SERVICE, SERVICE_NAME)ADTF_EXPORT_INTERFACE(IID_SERVICE,
                                                                         ucom::IService)ADTF_EXPORT_INTERFACE(
                IID_WORLD_INTERFACE, WorldService)
    ADTF_SERVICE_END()

public:
    WorldService(const tChar *);

    virtual ~WorldService();

    tResult ServiceInit(IException **__exception_ptr = NULL);

    tResult ServiceShutdown(IException **__exception_ptr = NULL);

    tResult ServiceEvent(tInt eventId,
                         tUInt32 param1 = 0,
                         tUInt32 param2 = 0,
                         tVoid *pvData = NULL,
                         tInt szData = 0,
                         IException **__exception_ptr = NULL);

private:
    std::map<string, tWorldData> map;

public:
    void Clear() {
        for (auto &pair : map) {
            pair.second.lock->LockWrite();
            free(pair.second.data);
            pair.second.data = nullptr;
            pair.second.lock->UnlockWrite();
            delete (pair.second.lock);
            pair.second.lock = nullptr;
        }
        map.clear();
    }

    template<typename Type>
    tResult Push(const string &key, const Type &value) {
        auto val = map.find(key);
        if (val != map.end()) {
            val->second.lock->LockWrite();
            memcpy(val->second.data, &value, sizeof(Type));
            val->second.lock->UnlockWrite();
        } else {
            tWorldData entry{};
            entry.lock = new cReadWriteMutex();
            entry.lock->Create(100);
            entry.data = malloc(sizeof(Type));
            memcpy(entry.data, &value, sizeof(Type));
            map[key] = entry;
        }

        RETURN_NOERROR;
    }

    template<typename Type>
    tResult Pull(const string &key, Type &result) {
        auto val = map.find(key);
        if (val == map.end()) {
#ifdef DEBUG_WORLD_SERVICE_LOG
            LOG_INFO(cString::Format("No value for '%s'", key.c_str()).GetPtr());
#endif
            RETURN_ERROR(-1);
        }
        val->second.lock->LockRead();
        memcpy(&result, val->second.data, sizeof(Type));
        val->second.lock->UnlockRead();
        RETURN_NOERROR;
    }

    /**
     * Pushs a vector into Map. Warning DO NOT USE Vectors in Vectors
     * (e.g. vector<<vector> or <Struct which contains vector>> is not supported!
     * @tparam Type of Vector e.g. Struct or primary Type
     * @param key for Map Entry
     * @param vectorIn which contains elements of Type
     * @return Always Success
     */
    template<typename Type>
    tResult Push(const string &key, const vector<Type> &vectorIn) {
        auto val = map.find(key);
        if (val != map.end()) {
            val->second.lock->LockWrite();
            val->second.data = realloc(val->second.data, sizeof(Type) * vectorIn.size());
            //This copys only flat vectors not vector<vector>> -> Seg-Fault
            memcpy(val->second.data, &vectorIn[0], sizeof(Type) * vectorIn.size());
            val->second.size = vectorIn.size();
            val->second.lock->UnlockWrite();
        } else {
            tWorldData entry{};
            entry.lock = new cReadWriteMutex();
            entry.lock->Create(100);
            entry.data = malloc(sizeof(Type) * vectorIn.size());
            //This copys only flat vectors not vector<vector>> -> Seg-Fault
            memcpy(entry.data, &vectorIn[0], sizeof(Type) * vectorIn.size());
            entry.size = vectorIn.size();
            map[key] = entry;
        }
        RETURN_NOERROR;
    }

    /**
     * Pulls a vector from Map. Warning DO NOT USE Vectors in Vectors
     * (e.g. vector<Struct which contains vector> is not supported!
     * @tparam Type of Vector e.g. Struct or primary Type
     * @param key for Map Entry
     * @param vectorIn which contains elements of Type
     * @return -1 Not found or Found
     */
    template<typename Type>
    tResult Pull(const string &key, vector<Type> &resultOut) {
        auto val = map.find(key);
        if (val == map.end()) {
            RETURN_ERROR(-1);
        }
        val->second.lock->LockRead();
        resultOut.resize(val->second.size);
        //Fastest way to copy content!
        memcpy(&resultOut[0], val->second.data, sizeof(Type) * val->second.size);
        val->second.lock->UnlockRead();
        RETURN_NOERROR;
    }
};

#endif // _WORLD_SERVICE_CLASS_HEADER_
