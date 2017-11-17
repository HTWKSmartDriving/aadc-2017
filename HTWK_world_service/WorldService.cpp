#include <WorldService.h>
#include "../../HTWK_Debug/EnableLogs.h"
ADTF_SERVICE_PLUGIN("World Service", OID_WORLD_SERVICE, WorldService)

WorldService::WorldService(const tChar *__info) : cService(__info) {
}

WorldService::~WorldService() {
}

tResult WorldService::ServiceInit(IException **__exception_ptr) {
    RETURN_IF_FAILED(cService::ServiceInit(__exception_ptr));

    Clear();

    RETURN_NOERROR;
}

tResult WorldService::ServiceShutdown(IException **__exception_ptr) {
    Clear();

    return cService::ServiceShutdown(__exception_ptr);
}

tResult WorldService::ServiceEvent(tInt eventId, tUInt32 param1, tUInt32 param2, tVoid *pvData, tInt szData,
                                   IException **__exception_ptr) {
    return cService::ServiceEvent(eventId, param1, param2, pvData, szData, __exception_ptr);
}

