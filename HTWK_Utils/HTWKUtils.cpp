#include "HTWKUtils.hpp"

std::string HTWKUtils::toString(double value) {
    std::ostringstream strs;
    strs << value;
    std::string valueStr = strs.str();
    return valueStr;
}

bool HTWKUtils::Equals(double f1, double f2, double delta) {
    return fabs(f1 - f2) <= delta;
}

bool HTWKUtils::Equals(HTWKPoint p1, HTWKPoint p2, double delta) {
    return Equals(p1.x(), p2.x(), delta) && Equals(p1.y(), p2.y(), delta);
}

void HTWKUtils::LogInfo(string str) {
    LOG_INFO(adtf_util::cString::Format(str.c_str()));
}

void HTWKUtils::LogWarn(string str) {
    LOG_WARNING(adtf_util::cString::Format(str.c_str()));
}

void HTWKUtils::LogError(string str) {
    LOG_ERROR(adtf_util::cString::Format(str.c_str()));
}