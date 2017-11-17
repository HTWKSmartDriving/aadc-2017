#ifndef PROJECT_HTWKUTILS_H
#define PROJECT_HTWKUTILS_H

#include <sstream>
#include <vector>
#include <stdlib.h>
#include <adtf_utils.h>
#include "HTWKPoint.hpp"

class HTWKUtils {
public:
    static std::string toString(double value);

    static bool Equals(double f1, double f2, double delta);

    static bool Equals(HTWKPoint p1, HTWKPoint p2, double delta);

    template<typename value_type>
    static std::pair<value_type, value_type> GetNearestElements(const value_type &element,
                                                                const std::vector<value_type> &list) {
        if (list.empty())
            throw 0; // Can't help you if we're empty.

        typename std::vector<value_type>::const_iterator it = std::lower_bound(list.begin(), list.end(), element);

        if (it == list.end())
            return std::make_pair(list.back(), list.back());
        else if (it == list.begin())
            return std::make_pair(list.front(), list.front());
        else
            return std::make_pair(*(it - 1), *(it));
    };

    static void LogInfo(string str);

    static void LogWarn(string str);

    static void LogError(string str);
};

template<typename value_type>
class CircleBuffer {
private:
    int currentIndex;
    int bufferSize;

public:
    std::vector<value_type> buffer;

    CircleBuffer(int size) : buffer(size) {
        bufferSize = size;
        currentIndex = 0;
    }

    void Insert(value_type element) {
        buffer[currentIndex] = element;
        currentIndex = (currentIndex + 1) % bufferSize;
    }
};

#endif //PROJECT_HTWKUTILS_H
