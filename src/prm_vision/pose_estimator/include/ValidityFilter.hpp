#ifndef _RESULT_FILTER_HPP_
#define _RESULT_FILTER_HPP_

#include <algorithm>
#include <chrono>
#include <limits>
#include <cmath>
#include <stdio.h>

enum ValidityFilterState
{
    TRACKING,
    IDLING,
    STOPPING,
};

class ValidityFilter
{
public:
    ValidityFilter(int lock_in_after, float max_distance, float min_distance, float max_shift_distance, float prev_len);

    ValidityFilter();
    ~ValidityFilter();

    bool shouldResetKalman(float x, float y, float z);
    bool positionValidity(float, float, float);
    bool distanceValidity(float, float, float);
    void updatePrev(float, float, float);
    int getLockInCounter();

    ValidityFilterState state = STOPPING;
    double _max_dt = 2000; // ms

    // Getters
    float *getPrevX() { return _prev_x; }
    float *getPrevY() { return _prev_y; }
    float *getPrevZ() { return _prev_z; }

protected:
    int _lock_in_after = 3; // lock in after n frames
    int _lock_in_counter = 0;

    // zero time point
    std::chrono::steady_clock::time_point _last_valid_time = std::chrono::steady_clock::time_point::min();

    float _max_distance = 10000.f; // mm
    float _min_distance = 10.f;    // mm

    int _prev_len = 5; // check the back n frames for max shift distance vilolation

    float _prev_x[20];
    float _prev_y[20];
    float _prev_z[20];
    int _prev_idx = 0;

    float _max_shift_distance = 150.f; // mm

    void incrementLockInCounter();
    void decrementLockInCounter();
    void resetLockInCounter();
};

#endif // _RESULT_FILTER_HPP_