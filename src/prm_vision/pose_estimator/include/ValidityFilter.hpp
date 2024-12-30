#ifndef _RESULT_FILTER_HPP_
#define _RESULT_FILTER_HPP_

#include <algorithm>
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

    bool isValid(float x, float y, float z, double dt);
    int get_lock_in_counter();

    ValidityFilterState state;

private:
    int lock_in_after = 3; // lock in after n frames
    int lock_in_counter = 0;

    int last_valid_time = 0; // sec

    float max_distance = 10000.f; // mm
    float min_distance = 10.f;    // mm

    double max_dt = 2000; // ms
    int prev_len = 5;     // check the back n frams for max shift distance vilolation

    float prev_x[20];
    float prev_y[20];
    float prev_z[20];
    int prev_idx = 0;

    float max_shift_distance = 150.f; // mm

    void update_prev(float, float, float);
    bool position_validity(float, float, float);
    bool distance_validity(float, float, float);
    void increment_lock_in_counter();
    void decrement_lock_in_counter();
    void reset_lock_in_counter();
};

#endif // _RESULT_FILTER_HPP_