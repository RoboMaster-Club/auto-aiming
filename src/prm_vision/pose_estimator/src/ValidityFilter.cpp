#include "ValidityFilter.hpp"
#include <cmath>
#include <algorithm>

ValidityFilter::ValidityFilter(int lock_in_after, float max_distance, float min_distance, float max_shift_distance, float prev_len)
{
    this->lock_in_after = lock_in_after;
    this->max_distance = max_distance;
    this->min_distance = min_distance;
    this->max_shift_distance = max_shift_distance;
    this->prev_len = std::min(prev_len, 20.0f);

    last_valid_time = 0;
    lock_in_counter = 0;
    prev_idx = 0;
    state = STOPPING;

    // Initialize prev arrays
    for (int i = 0; i < this->prev_len; i++)
    {
        prev_x[i] = 0;
        prev_y[i] = 0;
        prev_z[i] = 0;
    }
}

ValidityFilter::ValidityFilter() {}
ValidityFilter::~ValidityFilter() {}

bool ValidityFilter::isValid(float x, float y, float z, double dt)
{
    // Check distance validity
    if (!distance_validity(x, y, z))
    {
        decrement_lock_in_counter();
        return state == STOPPING;
    }

    // Check time since last valid detection
    if (dt > max_dt)
    {
        reset_lock_in_counter();
        update_prev(x, y, z);
        return false; // Invalid because of time lapse
    }

    // Check position validity
    int num_valid = position_validity(x, y, z);
    if (num_valid == 0)
    {
        decrement_lock_in_counter();
        update_prev(x, y, z);
        return state == STOPPING;
    }

    // Update state based on detection confidence
    increment_lock_in_counter();
    update_prev(x, y, z);

    return state == TRACKING;
}

void ValidityFilter::update_prev(float x, float y, float z)
{
    prev_x[prev_idx] = x;
    prev_y[prev_idx] = y;
    prev_z[prev_idx] = z;
    prev_idx = (prev_idx + 1) % prev_len;
}

bool ValidityFilter::distance_validity(float x, float y, float z)
{
    float dst = std::sqrt(x * x + y * y + z * z);
    return (dst <= max_distance && dst >= min_distance);
}

bool ValidityFilter::position_validity(float x, float y, float z)
{
    for (int i = 0; i < prev_len; i++)
    {
        float dx = x - prev_x[i];
        float dy = y - prev_y[i];
        float dz = z - prev_z[i];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < max_shift_distance)
        {
            return true; // Valid as soon as one match is found
        }
    }
    return false; // No valid matches found
}

void ValidityFilter::increment_lock_in_counter()
{
    if (++lock_in_counter >= lock_in_after)
    {
        state = TRACKING;
        lock_in_counter = lock_in_after; // Cap at max
    }
    else
    {
        state = IDLING;
    }
}

void ValidityFilter::decrement_lock_in_counter()
{
    if (--lock_in_counter <= 0)
    {
        state = STOPPING;
        lock_in_counter = 0; // Ensure no negative values
    }
    else
    {
        state = IDLING;
    }
}

void ValidityFilter::reset_lock_in_counter()
{
    lock_in_counter = 0;
    state = STOPPING;
}

int ValidityFilter::get_lock_in_counter()
{
    return lock_in_counter;
}
