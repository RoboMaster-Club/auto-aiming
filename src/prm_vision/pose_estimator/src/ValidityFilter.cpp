#include "ValidityFilter.hpp"

ValidityFilter::ValidityFilter(int lock_in_after, float max_distance, float min_distance, float max_shift_distance, float prev_len)
{
    this->lock_in_after = lock_in_after;
    this->max_distance = max_distance;
    this->min_distance = min_distance;
    this->max_shift_distance = max_shift_distance;
    this->prev_len = std::min(prev_len, 20.0f);

    // Initial state of the filter
    lock_in_counter = 0;
    prev_idx = 0;
    state = STOPPING;

    // Initialize arrays to track previous predictions
    for (int i = 0; i < this->prev_len; i++)
    {
        prev_x[i] = 0;
        prev_y[i] = 0;
        prev_z[i] = 0;
    }
}

ValidityFilter::ValidityFilter() {}
ValidityFilter::~ValidityFilter() {}

/**
 * @brief Check if the estimated pose is valid based on a validity filter.
 *
 * @param x The x-coordinate of the detected armor.
 * @param y The y-coordinate of the detected armor.
 * @param z The z-coordinate of the detected armor.
 * @return true If the pose is valid based on the validity filter and current aiming state.
 *
 * The return value is ONLY used to determine if we should reset the Kalman filter inside PoseEstimator.
 * We primarily use the state variable to determine the action to take (idle, track, stop).
 */
bool ValidityFilter::isValid(float x, float y, float z)
{
    // Calculate time difference since last valid detection
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_valid_time).count();

    // Check distance validity
    if (!distance_validity(x, y, z))
    {
        decrement_lock_in_counter();
        return state != STOPPING;
    }

    // Check time since last valid detection
    if (dt > max_dt)
    {
        reset_lock_in_counter();
        update_prev(x, y, z);
        return false; // Invalid because of time lapse
    }

    // Check position validity
    if (!position_validity(x, y, z))
    {
        decrement_lock_in_counter();
        update_prev(x, y, z);
        return state == STOPPING;
    }

    // Valid detection
    increment_lock_in_counter();
    update_prev(x, y, z);
    last_valid_time = now; // Update the last valid time

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
