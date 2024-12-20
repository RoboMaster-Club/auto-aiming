#include "ValidityFilter.hpp"

ValidityFilter::ValidityFilter(int lock_in_after, float max_distance, float min_distance, float max_shift_distance, float prev_len)
{
    this->lock_in_after = lock_in_after;
    this->max_distance = max_distance;
    this->min_distance = min_distance;
    this->max_shift_distance = max_shift_distance;
    this->prev_len = prev_len < 20 ? prev_len : 20;
}

ValidityFilter::ValidityFilter()
{
}

ValidityFilter::~ValidityFilter()
{
}

/**
 * @brief Get the lock in counter
 * 
 * @returns number of valid detections
 */
int ValidityFilter::get_lock_in_counter()
{
    return lock_in_counter;
}

/**
 * @brief Updates the coordinates of previous detections
 * 
 * @param x x coordinate
 * @param y y coordinate
 * @param z z coordinate
 */
void ValidityFilter::update_prev(float x, float y, float z)
{
    this->prev_coordinates[prev_idx][0] = x;
    this->prev_coordinates[prev_idx][1] = y;
    this->prev_coordinates[prev_idx][2] = z;
    prev_idx = (prev_idx + 1) % prev_len;
}

/**
 * @brief Check if the distance of the detection is valid
 * 
 * @param x x coordinate
 * @param y y coordinate
 * @param z z coordinate
 */
bool ValidityFilter::distance_validity(float x, float y, float z)
{
    float dst = std::sqrt(x * x + y * y + z * z);
    return dst <= max_distance && dst >= min_distance;
}

/**
 * @brief Check if the position of the detection is valid
 * 
 * The function compare previous detections with the current detection,
 * and count the previous detections that is significantly far from
 * the current detection.
 * 
 * @param x x coordinate
 * @param y y coordinate
 * @param z z coordinate
 */
int ValidityFilter::position_validity(float x, float y, float z)
{
    int num_valid = 0;

    for (int i = 0; i < prev_len; i++)
    {
        float dx = x - this->prev_coordinates[i][0];
        float dy = y - this->prev_coordinates[i][1];
        float dz = z - this->prev_coordinates[i][2];
        if (std::sqrt(dx * dx + dy * dy + dz * dz) < max_shift_distance)
        {
            num_valid++;
        }
    }
    return num_valid;
}

void ValidityFilter::increment_lock_in_counter()
{
    lock_in_counter++;
    if (lock_in_counter > lock_in_after)
    {
        state = TRACKING;
        lock_in_counter = lock_in_after;
    } else {
        state = IDLING;
    }
}

void ValidityFilter::decrement_lock_in_counter()
{
    lock_in_counter--;
    if (lock_in_counter != 0)
    {
        state = IDLING;
    }
    else
    {
        state = STOPPING;
        lock_in_counter = 0;
    }
}

void ValidityFilter::reset_lock_in_counter()
{
    lock_in_counter = 0;
    state = STOPPING;
}

/**
 * @brief Check if whether to reset the kalman filter
 * 
 * @param x x coordinate
 * @param y y coordinate
 * @param z z coordinate
 * @param dt delta time
 * 
 * @returns whether to reset the kalman filter
 */
bool ValidityFilter::validation(float x, float y, float z, double dt)
{

    // Invalid distance
    if (!distance_validity(x, y, z))
    {
        decrement_lock_in_counter();
        return state == STOPPING;
    }

    // New detection based on time
    if (dt > max_dt)
    {
        reset_lock_in_counter();
        increment_lock_in_counter();
        update_prev(x, y, z);
        return true;
    }

    int num_valid = position_validity(x, y, z);

    // Invalid detection
    if (num_valid == 0)
    {
        decrement_lock_in_counter();
        update_prev(x, y, z);
        return state == STOPPING;
    }

    // Valid detection
    else
    {
        increment_lock_in_counter();
        update_prev(x, y, z);
        return false;
    }

    // New detection based on overwriting prev positions
    if (num_valid > prev_len * 0.3)
    {
        increment_lock_in_counter();
        update_prev(x, y, z);
        return true;
    }
}