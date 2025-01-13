#include "PitchLookupModel.hpp"

PitchLookupModel::PitchLookupModel() {}

PitchLookupModel::PitchLookupModel(std::string filename) {
    this->load_file(filename);
}

/**
 * @brief Loads lookup table from file
 *
 * The first line of the file contains two space-separated integers: M, N,
 * where M represents the number of x coordinates (distance from the camera)
 * and N represents the number of z coordinates (height above the ground). 
 * The next M x N lines contain three space-separated numbers:
 * x coordinate, z coordinate, and pitch. Length measurements are in
 * millimeters, and angle measurements are in degrees.
 */
void PitchLookupModel::load_file(std::string filename) {
    FILE* fp = fopen(filename.c_str(), "r");
    if (fp == nullptr) {
        return;
    }

    int x_count = 0;
    int z_count = 0;
    fscanf(fp, "%d %d\n", &x_count, &z_count);

    this->pitch_lookup_table = std::vector<std::vector<float>>(
        x_count, std::vector<float>(z_count, 0));

    for (int row = 0; row < x_count; row++) {
        for (int col = 0; col < z_count; col++) {
            int x = 0, z = 0;
            float pitch = 0;
            int read_count = fscanf(fp, "%d %d %f\n", &x, &z, &pitch);

            if (read_count != 3) {
                fclose(fp);
                return;
            }

            if (x < this->lower_x) this->lower_x = x;
            if (x > this->upper_x) this->upper_x = x;
            if (z < this->lower_z) this->lower_z = z;
            if (z > this->upper_z) this->upper_z = z;

            this->pitch_lookup_table[row][col] = pitch;
        }
    }

    fclose(fp);
}

/**
 * @brief Lookup the corresponding pitch with distance and height
 *
 * @param distance Distance away from the camera in millimeters
 * @param height   Height above the ground in millimeters
 *
 * @return Pitch in degrees
 */
float PitchLookupModel::get_pitch(int distance, int height) {
    // Calculate the step size of the lookup table
    int step_size = (this->upper_x - this->lower_x) / 
                    (this->pitch_lookup_table.size() - 1);

    // Calculate the indices of the tightest bound containing distance and height
    int lower_step_x_idx = (distance - this->lower_x) / step_size;
    int upper_step_x_idx = (distance - this->lower_x + step_size - 1) / step_size;
    int lower_step_z_idx = (height - this->lower_z) / step_size;
    int upper_step_z_idx = (height - this->lower_z + step_size - 1) / step_size;

    // Calculate the actual values of the tightest bounds
    int lower_step_x = this->lower_x + step_size * lower_step_x_idx;
    int upper_step_x = this->lower_x + step_size * upper_step_x_idx;
    int lower_step_z = this->lower_z + step_size * lower_step_z_idx;
    int upper_step_z = this->lower_z + step_size * upper_step_z_idx;

    // Calculate the pitch of the four points with tightest bounds
    float pitch_low_low = this->pitch_lookup_table[lower_step_x_idx][lower_step_z_idx];
    float pitch_low_high = this->pitch_lookup_table[lower_step_x_idx][upper_step_z_idx];
    float pitch_high_low = this->pitch_lookup_table[upper_step_x_idx][lower_step_z_idx];
    float pitch_high_high = this->pitch_lookup_table[upper_step_x_idx][upper_step_z_idx];

    // Calculate pitch of the corresponding x position by fixing z to the lower bound
    float pitch_z_low = this->map(
        static_cast<float>(distance), 
        static_cast<float>(lower_step_x), 
        static_cast<float>(upper_step_x), 
        std::fmin(pitch_low_low, pitch_high_low), 
        std::fmax(pitch_low_low, pitch_high_low));

    // Calculate pitch of the corresponding x position by fixing z to the upper bound
    float pitch_z_high = this->map(
        static_cast<float>(distance), 
        static_cast<float>(lower_step_x), 
        static_cast<float>(upper_step_x), 
        std::fmin(pitch_low_high, pitch_high_high), 
        std::fmax(pitch_low_high, pitch_high_high));

    // Combine the calculated new bounds to calculate the new pitch
    return this->map(
        static_cast<float>(height), 
        static_cast<float>(lower_step_z), 
        static_cast<float>(upper_step_z), 
        std::fmin(pitch_z_low, pitch_z_high), 
        std::fmax(pitch_z_low, pitch_z_high));
}

/**
 * @brief Maps a value from a range to another range
 *
 * @param value   Value to be mapped
 * @param old_min The minimum of the old range
 * @param old_max The maximum of the old range
 * @param new_min The minimum of the new range
 * @param new_max The maximum of the new range
 *
 * @return Mapped value
 */
float PitchLookupModel::map(float value, float old_min, float old_max, 
                            float new_min, float new_max) {
    if (old_min > old_max || new_min > new_max) {
        return -1;
    }

    if (old_min == old_max) {
        return (new_min == new_max) ? new_min : -1;
    }

    return new_min + (value - old_min) * (new_max - new_min) / (old_max - old_min);
}
