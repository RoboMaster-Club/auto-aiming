#include "PitchLookupModel.hpp"

PitchLookupModel::PitchLookupModel() {}

PitchLookupModel::PitchLookupModel(std::string filename) {
    this->filename = filename;
    this->load_file();
}

/**
 * @brief Loads lookup table from file
 *
 * The first line of the file contains two space-separated integers: M, N,
 * where M represents the number of z coordinates (distance from the camera)
 * and N represents the number of y coordinates (height above the ground). 
 * The next M x N lines contain three space-separated numbers:
 * z coordinate, y coordinate, and pitch. Length measurements are in
 * millimeters, and angle measurements are in degrees.
 */
int PitchLookupModel::load_file() {
    FILE* fp = fopen(filename.c_str(), "r");
    if (fp == nullptr) {
        return -1;
    }

    int y_count = 0;
    int z_count = 0;
    fscanf(fp, "%d %d\n", &z_count, &y_count);

    this->pitch_lookup_table = std::vector<std::vector<float>>(
        z_count, std::vector<float>(y_count, 0));

    for (int row = 0; row < z_count; row++) {
        for (int col = 0; col < y_count; col++) {
            int z = 0, y = 0;
            float pitch = 0;
            int read_count = fscanf(fp, "%d %d %f\n", &z, &y, &pitch);

            if (read_count != 3) {
                fclose(fp);
                fp = NULL;
                return -2;
            }

            if (z < this->lower_z) this->lower_z = z;
            if (z > this->upper_z) this->upper_z = z;
            if (y < this->lower_y) this->lower_y = y;
            if (y > this->upper_y) this->upper_y = y;

            this->pitch_lookup_table[row][col] = pitch;
        }
    }

    fclose(fp);
    fp = NULL;
    return 1;
}

/**
 * @brief Writes the lookup table to a file
 * 
 * @param filename Name of the file to write the lookup table to
 */

void PitchLookupModel::write_file() {
    FILE* fp = fopen(filename.c_str(), "w");
    if (fp == nullptr) {
        return;
    }

    int z_count = this->pitch_lookup_table.size();
    int y_count = this->pitch_lookup_table[0].size();

    fprintf(fp, "%d %d\n", z_count, y_count);

    for (int row = 0; row < z_count; row++) {
        for (int col = 0; col < y_count; col++) {
            fprintf(fp, "%d %d %f\n", row, col, this->pitch_lookup_table[row][col]);
        }
    }

    fclose(fp);
    fp = NULL;
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
    int step_size = (this->upper_z - this->lower_z) / 
                    (this->pitch_lookup_table.size() - 1);

    // Calculate the indices of the tightest bound containing distance and height
    int lower_step_z_idx = (distance - this->lower_z) / step_size;
    int upper_step_z_idx = (distance - this->lower_z + step_size - 1) / step_size;
    int lower_step_y_idx = (height - this->lower_y) / step_size;
    int upper_step_y_idx = (height - this->lower_y + step_size - 1) / step_size;

    // Calculate the actual values of the tightest bounds
    int lower_step_z = this->lower_z + step_size * lower_step_z_idx;
    int upper_step_z = this->lower_z + step_size * upper_step_z_idx;
    int lower_step_y = this->lower_y + step_size * lower_step_y_idx;
    int upper_step_y = this->lower_y + step_size * upper_step_y_idx;

    // Calculate the pitch of the four points with tightest bounds
    float pitch_low_low = this->pitch_lookup_table[lower_step_z_idx][lower_step_y_idx];
    float pitch_low_high = this->pitch_lookup_table[lower_step_z_idx][upper_step_y_idx];
    float pitch_high_low = this->pitch_lookup_table[upper_step_z_idx][lower_step_y_idx];
    float pitch_high_high = this->pitch_lookup_table[upper_step_z_idx][upper_step_y_idx];

    // Calculate pitch of the corresponding z position by fixing y to the lower bound
    float pitch_y_low = this->map(
        static_cast<float>(distance), 
        static_cast<float>(lower_step_z), 
        static_cast<float>(upper_step_z), 
        std::fmin(pitch_low_low, pitch_high_low), 
        std::fmax(pitch_low_low, pitch_high_low));

    // Calculate pitch of the corresponding z position by fixing y to the upper bound
    float pitch_y_high = this->map(
        static_cast<float>(distance), 
        static_cast<float>(lower_step_z), 
        static_cast<float>(upper_step_z), 
        std::fmin(pitch_low_high, pitch_high_high), 
        std::fmax(pitch_low_high, pitch_high_high));

    assert(this->pitch_lookup_table.size() > 1);
    assert(this->pitch_lookup_table[0].size() > 1);
    assert(upper_z != lower_z);
    assert(step_size != 0);

    // Combine the calculated new bounds to calculate the new pitch
    return this->map(
        static_cast<float>(height), 
        static_cast<float>(lower_step_y), 
        static_cast<float>(upper_step_y), 
        std::fmin(pitch_y_low, pitch_y_high), 
        std::fmax(pitch_y_low, pitch_y_high));
}

/**
 * @brief Maps a value from a range to another range
 *
 * @param value   Value to be mapped
 * @param old_min The minimum of the old range
 * @param old_may The maximum of the old range
 * @param new_min The minimum of the new range
 * @param new_may The maximum of the new range
 *
 * @return Mapped value
 */
float PitchLookupModel::map(float value, float old_min, float old_may, 
                            float new_min, float new_may) {
    if (old_min > old_may || new_min > new_may) {
        return -1;
    }

    if (old_min == old_may) {
        return (new_min == new_may) ? new_min : -1;
    }

    return new_min + (value - old_min) * (new_may - new_min) / (old_may - old_min);
}