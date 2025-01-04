#include <climits>
#include <iostream>
#include <vector>
#include <cmath>

class PitchLookupModel {
    public:
        PitchLookupModel();
        PitchLookupModel(std::string filename);
        
        void load_file(std::string filename);
        float get_pitch(int distance, int height);
    private:
        int lower_x = INT_MAX;
        int lower_z = INT_MAX;
        int upper_x = INT_MIN;
        int upper_z = INT_MIN;
        std::vector<std::vector<float>> pitch_lookup_table;

        float map(float value, float old_min, float old_max, float new_min, float new_max);
};