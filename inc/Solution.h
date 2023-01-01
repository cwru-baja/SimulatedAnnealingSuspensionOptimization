#ifndef SUSPENSIONOPTIMIZATION_SOLUTION_H
#define SUSPENSIONOPTIMIZATION_SOLUTION_H

#include <random>
#include "vec3.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <fstream>

struct Geometry_t{
    double tire_radius;
    double kingpin_to_hub;
    double kingpin_to_ground;
    double front_to_back;
    double pedalbox_width;
    double preload_ratio;
};

struct SuspensionPoints_t{
    double theta, beta, x_one, x_two, x_three, y_three, z_three;
    double upper_y, x_four, x_five, x_six, y_six, z_six, travel;
};

struct GoalCharacteristics_t{
    double ideal_wheelbase;
    double ideal_track_width;
    std::vector<double> ideal_scrubs;
    std::vector<double> ideal_caster_trails;
    double ideal_ground_clearance;
    double ideal_motion_ratio;
    double ideal_recessional_travel;
    double ideal_camber_gain;
    double ideal_kp_size;

    double wheelbase_weight;
    double track_width_weight;
    std::vector<double> scrub_weights;
    std::vector<double> caster_trail_weights;
    double ground_clearance_weight;
    double motion_ratio_weight;
    double recessional_travel_weight;
    double camber_gain_weight;
    double kp_size_weight;
};

namespace Solution {

    class Solution {
    private:
        point3 points[6];
        double travel;
        Geometry_t geometry;
        double wheelbase;

    public:
        double getWheelbase() const;
        double getTrackWidth() const;
        const double *getScrubs() const;
        const double *getCasterTrails() const;
        double getGroundClearance() const;
        double getMotionRatio() const;
        double getRecessionalTravel() const;
        double getCamberGain() const;
        double *getHyperPos();

    private:
        double track_width;
        double scrubs[3];
        double caster_trails[3];
        double ground_clearance;
        double motion_ratio;
        double recessional_travel;
        double camber_gain;
        double kp_size;

        GoalCharacteristics_t goalCharacteristics;

        double scrub(vec3 kp) const;
        double casterTrail(vec3 kp, vec3 three) const;

    public:
        Solution();
        Solution(double * hyperpos, Geometry_t geometry, GoalCharacteristics_t goalCharacteristics);
        Solution(point3 points[6], double travel, Geometry_t geometry, GoalCharacteristics_t goalCharacteristics);
        Solution(const Solution &other);
        void updateSolution();
        double evalSolution();
        double evalSolutionMAE();
        void printPoints();

        void printSusValues();

        void saveToSldEqnFile(std::string filename);
    };
}

#endif //SUSPENSIONOPTIMIZATION_SOLUTION_H
