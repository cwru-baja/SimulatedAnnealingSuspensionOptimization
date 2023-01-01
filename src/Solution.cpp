#include "Solution.h"
#include "trilaterate.h"

#define NO_SCRUB 1

namespace Solution {
    Solution::Solution() {

    };

    Solution::Solution(const Solution &other) {
        for (int i = 0; i < 6; i++) {
            points[i] = other.points[i];
        }
        travel = other.travel;
        geometry = other.geometry;
        goalCharacteristics = other.goalCharacteristics;

        wheelbase = other.wheelbase;
        track_width = other.track_width;
        for (int i = 0; i < 3; i++) {
            scrubs[i] = other.scrubs[i];
            caster_trails[i] = other.caster_trails[i];
        }
        ground_clearance = other.ground_clearance;
        motion_ratio = other.motion_ratio;
        recessional_travel = other.recessional_travel;
        camber_gain = other.camber_gain;
        kp_size = other.kp_size;
    }


    Solution::Solution(point3 points[6], double travel, Geometry_t geometry, GoalCharacteristics_t goalCharacteristics) {
        for (int i = 0; i < 6; i++) {
            this->points[i] = points[i];
        }

        this->travel = travel;
        this->geometry = geometry;
        this->goalCharacteristics = goalCharacteristics;
    }

    Solution::Solution(double * hyperpos, Geometry_t geometry, GoalCharacteristics_t goalCharacteristics) {
        point3 one = {hyperpos[2], hyperpos[2] * tan(hyperpos[0]), 0};
        point3 two = {hyperpos[3], hyperpos[3] * tan(hyperpos[0]), 0};
        point3 three = {hyperpos[4], hyperpos[5], hyperpos[6]};
        point3 four = {hyperpos[8], hyperpos[8] * tan(hyperpos[1]) + hyperpos[7], 0};
        point3 five = {hyperpos[9], hyperpos[9] * tan(hyperpos[1]) + hyperpos[7], 0};
        point3 six = {hyperpos[10], hyperpos[11], hyperpos[12]};

        this->points[0] = one;
        this->points[1] = two;
        this->points[2] = three;
        this->points[3] = four;
        this->points[4] = five;
        this->points[5] = six;

        this->travel = hyperpos[13];
        this->geometry = geometry;
        this->goalCharacteristics = goalCharacteristics;
    }

    void Solution::updateSolution() {
        double comp_travel = geometry.preload_ratio * travel;
        double ext_travel =  (1-geometry.preload_ratio) * travel;

        double  r_one  = (points[0] - points[2]).mag(),
                r_two  = (points[1] - points[2]).mag(),
                r_four = (points[3] - points[5]).mag(),
                r_five  = (points[4] - points[5]).mag();

        vec3 kp_ride = points[2] - points[5];
        kp_size = kp_ride.mag();

        vec3 three_ext = trilaterate(points[0], points[1], points[2], r_one, r_two, ext_travel, true);
        vec3 three_comp = trilaterate(points[0], points[1], points[2], r_one, r_two, comp_travel, false);

        vec3 six_ext = trilaterate(points[3], points[4], three_ext, r_four, r_five, kp_size, false);
        vec3 six_comp = trilaterate(points[3], points[4], three_comp, r_four, r_five, kp_size, false);

        vec3 kp_ext = three_ext - six_ext;
        vec3 kp_comp = three_comp - six_comp;

        #ifndef NO_SCRUB
        double ride_scrub = scrub(kp_ride);

        double ext_scrub = scrub(kp_ext);

        double comp_scrub = scrub(kp_comp);
        #endif

        vec3 centers[] = {(points[2] + points[5]) / 2, (three_ext + six_ext) / 2, (three_comp + six_comp) / 2};

        const double tire_lerp = 0.345;

        vec3 tire_centers[] = {(points[2] + (points[5] - points[2]) * tire_lerp), (three_ext + (six_ext - three_ext) * tire_lerp), (three_comp + (six_comp - three_comp) * tire_lerp)};

        double comp_slope = (centers[0].y() - centers[2].y()) / (centers[0].x() - centers[2].x());
        double ext_slope = (centers[1].y() - centers[0].y()) / (centers[1].x() - centers[0].x());

        double camber_ride = atan2(kp_ride.z(), kp_ride.y());
        double camber_comp = atan2(kp_comp.z(), kp_comp.y());

        wheelbase = geometry.front_to_back + (points[2].x() + points[5].x()) / 2;
        track_width = geometry.pedalbox_width + (points[2].z() + points[5].z()) + 2 * geometry.kingpin_to_hub;

        #ifndef NO_SCRUB
        scrubs[0] = ride_scrub;
        scrubs[1] = ext_scrub;
        scrubs[2] = comp_scrub;
        #endif


        caster_trails[0] = casterTrail(kp_ride, points[2]);
        caster_trails[1] = casterTrail(kp_ext, three_ext);
        caster_trails[2] = casterTrail(kp_comp, three_comp);
        ground_clearance = geometry.kingpin_to_ground - three_comp.y();
        motion_ratio = comp_slope / ext_slope;
        recessional_travel = tire_centers[0].x() - tire_centers[2].x();
        camber_gain = (camber_ride - camber_comp) * 180 / M_PI;
    }

    double Solution::scrub(vec3 kp) const {
        double z_comp = kp.z() / kp.mag();
        double y_comp = kp.y() / kp.mag();

        return geometry.kingpin_to_hub - (geometry.kingpin_to_ground * z_comp / y_comp);
    }

    double Solution::casterTrail(vec3 kp, vec3 three) const {
        double x_comp = kp.x() / kp.mag();
        double y_comp = kp.y() / kp.mag();

        return -(geometry.kingpin_to_ground + three.y()) * x_comp / y_comp;
    }

    double Solution::evalSolution() {
        double wse = 0.0;

        wse += goalCharacteristics.wheelbase_weight * pow(wheelbase - goalCharacteristics.ideal_wheelbase, 2);
        wse += goalCharacteristics.track_width_weight * pow(track_width - goalCharacteristics.ideal_track_width, 2);

        for (int i = 0; i < 3; ++i) {
            wse += goalCharacteristics.scrub_weights[i] * pow(scrubs[i] - goalCharacteristics.ideal_scrubs[i], 2);
            wse += goalCharacteristics.caster_trail_weights[i] * pow(caster_trails[i] - goalCharacteristics.ideal_caster_trails[i], 2);
        }

        wse += goalCharacteristics.ground_clearance_weight * pow(ground_clearance - goalCharacteristics.ideal_ground_clearance, 2);
        wse += goalCharacteristics.motion_ratio_weight * pow(motion_ratio - goalCharacteristics.ideal_motion_ratio, 2);
        wse += goalCharacteristics.recessional_travel_weight * pow(recessional_travel - goalCharacteristics.ideal_recessional_travel, 2);
        wse += goalCharacteristics.camber_gain_weight * pow(camber_gain - goalCharacteristics.ideal_camber_gain, 2);
        wse += goalCharacteristics.kp_size_weight * pow(kp_size - goalCharacteristics.ideal_kp_size, 2);

        return wse;
    }

    double Solution::evalSolutionMAE() {
        double wae = 0.0;

        wae += goalCharacteristics.wheelbase_weight * abs(wheelbase - goalCharacteristics.ideal_wheelbase);
        wae += goalCharacteristics.track_width_weight * abs(track_width - goalCharacteristics.ideal_track_width);

        for (int i = 0; i < 3; ++i) {
            wae += goalCharacteristics.scrub_weights[i] * abs(scrubs[i] - goalCharacteristics.ideal_scrubs[i]);
            wae += goalCharacteristics.caster_trail_weights[i] * abs(caster_trails[i] - goalCharacteristics.ideal_caster_trails[i]);
        }

        wae += goalCharacteristics.ground_clearance_weight * abs(ground_clearance - goalCharacteristics.ideal_ground_clearance);
        wae += goalCharacteristics.motion_ratio_weight * abs(motion_ratio - goalCharacteristics.ideal_motion_ratio);
        wae += goalCharacteristics.recessional_travel_weight * abs(recessional_travel - goalCharacteristics.ideal_recessional_travel);
        wae += goalCharacteristics.camber_gain_weight * abs(camber_gain - goalCharacteristics.ideal_camber_gain);
        wae += goalCharacteristics.kp_size_weight * abs(kp_size - goalCharacteristics.ideal_kp_size);

        return wae;
    }

    double Solution::getWheelbase() const {
        return wheelbase;
    }

    double Solution::getTrackWidth() const {
        return track_width;
    }

    const double *Solution::getScrubs() const {
        return scrubs;
    }

    const double *Solution::getCasterTrails() const {
        return caster_trails;
    }

    double Solution::getGroundClearance() const {
        return ground_clearance;
    }

    double Solution::getMotionRatio() const {
        return motion_ratio;
    }

    double Solution::getRecessionalTravel() const {
        return recessional_travel;
    }

    double Solution::getCamberGain() const {
        return camber_gain;
    }

    double * Solution::getHyperPos() {
        double *hyperPos = new double[14];

        hyperPos[0] = atan((points[1][1] - points[0][1]) / (points[1][0] - points[0][0]));
        hyperPos[1] = atan((points[4][1] - points[3][1]) / (points[4][0] - points[3][0]));

        hyperPos[2] = points[0][0];
        hyperPos[3] = points[1][0];

        hyperPos[4] = points[2][0];
        hyperPos[5] = points[2][1];
        hyperPos[6] = points[2][2];

        hyperPos[7] = points[3][1] - points[3][0]*(points[4][1] - points[3][1]) / (points[4][0] - points[3][0]);

        hyperPos[8] = points[3][0];
        hyperPos[9] = points[4][0];

        hyperPos[10] = points[5][0];
        hyperPos[11] = points[5][1];
        hyperPos[12] = points[5][2];

        hyperPos[13] = travel;

        return hyperPos;
    }

    void Solution::printPoints() {
        std::cout << "Points: " << std::endl;
        for (int i = 0; i < 6; ++i) {
            std::cout << points[i] << std::endl;
        }

        std::cout << "Travel " << travel << std::endl << std::endl;
    }

    void Solution::printSusValues() {
        std::cout << std::endl << "Suspension Values: " << std::endl;

        printf("Wheelbase ideal %.3f actual %.3f\n", goalCharacteristics.ideal_wheelbase, wheelbase);
        printf("Track Width ideal %.3f actual %.3f\n", goalCharacteristics.ideal_track_width, track_width);

        #ifndef NO_SCRUB
        printf("Scrubs: (Ride, Ext, Comp)\n");
        for (int i = 0; i < 3; ++i) {
            printf("ideal %.3f actual %.3f\n", goalCharacteristics.ideal_scrubs[i], scrubs[i]);
        }
        #endif

        printf("Caster Trails: (Ride, Ext, Comp)\n");

        for (int i = 0; i < 3; ++i) {
            printf("ideal %.3f actual %.3f\n", goalCharacteristics.ideal_caster_trails[i], caster_trails[i]);
        }

        printf("Ground Clearance ideal %.3f actual %.3f\n", goalCharacteristics.ideal_ground_clearance, ground_clearance);
        printf("Motion Ratio ideal %.3f actual %.3f\n", goalCharacteristics.ideal_motion_ratio, motion_ratio);
        printf("Recessional Travel ideal %.3f actual %.3f\n", goalCharacteristics.ideal_recessional_travel, recessional_travel);
        printf("Camber Gain ideal %.3f actual %.3f\n", goalCharacteristics.ideal_camber_gain, camber_gain);
        printf("Kp Size ideal %.3f actual %.3f\n", goalCharacteristics.ideal_kp_size, kp_size);
    }

    void Solution::saveToSldEqnFile(std::string filename) {
        double *pos = this->getHyperPos();

        std::ofstream eqnfile;
        eqnfile.open(filename , std::ios::out | std::ios::trunc);

        eqnfile << "\"Theta\"= " << pos[0] << "rad" << std::endl << std::endl;
        eqnfile << "\"Beta\"= " << pos[1] << "rad" << std::endl << std::endl;

        eqnfile << "\"XOne\"= " << pos[2] << "in" << std::endl << std::endl;
        eqnfile << "\"XTwo\"= " << pos[3] << "in" << std::endl << std::endl;

        eqnfile << "\"XThree\"= " << pos[4] << "in" << std::endl << std::endl;
        eqnfile << "\"YThree\"= " << -1 * pos[5] << "in" << std::endl << std::endl;
        eqnfile << "\"ZThree\"= " << pos[6] << "in" << std::endl << std::endl;

        eqnfile << "\"YFour\"= " << pos[7] << "in" << std::endl << std::endl;
        eqnfile << "\"XFour\"= " << pos[8] << "in" << std::endl << std::endl;
        eqnfile << "\"XFive\"= " << pos[9] << "in" << std::endl << std::endl;

        eqnfile << "\"XSix\"= " << pos[10] << "in" << std::endl << std::endl;
        eqnfile << "\"YSix\"= " << pos[11] << "in" << std::endl << std::endl;
        eqnfile << "\"ZSix\"= " << pos[12] << "in" << std::endl << std::endl;

        eqnfile << "\"Travel\"= " << pos[13] << "in" << std::endl << std::endl;

        eqnfile.close();

        delete[] pos;
    }
} // Solution
