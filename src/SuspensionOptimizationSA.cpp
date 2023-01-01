#include "SimulatedAnnealing.h"
#include <stdio.h>
#include <json_struct.h>

JS_OBJ_EXT(Geometry_t, tire_radius, kingpin_to_hub, kingpin_to_ground, front_to_back, pedalbox_width, preload_ratio);
JS_OBJ_EXT(SuspensionPoints_t, theta, beta, x_one, x_two, x_three, y_three, z_three, upper_y, x_four, x_five, x_six, y_six, z_six, travel);
JS_OBJ_EXT(GoalCharacteristics_t, ideal_wheelbase, ideal_track_width, ideal_scrubs, ideal_caster_trails, ideal_ground_clearance, ideal_motion_ratio, ideal_recessional_travel, ideal_camber_gain, ideal_kp_size, wheelbase_weight, track_width_weight, scrub_weights, caster_trail_weights, ground_clearance_weight, motion_ratio_weight, recessional_travel_weight, camber_gain_weight, kp_size_weight);

const int input_dims = 14;

Geometry_t loadCarGeometry(std::string geometryFile) {
    std::ifstream file_stream(geometryFile);
    std::stringstream buffer;
    buffer << file_stream.rdbuf();
    std::string geometryStr = buffer.str();
    file_stream.close();

    JS::ParseContext parseContext(geometryStr);
    Geometry_t geometry;
    if (parseContext.parseTo(geometry) != JS::Error::NoError)
    {   
        std::string errorStr = parseContext.makeErrorString();
        std::cout << "Error parsing Car Geometry: " << errorStr << std::endl;
        exit(1);
    }

    return geometry;
}

GoalCharacteristics_t loadGoalCharacteristics(std::string goalFile) {
    std::ifstream file_stream(goalFile);
    std::stringstream buffer;
    buffer << file_stream.rdbuf();
    std::string goalStr = buffer.str();
    file_stream.close();

    JS::ParseContext parseContext(goalStr);

    GoalCharacteristics_t goal;
    if (parseContext.parseTo(goal) != JS::Error::NoError)
    {
        std::string errorStr = parseContext.makeErrorString();
        std::cout << "Error parsing Goal Characteristics: " << errorStr << std::endl;
        exit(1);
    }

    return goal;
}

double * loadHyperPoint(std::string hyperPointFile) {
    std::ifstream file_stream(hyperPointFile);
    std::stringstream buffer;
    buffer << file_stream.rdbuf();
    std::string hyperPointStr = buffer.str();
    file_stream.close();

    JS::ParseContext parseContext(hyperPointStr);
    SuspensionPoints_t suspensionPoints;
    if (parseContext.parseTo(suspensionPoints) != JS::Error::NoError)
    {   
        std::string errorStr = parseContext.makeErrorString();
        std::cout << "Error parsing " << hyperPointFile << ": " << errorStr << std::endl;
        exit(1);
    }

    double * hyperPoint = new double[input_dims];

    hyperPoint[0] = suspensionPoints.theta * M_PI / 180.0;
    hyperPoint[1] = suspensionPoints.beta * M_PI / 180.0;
    hyperPoint[2] = suspensionPoints.x_one;
    hyperPoint[3] = suspensionPoints.x_two;
    hyperPoint[4] = suspensionPoints.x_three;
    hyperPoint[5] = suspensionPoints.y_three;
    hyperPoint[6] = suspensionPoints.z_three;
    hyperPoint[7] = suspensionPoints.upper_y;
    hyperPoint[8] = suspensionPoints.x_four;
    hyperPoint[9] = suspensionPoints.x_five;
    hyperPoint[10] = suspensionPoints.x_six;
    hyperPoint[11] = suspensionPoints.y_six;
    hyperPoint[12] = suspensionPoints.z_six;
    hyperPoint[13] = suspensionPoints.travel;

    return hyperPoint;
}

void saveHyperPointToJson(double * hyperPoint, std::string fileName) {
    SuspensionPoints_t suspensionPoints;

    suspensionPoints.theta = hyperPoint[0] * 180.0 / M_PI;
    suspensionPoints.beta = hyperPoint[1] * 180.0 / M_PI;
    suspensionPoints.x_one = hyperPoint[2];
    suspensionPoints.x_two = hyperPoint[3];
    suspensionPoints.x_three = hyperPoint[4];
    suspensionPoints.y_three = hyperPoint[5];
    suspensionPoints.z_three = hyperPoint[6];
    suspensionPoints.upper_y = hyperPoint[7];
    suspensionPoints.x_four = hyperPoint[8];
    suspensionPoints.x_five = hyperPoint[9];
    suspensionPoints.x_six = hyperPoint[10];
    suspensionPoints.y_six = hyperPoint[11];
    suspensionPoints.z_six = hyperPoint[12];
    suspensionPoints.travel = hyperPoint[13];

    std::ofstream file_stream(fileName);
    file_stream << JS::serializeStruct(suspensionPoints);
    file_stream.close();
}

void annealToSolution(SimulatedAnnealing *sa) {
    Solution::Solution best = sa->anneal(600000, 0.5, 0.9999, 1.0 - 1e-5);

    std::cout << std::endl << "Best Solution: " << best.evalSolution() << std::endl;
    std::cout << "Best Solution MAE: " << best.evalSolutionMAE() << std::endl;
    sa->checkBestForClipping();
    best.printSusValues();
    best.saveToSldEqnFile("best.txt");

    double * bestHyperPoint = best.getHyperPos();
    saveHyperPointToJson(bestHyperPoint, "best.json");
    delete[] bestHyperPoint;

    std::cout << std::endl;
}

void checkHyperPos(double * hyperPos, SimulatedAnnealing *sa, std::string name) {
    Solution::Solution benchmark = sa->anneal(hyperPos, 0, 0.0001, 1e-50, 1.0, 1.0, 0);

    std::cout << std::endl << name << ": " << benchmark.evalSolution() << std::endl;
    std::cout << name << " MAE: " << benchmark.evalSolutionMAE() << std::endl;
    sa->checkBestForClipping();
    benchmark.printSusValues();
    benchmark.saveToSldEqnFile(name + ".txt");
    std::cout << std::endl << std::endl;
}

void hillClimbPos(double * hyperPos, SimulatedAnnealing *sa, std::string name) {
    Solution::Solution benchmark_clambered = sa->anneal(hyperPos, 500, 0.0001, 1e-50, 1.0, 0.999, 0);

    std::cout << std::endl << name << " Clambered: " << benchmark_clambered.evalSolution() << std::endl;
    std::cout << name << " Clambered MAE: " << benchmark_clambered.evalSolutionMAE() << std::endl;
    sa->checkBestForClipping();
    benchmark_clambered.printPoints();
    benchmark_clambered.printSusValues();
    benchmark_clambered.saveToSldEqnFile(name + "_clambered.txt");

    double * optimizedHyperPos = benchmark_clambered.getHyperPos();
    saveHyperPointToJson(optimizedHyperPos, name + "_clambered.json");
    delete[] optimizedHyperPos;

    std::cout << std::endl << std::endl;
}


int main(int argc, char **argv) {
    std::string geometryFile = "../car_geometry.json";
    std::string goalFile = "../goal_characteristics.json";
    std::string centerFile = "../search_centers.json";
    std::string radsFile = "../search_radii.json";

    char ** benchmarkFiles = (char **) malloc(sizeof(char *) * argc);

    unsigned int numBenchmarks = 0;

    for (int ind = 0; ind < argc; ind++) {
        if (strcmp(argv[ind], "-g") == 0) {
            geometryFile = argv[++ind];
        } else if (strcmp(argv[ind], "-c") == 0 && argc - ind > 1) {
            centerFile = argv[++ind];
        } else if (strcmp(argv[ind], "-r") == 0 && argc - ind > 1) {
            radsFile = argv[++ind];
        } else if (strcmp(argv[ind], "-b") == 0 && argc - ind > 1) {
            benchmarkFiles[numBenchmarks] = argv[++ind];
            numBenchmarks++;
        } else if (strcmp(argv[ind], "-t") == 0 && argc - ind > 1) {
            goalFile = argv[++ind];
        } else if (strcmp(argv[ind], "-h") == 0) {
            std::cout << "Usage: " << argv[0] << " [-g geometry_file] [-c center_file] [-r radius_file] [-t target_characteristics_file] [-b benchmark_file]" << std::endl;
            exit(0);
        }
    }



    Geometry_t geometry = loadCarGeometry(geometryFile);

    double * center = loadHyperPoint(centerFile);
    double * rads = loadHyperPoint(radsFile);

    GoalCharacteristics_t goal = loadGoalCharacteristics(goalFile);


    std::cout << "Suspension Optimiztion Running" << std::endl;
    SimulatedAnnealing sa(center, rads, input_dims, geometry, goal);

    // Simulated Annealing produces an optimal solution within bounds
    annealToSolution(&sa);

    for (int bench = 0; bench < numBenchmarks; bench++) {
        char * benchPath = benchmarkFiles[bench];
        double * benchmark = loadHyperPoint(benchPath);

        // Check an existing set of geometry points
        checkHyperPos(benchmark, &sa, benchPath);
        // Locally optimize an existing solution
        hillClimbPos(benchmark, &sa, benchPath);

        delete[] benchmark;
    }

    delete[] center;
    delete[] rads;
    free(benchmarkFiles);
}

