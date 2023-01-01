#ifndef SUSPENSIONOPTIMIZATION_SIMULATEDANNEALING_H
#define SUSPENSIONOPTIMIZATION_SIMULATEDANNEALING_H

#include "Solution.h"

class SimulatedAnnealing {
    private:
        Solution::Solution bestSolution;
        double bestCost;
        double temp;
        double radGain;
        double radGainDecay;
        std::default_random_engine randomEngine;
        double * hypercenter;
        double * hyperrads;
        int hyperdim;
        Geometry_t geometryConfig;
        GoalCharacteristics_t goalConfig;
    public:
        void generatePos(double *pos, double *nextPos);
        void getEnergy(Solution::Solution solution);
        SimulatedAnnealing(double * hypercenter, double * hyperrads, int hyperdim, Geometry_t geometryConfig, GoalCharacteristics_t goalConfig);
        Solution::Solution anneal(int iterations, double radGain, double tempRet, double radGainDecay);
        Solution::Solution anneal(double * hyperpos, int iterations, double radGain, double startTemp, double tempRet, double radGainDecay, int debug);
        void posCopy(double *from, double *to);

        void checkForClipping(Solution::Solution solution);
        void checkBestForClipping();
};


#endif //SUSPENSIONOPTIMIZATION_SIMULATEDANNEALING_H
