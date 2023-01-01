#include <random>
#include "SimulatedAnnealing.h"

/*
Search Space - 14D HyperSolid
Each dimension has a center and a radius

Solution Function takes in a point in search space
computes the cost
returns the cost

Generate a new point within a scalled down hypersolid of current point

Run the acceptance criteria

iterate
*/

void SimulatedAnnealing::generatePos(double *pos, double *nextPos) {
    for (int i = 0; i < this->hyperdim; i++) {
        double stepRad = this->hyperrads[i]*this->radGain;
        std::uniform_real_distribution<double> unif(-stepRad ,stepRad);
        nextPos[i] = pos[i] + unif(this->randomEngine);

        if (nextPos[i] > this->hypercenter[i] + this->hyperrads[i]) {
            nextPos[i] = this->hypercenter[i] + this->hyperrads[i];
        }
        else if (nextPos[i] < this->hypercenter[i] - this->hyperrads[i]) {
            nextPos[i] = this->hypercenter[i] - this->hyperrads[i];
        }
    }

    this->radGain *= this->radGainDecay;
}

void SimulatedAnnealing::posCopy(double *from, double *to) {
    for (int i = 0; i < this->hyperdim; i++) {
        to[i] = from[i];
    }
}

Solution::Solution SimulatedAnnealing::anneal(double * hyperpos, int iterations, double radGain, double startTemp, double tempRet, double radGainDecay, int debug) {
    this->radGain = radGain;
    this->radGainDecay = radGainDecay;

    int iter = 0;

    Solution::Solution prev = Solution::Solution(hyperpos, this->geometryConfig, this->goalConfig);
    prev.updateSolution();

    double prevCost = prev.evalSolution(), cost;
    this->bestSolution = prev;
    bestCost = prevCost;

    Solution::Solution current;

    double temperature = startTemp;

    double *prevPos = new double[this->hyperdim];
    this->posCopy(this->hypercenter, prevPos);

    double *currentPos = new double[this->hyperdim];

    while (iter < iterations) {

        this->generatePos(prevPos, currentPos);

        current = Solution::Solution(currentPos, this->geometryConfig, this->goalConfig);
        current.updateSolution();
        cost = current.evalSolution();


        if (cost < prevCost) {
            if (cost < bestCost) {
                bestSolution = current;
                bestCost = cost;
            }

            this->posCopy(currentPos, prevPos);
            prev = current;
            prevCost = cost;
        } else if (exp((prevCost - cost) / temperature) > this->randomEngine()) {
            std::cout << exp((prevCost - cost) / temperature) << std::endl;
            this->posCopy(currentPos, prevPos);
            prev = current;
            prevCost = cost;
        }

        temperature *= tempRet;

        iter++;
        printf("\rIter %07d: Temp: %.5e Gain: %.5e Cost: %0.04f", iter, temperature, this->radGain, cost);
    }

    return this->bestSolution;
}

Solution::Solution SimulatedAnnealing::anneal(int iterations, double radGain, double tempRet, double radGainDecay) {
    return anneal(this->hypercenter, iterations, radGain, 1e10, tempRet, radGainDecay, 0);
}

SimulatedAnnealing::SimulatedAnnealing(double * hypercenter, double * hyperrads, int hyperdim, Geometry_t geometryConfig, GoalCharacteristics_t goalConfig)
: randomEngine(std::random_device()()),
  bestSolution(Solution::Solution(hypercenter, geometryConfig, goalConfig)) {
    this->hypercenter = hypercenter;
    this->hyperrads = hyperrads;
    this->hyperdim = hyperdim;
    this->geometryConfig = geometryConfig;
    this->goalConfig = goalConfig;
}

void SimulatedAnnealing::checkForClipping(Solution::Solution solution) {
    double *pos = solution.getHyperPos();

    for (int i = 0; i < this->hyperdim; i++) {
        if (abs(pos[i] - (this->hypercenter[i] + this->hyperrads[i])) < 1e-10) {
            std::cout << "Clipping on " << i << " " << pos[i] << std::endl;
        }
        else if (abs(pos[i] - (this->hypercenter[i] - this->hyperrads[i])) < 1e-10) {
            std::cout << "Clipping on " << i << " " << pos[i] << std::endl;
        }
    }

    delete [] pos;
}

void SimulatedAnnealing::checkBestForClipping() {
    this->checkForClipping(this->bestSolution);
}
