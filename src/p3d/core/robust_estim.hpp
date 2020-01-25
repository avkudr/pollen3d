#ifndef ROBUST_ESTIMATOR_H
#define ROBUST_ESTIMATOR_H

#include <vector>
#include <random>
#include <numeric>
#include <functional>
#include <algorithm>
#include <iostream>
#include <iterator>

class EstimationProblem{

public:
    // Functions to overload in your class:
    virtual double estimErrorForSample(int i) = 0;
    virtual double estimModelFromSamples(std::vector<int> samplesIdx) = 0;
    virtual    int getTotalNbSamples() const = 0;

    int getNbParams()     const{return nbParams;}
    int getNbMinSamples() const{return nbMinSamples;}

protected:
    void setNbParams(int i)    {nbParams     = i;}
    void setNbMinSamples(int i){nbMinSamples = i;}

private:
    int nbParams = -1;
    int nbMinSamples = -1;
};

static std::random_device rd;  // random device engine, usually based on /dev/random on UNIX-like systems
static std::mt19937 rng(rd()); // initialize Mersennes' twister using rd to generate the seed
//static std::mt19937 rng((unsigned int) - 1);

// Base class for Robust Estimators
class AbstractEstimator
{
public:
    // Generate X !different! random numbers from 0 to N-1
    // X - minimal number of samples
    // N - total number of samples
    // generated numbers are indices of data points

    std::vector<int> randomSampleIdx(){

        int minNbSamples = problem->getNbMinSamples();
        int totalNbSamples = problem->getTotalNbSamples();

        std::vector<int> allIdx(totalNbSamples);
        std::iota (std::begin(allIdx), std::end(allIdx), 0); // Fill with 0, 1, ..., totalNbSamples-1.

        // shuffle the elements of allIdx : Fisher–Yates shuffle
        for (int i = 0; i < minNbSamples; i++){
            std::uniform_int_distribution<int> dist(0,totalNbSamples-i-1);
            int randInt = dist(rng);
            std::swap(allIdx[totalNbSamples-i-1],allIdx[randInt]);
        }

        //take last <minNbSamples> elements
        std::vector<int> idx( allIdx.end() - minNbSamples, allIdx.end());
//        std::cout << "[";
//        for (auto i : idx) std::cout << i << ", ";
//        std::cout << "]\n";
        return idx;
    }

    double getInliersFraction() const {return inliersFraction;}
    std::vector<int> getInliersIndices() const {return inliersIdx;}

protected:
    void getInliers(double thres){
        int totalNbSamples = problem->getTotalNbSamples();
        inliersIdx.clear();
        for(int j = 0; j < totalNbSamples; j++){
            double error = problem->estimErrorForSample(j);
            error = error*error;
            if (error < thres){
                inliersIdx.push_back(j);
            }
        }
        this->inliersFraction = (double)(inliersIdx.size()) / (double)(totalNbSamples);
    }

    EstimationProblem * problem;
    std::vector<int> bestIdxSet;
    std::vector<int> inliersIdx;
    double inliersFraction;
};

class RANSAC : public AbstractEstimator
{
public:
    RANSAC(){

    }

    void solve(EstimationProblem * pb, double thres = 0.1, int nbIter = 10000){
        problem = pb;

        int totalNbSamples = problem->getTotalNbSamples();
        for (int i = 0; i < nbIter; i++){
            std::vector<int> indices = randomSampleIdx();

            problem->estimModelFromSamples(indices);

            //getInliersNb
            int nbInliers = 0;
            for(int j = 0; j < totalNbSamples; j++){
                double error = problem->estimErrorForSample(j);
                error = error*error;
                if (error < thres){
                    nbInliers++;
                }
            }

            double inliersFraction = (double)(nbInliers) / (double)(problem->getTotalNbSamples());
            if (inliersFraction > this->inliersFraction){
                this->inliersFraction = inliersFraction;
                this->bestIdxSet = indices;
            }
        }

        problem->estimModelFromSamples(bestIdxSet);
        getInliers(thres);
        problem->estimModelFromSamples(inliersIdx);
    }
};

class LMedS : public AbstractEstimator
{
public:
    LMedS(){
        med = 1000000.0;
    }

    template <typename T>
    double median(std::vector<T> & v){
        std::sort(v.begin(), v.end());
        if (v.size() % 2 == 0){
            return (double)(v[v.size()/2-1] + v[v.size()/2]) / 2;
        }else{
            return v[v.size()/2];
        }
    }

    void solve(EstimationProblem * pb, double thres = 0.1, int nbIter = 1000){
        problem = pb;
        int totalNbSamples = problem->getTotalNbSamples();

        for (int i = 0; i < nbIter; i++){
            std::vector<int> indices = randomSampleIdx();

            problem->estimModelFromSamples(indices);

            std::vector<double> errorsVec(totalNbSamples);
            for(int j = 0; j < problem->getTotalNbSamples(); j++){
                double error = problem->estimErrorForSample(j);
                //errorsVec[j] = error; // error must be squared!
                errorsVec[j] = error*error; // error must be squared!
            }
            double med = median(errorsVec);
            if (med < this->med){
                this->med = med;
                this->bestIdxSet = indices;
            }
        }

        problem->estimModelFromSamples(bestIdxSet);
        getInliers(thres);
        problem->estimModelFromSamples(inliersIdx);
    }

private:
    double med;
};

#endif // ROBUST_ESTIMATOR_H
