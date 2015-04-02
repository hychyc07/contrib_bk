#ifndef GMM_H
#define GMM_H

#include <iostream>
#include <vector>
#include <cv.h>

// Gaussian Mixture Model
//
class Gmm 
{
public:
    Gmm() {};

    void init(const cv::Mat &data);

    // run expectation maximization algorithm
    int runEM(int maxIterations, bool printProgress);

    cv::Mat predict(const cv::Mat &samples) const;

    inline int size() const { return numComponents; }
    inline cv::Mat means() const { return mu; };
    inline std::vector<cv::Mat> sigmas() const { return sigma; };

private:

    int numComponents; 
    std::vector<cv::Mat> sigma;
    cv::Mat mu;

    cv::Mat data;
};

#endif
