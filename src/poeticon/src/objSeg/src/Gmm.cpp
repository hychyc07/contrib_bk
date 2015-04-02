#include <iCub/Gmm.h>
#include <boost/progress.hpp>



cv::Mat cv_mean(const cv::Mat &m, int axis)
{

    int rows;
    int cols;

    if (axis == 0)
    {
        rows = 1;
        cols = m.cols;
    }
    else if (axis == 1)
    {
        rows = m.rows;
        cols = 1;
    }
    else 
    {
        std::cerr << "cv_mean: axis must be 0 or 1" << std::endl;
        return cv::Mat();
    }

    cv::Mat dst(rows, cols, m.type());
    
    if (axis == 0)
    {
        for (int r=0; r < m.rows; r++)
        {
            dst += cv::mean(m.col(r));
        }
    }
    else
    {
        for (int c=0; c < m.cols; c++)
        {
            dst += cv::mean(m.col(c));
        }
    }
    return dst;
}

void Gmm::init(const cv::Mat &X)
{
    data = X.clone();

    int subsetSize = (data.rows/numComponents);
    for (int k=0; k<numComponents; k++)
    {
        cv::Mat subset = data.rowRange(k*subsetSize, (k+1)*subsetSize);
        
        cv::Mat tmp = mu.row(k);
        cv_mean(subset, 0).row(0).copyTo(tmp);

    }
}

int Gmm::runEM(int maxIterations, bool printProgress)
{
    return 0;
}

cv::Mat Gmm::predict(const cv::Mat &samples) const
{
    return cv::Mat();
}

