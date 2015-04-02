/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Bjoern Browatzki
 * email:   bjoern.browatzki@tuebingen.mpg.de
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <sstream>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <yarp/math/Math.h>

#include "iCub/Util.h"


Vector Util::normalize(const Vector &v)
{
    Vector res;
    double l = yarp::math::norm(v);
    for (int i = 0; i < v.length(); ++i)
        res.push_back(v[i] / l);
    return res;
}

// always stay away a few degrees from the joint limits
void Util::safePositionMove(Vector &target, IPositionControl *pos, IControlLimits *lim)
{
    double safeDist = 0;
    int nAxes;
    pos->getAxes(&nAxes);
    if (target.size() > nAxes)
    {
        std::cerr << "safePositionMove: size of target vector exceeds number of joints!" << std::endl;
        return;
    }
    for (int jnt = 0; jnt < target.size(); ++jnt)
    {
        double min, max;
        lim->getLimits(jnt, &min, &max);
        if (target[jnt] < min+safeDist) target[jnt] = min+safeDist;
        if (target[jnt] > max-safeDist) target[jnt] = max-safeDist;
    }
    pos->positionMove(target.data());
}

// always stay away a few degrees from the joint limits
void Util::safePositionMove(int jnt, double val, IPositionControl *pos, IControlLimits *lim)
{
    double safeDist = 0;
    int nAxes;
    pos->getAxes(&nAxes);
    if (jnt >= nAxes)
    {
        std::cerr << "safePositionMove: specified joint index exceeds number of joints!" << std::endl;
        return;
    }
    double min, max;
    lim->getLimits(jnt, &min, &max);
    if (val < min+safeDist) val =  min+safeDist;
    if (val > max-safeDist) val =  max-safeDist;
    pos->positionMove(jnt, val);
}

std::string Util::toString(const Vector &vec)
{
    std::stringstream stream;
    if (vec.size() > 0)
    {
        stream << vec[0];
    }
    for (int i=1; i<vec.size(); i++)
    {
        stream << " " << vec[i];
    }
    return stream.str();
}


// calculate angular difference on sphere (degrees)
double Util::calcCentralAngle(const cv::Vec2f &vp1, const cv::Vec2f &vp2)
{
    calcCentralAngle(vp1[0], vp2[0], vp1[1], vp2[1]); 
}
double Util::calcCentralAngle(double e1_deg, double e2_deg, double r1_deg, double r2_deg)
{
    double e1 = (90.0-e1_deg)*Util::DEG2RAD;
    double e2 = (90.0-e2_deg)*Util::DEG2RAD;
    double r1 = r1_deg*Util::DEG2RAD;
    double r2 = r2_deg*Util::DEG2RAD;
    double dr = r2-r1;
    return acos(sin(e1)*sin(e2)+cos(e1)*cos(e2)*cos(dr))*Util::RAD2DEG;
}

cv::Point Util::cog(const cv::Mat &img)
{
    cv::Point cog(0,0);
    for (int y = 0; y < img.rows; y++) 
    {
        for (int x = 0; x < img.cols; x++) 
        {
            if (img.at<cv::Vec3b>(y,x) != cv::Vec3b())
            {
                cog += cv::Point(x,y);
            }
        }
    }
    cog.x /= img.rows*img.cols;
    cog.y /= img.rows*img.cols;
    return cog;
}

cv::Point Util::cog(const std::vector<cv::Point> &points)
{
    cv::Point cog(0,0);
    std::vector<cv::Point>::const_iterator pt;
    for (pt = points.begin(); pt != points.end(); ++pt)
    {
        //std::cout << *pt << std::endl;;
        cog += *pt; 
    }
    cog.x /= points.size();
    cog.y /= points.size();
    return cog;
}

cv::Mat Util::removeBorder(const cv::Mat &img, const cv::Vec3b &color)
{   

    CV_Assert(img.type() == CV_8UC3);

    int top = 0;
    int bottom = 0;
    int left = 0;
    int right = 0;

    for (int y = 0; y < img.rows; y++)
    {
        for (int x = 0; x < img.cols; x++)
        {
            if (img.at<cv::Vec3b>(y,x) != color)
            {
                top = y;
                goto BOTTOM;
            }
        }
    }

BOTTOM:
    for (int y = img.rows-1; y >= 0; y--)
    {
        for (int x = 0; x < img.cols; x++)
        {
            if (img.at<cv::Vec3b>(y,x) != color)
            {
                bottom = y;
                goto LEFT;
            }
        }
    }

LEFT:
    for (int x = 0; x < img.cols; x++)
    {
        for (int y = 0; y < img.rows; y++)
        {
            if (img.at<cv::Vec3b>(y,x) != color)
            {
                left = x;
                goto RIGHT;
            }
        }
    }

RIGHT:
    for (int x = img.cols-1; x >= 0 ; x--)
    {
        for (int y = 0; y < img.rows; y++)
        {
            if (img.at<cv::Vec3b>(y,x) != color)
            {
                right = x;
                goto DONE;
            }
        }
    }

DONE:
    int w = right-left;
    int h = bottom - top;
    if (w*h == 0)
        return img;

    return cv::Mat(img, cv::Rect(left,top, w, h));

}

void Util::imagesc(const cv::Mat &img_, const std::string &wndname, 
        float scaleX, float scaleY, int delay)
{
	cv::Mat img;
	cv::Mat img_32F;
	img_.convertTo(img_32F, CV_32F);
	cv::resize(img_32F, img, cv::Size(), scaleX, scaleY, cv::INTER_NEAREST);

    cv::normalize(img, img, 0, 1, cv::NORM_MINMAX);
    cv::imshow(wndname, img);
    cv::waitKey(delay);
}

cv::Mat Util::GetColorcoded(const cv::Mat& img_32F)
{
    if (img_32F.empty())
        return img_32F;

    double minVal, maxVal;
    cv::minMaxLoc(img_32F, &minVal, &maxVal);
    return GetColorcoded(img_32F, minVal, maxVal);
}


cv::Vec3f jetColorMap(double value, double min,double max)
{
  double rgb[3];
  unsigned char c1=144;
  float max4=(max-min)/4;
  value-=min;
  if(value==HUGE_VAL)
    {rgb[0]=rgb[1]=rgb[2]=255;}
  else if(value<0)
    {rgb[0]=rgb[1]=rgb[2]=0;}
  else if(value<max4)
    {rgb[0]=0;rgb[1]=0;rgb[2]=c1+(unsigned char)((255-c1)*value/max4);}
  else if(value<2*max4)
    {rgb[0]=0;rgb[1]=(unsigned char)(255*(value-max4)/max4);rgb[2]=255;}
  else if(value<3*max4)
    {rgb[0]=(unsigned char)(255*(value-2*max4)/max4);rgb[1]=255;rgb[2]=255-rgb[0];}
  else if(value<max)
    {rgb[0]=255;rgb[1]=(unsigned char)(255-255*(value-3*max4)/max4);rgb[2]=0;}
  else {rgb[0]=255;rgb[1]=rgb[2]=0;}
  return cv::Vec3f(rgb[2], rgb[1], rgb[0]);
}


cv::Vec3f coldColorMap(double value, double min,double max)
{
  double rgb[3];
  float max3=(max-min)/3;
  value-=min;
  if(value==HUGE_VAL)
    {rgb[0]=rgb[1]=rgb[2]=255;}
  else if(value<0)
    {rgb[0]=rgb[1]=rgb[2]=0;}
  else if(value<max3)
    {rgb[0]=0;rgb[1]=0;rgb[2]=(unsigned char)(255*value/max3);}
  else if(value<2*max3)
    {rgb[0]=0;rgb[1]=(unsigned char)(255*(value-max3)/max3);rgb[2]=255;}
  else if(value<max)
    {rgb[0]=(unsigned char)(255*(value-2*max3)/max3);rgb[1]=255;rgb[2]=255;}
  else {rgb[0]=rgb[1]=rgb[2]=255;}
  return cv::Vec3f(rgb[2], rgb[1], rgb[0]);
}

cv::Vec3f hotColorMap(double value, double min,double max)
{
  double max3=(max-min)/3;
  double rgb[3];
  value-=min;
  if(value==HUGE_VAL)
    {rgb[0]=rgb[1]=rgb[2]=255;}
  else if(value<0)
    {rgb[0]=rgb[1]=rgb[2]=0;}
  else if(value<max3)
    {rgb[0]=(unsigned char)(255*value/max3);rgb[1]=0;rgb[2]=0;}
  else if(value<2*max3)
    {rgb[0]=255;rgb[1]=(unsigned char)(255*(value-max3)/max3);rgb[2]=0;}
  else if(value<max)
    {rgb[0]=255;rgb[1]=255;rgb[2]=(unsigned char)(255*(value-2*max3)/max3);}
  else {rgb[0]=rgb[1]=rgb[2]=255;}
  return cv::Vec3f(rgb[2], rgb[1], rgb[0]);
}

cv::Vec3f grayColorMap(double value, double min,double max)
{
    double rgb[3];
    max-=min;
    value-=min;
    rgb[0]=rgb[1]=rgb[2]=(unsigned char)(255*value/max);
    return cv::Vec3f(rgb[2], rgb[1], rgb[0]);
}

cv::Mat Util::GetColorcoded(const cv::Mat& img_32F, double min, double max)
{
    float H,S,V;
    cv::Mat hsvImage(img_32F.size(), CV_8UC3);
    int hsvBlue = 180*2/3;
    
    if (min > max)
    {
        std::swap(min, max);
    }

    double diff = max-min;
    if (diff == 0)
    {
        diff = 1;
    }

    bool hsv = false;

    for (int i = 0; i < img_32F.rows; i++)
    {

        for (int j = 0; j < img_32F.cols; j++)
        {
            double val = (double)img_32F.at<float>(i,j);
            val = std::max(std::min(max, val), min);
            val = ((val-min)/diff);
            if (hsv)
            {
                if (val == 0)
                {
                    H = 0;
                    S = 0;
                    V = 0;
                }
                else
                {
                    H = val * hsvBlue;
                    S = 255;
                    V = 255;
                }

                hsvImage.at<cv::Vec3b>(i,j)[0] = hsvBlue - H;
                hsvImage.at<cv::Vec3b>(i,j)[1] = S;
                hsvImage.at<cv::Vec3b>(i,j)[2] = V;
            }
            else
            {
                hsvImage.at<cv::Vec3b>(i,j) = jetColorMap(val, 0, 1);
            }
        }
    }

    if (hsv)
        cv::cvtColor(hsvImage, hsvImage, CV_HSV2BGR);

    return hsvImage;
}

cv::Mat Util::vstack(const std::vector<cv::Mat> &mats)
{
    if (mats.empty())
        return cv::Mat();

    // we need to know the total number of rows to create the stacked matrix
    int nRows = 0;
    int nCols = 0;
    int datatype = mats.front().type();
    std::vector<cv::Mat>::const_iterator it;
    for (it = mats.begin(); it != mats.end(); ++it)
    {
        if (it->empty())
            continue;
        if (it->rows == 0 || it->cols == 0)
            continue;
        nCols = it->cols;
        nRows += it->rows;
    }

    // copy data to stacked matrix
    int startRow = 0;
    int endRow = 0;
    cv::Mat stacked(nRows, nCols, datatype);
    for (it = mats.begin(); it != mats.end(); ++it)
    {
        if (it->empty())
            continue;
        if (it->rows == 0 || it->cols == 0)
            continue;

        // make sure all mats have same num of cols and data type
        if (it->cols != nCols)
        {
            std::cerr << "ERROR - ipa_Utils::vstack: All mats must have the same number of column" << std::endl;
            std::cerr << "\t Current mat has " << it->cols << " colums. Expected: " << nCols << std::endl;
        }
        CV_Assert(it->cols == nCols);
        CV_Assert(it->type() == datatype);

        startRow = endRow;
        endRow = startRow + it->rows;
    	cv::Mat mat = stacked.rowRange(startRow, endRow);
    	it->copyTo(mat);
    }

    return stacked;
}

void Util::writeMatTxt(const std::string &filename, const cv::Mat &m)
{
    std::ofstream f(filename.c_str(), std::ios::out);
    if (!f.is_open())
    {
        std::cerr << "ERROR - writeMatTxt: Could not open file '" << filename << "' for writing." << std::endl;
        return;
    }
    if (m.channels() != 1)
    {
        std::cerr << "ERROR - writeMatTxt: Only single channel matrices are supported." << std::endl;
        return;
    }

    typedef float T;
    if (m.type() == CV_32F)
        typedef float T;
    else if (m.type() == CV_8U)
        typedef uchar T;
    else if (m.type() == CV_32S)
        typedef int T;
    else
    {
        std::cerr << "ERROR - writeMatTxt: Type not supported." << std::endl;
        return;
    }

    for (int i = 0; i < m.rows; i++)
    {
        for (int j = 0; j < m.cols; j++)
        {
            f << m.at<T>(i,j) << " "; 
        }
        f << std::endl;
    }

    f.close();
}

cv::Mat Util::tileImgs(const std::vector<cv::Mat> &imgs, int nRows, int nCols, 
        const std::vector<std::string> captions, const cv::Scalar &clBgr)
{
    if (imgs.empty())
        return cv::Mat();

    int nimgs = (int)imgs.size();

    CV_Assert(nRows*nCols >= nimgs);  
    int w = 0;
    int h = 0;
    for (int i = 0; i < nimgs; i++)
    {
        w = std::max(w, imgs[i].cols);
        h = std::max(h, imgs[i].rows);
    }

    cv::Rect dstRoi;
    dstRoi.width  = w;
    dstRoi.height = h;

    cv::Mat tiled(h*nRows, w*nCols, imgs[0].type(), clBgr);

    for (int i = 0; i < nimgs; i++)
    {
        int c = i % nCols;
        int r = i / nCols;

        dstRoi.x = c * w;
        dstRoi.y = r * h;

        cv::Mat dst = tiled(dstRoi);
        imgs[i].copyTo(dst);
        if (captions.size() > i)
        {
            cv::putText(dst, captions[i], cv::Point(0,20), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar::all(0)); 
        }
    }
    return tiled;
    
}

cv::Mat Util::drawHistogram(const cv::Mat &hist, int width, int height)
{
  double histMax = 0;
  cv::minMaxLoc(hist, 0, &histMax, 0, 0);

  cv::Mat imgHist(height, width, CV_8U, cv::Scalar(0));

    for(int i=0; i<hist.cols-1; i++)
    {
        float histValue = hist.at<float>(0,i);
        float nextValue = hist.at<float>(0,i+1);
 
        cv::Point pt1(width*i/(double)hist.cols, height);
        cv::Point pt2(width*i/(double)hist.cols+width/(double)hist.cols, height);
        cv::Point pt3(width*i/(double)hist.cols+width/(double)hist.cols, height-nextValue*height/histMax);
        cv::Point pt4(width*i/(double)hist.cols, height-histValue*height/histMax);
 
        int numPts = 5;
        cv::Point pts[] = {pt1, pt2, pt3, pt4, pt1};
 
        cv::fillConvexPoly(imgHist, pts, numPts, cv::Scalar(255));
    }
    return imgHist;
}
