#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <fftm.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


int threshold_value1 = 100;
int threshold_value2 = 100;
void barCB(int, void* ){

}
int main( int argc, char** argv )
{

    //Mat frame;
    std::vector<Mat> templateList;
    string path = ros::package::getPath("locationCamera");
    stringstream ss;
    ss << path << "/images/robotAtAngle.png";
    Mat measuredRobot = imread( ss.str(), IMREAD_GRAYSCALE );
    imshow("measuredRobot",measuredRobot);
    ss.str("");
    ss << path << "/images/temp2.png";
    Mat robotTemplate = imread( ss.str(), IMREAD_GRAYSCALE );
    // Define the motion model
    const int warp_mode = MOTION_AFFINE;

    // Set a 2x3 or 3x3 warp matrix depending on the motion model.
    Mat warp_matrix;

    // Initialize the matrix to identity
    if ( warp_mode == MOTION_HOMOGRAPHY )
        warp_matrix = Mat::eye(3, 3, CV_32F);
    else
        warp_matrix = Mat::eye(2, 3, CV_32F);

    // Specify the number of iterations.
    int number_of_iterations = 50000;

    // Specify the threshold of the increment
    // in the correlation coefficient between two iterations
    double termination_eps = 1e-10;

    // Define termination criteria
    TermCriteria criteria (TermCriteria::COUNT+TermCriteria::EPS, number_of_iterations, termination_eps);

    // Run the ECC algorithm. The results are stored in warp_matrix.
    findTransformECC(
                     measuredRobot,
                     robotTemplate,
                     warp_matrix,
                     warp_mode,
                     criteria
                 );

    // Storage for warped image.
    Mat im2_aligned;

    if (warp_mode != MOTION_HOMOGRAPHY)
        // Use warpAffine for Translation, Euclidean and Affine
        warpAffine(robotTemplate, im2_aligned, warp_matrix, measuredRobot.size(), INTER_LINEAR + WARP_INVERSE_MAP);
    else
        // Use warpPerspective for Homography
        warpPerspective (robotTemplate, im2_aligned, warp_matrix, measuredRobot.size(),INTER_LINEAR + WARP_INVERSE_MAP);

    // Show final result
    imshow("Image 1", measuredRobot);
    imshow("Image 2", robotTemplate);
    imshow("Image 2 Aligned", im2_aligned);
    waitKey(0);


}






