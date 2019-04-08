#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include "popt_pp.h"

#include "configuration.h"

using namespace std;
using namespace cv;

vector< vector< Point3d > > object_points;
vector< vector< Point2f > > imagePoints1, imagePoints2;
vector< Point2f > corners1, corners2;
vector< vector< Point2d > > cam1_img_points, cam2_img_points;
vector<string> goodImageListGlobal;

Size image_size;

int s_offset = START_OFFSET; 
int e_offset = END_OFFSET; 
int skip_frame = FRAME_SKIP;

void extractImagePoints(int board_width, int board_height,
			float square_size, char* image_directory_left, char* image_directory_right,
			int verb_disp)
{
    Size board_size = Size(board_width,board_height);
    int board_n = board_width * board_height;
   
    vector<String> image_filenames_left;
    vector<String> image_filenames_right;

    glob(image_directory_left, image_filenames_left);
    glob(image_directory_right, image_filenames_right);

    if (image_filenames_left.size() > 0 && image_filenames_right.size() > 0)
        std::cout << "Identified " << image_filenames_left.size() << " images." << std::endl << std::endl;
    else 
    {
        std::cout << "Could not identify any images. Check calibration image path and try again." << std::endl << std::endl;
	return;
    }
    int valid_stereo=0;

    if (verb_disp) {
        cv::namedWindow("left");
        cv::namedWindow("right");
    }

    for (int im_ctr = s_offset; im_ctr < image_filenames_left.size() - e_offset; im_ctr += skip_frame) 
    {
        std::cout << "Here:" << std::endl;
        cv::Mat cam1_image = imread(image_filenames_left[im_ctr],0);
        cv::Mat cam2_image = imread(image_filenames_right[im_ctr],0);

	image_size = cam1_image.size();

        bool found1 = false, found2 = false;
	found1 = cv::findChessboardCorners(cam1_image, board_size, corners1, 
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        std::cout << "Left : " << found1 << std::endl;
        found2 = cv::findChessboardCorners(cam2_image, board_size, corners2,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        std::cout << "Right : " << found2 << std::endl;

	if (found1 && found2) 
        {
	    valid_stereo = valid_stereo + 1;
	    cv::cornerSubPix(cam1_image, corners1, cv::Size(5, 5), cv::Size(-1, -1), 
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
	    cv::cornerSubPix(cam2_image, corners2, cv::Size(5, 5), cv::Size(-1, -1), 
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));	    

	    imagePoints1.push_back(corners1);
	    imagePoints2.push_back(corners2);

    	    vector<cv::Point3d> obj;
    	    for( int i = 0; i < board_height; ++i ) {
                for( int j = 0; j < board_width; ++j ) {
                       obj.push_back(Point3d(double( (float)j * square_size ), 
					     double( (float)i * square_size ), 0));
		}
	    }
	    object_points.push_back(obj);
	    std::cout << "PASSED: Image triplet detected." << std::endl;

	    if (verb_disp) 
            {
		Mat cimg1,cimg2,cimg1_r,cimg2_r;
      		cvtColor(cam1_image, cimg1, COLOR_GRAY2BGR);
      		cvtColor(cam2_image, cimg2, COLOR_GRAY2BGR);
		
		cv::drawChessboardCorners(cimg1, board_size, corners1, found1);
		cv::drawChessboardCorners(cimg2, board_size, corners2, found2);

		double sf = 240./MAX(cam1_image.rows, cam1_image.cols);
		cv::resize(cimg1, cimg1_r, Size(), sf, sf);
		cv::resize(cimg2, cimg2_r, Size(), sf, sf);
		cv::imshow("left",cimg1_r);
		cv::imshow("right",cimg2_r);
	        cv::waitKey(50);
	    }

	}
	else {
	    std::cout << "FAILED: Image pair excluded." << std::endl;
	}
    }
    
    if (verb_disp) {
        cv::destroyWindow("left");
        cv::destroyWindow("right");
    }
    std::cout << std::endl; 
    std::cout << "Detected " << valid_stereo << " pairs for calibration." << std::endl;
    for (int i = 0; i < imagePoints1.size(); i++) 
    {
        vector< Point2d > v1, v2, v3;
        for (int j = 0; j < imagePoints1[i].size(); j++) 
	{
            v1.push_back(Point2d((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
            v2.push_back(Point2d((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
        }
        cam1_img_points.push_back(v1);
        cam2_img_points.push_back(v2);
    }
}






int main(int argc, char const *argv[])
{
    std::cout << std::endl << "Stereo Fisheye Model Camera Calibration\n" << std::endl;

    int board_width, board_height;
    float square_size;
    char* image_directory_left;
    char* image_directory_right;
    int verb_disp;

    static struct poptOption options[] = {
    { "img_dir_l",'l',POPT_ARG_STRING,&image_directory_left,0,"Directory containing images","STR" },
    { "img_dir_r",'r',POPT_ARG_STRING,&image_directory_right,0,"Directory containing images","STR" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
    };

    POpt popt(NULL, argc, argv, options, 0);
    int c;
    while((c = popt.getNextOpt()) >= 0) {}

    board_width = BOARD_WIDTH;
    board_height = BOARD_HEIGHT;
    square_size = SQUARE_SIZE;
    verb_disp = VERBOSITY_FLAG;

    std::cout << "BW : " << board_width << " BH : " << board_height << " SS : " << square_size << " VD : " << verb_disp << std::endl;

    extractImagePoints(board_width,board_height,square_size,image_directory_left,image_directory_right,verb_disp);

    std::cout << std::endl << "Image points have been extracted. " << std::endl << std::endl;

    std::cout << "Calibration will begin using " << cam1_img_points.size();
    std::cout << " image pairs." << std::endl << std::endl;

    int flag = 0;
    flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flag |= cv::fisheye::CALIB_CHECK_COND;
    flag |= cv::fisheye::CALIB_FIX_SKEW;
    //flag |= cv::fisheye::CALIB_FIX_K2;
    //flag |= cv::fisheye::CALIB_FIX_K3;
    //flag |= cv::fisheye::CALIB_FIX_K4;

    // Calibrating stereo cameras
    std::cout << "Calibrating... " << std::endl << std::endl;
    cv::Matx33d K1_b1, K2_b1, R_b1;
    cv::Vec3d T_b1;
    cv::Vec4d D1_b1, D2_b1;
    cv::fisheye::stereoCalibrate(object_points, cam1_img_points, cam2_img_points,
    				K1_b1, D1_b1, K2_b1, D2_b1, image_size, R_b1, T_b1, flag,
			        cv::TermCriteria(3, 12, 0));
    cv::Mat R1_b1, R2_b1, P1_b1, P2_b1, Q_b1;
    cv::fisheye::stereoRectify(K1_b1, D1_b1, K2_b1, D2_b1, image_size, R_b1, T_b1, R1_b1, R2_b1, 
				P1_b1, P2_b1, Q_b1, CV_CALIB_ZERO_DISPARITY, image_size, 0.0, 1.0);

    std::cout << "Saving parameter files.." << std::endl << std::endl;
    
    // Create parameter file
    cv::FileStorage fs1(paramFile, cv::FileStorage::WRITE);
    fs1 << "K1" << Mat(K1_b1);
    fs1 << "K2" << Mat(K2_b1);
    fs1 << "D1" << D1_b1;
    fs1 << "D2" << D2_b1;
    fs1 << "R" << Mat(R_b1);
    fs1 << "T" << T_b1;
    fs1 << "R1" << R1_b1;
    fs1 << "R2" << R2_b1;
    fs1 << "P1" << P1_b1;
    fs1 << "P2" << P2_b1;
    fs1 << "Q" << Q_b1;
    fs1.release();
    std::cout << "Saved B-1 parameter file." << std::endl << std::endl;
    
    return 1;
}
