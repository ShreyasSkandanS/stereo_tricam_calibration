#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include "popt_pp.h"

using namespace std;
using namespace cv;

vector< vector< Point3d > > object_points;
vector< vector< Point2f > > imagePoints1, imagePoints2, imagePoints3;
vector< Point2f > corners1, corners2, corners3;
vector< vector< Point2d > > cam1_img_points, cam2_img_points, cam3_img_points;
vector<string> goodImageListGlobal;

Size image_size;

// Image extraction parameters
int s_offset = 15; // skip the first s_offset frames
int e_offset = 15; // skip the last e_offset frames
int skip_frame = 5; // use 1 in ever skip_frame frames

void extractImagePoints(int board_width, int board_height,
			float square_size, char* image_directory,
			int verb_disp)
{
    Size board_size = Size(board_width,board_height);
    int board_n = board_width * board_height;
    
   
    vector<String> image_filenames;
    glob(image_directory,image_filenames);
    if (image_filenames.size() > 0)
        std::cout << "Identified " << image_filenames.size() << " images." << std::endl << std::endl;
    else 
    {
        std::cout << "Could not identify any images. Check calibration image path and try again." << std::endl << std::endl;
	return;
    }
    int valid_triplet = 0;

    if (verb_disp) {
        cv::namedWindow("cam1");
        cv::namedWindow("cam2");
        cv::namedWindow("cam3");
    }

    for (int im_ctr = s_offset; im_ctr < image_filenames.size() - e_offset; im_ctr += skip_frame) 
    {
        cv::Mat full_image = imread(image_filenames[im_ctr],0);

	//cv::imshow("full_image",full_image);
	//cv::waitKey(1);

	cv::Mat cam1_image, cam2_image, cam3_image;
	//cv::Rect cam1_roi(0,2*(full_image.rows/3),full_image.cols,(full_image.rows/3));
	//cv::Rect cam2_roi(0,1*(full_image.rows/3),full_image.cols,(full_image.rows/3));
	//cv::Rect cam3_roi(0,0*(full_image.rows/3),full_image.cols,(full_image.rows/3));

	cv::Rect cam1_roi(0,0*(full_image.rows/3),full_image.cols,(full_image.rows/3));
	cv::Rect cam2_roi(0,1*(full_image.rows/3),full_image.cols,(full_image.rows/3));
	cv::Rect cam3_roi(0,2*(full_image.rows/3),full_image.cols,(full_image.rows/3));

	cam1_image = full_image(cam1_roi);
	cam2_image = full_image(cam2_roi);
	cam3_image = full_image(cam3_roi);
	image_size = cam1_image.size();

	//cv::imshow("cam1",cam1_image);
	//cv::imshow("cam2",cam2_image);
	//cv::imshow("cam3",cam3_image);
	//cv::waitKey(1);

        bool found1 = false, found2 = false, found3 = false;
	found1 = cv::findChessboardCorners(cam1_image, board_size, corners1, 
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        found2 = cv::findChessboardCorners(cam2_image, board_size, corners2,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        found3 = cv::findChessboardCorners(cam3_image, board_size, corners3,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

	if (found1 && found2 && found3) 
        {
	    valid_triplet = valid_triplet + 1;
	    cv::cornerSubPix(cam1_image, corners1, cv::Size(5, 5), cv::Size(-1, -1), 
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
	    cv::cornerSubPix(cam2_image, corners2, cv::Size(5, 5), cv::Size(-1, -1), 
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));	    
	    cv::cornerSubPix(cam3_image, corners3, cv::Size(5, 5), cv::Size(-1, -1), 
			cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

	    imagePoints1.push_back(corners1);
	    imagePoints2.push_back(corners2);
	    imagePoints3.push_back(corners3);

    	    vector<cv::Point3d> obj;
    	    for( int i = 0; i < board_height; ++i ) {
                for( int j = 0; j < board_width; ++j ) {
                       //cout << (j*square_size) << " " << (i*square_size) << endl;
                       obj.push_back(Point3d(double( (float)j * square_size ), 
					     double( (float)i * square_size ), 0));
		}
	    }
	    object_points.push_back(obj);
	    std::cout << "PASSED: Image triplet detected." << std::endl;

	    if (verb_disp) 
            {
		Mat cimg1,cimg2,cimg3,cimg1_r,cimg2_r,cimg3_r;
      		cvtColor(cam1_image, cimg1, COLOR_GRAY2BGR);
      		cvtColor(cam2_image, cimg2, COLOR_GRAY2BGR);
      		cvtColor(cam3_image, cimg3, COLOR_GRAY2BGR);
		
		cv::drawChessboardCorners(cimg1, board_size, corners1, found1);
		cv::drawChessboardCorners(cimg2, board_size, corners2, found2);
		cv::drawChessboardCorners(cimg3, board_size, corners3, found3);

		double sf = 240./MAX(cam1_image.rows, cam1_image.cols);
		cv::resize(cimg1, cimg1_r, Size(), sf, sf);
		cv::resize(cimg2, cimg2_r, Size(), sf, sf);
		cv::resize(cimg3, cimg3_r, Size(), sf, sf);
		cv::imshow("cam1",cimg1_r);
		cv::imshow("cam2",cimg2_r);
	        cv::imshow("cam3",cimg3_r);
	        cv::waitKey(50);
	    }

	}
	else {
	    std::cout << "FAILED: Image triplet excluded." << std::endl;
	}
    }
    
    if (verb_disp) {
        cv::destroyWindow("cam1");
        cv::destroyWindow("cam2");
        cv::destroyWindow("cam3"); 
    }
    std::cout << std::endl; 
    std::cout << "Detected " << valid_triplet << " triplets for calibration." << std::endl;
    for (int i = 0; i < imagePoints1.size(); i++) 
    {
        vector< Point2d > v1, v2, v3;
        for (int j = 0; j < imagePoints1[i].size(); j++) 
	{
            v1.push_back(Point2d((double)imagePoints1[i][j].x, (double)imagePoints1[i][j].y));
            v2.push_back(Point2d((double)imagePoints2[i][j].x, (double)imagePoints2[i][j].y));
            v3.push_back(Point2d((double)imagePoints3[i][j].x, (double)imagePoints3[i][j].y));
        }
        cam1_img_points.push_back(v1);
        cam2_img_points.push_back(v2);
	cam3_img_points.push_back(v3);
    }
}






int main(int argc, char const *argv[])
{
    std::cout << std::endl << "Tri-imager Camera Calibration\n" << std::endl;

    int board_width, board_height;
    float square_size;
    char* image_directory;
    char* b1_param_file;
    char* b2_param_file;
    int verb_disp;


    static struct poptOption options[] = {
    { "board_width",'w',POPT_ARG_INT,&board_width,0,"Checkerboard width","NUM" },
    { "board_height",'h',POPT_ARG_INT,&board_height,0,"Checkerboard height","NUM" },
    { "square_size",'s',POPT_ARG_FLOAT,&square_size,0,"Checkerboard square size","NUM" },
    { "img_dir",'d',POPT_ARG_STRING,&image_directory,0,"Directory containing images","STR" },
    { "b1_param_file",'c',POPT_ARG_STRING,&b1_param_file,0,"Destination for b1 calib file","STR" },
    { "b2_param_file",'f',POPT_ARG_STRING,&b2_param_file,0,"Destination for b2 calib file","STR" },
    { "verb_disp",'v',POPT_ARG_INT,&verb_disp,0,"Display intermediate values for debugging","NUM"},
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
    };

    POpt popt(NULL, argc, argv, options, 0);
    int c;
    while((c = popt.getNextOpt()) >= 0) {}


    extractImagePoints(board_width,board_height,square_size,image_directory,verb_disp);

    std::cout << std::endl << "Image points have been extracted. " << std::endl << std::endl;

    std::cout << "Calibration will begin using " << cam1_img_points.size();
    std::cout << " image triplets." << std::endl << std::endl;

    int flag = 0;
    flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    flag |= cv::fisheye::CALIB_CHECK_COND;
    flag |= cv::fisheye::CALIB_FIX_SKEW;
    //flag |= cv::fisheye::CALIB_FIX_K2;
    //flag |= cv::fisheye::CALIB_FIX_K3;
    //flag |= cv::fisheye::CALIB_FIX_K4;

    // Calibrating for Baseline-1 (B-1) | Narrower baseline between Cam-1 and Cam-2
    std::cout << "Calibrating baseline B-1... " << std::endl << std::endl;
    cv::Matx33d K1_b1, K2_b1, R_b1;
    cv::Vec3d T_b1;
    cv::Vec4d D1_b1, D2_b1;
    cv::fisheye::stereoCalibrate(object_points, cam1_img_points, cam2_img_points,
    				K1_b1, D1_b1, K2_b1, D2_b1, image_size, R_b1, T_b1, flag,
			        cv::TermCriteria(3, 12, 0));
    cv::Mat R1_b1, R2_b1, P1_b1, P2_b1, Q_b1;
    cv::fisheye::stereoRectify(K1_b1, D1_b1, K2_b1, D2_b1, image_size, R_b1, T_b1, R1_b1, R2_b1, 
				P1_b1, P2_b1, Q_b1, CV_CALIB_ZERO_DISPARITY, image_size, 0.0, 1.1);
    

    // Calibrating for Baseline-2 (B-2) | Wider baseline between Cam-1 and Cam-3
    std::cout << "Calibrating baseline B-2... " << std::endl << std::endl;
    cv::Matx33d K1_b2, K2_b2, R_b2;
    cv::Vec3d T_b2;
    cv::Vec4d D1_b2, D2_b2;
    cv::fisheye::stereoCalibrate(object_points, cam1_img_points, cam3_img_points,
    				K1_b2, D1_b2, K2_b2, D2_b2, image_size, R_b2, T_b2, flag,
			        cv::TermCriteria(3, 12, 0));
    cv::Mat R1_b2, R2_b2, P1_b2, P2_b2, Q_b2;
    cv::fisheye::stereoRectify(K1_b2, D1_b2, K2_b2, D2_b2, image_size, R_b2, T_b2, R1_b2, R2_b2,
				P1_b2, P2_b2, Q_b2, CV_CALIB_ZERO_DISPARITY, image_size, 0.0, 1.1);

    std::cout << "Saving parameter files.." << std::endl << std::endl;
    
    // Create parameter file for B-1
    cv::FileStorage fs1(b1_param_file, cv::FileStorage::WRITE);
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

    // Create parameter file for B-2
    cv::FileStorage fs2(b2_param_file, cv::FileStorage::WRITE);
    fs2 << "K1" << Mat(K1_b2);
    fs2 << "K2" << Mat(K2_b2);
    fs2 << "D1" << D1_b2;
    fs2 << "D2" << D2_b2;
    fs2 << "R" << Mat(R_b2);
    fs2 << "T" << T_b2;
    fs2 << "R1" << R1_b2;
    fs2 << "R2" << R2_b2;
    fs2 << "P1" << P1_b2;
    fs2 << "P2" << P2_b2;
    fs2 << "Q" << Q_b2;
    fs2.release();
    std::cout << "Saved B-2 parameter file." << std::endl << std::endl;
    
    return 1;
}
