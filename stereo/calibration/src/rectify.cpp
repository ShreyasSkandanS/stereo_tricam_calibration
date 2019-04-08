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

// Manually define dimensions
#define IMG_WIDTH 960
#define IMG_HEIGHT 800

void rectifyImageSequence(char* image_directory, char* param_file, int verb_disp, bool reuse_flag, int baseline_opt)
{
    vector<String> image_filenames;
    glob(image_directory,image_filenames);
    if (image_filenames.size() > 0)
        std::cout << "Identified " << image_filenames.size() << " images." << std::endl << std::endl;
    else 
    {
        std::cout << "Could not identify any images. Check rectification image path and try again." << std::endl << std::endl;
	return;
    }

    Vec3d T;
    Vec4d D1,D2;
    cv::Mat R1, R2, P1, P2, Q, K1, K2, R;
    std::cout <<"Calibration file : "<< param_file << std::endl;
    cv::FileStorage fs(param_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "Failed to open calibration parameter file." << std::endl;
    	return;
    }
    fs["K1"] >> K1;
    fs["K2"] >> K2;
    fs["R"] >> R;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;
    fs["T"] >> T;
    fs["D1"] >> D1;
    fs["D2"] >> D2;

    double* data_k1c = reinterpret_cast<double*>(K1.data);
    Matx33d K1_c(data_k1c);
    double* data_k2c = reinterpret_cast<double*>(K2.data);
    Matx33d K2_c(data_k2c);
    double* data_R = reinterpret_cast<double*>(R.data);
    Matx33d R_c(data_R);

    std::cout << "Read status: " << fs.isOpened() << std::endl;
    std::cout << "Finished reading in parameter values." << std::endl;

    cv::Mat rmap[2][2];

    cv::Size image_size;
    image_size.height = IMG_HEIGHT;
    image_size.width = IMG_WIDTH;

    if (reuse_flag)
    {
	std::cout << "Re-estimating rectification parameters.. " << std::endl << std::endl;
        cv::fisheye::stereoRectify(K1, D1, K2, D2, image_size, R, T, 
			       R1, R2, P1, P2, Q, CV_CALIB_ZERO_DISPARITY,
			       image_size, 0.0, 1.1);
        cv::fisheye::initUndistortRectifyMap(K1, D1, R1, P1, image_size, 
					CV_16SC2, rmap[0][0], rmap[0][1]);
        cv::fisheye::initUndistortRectifyMap(K2, D2, R2, P2, image_size, 
					CV_16SC2, rmap[1][0], rmap[1][1]);
	if (verb_disp) {
	    std::cout << "\n\n--------------------- RE-ESTIMATED RECTIFICATION ----------------------\n";
            std::cout << "Rect (R1): \n" << R1 << std::endl << std::endl << "Rect (R2): \n" << R2 << std::endl << std::endl ;
            std::cout << "Rect (P1): \n" << P1 << std::endl << std::endl << "Rect (P2): \n" << P2 << std::endl << std::endl ;
            std::cout << "Rect (Q): \n" << Q << std::endl;
	    std::cout << "-----------------------------------------------------------------------\n";
        }
    }
    else
    {
	std::cout << "Using original rectification parameters.. " << std::endl << std::endl;
        cv::fisheye::initUndistortRectifyMap(K1_c, D1, R1, P1, image_size, 
					CV_16SC2, rmap[0][0], rmap[0][1]);
        cv::fisheye::initUndistortRectifyMap(K2_c, D2, R2, P2, image_size, 
					CV_16SC2, rmap[1][0], rmap[1][1]);
    }

    for (int im_ctr = 0; im_ctr < image_filenames.size(); im_ctr++) 
    {
	std::cout << "Image Pair : " << im_ctr << std::endl;
        cv::Mat full_image = imread(image_filenames[im_ctr],0);

	//cv::imshow("full_image",full_image);
	//cv::waitKey(1);

	cv::Mat cam1_image, cam2_image, cam3_image;
	cv::Rect cam1_roi(0,2*(full_image.rows/3),full_image.cols,(full_image.rows/3));
	cv::Rect cam2_roi(0,1*(full_image.rows/3),full_image.cols,(full_image.rows/3));
	cv::Rect cam3_roi(0,0*(full_image.rows/3),full_image.cols,(full_image.rows/3));
	cam1_image = full_image(cam1_roi);
	cam2_image = full_image(cam2_roi);
	cam3_image = full_image(cam3_roi);

	//cv::imshow("cam1",cam1_image);
	//cv::imshow("cam2",cam2_image);
	//cv::imshow("cam3",cam3_image);
	//cv::waitKey(1);
        
	cv::Mat rect_cam1, rect_cam2, rect_cam3;
	cv::Mat rect_left, rect_right;
	if (baseline_opt == 0) {
	    cv::remap(cam1_image,rect_cam1,rmap[0][0],rmap[0][1],INTER_LINEAR);
            cv::remap(cam2_image,rect_cam2,rmap[1][0],rmap[1][1],INTER_LINEAR);	

	    //cv::imshow("rectcam1",rect_cam1);
	    //cv::imshow("rectcam2",rect_cam2);
	    //cv::waitKey(1);

	    rect_left = rect_cam1;
	    rect_right = rect_cam2;
	}
	else {
	    cv::remap(cam1_image,rect_cam1,rmap[0][0],rmap[0][1],INTER_LINEAR);
            cv::remap(cam3_image,rect_cam3,rmap[1][0],rmap[1][1],INTER_LINEAR);	

	    //cv::imshow("rectcam1",rect_cam1);
	    //cv::imshow("rectcam3",rect_cam3);
	    //cv::waitKey(1);

	    rect_left = rect_cam1;
	    rect_right = rect_cam3;
	}

	if (verb_disp) 
        {
            cv::Mat canvas;
            double sf;
            int w, h;
            sf = 480./MAX(image_size.width, image_size.height);
            w = cvRound(image_size.width*sf);
            h = cvRound(image_size.height*sf);
            canvas.create(h, w*2, CV_8UC3);

	    cv::Mat cl_img, cr_img;
	    cv::cvtColor(rect_left,cl_img,COLOR_GRAY2BGR);
	    cv::cvtColor(rect_right,cr_img,COLOR_GRAY2BGR);

	    cv::Mat canvasPart_1 = canvas(Rect(0,0,w,h));
	    cv::resize(cl_img,canvasPart_1,canvasPart_1.size(),0,0,INTER_AREA);

	    cv::Mat canvasPart_2 = canvas(Rect(w,0,w,h));
	    cv::resize(cr_img,canvasPart_2,canvasPart_2.size(),0,0,INTER_AREA);  
 
	    for(int j = 0; j < canvas.rows; j += 16 )
	        line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

	    cv::imshow("Rectified Images", canvas);

	    //char savefilenamer[50];
	    //sprintf(savefilenamer,"%s%05d.png","/home/shreyas/savedimgs/rect/rect",(int)im_idx);
            //cv::imwrite(savefilenamer,canvas);

	    char ch = (char)waitKey();
	    if( ch == 27 || ch == 'q' || ch == 'Q')
	        return;         
	}
    }
}


int main(int argc, char const *argv[])
{
    std::cout << std::endl << "Tri-imager Camera Rectification\n" << std::endl;

    char* image_directory;
    char* param_file;
    bool reuse_flag = false;
    int verb_disp;
    int baseline_opt;

    static struct poptOption options[] = {
    { "img_dir",'d',POPT_ARG_STRING,&image_directory,0,"Directory containing images","STR" },
    { "param_file",'c',POPT_ARG_STRING,&param_file,0,"Destination for b1 calib file","STR" },
    { "verb_disp",'v',POPT_ARG_INT,&verb_disp,0,"Display intermediate values for debugging","NUM"},
    { "baseline_opt",'b',POPT_ARG_INT,&baseline_opt,0,"Pick which baseline - b1/b2","NUM"},
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
    };

    POpt popt(NULL, argc, argv, options, 0);
    int c;
    while((c = popt.getNextOpt()) >= 0) {}

    rectifyImageSequence(image_directory,param_file,verb_disp,reuse_flag,baseline_opt);
 
    return 1;
}
