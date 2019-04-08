// OPENCV LIBRARIES
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// OTHER INCLUDED LIBRARIES
#include <stdio.h>
#include <stdlib.h> 
#include <vector>
#include <string>
#include <math.h>
#include <sstream>
#include <fstream>

using namespace std;


void print_vec(std::vector<double> const &v)
{
  for(auto const &e : v)
	std::cout << e << ", ";
  std::cout << std::endl;
}

void getCSVElements(std::string line, std::string &field)
{
        std::string::size_type start_idx;
        std::string::size_type end_idx;
        start_idx = line.find("[") + 1;
        end_idx   = line.find("]");
        field = line.substr(start_idx, end_idx - start_idx);
}

cv::Mat vectorToCVMat(std::vector<double> vector, int rows, int cols)
{
	cv::Mat Mat_out(rows,cols,CV_64FC1);
	for (int r = 0; r < rows; r++) {
		for (int c = 0; c < cols; c++) {
			Mat_out.at<double>(r,c) = vector[c + (rows*r)];
		}
	}
	//std::cout << "Output Matrix : " << Mat_out << std::endl;
	return Mat_out;
}

void extractDataField(std::string field, std::vector<double> &data)
{
	std::istringstream csv_string(field);
	string numbers;
        while (std::getline(csv_string, numbers, ',')) {
        	std::istringstream iss(numbers);
                double i;
                iss >> i;
                data.push_back(i);
        }
}

cv::Mat extractFullField(std::string line, int rows, int cols)
{
	std::vector<double> vector_v;
	std::string field;
	getCSVElements(line, field);
	extractDataField(field, vector_v);
	//std::cout << "Added " << vector_v.size() << " elements to vector." << std::endl;
	cv::Mat matrix_out = vectorToCVMat(vector_v, rows, cols);
	return matrix_out;
}

int main(int argc, char **argv)
{
	if (argc < 5)
	{
		std::cout << "Not enough input arguments. Try again.." << std::endl;
		std::cout << "[FORMAT] ./convert left_in_file right_in_file extrin_file output_file width height" << std::endl;
		exit(1);
	}

	std::string l_intrinsics_file = argv[1];
	std::string r_intrinsics_file = argv[2];
	std::string extrinsics_file = argv[3];
	std::string save_file = argv[4];

	std::string img_width_s = argv[5];
	std::string img_height_s = argv[6];
	int IMG_WIDTH = std::atoi(img_width_s.c_str());
	int IMG_HEIGHT = std::atoi(img_height_s.c_str());

	std::cout << "Extracting left camera intrinsics.." << std::endl;
	std::ifstream left_intrinsics(l_intrinsics_file.c_str());
	std::string line;
	int l_line_ctr = 0;
	cv::Mat K1;
	cv::Mat d1_mat;
	std::vector<double> d1;
	while (std::getline(left_intrinsics, line)) 
	{
		l_line_ctr = l_line_ctr + 1;
		std::istringstream iss(line);
		if (l_line_ctr == 3) {
			K1 = extractFullField(line, 3, 3);
		}
		if (l_line_ctr == 8) {
			d1_mat = extractFullField(line, 1, 4);
#if 0
                        d1[0] = d1_mat.at<double>(0,0);
                        d1[1] = d1_mat.at<double>(0,1);
                        d1[2] = d1_mat.at<double>(0,2);
                        d1[3] = d1_mat.at<double>(0,3);
#else
			d1.push_back(d1_mat.at<double>(0,0));
			d1.push_back(d1_mat.at<double>(0,1));
			d1.push_back(d1_mat.at<double>(0,2));
			d1.push_back(d1_mat.at<double>(0,3));
#endif
		}
	}

	std::cout << std::endl << "Extracting right camera intrinsics.." << std::endl;
        std::ifstream right_intrinsics(r_intrinsics_file.c_str());
        int r_line_ctr = 0;
	cv::Mat K2;
	cv::Mat d2_mat;
	std::vector<double> d2;
        while (std::getline(right_intrinsics, line))
        {
                r_line_ctr = r_line_ctr + 1;
                std::istringstream iss(line);
                if (r_line_ctr == 3) {
                        K2 = extractFullField(line, 3, 3);
                }
                if (r_line_ctr == 8) {
                        d2_mat = extractFullField(line, 1, 4);
#if 0
                        d2[0] = d2_mat.at<double>(0,0);
                        d2[1] = d2_mat.at<double>(0,1);
                        d2[2] = d2_mat.at<double>(0,2);
                        d2[3] = d2_mat.at<double>(0,3);
#else
			d2.push_back(d2_mat.at<double>(0,0));
			d2.push_back(d2_mat.at<double>(0,1));
			d2.push_back(d2_mat.at<double>(0,2));
			d2.push_back(d2_mat.at<double>(0,3));
#endif
                }
        }

	std::cout << std::endl << "Exracting L->R camera extrinsics.." << std::endl;
	std::ifstream extrinsics(extrinsics_file.c_str());
	int ex_line_ctr = 0;
	cv::Matx33d R1_2;
	cv::Vec3d T1_2;
		
	cv::Mat temp_mat;
	while (std::getline(extrinsics, line))
	{
		ex_line_ctr = ex_line_ctr + 1;
		std::istringstream iss(line);
		if (ex_line_ctr == 20) {
			temp_mat = extractFullField(line, 1,4);
			R1_2(0,0) = temp_mat.at<double>(0,0);
			R1_2(0,1) = temp_mat.at<double>(0,1);
			R1_2(0,2) = temp_mat.at<double>(0,2);
			T1_2[0] = temp_mat.at<double>(0,3);
		}
		if (ex_line_ctr == 21) {
			temp_mat = extractFullField(line, 1,4);
			R1_2(1,0) = temp_mat.at<double>(0,0);
			R1_2(1,1) = temp_mat.at<double>(0,1);
			R1_2(1,2) = temp_mat.at<double>(0,2);
			T1_2[1] = temp_mat.at<double>(0,3);	
		}
		if (ex_line_ctr == 22) {
                        temp_mat = extractFullField(line, 1,4);
                        R1_2(2,0) = temp_mat.at<double>(0,0);
                        R1_2(2,1) = temp_mat.at<double>(0,1);
                        R1_2(2,2) = temp_mat.at<double>(0,2);
                        T1_2[2] = temp_mat.at<double>(0,3);
		}
	}


	std::cout << std::endl << std::endl;
	std::cout << "Final Parameters: " << std::endl << std::endl;

	std::cout << "Rotation Matrix : " << R1_2 << std::endl;
	std::cout << "Translation Matrix : " << T1_2 << std::endl;
	cv::Matx33d K1_c = K1;
	cv::Matx33d K2_c = K2;
	std::cout << "K1 : " << K1_c << std::endl;
	std::cout << "K2 : " << K2_c << std::endl;
	std::cout << "D1 : "; print_vec(d1);
	std::cout << "D2 : "; print_vec(d2);

	cv::Size image_size(IMG_WIDTH,IMG_HEIGHT);
	std::cout << "Width : " << image_size.width << " Height : " << image_size.height << std::endl;
	std::cout << "image_size: " << image_size << std::endl;

	cv::Mat R1_o, R2_o, P1_o, P2_o, Q_o;
	cv::fisheye::stereoRectify(K1_c, d1, K2_c, d2, image_size, R1_2, T1_2, R1_o, R2_o, 
				P1_o, P2_o, Q_o, CV_CALIB_ZERO_DISPARITY);

	std::cout << std::endl << "Rectification Parameters Calculated..." << std::endl;
	std::cout << "R1 : " << R1_o << std::endl;
	std::cout << "R2 : " << R2_o << std::endl;
	std::cout << "P1 : " << P1_o << std::endl;
	std::cout << "P2 : " << P2_o << std::endl;
	std::cout << "Q  : " << Q_o << std::endl;

	std::cout << "Saving parameter files.." << std::endl << std::endl;
    	cv::FileStorage fs1(save_file, cv::FileStorage::WRITE);
    	fs1 << "K1" << cv::Mat(K1_c);
    	fs1 << "K2" << cv::Mat(K2_c);
    	fs1 << "D1" << d1;
    	fs1 << "D2" << d2;
    	fs1 << "R" << cv::Mat(R1_2);
    	fs1 << "T" << T1_2;
    	fs1 << "R1" << R1_o;
    	fs1 << "R2" << R2_o;
    	fs1 << "P1" << P1_o;
    	fs1 << "P2" << P2_o;
    	fs1 << "Q" << Q_o;
    	fs1.release();
    	std::cout << "Saved parameter file." << std::endl << std::endl;

}



