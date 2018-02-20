#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
//
//
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>


using namespace cv;
using namespace std;


namespace CVUtils
{

	cv::Mat PoseEstimateNotes(vector<cv::Point3d> points, vector<cv::Point2d> projections);

	Vec3b MeanColor(Mat& imageCopy);

	bool ExpandColor(int row, int column, Mat& imageCopy, Vec3b& note_color, Vec3b& mark_color, std::function<bool(Vec3b acolor)> BelogsToNote, int minimum_amount_pixels, Point2d& center);

	vector<Point2d> FindNotesCenters(Mat& image, Vec3b note_color, std::function<bool(Vec3b acolor)> BelogsToNote);

	// Mat camera(4,4,CV_64FC1) camera1  and 2 
	Mat Homografy(Mat& camera1, Mat& camera2);

	void AKAZE(Mat& img1, Mat& img2, Mat& homography, float nn_match_ratio = 0.9f, float inlier_threshold = 2.5f);

	void DisplayCamera();


}




