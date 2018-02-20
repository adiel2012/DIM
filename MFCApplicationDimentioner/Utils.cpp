#include "stdafx.h"
#include "Utils.h"


cv::Mat CVUtils::PoseEstimateNotes(vector<cv::Point3d> points, vector<cv::Point2d> projections)
{

	int num_points = points.size();

	cv::Mat points3D(4, num_points, CV_64FC1);
	cv::Mat points2D(3, num_points, CV_64FC1);

	for (size_t i = 0; i < num_points; i++)
	{
		Point3d pp = points[i];
		points3D.at<double>(i, 0) = pp.x;
		points3D.at<double>(i, 1) = pp.y;
		points3D.at<double>(i, 2) = pp.z;

		Point2d pp2 = projections[i];
		points2D.at<double>(i, 0) = pp2.x;
		points2D.at<double>(i, 1) = pp2.y;
	}

	cv::Mat inv1 = points3D;   // for the size
	invert(points3D.t(), inv1);

	cv::Mat camera = (inv1 * points2D.t()).t();

	return camera;
}

Vec3b CVUtils::MeanColor(Mat& imageCopy)
{
	double asum[3] = {0,0,0};
	for (size_t i = 0; i < imageCopy.cols; i++)
	{
		for (size_t j = 0; j < imageCopy.rows; j++)
		{
			Vec3b pto = imageCopy.at<Vec3b>(j, i);

			asum[0] += pto[0];
			asum[1] += pto[1];
			asum[2] += pto[2];
		}
	}
	int numpoints = imageCopy.cols * imageCopy.rows;
	return Vec3b(asum[0] / numpoints, asum[1] / numpoints, asum[2] / numpoints);

}

bool CVUtils::ExpandColor(int row, int column, Mat& imageCopy, Vec3b& note_color, Vec3b& mark_color, std::function<bool(Vec3b acolor)> BelogsToNote, int minimum_amount_pixels, Point2d& center)
{
	int quantity = 0;
	auto mm = new int[8][2]{ {0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1} };

	std::deque<Point2d> buffer;



	//add point to buffer
	buffer.push_front(Point2d(column, row));

	//mark original point
	auto aptr = imageCopy.at<Vec3b>(column, row);
	aptr[0] = mark_color[0];
	aptr[1] = mark_color[1];
	aptr[2] = mark_color[2];

	double acummulate_x = column, acummulate_y = row;
	int num_points = 1;

	while (buffer.size() > 0)
	{
		Point2d curr = buffer[0];  buffer.pop_front();
		for (size_t i = 0; i < 8; i++)
		{
			Point2d piv(curr.x + mm[i][0], curr.y + mm[i][1]);
			if (piv.x >= 0 && piv.y >= 0 && piv.x < imageCopy.cols && piv.y < imageCopy.rows)
			{
				Vec3b pto = imageCopy.at<Vec3b>(piv.y, piv.x);   //
				if (BelogsToNote(pto))
				{
					//mark original point
					aptr = imageCopy.at<Vec3b>(piv);
					aptr[0] = mark_color[0];
					aptr[1] = mark_color[1];
					aptr[2] = mark_color[2];

					// add the point
					buffer.push_front(piv);

					acummulate_x += piv.x;
					acummulate_y += piv.y;
					num_points++;
				}
			}
			
		}
	}

	center.x = acummulate_x / num_points;  // TODO  round
	center.y = acummulate_y / num_points;


	return quantity >= minimum_amount_pixels;
}

vector<Point2d> CVUtils::FindNotesCenters(Mat& image, Vec3b note_color, std::function<bool(Vec3b acolor)> BelogsToNote)
{

	vector<Point2d> result;
	Mat& imageCopy(image);


	Vec3b mark_color; //TODO
	int minimum_amount_pixels = 30;

	for (size_t j = 0; j < image.rows; j++)
	{
		for (size_t i = 0; i < image.cols; i++)
		{
			Vec3b curr_color = image.at<Vec3b>(j, i);
			if (BelogsToNote(curr_color))
			{
				Point2d center;
				if (ExpandColor(j, i, imageCopy, note_color, mark_color, BelogsToNote, minimum_amount_pixels, center))
				{
					result.push_back(center);
				}
			}

		}
	}

	return result;
}

// Mat camera(4,4,CV_64FC1) camera1  and 2 
Mat CVUtils::Homografy(Mat& camera1, Mat& camera2)
{
	Mat result = camera1.inv() * camera2;
	return result;
}

void CVUtils::AKAZE(Mat& img1, Mat& img2, Mat& homography, float nn_match_ratio, float inlier_threshold)
{

	vector<KeyPoint> kpts1, kpts2;
	Mat desc1, desc2;

	Ptr<cv::AKAZE> akaze = AKAZE::create();
	akaze->detectAndCompute(img1, noArray(), kpts1, desc1);
	akaze->detectAndCompute(img2, noArray(), kpts2, desc2);

	BFMatcher matcher(NORM_HAMMING);
	vector< vector<DMatch> > nn_matches;
	matcher.knnMatch(desc1, desc2, nn_matches, 2);

	vector<KeyPoint> matched1, matched2, inliers1, inliers2;
	vector<DMatch> good_matches;
	for (size_t i = 0; i < nn_matches.size(); i++) {
		DMatch first = nn_matches[i][0];
		float dist1 = nn_matches[i][0].distance;
		float dist2 = nn_matches[i][1].distance;

		if (dist1 < nn_match_ratio * dist2) {
			matched1.push_back(kpts1[first.queryIdx]);
			matched2.push_back(kpts2[first.trainIdx]);
		}
	}

	for (unsigned i = 0; i < matched1.size(); i++) {
		Mat col = Mat::ones(3, 1, CV_64F);
		col.at<double>(0) = matched1[i].pt.x;
		col.at<double>(1) = matched1[i].pt.y;

		col = homography * col;
		col /= col.at<double>(2);
		double dist = sqrt(pow(col.at<double>(0) - matched2[i].pt.x, 2) +
			pow(col.at<double>(1) - matched2[i].pt.y, 2));

		if (dist < inlier_threshold) {
			int new_i = static_cast<int>(inliers1.size());
			inliers1.push_back(matched1[i]);
			inliers2.push_back(matched2[i]);
			good_matches.push_back(DMatch(new_i, new_i, 0));
		}
	}

	Mat res;
	drawMatches(img1, inliers1, img2, inliers2, good_matches, res);
	//imwrite("res.png", res);

	double inlier_ratio = inliers1.size() * 1.0 / matched1.size();
	cout << "A-KAZE Matching Results" << endl;
	cout << "*******************************" << endl;
	cout << "# Keypoints 1:                        \t" << kpts1.size() << endl;
	cout << "# Keypoints 2:                        \t" << kpts2.size() << endl;
	cout << "# Matches:                            \t" << matched1.size() << endl;
	cout << "# Inliers:                            \t" << inliers1.size() << endl;
	cout << "# Inliers Ratio:                      \t" << inlier_ratio << endl;
	cout << endl;



}

void CVUtils::DisplayCamera()
{

	Mat image;
	image = imread("D:\\Adiel\\opencv\\MFCDimentioner\\MFCApplicationDimentioner\\pink.png", CV_LOAD_IMAGE_COLOR);   // Read the file
	Vec3b bg_color = CVUtils::MeanColor(image);
	//Vec3b tempVal = mean(image);

	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return;


	Mat edges;
	namedWindow("edges", 1);
	for (;;)
	{
		Mat frame;
		cap >> frame; // get a new frame from camera
		//cvtColor(frame, edges, COLOR_BGR2GRAY);
		//GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);
		//Canny(edges, edges, 0, 30, 3);

		std::vector<Point2d> points = CVUtils::FindNotesCenters(frame, bg_color, [&](Vec3b acolor){
			return abs(acolor[0] - bg_color[0]) + abs(acolor[1] - bg_color[1]) + abs(acolor[2] - bg_color[2])   <  130;
		});


		imshow("edges", frame);
		if (waitKey(30) >= 0) break;
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
}

