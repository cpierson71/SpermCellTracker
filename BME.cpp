// BME.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

struct tailData {
	double angle;
	cv::Point tailLocation;
};

void processFrame(cv::Mat&, cv::Mat&, std::vector<cv::Vec3f>&);

cv::Mat morphSkeleton(cv::Mat);

tailData skeletonAngle(cv::Mat&);



int main()
{
	cv::Mat frame;
	cv::Mat fgMask;
	cv::Mat skel;
	std::vector<cv::Vec3f> circles;
    std::vector<double> angles;
	std::vector<cv::Point> tailLocations;
	cv::VideoCapture cap = cv::VideoCapture("Movie 12.avi");
    bool loop = false;


    int history = 500;
    double varThreshold = 20.0;
    bool detectShadows = false;
    cv::Ptr<cv::BackgroundSubtractorMOG2> pMOG2 = cv::createBackgroundSubtractorMOG2(history, varThreshold, detectShadows);

    if (cap.get(CV_CAP_PROP_FRAME_COUNT) > 0) {
        loop = true;
    }
    else {
        std::cout << "Video could not be read" << std::endl;
        std::cout << "Press enter" << std::endl;
        std::cin.get();
    }
	while (loop) {
		cap >> frame;
		if (!frame.empty() && !(cap.get(CV_CAP_PROP_POS_FRAMES) == cap.get(CV_CAP_PROP_FRAME_COUNT))) {
			pMOG2->apply(frame, fgMask);
			cv::imshow("Foreground", fgMask);

			if (cap.get(CV_CAP_PROP_POS_FRAMES) != 1) {

				processFrame(fgMask, skel, circles);

				//Need to mix skeleton into frame
				cv::Mat blue(frame.rows, frame.cols, CV_8U);
				std::vector<int> from_to{ 0,0 };
				cv::mixChannels(frame, blue, from_to);

				cv::Mat sum(frame.rows, frame.cols, CV_8U);
				// Dilate skeleton to make it more visible
				//cv::dilate(skel, skel, cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(5, 5)));
				//cv::medianBlur(skel, skel, 3);


				cv::Mat nonZeroCoordinates;
				cv::findNonZero(skel, nonZeroCoordinates);
				for (int i = 0; i < nonZeroCoordinates.total(); i++) {
					//sum.at<uchar>(nonZeroCoordinates.at<cv::Point>(i)) = 255;
					cv::circle(frame, nonZeroCoordinates.at<cv::Point>(i), 1, cv::Scalar(255, 0,00));
				}

                tailData tail = skeletonAngle(skel);
                angles.push_back(tail.angle);
				tailLocations.push_back(tail.tailLocation);

			}


			for (size_t i = 0; i < circles.size(); i++)
			{
				cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				int radius = cvRound(circles[i][2]);
				//std::cout << radius << std::endl;
				// circle center
				cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
				// circle outline
				cv::circle(frame, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
			}

            cv::String progress = "Frame " + std::to_string(int(cap.get(CV_CAP_PROP_POS_FRAMES))) + " out of " + std::to_string(int(cap.get(CV_CAP_PROP_FRAME_COUNT)));
            cv::putText(frame, progress, cv::Point(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0),1);
			cv::imshow("Frame", frame);

			//if (!skel.empty())
				//cv::imshow("Foreground", skel);

		}
		else {
            break;
		}

		char k = cv::waitKey(10);
		if (k == 27) {
            break;
		}

	}

	cap.release();
	cv::destroyAllWindows();

    std::ofstream tailDataFile("tailData.txt");
	for (int n = 0; n < angles.size(); n++) {
		tailDataFile << angles[n] << '\t' << tailLocations[n].x << '\t' << tailLocations[n].y << std::endl;
	}

	std::cout << "Done" << std::endl;
    return 0;
}


void processFrame(cv::Mat& frame, cv::Mat& skel, std::vector<cv::Vec3f>& circles)
{
	cv::Mat kernel = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(5, 5));
	cv::medianBlur(frame, frame, 7);
	skel = morphSkeleton(frame);
	cv::morphologyEx(frame, frame, cv::MorphTypes::MORPH_OPEN, kernel);
	cv::medianBlur(frame, frame, 21);
	cv::HoughCircles(frame, circles, cv::HOUGH_GRADIENT, 2, frame.rows / 6, 100, 35, 20, 35);

	return;
}

cv::Mat morphSkeleton(cv::Mat input)
{
	cv::Mat frame;
	input.copyTo(frame);
	cv::Mat skel(frame.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp;
	cv::Mat erosion;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
	bool done;
	do
	{
		cv::erode(frame, erosion, element);
		cv::dilate(erosion, temp, element);
		cv::subtract(frame, temp, temp);
		cv::bitwise_or(skel, temp, skel);
		erosion.copyTo(frame);

		done = (cv::countNonZero(frame) == 0);
	} while (!done);

	return skel;
}

tailData skeletonAngle(cv::Mat& skel)
{
	tailData tail;
    double angle;
	cv::Point tailLocation;
    cv::Mat nonZeroCoordinates;

    int minX = skel.cols + 1;
    int maxX = -1;
    int minY = skel.rows + 1;
    int maxY = -1;
    cv::findNonZero(skel, nonZeroCoordinates);
    for (int i = 0; i < nonZeroCoordinates.total(); i++) {
        int xVal = nonZeroCoordinates.at<cv::Point>(i).x;
        int yVal = nonZeroCoordinates.at<cv::Point>(i).y;
        if (xVal < minX)
            minX = xVal;
        else if (xVal > maxX)
            maxX = xVal;
        if (yVal < minY)
            minY = yVal;
        else if (yVal > maxY)
            maxY = yVal;
    }


	tailLocation = cv::Point(maxX, maxY);
    double yDiff = maxY - minY;
    double xDiff = maxX - minX;
    if (xDiff == 0)
        xDiff = xDiff + 0.001;

    angle = atan(yDiff / xDiff);
	tail.angle = angle;
	tail.tailLocation = tailLocation;

    return tail;
}

