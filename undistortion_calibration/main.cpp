/* Copyright 2021 iwatake2222

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
/*** Include ***/
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <vector>
#include <numeric>

#include <opencv2/opencv.hpp>


/*** Macro ***/

/*** Global variable ***/

/*** Function ***/
static inline float Deg2Rad(float deg) { return static_cast<float>(deg * M_PI / 180.0); }
static inline float Rad2Deg(float rad) { return static_cast<float>(rad * 180.0 / M_PI); }

int main(int argc, char *argv[])
{
    static constexpr int32_t kHorizonalCrossCount = 7;
    static constexpr int32_t kVerticalCrossCount = 6;
    static const std::vector<std::string> image_path_list = {
        RESOURCE_DIR"/chessboard/left01.jpg",
        RESOURCE_DIR"/chessboard/left02.jpg",
        RESOURCE_DIR"/chessboard/left03.jpg",
        RESOURCE_DIR"/chessboard/left04.jpg",
        RESOURCE_DIR"/chessboard/left05.jpg",
        RESOURCE_DIR"/chessboard/left06.jpg",
        RESOURCE_DIR"/chessboard/left07.jpg",
        RESOURCE_DIR"/chessboard/left08.jpg",
        RESOURCE_DIR"/chessboard/left09.jpg",
        RESOURCE_DIR"/chessboard/left11.jpg",
        RESOURCE_DIR"/chessboard/left12.jpg",
        RESOURCE_DIR"/chessboard/left13.jpg",
        RESOURCE_DIR"/chessboard/left14.jpg",
    };

    static const cv::Size image_size = cv::imread(image_path_list[0]).size();
    static const cv::Size chessboard_pattern(kHorizonalCrossCount, kVerticalCrossCount);


    std::vector<cv::Point3f> object_point;
    for (int32_t i = 0; i < kHorizonalCrossCount * kVerticalCrossCount; i++) {
        object_point.push_back(cv::Point3f(static_cast<float>(i / kHorizonalCrossCount), static_cast<float>(i % kHorizonalCrossCount), 0.0f));
    }

    std::vector<std::vector<cv::Point3f>> object_point_list;
    std::vector<std::vector<cv::Point2f>> image_point_list;
    for (int32_t i = 0; i < image_path_list.size(); i++) {
        cv::Mat image_chessboard = cv::imread(image_path_list[i]);
        if (image_chessboard.channels() == 3) cv::cvtColor(image_chessboard, image_chessboard, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> image_point;
        bool found = findChessboardCorners(image_chessboard, chessboard_pattern, image_point, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            printf("corner found\n");
            object_point_list.push_back(object_point);
            image_point_list.push_back(image_point);
            //drawChessboardCorners(image_chessboard, chessboard_pattern, cv::Mat(image_point), true);
            //cv::imshow("test", image_chessboard);
            //cv::waitKey(-1);
        } else {
            printf("corner not found\n");
        }
    }
    

    std::vector<cv::Mat> rvec;
    std::vector<cv::Mat> tvec;
    cv::Mat K;
    cv::Mat dist_coeff;
    calibrateCamera(object_point_list, image_point_list, image_size, K, dist_coeff, rvec, tvec, cv::CALIB_FIX_K3);
    cv::Mat mapx, mapy;
    cv::initUndistortRectifyMap(K, dist_coeff, cv::Mat(), K, image_size, CV_32FC1, mapx, mapy);

    cv::FileStorage fs("calib.yaml", cv::FileStorage::WRITE);
    fs << "camera_matrix" << K;
    fs << "dist_coeff" << dist_coeff;
    fs << "rvec" << rvec;
    fs << "tvec" << tvec;
    fs << "mapx" << mapx;
    fs << "mapy" << mapy;
    fs.release();

    for (int32_t i = 0; i < image_path_list.size(); i++) {
        cv::Mat image_chessboard = cv::imread(image_path_list[i]);
        cv::Mat image_undistorted;
#if 0
        cv::undistort(image_chessboard, image_undistorted, K, dist_coeff);
#else
        cv::remap(image_chessboard, image_undistorted, mapx, mapy, cv::INTER_LINEAR);
#endif
        cv::imshow("image_original", image_chessboard);
        cv::imshow("image_undistorted", image_undistorted);
        cv::waitKey(-1);
    }

    return 0;
}
