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

#include <opencv2/opencv.hpp>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

#include "curve_fitting.h"

/*** Macro ***/
static constexpr char kWindowMain[] = "WindowMain";

static constexpr int32_t kWidth = 1280;
static constexpr int32_t kHeight = 720;


/*** Global variable ***/
static std::vector<cv::Point2f> point_list;

/*** Function ***/
static void CallbackMouseMain(int32_t event, int32_t x, int32_t y, int32_t flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONUP) {
    } else if (event == cv::EVENT_LBUTTONDOWN) {
        cv::Point2f point(static_cast<float>(x), static_cast<float>(y));
        point_list.push_back(point);
    } else {
    }
}

int main(int argc, char* argv[])
{
    cvui::init(kWindowMain);

    cv::setMouseCallback(kWindowMain, CallbackMouseMain);
    
    while (true) {
        cv::Mat image = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(70, 70, 70));
        for (const auto& point : point_list) {
            cv::circle(image, point, 5, cv::Scalar(255, 0, 0), 2);
        }

        double a = 0;
        double b = 0;
        double c = 0;
        if (CurveFitting::SolveLinearRegression<float>(point_list, a, b)) {
            for (int32_t x = 0; x < image.cols - 1; x++) {
                int32_t x0 = x + 0;
                int32_t x1 = x + 1;
                int32_t y0 = static_cast<int32_t>(a * x0 + b);
                int32_t y1 = static_cast<int32_t>(a * x1 + b);
                cv::line(image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(255, 0, 0));
            }
        }

        if (CurveFitting::SolveQuadraticRegression<float>(point_list, a, b, c)) {
            for (int32_t x = 0; x < image.cols - 1; x++) {
                int32_t x0 = x + 0;
                int32_t x1 = x + 1;
                int32_t y0 = static_cast<int32_t>(a * x0 * x0 + b * x0 + c);
                int32_t y1 = static_cast<int32_t>(a * x1 * x1 + b * x1 + c);
                cv::line(image, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(0, 255, 0));
            }
        }

        cv::imshow(kWindowMain, image);
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
        if (key == 'r') {
            /* reset */
            point_list.clear();
        }
    }

    return 0;
}
