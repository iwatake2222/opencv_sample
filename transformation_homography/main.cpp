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

/*** Macro ***/
static constexpr char kWindowMain[] = "WindowMain";
static constexpr char kWindowOutput[] = "WindowOutput";
static constexpr float kFovDeg = 130.0f;


/*** Global variable ***/
static std::vector<cv::Point2f> selecting_point_list;

/*** Function ***/


static void loop_main(const cv::Mat& image_org)
{
    cv::Mat& image = image_org.clone();
    cvui::context(kWindowMain);
    for (int32_t i = 1; i < static_cast<int32_t>(selecting_point_list.size()); i++) {
        cv::line(image, selecting_point_list[i - 1], selecting_point_list[i], cv::Scalar(255, 0, 0), 2);
    }
    cvui::imshow(kWindowMain, image);


    if (selecting_point_list.size() >= 4) {
        cvui::context(kWindowOutput);
        float width = std::abs(selecting_point_list[1].x - selecting_point_list[2].x);
        float height = std::abs(selecting_point_list[0].y - selecting_point_list[1].y);
        height *= 5;
        cv::Point2f dst_point_list[] = {
            { 0, 0 },
            { 0, height },
            { width, height },
            { width, 0 },
        };

        cv::Mat mat_transform = cv::getPerspectiveTransform(selecting_point_list.data(), dst_point_list);
        cv::Mat mat_output = cv::Mat(cv::Size(static_cast<int32_t>(width), static_cast<int32_t>(height)), CV_8UC3, cv::Scalar(70, 70, 70));
        cv::warpPerspective(image_org, mat_output, mat_transform, mat_output.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

        cvui::imshow(kWindowOutput, mat_output);
        selecting_point_list.clear();
    }
}


static void CallbackMouseMain(int32_t event, int32_t x, int32_t y, int32_t flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONUP) {
    } else if (event == cv::EVENT_LBUTTONDOWN) {
        cv::Point2f point(static_cast<float>(x), static_cast<float>(y));
        selecting_point_list.push_back(point);
    } else {
    }
}


int main(int argc, char* argv[])
{
    cvui::init(kWindowMain);
    cvui::init(kWindowOutput);

    cv::setMouseCallback(kWindowMain, CallbackMouseMain);

    static const std::string image_path = RESOURCE_DIR"/dashcam_00.jpg";
    cv::Mat image_org = cv::imread(image_path);


    while (true) {
        loop_main(image_org);
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
    }

    return 0;
}
