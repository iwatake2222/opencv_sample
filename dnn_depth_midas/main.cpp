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
#include <array>
#include <numeric>
#include <algorithm>

#include <opencv2/opencv.hpp>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

#include "common_helper_cv.h"
#include "depth_engine.h"

/*** Macro ***/
static constexpr char kInputImageFilename[] = RESOURCE_DIR"/parrot.jpg";


/*** Global variable ***/


/*** Function ***/
int main(int argc, char *argv[])
{
    cvui::init("Output");   // use cvui for track bar

    /* Initialize Model */
    DepthEngine depth_engine;
    depth_engine.Initialize();

    /* Find source image */
    std::string input_name = (argc > 1) ? argv[1] : kInputImageFilename;
    cv::VideoCapture cap;   /* if cap is not opened, src is still image */
    if (!CommonHelper::FindSourceImage(input_name, cap)) {
        return -1;
    }
    
    /* Process for each frame */
    for (int32_t frame_cnt = 0; ; frame_cnt++) {
        /* Read image */
        cv::Mat image_input;
        if (cap.isOpened()) {
            cap.read(image_input);
        } else {
            image_input = cv::imread(input_name);
        }
        if (image_input.empty()) break;

        int32_t input_height = (std::min)(400, image_input.cols);
        cv::resize(image_input, image_input, cv::Size((input_height * image_input.cols) / image_input.rows, input_height));

        /* Estimate depth */
        cv::Mat mat_depth;
        depth_engine.Process(image_input, mat_depth);

        /* Draw Depth */
        cv::Mat mat_depth_normlized255;
        depth_engine.NormalizeMinMax(mat_depth, mat_depth_normlized255);
        cv::resize(mat_depth_normlized255, mat_depth_normlized255, image_input.size());
        cv::Mat image_depth;
        cv::applyColorMap(mat_depth_normlized255, image_depth, cv::COLORMAP_JET);
        
        
        /* Draw Image (only near object) */
        cv::Mat image_output = cv::Mat(image_input.size(), CV_8UC3, cv::Scalar(0, 0, 0));
        static int32_t depth_threshold = 255;
        for (int32_t i = 0; i < image_output.total(); i++) {
            if (mat_depth_normlized255.at<uint8_t>(i) <= depth_threshold) {
                image_output.at<cv::Vec3b>(i) = image_input.at<cv::Vec3b>(i);
            }
        }
        cvui::trackbar<int32_t>(image_output, 10, 10, 200, &depth_threshold, 0, 255);

        cv::imshow("Input", image_input);
        cv::imshow("Depth", image_depth);
        cv::imshow("Output", image_output);
        
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
    }

    depth_engine.Finalize();
    cv::waitKey(-1);

    return 0;
}
