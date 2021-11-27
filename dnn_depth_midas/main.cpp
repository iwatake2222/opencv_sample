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

#include "common_helper_cv.h"
#include "depth_engine.h"

/*** Macro ***/
static constexpr char kInputImageFilename[] = RESOURCE_DIR"/room_00.jpg";


/*** Global variable ***/


/*** Function ***/
int main(int argc, char *argv[])
{
    /* Initialize Model */
    DepthEngine depth_engine;
    depth_engine.Initialize("");

    /* Find source image */
    std::string input_name = (argc > 1) ? argv[1] : kInputImageFilename;
    cv::VideoCapture cap;   /* if cap is not opened, src is still image */
    if (!CommonHelper::FindSourceImage(input_name, cap)) {
        return -1;
    }

    /* Process for each frame */
    int32_t frame_cnt = 0;
    for (frame_cnt = 0; cap.isOpened() || frame_cnt < 1; frame_cnt++) {
        /* Read image */
        cv::Mat image_input;
        if (cap.isOpened()) {
            cap.read(image_input);
        } else {
            image_input = cv::imread(input_name);
        }
        if (image_input.empty()) break;

        /* Detect face */
        cv::Mat mat_depth;
        depth_engine.Process(image_input, mat_depth);

        /* Draw Result */
        cv::Mat image_depth;
        //cv::applyColorMap(mat_depth, image_depth, cv::COLORMAP_JET);
        image_depth = mat_depth;
        cv::resize(image_depth, image_depth, image_input.size());

        cv::imshow("Input", image_input);
        cv::imshow("Depth", image_depth);
        int32_t key = cv::waitKey(1);
        if (key == 'q') break;
    }

    depth_engine.Finalize();
    cv::waitKey(-1);

    return 0;
}
