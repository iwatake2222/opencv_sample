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
#include "camera_model.h"

/*** Macro ***/
static constexpr char kInputImageFilename[] = RESOURCE_DIR"/room_00.jpg";
static constexpr int32_t kWidth = 640;
static constexpr int32_t kHeight = 480;
static constexpr float   kFovDeg = 60.0f;

/*** Global variable ***/
static CameraModel camera_2d_to_3d;
static CameraModel camera_3d_to_2d;

void initialize_camera(int32_t width, int32_t height)
{
    camera_2d_to_3d.parameter.SetIntrinsic(width, height, CameraModel::FocalLength(width, kFovDeg));
    camera_2d_to_3d.parameter.SetDist({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
    camera_2d_to_3d.parameter.SetExtrinsic(
        { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
        { 0.0f, 0.0f, 0.0f }, true);   /* tvec (in world coordinate) */

    camera_3d_to_2d.parameter.SetIntrinsic(kWidth, kHeight, CameraModel::FocalLength(kWidth, kFovDeg));
    camera_3d_to_2d.parameter.SetDist({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
    //camera_3d_to_2d.parameter.SetDist({ -0.1f, 0.01f, -0.005f, -0.001f, 0.0f });
    //camera_3d_to_2d.parameter.SetExtrinsic(
    //    { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
    //    { 0.0f, 0.0f, 0.0f }, true);   /* tvec (in world coordinate) */

    camera_3d_to_2d.parameter.SetExtrinsic(
        { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
        { 0.0f, 0.0f, 0.0f }, true);   /* tvec (in world coordinate) */

}

/*** Function ***/

static void CallbackMouseMain(int32_t event, int32_t x, int32_t y, int32_t flags, void* userdata)
{
    static constexpr float kIncAnglePerPx = 0.01f;
    static constexpr int32_t kInvalidValue = -99999;
    static cv::Point s_drag_previous_point = { kInvalidValue, kInvalidValue };
    if (event == cv::EVENT_LBUTTONUP) {
        s_drag_previous_point.x = kInvalidValue;
        s_drag_previous_point.y = kInvalidValue;
    } else if (event == cv::EVENT_LBUTTONDOWN) {
        s_drag_previous_point.x = x;
        s_drag_previous_point.y = y;
    } else {
        if (s_drag_previous_point.x != kInvalidValue) {
            camera_3d_to_2d.parameter.yaw() += kIncAnglePerPx * (x - s_drag_previous_point.x);
            camera_3d_to_2d.parameter.pitch() -= kIncAnglePerPx * (y - s_drag_previous_point.y);
            s_drag_previous_point.x = x;
            s_drag_previous_point.y = y;
        }
        camera_3d_to_2d.parameter.yaw() = (std::min)(Deg2Rad(90.0f), (std::max)(camera_3d_to_2d.parameter.yaw(), Deg2Rad(-90.0f)));
        camera_3d_to_2d.parameter.pitch() = (std::min)(Deg2Rad(90.0f), (std::max)(camera_3d_to_2d.parameter.pitch(), Deg2Rad(-90.0f)));

        /* todo: rotate oi camera coordinate */
    }
}


static void TreatKeyInputMain(int32_t key)
{
    static constexpr float kIncPosPerFrame = 20.f;
    key &= 0xFF;
    switch (key) {
    case 'w':
        camera_3d_to_2d.parameter.z() -= kIncPosPerFrame;
        break;
    case 'W':
        camera_3d_to_2d.parameter.z() -= kIncPosPerFrame * 3;
        break;
    case 's':
        camera_3d_to_2d.parameter.z() += kIncPosPerFrame;
        break;
    case 'S':
        camera_3d_to_2d.parameter.z() += kIncPosPerFrame * 3;
        break;
    case 'a':
        camera_3d_to_2d.parameter.x() += kIncPosPerFrame;
        break;
    case 'A':
        camera_3d_to_2d.parameter.x() += kIncPosPerFrame * 3;
        break;
    case 'd':
        camera_3d_to_2d.parameter.x() -= kIncPosPerFrame;
        break;
    case 'D':
        camera_3d_to_2d.parameter.x() -= kIncPosPerFrame * 3;
        break;
    case 'z':
        camera_3d_to_2d.parameter.y() += kIncPosPerFrame;
        break;
    case 'Z':
        camera_3d_to_2d.parameter.y() += kIncPosPerFrame * 3;
        break;
    case 'x':
        camera_3d_to_2d.parameter.y() -= kIncPosPerFrame;
        break;
    case 'X':
        camera_3d_to_2d.parameter.y() -= kIncPosPerFrame * 3;
        break;
    case 'q':
        camera_3d_to_2d.parameter.roll() += 0.1f;
        break;
    case 'e':
        camera_3d_to_2d.parameter.roll() -= 0.1f;
        break;
    }
}

static bool CheckIfPointInArea(const cv::Point& p, const cv::Size& r)
{
    if (p.x < 0 || p.y < 0 || p.x >= r.width || p.y >= r.height) return false;
    return true;
}


int main(int argc, char* argv[])
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
    for (frame_cnt = 0; cap.isOpened() || frame_cnt < 1 || true; frame_cnt++) { // todo
        /* Read image */
        cv::Mat image_input;
        if (cap.isOpened()) {
            cap.read(image_input);
        } else {
            image_input = cv::imread(input_name);
        }
        if (image_input.empty()) break;

        cv::resize(image_input, image_input, cv::Size(), 0.5, 0.5); // todo

        if (frame_cnt == 0) initialize_camera(image_input.cols, image_input.rows);

        /* Detect face */
        cv::Mat mat_depth;
        depth_engine.Process(image_input, mat_depth);

        /* Draw Result */
        cv::Mat image_depth;
        //cv::applyColorMap(mat_depth, image_depth, cv::COLORMAP_JET);
        image_depth = mat_depth;
        cv::resize(image_depth, image_depth, image_input.size());

        std::vector<cv::Point3f> object_point_list;
        for (int32_t y = 0; y < image_depth.rows; y+=1) {
            for (int32_t x = 0; x < image_depth.cols; x+=1) {
                cv::Point3f object_point;
                float Z = image_depth.at<uint8_t>(cv::Point(x, y));
                camera_2d_to_3d.ProjectImage2PosInCamera(cv::Point2f(x, y), Z, object_point);
                object_point_list.push_back(object_point);
                //printf("%d %d, %.3f, %.3f, %.3f\n", x, y, object_point.x, object_point.y, object_point.z);
            }
        }

        std::vector<cv::Point2f> image_point_list;
        //cv::projectPoints(object_point_list, camera_3d_to_2d.parameter.rvec, camera_3d_to_2d.parameter.tvec, camera_3d_to_2d.parameter.K, camera_3d_to_2d.parameter.dist_coeff, image_point_list);
        for (auto& p : object_point_list) {
            cv::Point2f image_point;
            camera_3d_to_2d.ProjectWorld2Image(p, image_point);
            image_point_list.push_back(image_point);
        }

        /* Draw the result */
        cv::Mat mat_output = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(0, 0, 0));
        for (int32_t i = 0; i < image_point_list.size(); i++) {
            if (CheckIfPointInArea(image_point_list[i], mat_output.size())) {
                cv::circle(mat_output, image_point_list[i], 2, image_input.at<cv::Vec3b>(i), -1);
            }
        }

        cv::imshow("Input", image_input);
        cv::imshow("Depth", image_depth);
        cv::imshow("Reconstruction", mat_output);
        
        
        int32_t key = cv::waitKey(1);
        if (key == 'q') break;
        TreatKeyInputMain(key);
        cv::setMouseCallback("Reconstruction", CallbackMouseMain);
    }

    depth_engine.Finalize();
    cv::waitKey(-1);

    return 0;
}
