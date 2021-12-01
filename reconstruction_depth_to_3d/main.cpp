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
#include <chrono>

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

/*** Function ***/
void initialize_camera(int32_t width, int32_t height)
{
    camera_2d_to_3d.parameter.SetIntrinsic(width, height, CameraModel::FocalLength(width, kFovDeg));
    camera_2d_to_3d.parameter.SetDist({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
    camera_2d_to_3d.parameter.SetExtrinsic(
        { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
        { 0.0f, 0.0f, 0.0f }, true);   /* tvec (Oc - Ow in world coordinate. X+= Right, Y+ = down, Z+ = far) */
                                       /* tvec must be zero, so that calculated Mc(Location in camera cooridinate) becomes the same as Mw(location in world coordinate), and can be used to convert from 3D to 2D later */

    camera_3d_to_2d.parameter.SetIntrinsic(kWidth, kHeight, CameraModel::FocalLength(kWidth, kFovDeg));
    camera_3d_to_2d.parameter.SetDist({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
    camera_3d_to_2d.parameter.SetExtrinsic(
        { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
        { 0.0f, 0.0f, 0.0f }, true);   /* tvec (Oc - Ow in world coordinate. X+= Right, Y+ = down, Z+ = far) */

}

static void CallbackMouseMain(int32_t event, int32_t x, int32_t y, int32_t flags, void* userdata)
{
    static constexpr float kIncAnglePerPx = 0.1f;
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
            float delta_yaw = kIncAnglePerPx * (x - s_drag_previous_point.x);
            float pitch_delta = -kIncAnglePerPx * (y - s_drag_previous_point.y);
            camera_3d_to_2d.parameter.RotateCameraAngle(pitch_delta, delta_yaw, 0);
            s_drag_previous_point.x = x;
            s_drag_previous_point.y = y;
        }
    }
}

static void TreatKeyInputMain(int32_t key)
{
    static constexpr float kIncPosPerFrame = 10.0f;
    key &= 0xFF;
    switch (key) {
    case 'w':
        camera_3d_to_2d.parameter.MoveCameraPos(0, 0, kIncPosPerFrame, false);
        break;
    case 'W':
        camera_3d_to_2d.parameter.MoveCameraPos(0, 0, kIncPosPerFrame, true);
        break;
    case 's':
        camera_3d_to_2d.parameter.MoveCameraPos(0, 0, -kIncPosPerFrame, false);
        break;
    case 'S':
        camera_3d_to_2d.parameter.MoveCameraPos(0, 0, -kIncPosPerFrame, true);
        break;
    case 'a':
        camera_3d_to_2d.parameter.MoveCameraPos(-kIncPosPerFrame, 0, 0, false);
        break;
    case 'A':
        camera_3d_to_2d.parameter.MoveCameraPos(-kIncPosPerFrame, 0, 0, true);
        break;
    case 'd':
        camera_3d_to_2d.parameter.MoveCameraPos(kIncPosPerFrame, 0, 0, false);
        break;
    case 'D':
        camera_3d_to_2d.parameter.MoveCameraPos(kIncPosPerFrame, 0, 0, true);
        break;
    case 'z':
        camera_3d_to_2d.parameter.MoveCameraPos(0, -kIncPosPerFrame, 0, false);
        break;
    case 'Z':
        camera_3d_to_2d.parameter.MoveCameraPos(0, -kIncPosPerFrame, 0, true);
        break;
    case 'x':
        camera_3d_to_2d.parameter.MoveCameraPos(0, kIncPosPerFrame, 0, false);
        break;
    case 'X':
        camera_3d_to_2d.parameter.MoveCameraPos(0, kIncPosPerFrame, 0, true);
        break;
    case 'q':
        camera_3d_to_2d.parameter.RotateCameraAngle(0, 0, 2.0f);
        break;
    case 'e':
        camera_3d_to_2d.parameter.RotateCameraAngle(0, 0, -2.0f);
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

    /* Read image */
    cv::Mat image_input;
    if (cap.isOpened()) {
        cap.read(image_input);
    } else {
        image_input = cv::imread(input_name);
    }
    if (image_input.empty()) return -1;

    cv::resize(image_input, image_input, cv::Size(), 0.5, 0.5);

    initialize_camera(image_input.cols, image_input.rows);

    /* Estimate depth */
    cv::Mat mat_depth;
    depth_engine.Process(image_input, mat_depth);

    while(true) {
        /* Draw depth */
        cv::Mat image_depth;
        //cv::applyColorMap(mat_depth, image_depth, cv::COLORMAP_JET);
        image_depth = mat_depth;
        cv::resize(image_depth, image_depth, image_input.size());

        /* Convert px,py,depth(Zc) -> Xc,Yc,Zc(in camera_2d_to_3d)(=Xw,Yw,Zw) */
        std::vector<float> depth_list;
        for (int32_t y = 0; y < image_depth.rows; y += 1) {
            for (int32_t x = 0; x < image_depth.cols; x += 1) {
                float Z = image_depth.at<uint8_t>(cv::Point(x, y));
                depth_list.push_back(Z);
            }
        }
        std::vector<cv::Point3f> object_point_list;
        camera_2d_to_3d.ProjectImage2PosInCamera(depth_list, object_point_list);

        /* Project 3D to 2D(new image) */
        std::vector<cv::Point2f> image_point_list;
        camera_3d_to_2d.ProjectWorld2Image(object_point_list, image_point_list);

        /* Draw the result */
        cv::Mat mat_output = cv::Mat(camera_3d_to_2d.parameter.height, camera_3d_to_2d.parameter.width, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat mat_z_buffer = cv::Mat(mat_output.size(), CV_32FC1, 999999);
        for (int32_t i = 0; i < image_point_list.size(); i++) {
            if (CheckIfPointInArea(image_point_list[i], mat_output.size())) {
                float& z_old = mat_z_buffer.at<float>(image_point_list[i]);
                if (depth_list[i] < z_old) {  /* z_old is far */
                    z_old = depth_list[i];
                    cv::circle(mat_output, image_point_list[i], 4, image_input.at<cv::Vec3b>(i), -1);
                }
            }
        }

        cv::imshow("Input", image_input);
        cv::imshow("Depth", image_depth);
        cv::imshow("Reconstruction", mat_output);
        
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
        TreatKeyInputMain(key);
        cv::setMouseCallback("Reconstruction", CallbackMouseMain);
    }

    depth_engine.Finalize();
    cv::waitKey(-1);

    return 0;
}
