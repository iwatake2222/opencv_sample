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
static constexpr char kInputImageFilename[] = RESOURCE_DIR"/room_02.jpg";
static constexpr float   kCamera2d3dFovDeg = 80.0f;
static constexpr int32_t kCamera3d2dWidth = 640;
static constexpr int32_t kCamera3d2dHeight = 480;
static constexpr float   kCamera3d2dFovDeg = 80.0f;
#define NORMALIZE_BY_255

/*** Global variable ***/
static CameraModel camera_2d_to_3d;
static CameraModel camera_3d_to_2d;

/*** Function ***/
void InitializeCamera(int32_t width, int32_t height)
{
    camera_2d_to_3d.SetIntrinsic(width, height, FocalLength(width, kCamera2d3dFovDeg));
    camera_2d_to_3d.SetDist({ -0.1f, 0.01f, -0.005f, -0.001f, 0.0f });
    //camera_2d_to_3d.SetDist({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
    camera_2d_to_3d.SetExtrinsic(
        { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
        { 0.0f, 0.0f, 0.0f }, true);   /* tvec (Oc - Ow in world coordinate. X+= Right, Y+ = down, Z+ = far) */

    camera_3d_to_2d.SetIntrinsic(kCamera3d2dWidth, kCamera3d2dHeight, FocalLength(kCamera3d2dWidth, kCamera3d2dFovDeg));
    camera_3d_to_2d.SetDist({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
    camera_3d_to_2d.SetExtrinsic(
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
            camera_3d_to_2d.RotateCameraAngle(pitch_delta, delta_yaw, 0);
            s_drag_previous_point.x = x;
            s_drag_previous_point.y = y;
        }
    }
}

static void TreatKeyInputMain(int32_t key)
{
#ifdef NORMALIZE_BY_255
    static constexpr float kIncPosPerFrame = 10.0f;
#else
    static constexpr float kIncPosPerFrame = 0.0005f;
#endif
    key &= 0xFF;
    switch (key) {
    case 'w':
        camera_3d_to_2d.MoveCameraPos(0, 0, kIncPosPerFrame, false);
        break;
    case 'W':
        camera_3d_to_2d.MoveCameraPos(0, 0, kIncPosPerFrame, true);
        break;
    case 's':
        camera_3d_to_2d.MoveCameraPos(0, 0, -kIncPosPerFrame, false);
        break;
    case 'S':
        camera_3d_to_2d.MoveCameraPos(0, 0, -kIncPosPerFrame, true);
        break;
    case 'a':
        camera_3d_to_2d.MoveCameraPos(-kIncPosPerFrame, 0, 0, false);
        break;
    case 'A':
        camera_3d_to_2d.MoveCameraPos(-kIncPosPerFrame, 0, 0, true);
        break;
    case 'd':
        camera_3d_to_2d.MoveCameraPos(kIncPosPerFrame, 0, 0, false);
        break;
    case 'D':
        camera_3d_to_2d.MoveCameraPos(kIncPosPerFrame, 0, 0, true);
        break;
    case 'z':
        camera_3d_to_2d.MoveCameraPos(0, -kIncPosPerFrame, 0, false);
        break;
    case 'Z':
        camera_3d_to_2d.MoveCameraPos(0, -kIncPosPerFrame, 0, true);
        break;
    case 'x':
        camera_3d_to_2d.MoveCameraPos(0, kIncPosPerFrame, 0, false);
        break;
    case 'X':
        camera_3d_to_2d.MoveCameraPos(0, kIncPosPerFrame, 0, true);
        break;
    case 'q':
        camera_3d_to_2d.RotateCameraAngle(0, 0, 2.0f);
        break;
    case 'e':
        camera_3d_to_2d.RotateCameraAngle(0, 0, -2.0f);
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

    InitializeCamera(image_input.cols, image_input.rows);

    /* Estimate depth */
    cv::Mat mat_depth;
    depth_engine.Process(image_input, mat_depth);

    /* Draw depth */
    cv::Mat mat_depth_normlized255;
    depth_engine.NormalizeMinMax(mat_depth, mat_depth_normlized255);
    cv::Mat image_depth;
    cv::applyColorMap(mat_depth_normlized255, image_depth, cv::COLORMAP_JET);
    cv::resize(image_depth, image_depth, image_input.size());

    /* Normalize depth for 3D reconstruction */
    cv::Mat mat_depth_normlized;
#ifdef NORMALIZE_BY_255
    mat_depth_normlized255.convertTo(mat_depth_normlized, CV_32FC1);
#else
    depth_engine.NormalizeScaleShift(mat_depth, mat_depth_normlized, 1.0f, 0.0f);
#endif
    cv::resize(mat_depth_normlized, mat_depth_normlized, image_input.size());

    /* Generate depth list */
    std::vector<float> depth_list;
    for (int32_t y = 0; y < mat_depth_normlized.rows; y ++) {
        for (int32_t x = 0; x < mat_depth_normlized.cols; x ++) {
            float Z = mat_depth_normlized.at<float>(cv::Point(x, y));
            depth_list.push_back(Z);
        }
    }

    /* Convert px,py,depth(Zc) -> Xc,Yc,Zc(in camera_2d_to_3d)(=Xw,Yw,Zw) */
    std::vector<cv::Point3f> object_point_list;
    camera_2d_to_3d.ConvertImage2World(depth_list, object_point_list);

    while(true) {
        /* Project 3D to 2D(new image) */
        std::vector<cv::Point2f> image_point_list;
        camera_3d_to_2d.ConvertWorld2Image(object_point_list, image_point_list);

        /* Generate object points in camera coordinate to draw the object in Zc order, from far to near (instead of using Z buffer) */
        std::vector<cv::Point3f> object_point_in_camera_list;
        camera_3d_to_2d.ConvertWorld2Camera(object_point_list, object_point_in_camera_list);

        /* Argsort by depth (index_0 = Far, index_len-1 = Near)*/
        std::vector<int32_t> indices_depth(object_point_in_camera_list.size());
        std::iota(indices_depth.begin(), indices_depth.end(), 0);
        std::sort(indices_depth.begin(), indices_depth.end(), [&object_point_in_camera_list](int32_t i1, int32_t i2) {
            return object_point_in_camera_list[i1].z > object_point_in_camera_list[i2].z;
        });

        /* Draw the result */
        cv::Mat mat_output = cv::Mat(camera_3d_to_2d.height, camera_3d_to_2d.width, CV_8UC3, cv::Scalar(0, 0, 0));
        for (int32_t i : indices_depth) {
            if (CheckIfPointInArea(image_point_list[i], mat_output.size())) {
                cv::circle(mat_output, image_point_list[i], 4, image_input.at<cv::Vec3b>(i), -1);
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
