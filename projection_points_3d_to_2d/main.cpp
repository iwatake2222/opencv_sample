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

#include "camera_model.h"


/*** Macro ***/
static constexpr char kWindowMain[] = "WindowMain";
static constexpr char kWindowParam[] = "WindowParam";

static constexpr int32_t kWidth  = 1280;
static constexpr int32_t kHeight = 720;
static constexpr float   kFovDeg = 80.0f;

static constexpr int32_t kPointRange  = 100;
static constexpr float kPointInterval = 5.0f;
static constexpr int32_t kPointNum    = static_cast<int32_t>(kPointRange / kPointInterval) + 1;


/*** Global variable ***/
static bool is_floor_mode = true;
static CameraModel camera;

/*** Function ***/
void ResetCameraPoseFloor()
{
    camera.SetExtrinsic(
        { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
        //{ 0.0f, 10.0f, 0.0f }, false);   /* tvec (Oc - Ow in world coordinate. X+= Right, Y+ = down, Z+ = far) */
        { 0.0f, -10.0f, 0.0f }, true);   /* tvec (Oc - Ow in world coordinate. X+= Right, Y+ = down, Z+ = far) */
}

void ResetCameraPoseWall()
{
    camera.SetExtrinsic(
        { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
        //{ 0.0f, 0.0f, 100.0f }, false);   /* tvec (Oc - Ow in world coordinate. X+= Right, Y+ = down, Z+ = far) */
        { 0.0f, 0.0f, -100.0f }, true);   /* tvec (Oc - Ow in world coordinate. X+= Right, Y+ = down, Z+ = far) */
}

void ResetCamera(int32_t width, int32_t height)
{
    camera.SetIntrinsic(width, height, FocalLength(width, kFovDeg));
    camera.SetDist({ -0.1f, 0.01f, -0.005f, -0.001f, 0.0f });
    ResetCameraPoseFloor();
}


static bool CheckIfPointInArea(const cv::Point& p, const cv::Size& r)
{
    if (p.x < 0 || p.y < 0 || p.x >= r.width || p.y >= r.height) return false;
    return true;
}

static void loop_main()
{
    cvui::context(kWindowMain);

    /* Generate object points (3D: world coordinate) */
    std::vector<cv::Point3f> object_point_list;
    if (is_floor_mode) {
        for (float x = -kPointRange; x <= kPointRange; x += kPointInterval) {
            for (float z = 0; z <= kPointRange; z += kPointInterval) {
                object_point_list.push_back(cv::Point3f(x, 0, z));
            }
        }
    } else {
        for (float x = -kPointRange; x <= kPointRange; x += kPointInterval) {
            for (float y = -kPointRange; y <= kPointRange; y += kPointInterval) {
                object_point_list.push_back(cv::Point3f(x, y, 0));
            }
        }
    }

    /* Convert to image points (2D) */
    std::vector<cv::Point2f> image_point_list;
    camera.ConvertWorld2Image(object_point_list, image_point_list);

    /* Draw the result */
    cv::Mat mat_output = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(70, 70, 70));
    for (int32_t i = 0; i < image_point_list.size(); i++) {
        if (CheckIfPointInArea(image_point_list[i], mat_output.size())) {
            if (i % kPointNum != 0) {
                cv::line(mat_output, image_point_list[i - 1], image_point_list[i], cv::Scalar(220, 0, 0));
            }
            cv::circle(mat_output, image_point_list[i], 2, cv::Scalar(220, 0, 0));
            cv::putText(mat_output, std::to_string(i), image_point_list[i], 0, 0.4, cv::Scalar(0, 255, 0));
        }
    }

    cvui::imshow(kWindowMain, mat_output);
}



#define MAKE_GUI_SETTING_FLOAT(VAL, LABEL, STEP, FORMAT, RANGE0, RANGE1) {\
cvui::beginColumn(-1, -1, 2);\
double temp_double_current = static_cast<double>(VAL);\
double temp_double_new = temp_double_current;\
float temp_float_current = VAL;\
float temp_float_new = temp_float_current;\
cvui::text(LABEL);\
cvui::counter(&temp_double_new, STEP, FORMAT);\
cvui::trackbar<float>(200, &temp_float_new, RANGE0, RANGE1);\
if (temp_double_new != temp_double_current) VAL = static_cast<float>(temp_double_new);\
if (temp_float_new != temp_float_current) VAL = temp_float_new;\
cvui::endColumn();\
}

static void loop_param()
{
    cvui::context(kWindowParam);
    cv::Mat mat = cv::Mat(1000, 300, CV_8UC3, cv::Scalar(70, 70, 70));

    cvui::beginColumn(mat, 10, 10, -1, -1, 2);
    {
        cvui::text("Reset Camera Parameter");
        cvui::beginRow(-1, -1, 10);
        if (cvui::button(120, 20, "Floor Pattern")) {
            is_floor_mode = true;
            ResetCameraPoseFloor();
        }
        if (cvui::button(120, 20, "Wall Pattern")) {
            is_floor_mode = false;
            ResetCameraPoseWall();
        }
        cvui::endRow();

        cvui::text("Camera Parameter (Intrinsic)");
        MAKE_GUI_SETTING_FLOAT(camera.fx(), "Focal Length", 10.0f, "%.0Lf", 0.0f, 1000.0f);
        camera.fy() = camera.fx();

        MAKE_GUI_SETTING_FLOAT(camera.dist_coeff.at<float>(0), "dist: k1", 0.00001f, "%.05Lf", -0.4f, 0.4f);
        MAKE_GUI_SETTING_FLOAT(camera.dist_coeff.at<float>(1), "dist: k2", 0.00001f, "%.05Lf", -0.1f, 0.1f);
        MAKE_GUI_SETTING_FLOAT(camera.dist_coeff.at<float>(2), "dist: p1", 0.00001f, "%.05Lf", -0.1f, 0.1f);
        MAKE_GUI_SETTING_FLOAT(camera.dist_coeff.at<float>(3), "dist: p2", 0.00001f, "%.05Lf", -0.1f, 0.1f);
        MAKE_GUI_SETTING_FLOAT(camera.dist_coeff.at<float>(4), "dist: k3", 0.00001f, "%.05Lf", -0.1f, 0.1f);

        camera.UpdateNewCameraMatrix();

        cvui::text("Camera Parameter (Extrinsic)");
        float pitch_deg = Rad2Deg(camera.rx());
        MAKE_GUI_SETTING_FLOAT(pitch_deg, "Pitch", 1.0f, "%.0Lf", -90.0f, 90.0f);
        float yaw_deg = Rad2Deg(camera.ry());
        MAKE_GUI_SETTING_FLOAT(yaw_deg, "Yaw", 1.0f, "%.0Lf", -90.0f, 90.0f);
        float roll_deg = Rad2Deg(camera.rz());
        MAKE_GUI_SETTING_FLOAT(roll_deg, "Roll", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera.SetCameraAngle(pitch_deg, yaw_deg, roll_deg);

        float x = -camera.tx();
        float y = -camera.ty();
        float z = -camera.tz();
        MAKE_GUI_SETTING_FLOAT(x, "X", 1.0f, "%.0Lf", -20.0f, 20.0f);
        MAKE_GUI_SETTING_FLOAT(y, "Y", 1.0f, "%.0Lf", -20.0f, 20.0f);
        MAKE_GUI_SETTING_FLOAT(z, "Z", 1.0f, "%.0Lf", -20.0f, 20.0f);
        camera.SetCameraPos(x, y, z, false);
    }
    cvui::endColumn();

    cvui::imshow(kWindowParam, mat);
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
            camera.RotateCameraAngle(pitch_delta, delta_yaw, 0);
            s_drag_previous_point.x = x;
            s_drag_previous_point.y = y;
        }
    }
}


static void TreatKeyInputMain(int32_t key)
{
    static constexpr float kIncPosPerFrame = 0.8f;
    key &= 0xFF;
    switch (key) {
    case 'w':
        camera.MoveCameraPos(0, 0, kIncPosPerFrame, false);
        break;
    case 'W':
        camera.MoveCameraPos(0, 0, kIncPosPerFrame, true);
        break;
    case 's':
        camera.MoveCameraPos(0, 0, -kIncPosPerFrame, false);
        break;
    case 'S':
        camera.MoveCameraPos(0, 0, -kIncPosPerFrame, true);
        break;
    case 'a':
        camera.MoveCameraPos(-kIncPosPerFrame, 0, 0, false);
        break;
    case 'A':
        camera.MoveCameraPos(-kIncPosPerFrame, 0, 0, true);
        break;
    case 'd':
        camera.MoveCameraPos(kIncPosPerFrame, 0, 0, false);
        break;
    case 'D':
        camera.MoveCameraPos(kIncPosPerFrame, 0, 0, true);
        break;
    case 'z':
        camera.MoveCameraPos(0, -kIncPosPerFrame, 0, false);
        break;
    case 'Z':
        camera.MoveCameraPos(0, -kIncPosPerFrame, 0, true);
        break;
    case 'x':
        camera.MoveCameraPos(0, kIncPosPerFrame, 0, false);
        break;
    case 'X':
        camera.MoveCameraPos(0, kIncPosPerFrame, 0, true);
        break;
    case 'q':
        camera.RotateCameraAngle(0, 0, 2.0f);
        break;
    case 'e':
        camera.RotateCameraAngle(0, 0, -2.0f);
        break;
    }
}

int main(int argc, char* argv[])
{
    cvui::init(kWindowMain);
    cvui::init(kWindowParam);

    cv::setMouseCallback(kWindowMain, CallbackMouseMain);

    ResetCamera(kWidth, kHeight);

    while (true) {
        loop_main();
        loop_param();
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
        TreatKeyInputMain(key);
    }

    return 0;
}
