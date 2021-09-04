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
static constexpr float kFovDeg = 130.0f;


/*** Global variable ***/
static CameraModel camera_real;
static CameraModel camera_top;


/*** Function ***/
void ResetCameraPose()
{
    camera_real.parameter.SetExtrinsic(
        { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
        { 0.0f, 1.0f, 0.0f });   /* tvec */
    camera_top.parameter.SetExtrinsic(
        { 90.0f, 0.0f, 0.0f },    /* rvec [deg] */
        {  0.0f, 8.0f, 7.0f });   /* tvec */  /* tvec is in camera coordinate, so Z is height because pitch = 90 */
}

void ResetCamera(int32_t width, int32_t height)
{
    camera_real.parameter.SetIntrinsic(width, height, CameraModel::FocalLength(width, kFovDeg));
    camera_top.parameter.SetIntrinsic(width, height, CameraModel::FocalLength(width, kFovDeg));
    ResetCameraPose();
}


static void loop_main(const cv::Mat& image_org)
{
    cvui::context(kWindowMain);

    /*** Generate mapping b/w object points (3D: world coordinate) and image points (real camera) */
    std::vector<cv::Point3f> object_point_list = {   /* Target area (possible road area) */
        { -1.0f, 0, 10.0f },
        {  1.0f, 0, 10.0f },
        { -1.0f, 0,  3.0f },
        {  1.0f, 0,  3.0f },
    };
    std::vector<cv::Point2f> image_point_real_list;
    cv::projectPoints(object_point_list, camera_real.parameter.rvec, camera_real.parameter.tvec, camera_real.parameter.K, camera_real.parameter.dist_coeff, image_point_real_list);

    /* Convert to image points (2D) using the top view camera (virtual camera) */
    std::vector<cv::Point2f> image_point_top_list;
    cv::projectPoints(object_point_list, camera_top.parameter.rvec, camera_top.parameter.tvec, camera_top.parameter.K, camera_top.parameter.dist_coeff, image_point_top_list);

    /* Perspective Transform */
    cv::Mat mat_transform = cv::getPerspectiveTransform(&image_point_real_list[0], &image_point_top_list[0]);
    cv::Mat mat_output = cv::Mat(image_org.size(), CV_8UC3, cv::Scalar(70, 70, 70));
    cv::warpPerspective(image_org, mat_output, mat_transform, mat_output.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

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
    cv::Mat mat = cv::Mat(800, 300, CV_8UC3, cv::Scalar(70, 70, 70));

    cvui::beginColumn(mat, 10, 10, -1, -1, 2);
    {
        if (cvui::button(120, 20, "Reset")) {
            ResetCameraPose();
        }

        cvui::text("Camera Parameter (Intrinsic)");
        MAKE_GUI_SETTING_FLOAT(camera_real.parameter.fx(), "Focal Length", 10.0f, "%.0Lf", 0.0f, 1000.0f);
        camera_real.parameter.fy() = camera_real.parameter.fx();
        camera_top.parameter.fx() = camera_real.parameter.fx();
        camera_top.parameter.fy() = camera_real.parameter.fy();

        cvui::text("Top Camera Parameter (Extrinsic)");
        float temp_deg = Rad2Deg(camera_top.parameter.pitch());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Pitch", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_top.parameter.pitch() = Deg2Rad(temp_deg);

        temp_deg = Rad2Deg(camera_top.parameter.yaw());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Yaw", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_top.parameter.yaw() = Deg2Rad(temp_deg);

        temp_deg = Rad2Deg(camera_top.parameter.roll());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Roll", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_top.parameter.roll() = Deg2Rad(temp_deg);

        cvui::text("Real Camera Parameter (Extrinsic)");
        MAKE_GUI_SETTING_FLOAT(camera_real.parameter.y(), "Height", 1.0f, "%.0Lf", 0.0f, 5.0f);

        temp_deg = Rad2Deg(camera_real.parameter.pitch());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Pitch", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_real.parameter.pitch() = Deg2Rad(temp_deg);

        temp_deg = Rad2Deg(camera_real.parameter.yaw());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Yaw", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_real.parameter.yaw() = Deg2Rad(temp_deg);

        temp_deg = Rad2Deg(camera_real.parameter.roll());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Roll", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_real.parameter.roll() = Deg2Rad(temp_deg);

    }
    cvui::endColumn();

    cvui::imshow(kWindowParam, mat);
}

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
            camera_top.parameter.yaw() += kIncAnglePerPx * (x - s_drag_previous_point.x);
            camera_top.parameter.pitch() -= kIncAnglePerPx * (y - s_drag_previous_point.y);
            s_drag_previous_point.x = x;
            s_drag_previous_point.y = y;
        }
        camera_top.parameter.yaw() = (std::min)(Deg2Rad(90.0f), (std::max)(camera_top.parameter.yaw(), Deg2Rad(-90.0f)));
        camera_top.parameter.pitch() = (std::min)(Deg2Rad(90.0f), (std::max)(camera_top.parameter.pitch(), Deg2Rad(-90.0f)));
    }
}


static void TreatKeyInputMain(int32_t key)
{
    static constexpr float kIncPosPerFrame = 0.8f;
    key &= 0xFF;
    switch (key) {
    case 'w':
        camera_top.parameter.z() -= kIncPosPerFrame;
        break;
    case 'W':
        camera_top.parameter.z() -= kIncPosPerFrame * 3;
        break;
    case 's':
        camera_top.parameter.z() += kIncPosPerFrame;
        break;
    case 'S':
        camera_top.parameter.z() += kIncPosPerFrame * 3;
        break;
    case 'a':
        camera_top.parameter.x() += kIncPosPerFrame;
        break;
    case 'A':
        camera_top.parameter.x() += kIncPosPerFrame * 3;
        break;
    case 'd':
        camera_top.parameter.x() -= kIncPosPerFrame;
        break;
    case 'D':
        camera_top.parameter.x() -= kIncPosPerFrame * 3;
        break;
    case 'z':
        camera_top.parameter.y() += kIncPosPerFrame;
        break;
    case 'Z':
        camera_top.parameter.y() += kIncPosPerFrame * 3;
        break;
    case 'x':
        camera_top.parameter.y() -= kIncPosPerFrame;
        break;
    case 'X':
        camera_top.parameter.y() -= kIncPosPerFrame * 3;
        break;
    case 'q':
        camera_top.parameter.roll() += 0.1f;
        break;
    case 'e':
        camera_top.parameter.roll() -= 0.1f;
        break;
    }
}

int main(int argc, char* argv[])
{
    cvui::init(kWindowMain);
    cvui::init(kWindowParam);

    cv::setMouseCallback(kWindowMain, CallbackMouseMain);

    static const std::string image_path = RESOURCE_DIR"/dashcam_00.jpg";
    cv::Mat image_org = cv::imread(image_path);

    ResetCamera(image_org.cols, image_org.rows);

    while (true) {
        loop_main(image_org);
        loop_param();
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
        TreatKeyInputMain(key);
    }

    return 0;
}
