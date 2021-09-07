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

static constexpr int32_t kWidth = 1280;
static constexpr int32_t kHeight = 720;
static constexpr float   kFovDeg = 80.0f;


/*** Global variable ***/
static CameraModel camera;


/*** Function ***/
void ResetCameraPose()
{
    camera.parameter.SetExtrinsic(
        { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
        { 0.0f, 0.0f, 7.0f });   /* tvec */
}

void ResetCamera(int32_t width, int32_t height)
{
    camera.parameter.SetIntrinsic(width, height, CameraModel::FocalLength(width, kFovDeg));
    camera.parameter.SetDist({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
    ResetCameraPose();
}

static void loop_main(const cv::Mat& image_org)
{
    cvui::context(kWindowMain);

    /* Generate object points (3D: world coordinate) */
    std::vector<cv::Point3f> object_point_list;
    float aspect = static_cast<float>(image_org.cols) / image_org.rows;
    object_point_list.push_back(cv::Point3f(-1 * aspect, -1, 0));
    object_point_list.push_back(cv::Point3f(1 * aspect, -1, 0));
    object_point_list.push_back(cv::Point3f(1 * aspect, 1, 0));
    object_point_list.push_back(cv::Point3f(-1 * aspect, 1, 0));

    /* Rotate object */
    static float x_deg, y_deg;
    CameraModel::RotateObject(x_deg, y_deg, 0, object_point_list);
    x_deg += 4.9f;  /* don't use a good cut-off number to avoid gimbal lock */
    y_deg += 5.1f;

    /* Convert to image points (2D) */
    std::vector<cv::Point2f> image_point_list;
    cv::projectPoints(object_point_list, camera.parameter.rvec, camera.parameter.tvec, camera.parameter.K, camera.parameter.dist_coeff, image_point_list);

    /* Affine transform */
    cv::Point2f pts1[] = { cv::Point2f(0, 0), cv::Point2f(image_org.cols - 1.0f, 0) , cv::Point2f(image_org.cols - 1.0f, image_org.rows - 1.0f) , cv::Point2f(0, image_org.rows - 1.0f) };
    cv::Mat mat_affine = cv::getPerspectiveTransform(pts1, &image_point_list[0]);
    cv::Mat mat_output = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(70, 70, 70));
    cv::warpPerspective(image_org, mat_output, mat_affine, mat_output.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);

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
        if (cvui::button(120, 20, "Reset")) {
            ResetCameraPose();
        }

        cvui::text("Camera Parameter (internal)");
        MAKE_GUI_SETTING_FLOAT(camera.parameter.fx(), "Focal Length", 10.0f, "%.0Lf", 0.0f, 1000.0f);
        camera.parameter.fy() = camera.parameter.fx();

        MAKE_GUI_SETTING_FLOAT(camera.parameter.dist_coeff.at<float>(0), "dist: k1", 0.00001f, "%.05Lf", -0.4f, 0.4f);
        MAKE_GUI_SETTING_FLOAT(camera.parameter.dist_coeff.at<float>(1), "dist: k2", 0.00001f, "%.05Lf", -0.1f, 0.1f);
        MAKE_GUI_SETTING_FLOAT(camera.parameter.dist_coeff.at<float>(2), "dist: p1", 0.00001f, "%.05Lf", -0.1f, 0.1f);
        MAKE_GUI_SETTING_FLOAT(camera.parameter.dist_coeff.at<float>(3), "dist: p2", 0.00001f, "%.05Lf", -0.1f, 0.1f);
        MAKE_GUI_SETTING_FLOAT(camera.parameter.dist_coeff.at<float>(4), "dist: k3", 0.00001f, "%.05Lf", -0.1f, 0.1f);
        camera.parameter.UpdateNewCameraMatrix();

        cvui::text("Camera Parameter (external)");
        float temp_deg = Rad2Deg(camera.parameter.pitch());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Pitch", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera.parameter.pitch() = Deg2Rad(temp_deg);

        temp_deg = Rad2Deg(camera.parameter.yaw());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Yaw", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera.parameter.yaw() = Deg2Rad(temp_deg);

        temp_deg = Rad2Deg(camera.parameter.roll());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Roll", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera.parameter.roll() = Deg2Rad(temp_deg);

        MAKE_GUI_SETTING_FLOAT(camera.parameter.x(), "X", 1.0f, "%.0Lf", -20.0f, 20.0f);
        MAKE_GUI_SETTING_FLOAT(camera.parameter.y(), "Y", 1.0f, "%.0Lf", -20.0f, 20.0f);
        MAKE_GUI_SETTING_FLOAT(camera.parameter.z(), "Z", 1.0f, "%.0Lf", -20.0f, 20.0f);
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
            camera.parameter.yaw() += kIncAnglePerPx * (x - s_drag_previous_point.x);
            camera.parameter.pitch() -= kIncAnglePerPx * (y - s_drag_previous_point.y);
            s_drag_previous_point.x = x;
            s_drag_previous_point.y = y;
        }
        camera.parameter.yaw() = (std::min)(Deg2Rad(90.0f), (std::max)(camera.parameter.yaw(), Deg2Rad(-90.0f)));
        camera.parameter.pitch() = (std::min)(Deg2Rad(90.0f), (std::max)(camera.parameter.pitch(), Deg2Rad(-90.0f)));
    }
}

static void TreatKeyInputMain(int32_t key)
{
    static constexpr float kIncPosPerFrame = 0.8f;
    key &= 0xFF;
    switch (key) {
    case 'w':
        camera.parameter.z() -= kIncPosPerFrame;
        break;
    case 'W':
        camera.parameter.z() -= kIncPosPerFrame * 3;
        break;
    case 's':
        camera.parameter.z() += kIncPosPerFrame;
        break;
    case 'S':
        camera.parameter.z() += kIncPosPerFrame * 3;
        break;
    case 'a':
        camera.parameter.x() += kIncPosPerFrame;
        break;
    case 'A':
        camera.parameter.x() += kIncPosPerFrame * 3;
        break;
    case 'd':
        camera.parameter.x() -= kIncPosPerFrame;
        break;
    case 'D':
        camera.parameter.x() -= kIncPosPerFrame * 3;
        break;
    case 'z':
        camera.parameter.y() += kIncPosPerFrame;
        break;
    case 'Z':
        camera.parameter.y() += kIncPosPerFrame * 3;
        break;
    case 'x':
        camera.parameter.y() -= kIncPosPerFrame;
        break;
    case 'X':
        camera.parameter.y() -= kIncPosPerFrame * 3;
        break;
    case 'q':
        camera.parameter.roll() += 0.1f;
        break;
    case 'e':
        camera.parameter.roll() -= 0.1f;
        break;
    }
}

int main(int argc, char* argv[])
{
    cvui::init(kWindowMain);
    cvui::init(kWindowParam);

    cv::setMouseCallback(kWindowMain, CallbackMouseMain);

    static const std::string image_path = RESOURCE_DIR"/baboon.jpg";
    cv::Mat image_org = cv::imread(image_path);

    ResetCamera(kWidth, kHeight);

    while (true) {
        loop_main(image_org);
        loop_param();
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
        TreatKeyInputMain(key);
    }

    return 0;
}
