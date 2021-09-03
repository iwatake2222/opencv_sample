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
static constexpr char kWindowParam[] = "WindowParam";

static constexpr int32_t kWidth  = 1280;
static constexpr int32_t kHeight = 720;
static constexpr float   kFovDeg = 80.0f;

static constexpr int32_t kPointRange  = 100;
static constexpr float kPointInterval = 5.0f;
static constexpr int32_t kPointNum    = static_cast<int32_t>(kPointRange / kPointInterval) + 1;


/*** Global variable ***/
static bool is_floor_mode = true;


/*** Function ***/
static inline float Deg2Rad(float deg) { return static_cast<float>(deg * M_PI / 180.0); }
static inline float Rad2Deg(float rad) { return static_cast<float>(rad * 180.0 / M_PI); }

static float FocalLength()
{
    /* (w/2) / f = tan(fov/2) */
    return (kWidth / 2) / std::tanf(Deg2Rad(kFovDeg / 2));
}

typedef struct CameraParameter_ {
    float& pitch() { return rvec.at<float>(0); }
    float& yaw() { return rvec.at<float>(1); }
    float& roll() { return rvec.at<float>(2); }
    float& x() { return tvec.at<float>(0); }
    float& y() { return tvec.at<float>(1); }
    float& z() { return tvec.at<float>(2); }
    float& fx() { return camera_matrix.at<float>(0); }
    float& cx() { return camera_matrix.at<float>(2); }
    float& fy() { return camera_matrix.at<float>(4); }
    float& cy() { return camera_matrix.at<float>(5); }

    /* float, 3 x 1, pitch,  yaw, roll */
    cv::Mat rvec;

    /* float, 3 x 1, (X, Y, Z): horizontal, vertical, depth */
    cv::Mat tvec;

    /* float, 3 x 3 */
    cv::Mat camera_matrix;

    /* float, 5 x 1 */
    cv::Mat dist_coeff;

    /* Default Parameters */
    void ResetFloor()
    {
        rvec = (cv::Mat_<float>(3, 1) << Deg2Rad(0.0f), Deg2Rad(0.0f), Deg2Rad(0.0f));
        tvec = (cv::Mat_<float>(3, 1) << 0.0f, 10.0f, 0.0f);
        camera_matrix = (cv::Mat_<float>(3, 3) <<
            FocalLength(),             0,  kWidth / 2.f,
                        0, FocalLength(), kHeight / 2.f,
                        0,             0,             1);
        dist_coeff = (cv::Mat_<float>(5, 1) << -0.1f, 0.01f, -0.005f, -0.001f, 0.0f);
    }

    void ResetWall()
    {
        rvec = (cv::Mat_<float>(3, 1) << Deg2Rad(0.0f), Deg2Rad(0.0f), Deg2Rad(0.0f));
        tvec = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 100.0f);
        camera_matrix = (cv::Mat_<float>(3, 3) <<
            FocalLength(),             0,  kWidth / 2.f,
                        0, FocalLength(), kHeight / 2.f,
                        0,             0,             1);
        dist_coeff = (cv::Mat_<float>(5, 1) << -0.1f, 0.01f, -0.005f, -0.001f, 0.0f);
    }

    CameraParameter_() { ResetFloor(); }
} CameraParameter;

static CameraParameter camera_parameter;

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
    cv::projectPoints(object_point_list, camera_parameter.rvec, camera_parameter.tvec, camera_parameter.camera_matrix, camera_parameter.dist_coeff, image_point_list);

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
            camera_parameter.ResetFloor();
        }
        if (cvui::button(120, 20, "Wall Pattern")) {
            is_floor_mode = false;
            camera_parameter.ResetWall();
        }
        cvui::endRow();

        cvui::text("Camera Parameter (internal)");
        MAKE_GUI_SETTING_FLOAT(camera_parameter.fx(), "Focal Length", 10.0f, "%.0Lf", 0.0f, 1000.0f);
        camera_parameter.fy() = camera_parameter.fx();

        MAKE_GUI_SETTING_FLOAT(camera_parameter.dist_coeff.at<float>(0), "dist: k1", 0.00001f, "%.05Lf", -0.4f, 0.4f);
        MAKE_GUI_SETTING_FLOAT(camera_parameter.dist_coeff.at<float>(1), "dist: k2", 0.00001f, "%.05Lf", -0.1f, 0.1f);
        MAKE_GUI_SETTING_FLOAT(camera_parameter.dist_coeff.at<float>(2), "dist: p1", 0.00001f, "%.05Lf", -0.1f, 0.1f);
        MAKE_GUI_SETTING_FLOAT(camera_parameter.dist_coeff.at<float>(3), "dist: p2", 0.00001f, "%.05Lf", -0.1f, 0.1f);
        MAKE_GUI_SETTING_FLOAT(camera_parameter.dist_coeff.at<float>(4), "dist: k3", 0.00001f, "%.05Lf", -0.1f, 0.1f);

        cvui::text("Camera Parameter (external)");
        float temp_deg = Rad2Deg(camera_parameter.pitch());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Pitch", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_parameter.pitch() = Deg2Rad(temp_deg);

        temp_deg = Rad2Deg(camera_parameter.yaw());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Yaw", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_parameter.yaw() = Deg2Rad(temp_deg);

        temp_deg = Rad2Deg(camera_parameter.roll());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Roll", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_parameter.roll() = Deg2Rad(temp_deg);

        MAKE_GUI_SETTING_FLOAT(camera_parameter.x(), "X", 1.0f, "%.0Lf", -20.0f, 20.0f);
        MAKE_GUI_SETTING_FLOAT(camera_parameter.y(), "Y", 1.0f, "%.0Lf", -20.0f, 20.0f);
        MAKE_GUI_SETTING_FLOAT(camera_parameter.z(), "Z", 1.0f, "%.0Lf", -20.0f, 20.0f);
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
            camera_parameter.yaw() += kIncAnglePerPx * (x - s_drag_previous_point.x);
            camera_parameter.pitch() -= kIncAnglePerPx * (y - s_drag_previous_point.y);
            s_drag_previous_point.x = x;
            s_drag_previous_point.y = y;
        }
        camera_parameter.yaw() = (std::min)(Deg2Rad(90.0f), (std::max)(camera_parameter.yaw(), Deg2Rad(-90.0f)));
        camera_parameter.pitch() = (std::min)(Deg2Rad(90.0f), (std::max)(camera_parameter.pitch(), Deg2Rad(-90.0f)));
    }
}


static void TreatKeyInputMain(int32_t key)
{
    static constexpr float kIncPosPerFrame = 0.8f;
    key &= 0xFF;
    switch (key) {
    case 'w':
        camera_parameter.z() -= kIncPosPerFrame;
        break;
    case 'W':
        camera_parameter.z() -= kIncPosPerFrame * 3;
        break;
    case 's':
        camera_parameter.z() += kIncPosPerFrame;
        break;
    case 'S':
        camera_parameter.z() += kIncPosPerFrame * 3;
        break;
    case 'a':
        camera_parameter.x() += kIncPosPerFrame;
        break;
    case 'A':
        camera_parameter.x() += kIncPosPerFrame * 3;
        break;
    case 'd':
        camera_parameter.x() -= kIncPosPerFrame;
        break;
    case 'D':
        camera_parameter.x() -= kIncPosPerFrame * 3;
        break;
    case 'z':
        camera_parameter.y() += kIncPosPerFrame;
        break;
    case 'Z':
        camera_parameter.y() += kIncPosPerFrame * 3;
        break;
    case 'x':
        camera_parameter.y() -= kIncPosPerFrame;
        break;
    case 'X':
        camera_parameter.y() -= kIncPosPerFrame * 3;
        break;
    case 'q':
        camera_parameter.roll() += 0.1f;
        break;
    case 'e':
        camera_parameter.roll() -= 0.1f;
        break;
    }
}

int main(int argc, char* argv[])
{
    cvui::init(kWindowMain);
    cvui::init(kWindowParam);

    cv::setMouseCallback(kWindowMain, CallbackMouseMain);

    while (true) {
        loop_main();
        loop_param();
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
        TreatKeyInputMain(key);
    }

    return 0;
}
