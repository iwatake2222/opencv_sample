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

static constexpr int32_t kWidth = 1280;
static constexpr int32_t kHeight = 720;
static constexpr float   kFovDeg = 130.0f;


/*** Global variable ***/


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
    void ResetReal()
    {
        rvec = (cv::Mat_<float>(3, 1) << Deg2Rad(0.0f), Deg2Rad(0.0f), Deg2Rad(0.0f));
        tvec = (cv::Mat_<float>(3, 1) << 0.0f, 1.0f, 0.0f);
        camera_matrix = (cv::Mat_<float>(3, 3) <<
            FocalLength(),             0,  kWidth / 2.f,
                        0, FocalLength(), kHeight / 2.f,
                        0,             0,             1);
        dist_coeff = (cv::Mat_<float>(5, 1) << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }

    void ResetTopView()
    {
        ResetReal();
        rvec = (cv::Mat_<float>(3, 1) << Deg2Rad(90.0f), Deg2Rad(0.0f), Deg2Rad(0.0f));
        tvec = (cv::Mat_<float>(3, 1) << 0.0f, 8.0f, 7.0f);        /* tvec is in camera coordinate, so Z is height because pitch = 90 */
    }

    CameraParameter_() { ResetReal(); }
} CameraParameter;

static CameraParameter camera_parameter_real;
static CameraParameter camera_parameter_topview;


void RotateObject(float x_deg, float y_deg, float z_deg, std::vector<cv::Point3f>& object_point_list)
{
    float x_rad = Deg2Rad(x_deg);
    float y_rad = Deg2Rad(y_deg);
    float z_rad = Deg2Rad(z_deg);

    cv::Mat R_x = (cv::Mat_<float>(3, 3) <<
                1, 0, 0,
                0, std::cos(x_rad), -std::sin(x_rad),
                0, std::sin(x_rad), std::cos(x_rad));

    cv::Mat R_y = (cv::Mat_<float>(3, 3) <<
                std::cos(y_rad), 0, std::sin(y_rad),
                0, 1, 0,
                -std::sin(y_rad), 0, std::cos(y_rad));

    cv::Mat R_z = (cv::Mat_<float>(3, 3) <<
                std::cos(z_rad), -std::sin(z_rad), 0,
                std::sin(z_rad), std::cos(z_rad), 0,
                0, 0, 1);

    for (auto& object_point : object_point_list) {
        cv::Mat p = (cv::Mat_<float>(3, 1) << object_point.x, object_point.y, object_point.z);
        p = R_x * R_y * R_z * p;
        object_point.x = p.at<float>(0);
        object_point.y = p.at<float>(1);
        object_point.z = p.at<float>(2);
    }
}

void MoveObject(float x, float y, float z, std::vector<cv::Point3f>& object_point_list)
{
    for (auto& object_point : object_point_list) {
        object_point.x += x;
        object_point.y += y;
        object_point.z += z;
    }
}

void CreateMapping(cv::Size size, std::vector<cv::Point2f>& image_point_list, std::vector<cv::Point3f>& object_point_list)
{
    /*** Select target area (possible road area) and calculate corresponding points in world coordinate ***/
    /* todo: consider camera parameter (external) like pitch, yaw, roll */
    static constexpr float kMargin = 10.0f;
    image_point_list = {
        { kMargin, size.height / 2.0f + kMargin },
        { size.width - kMargin, size.height / 2.0f + kMargin },
        { size.width - kMargin, size.height - kMargin },
        { kMargin, size.height - kMargin },
    };

    float focal_length = camera_parameter_real.fx();
    float camera_y = camera_parameter_real.y();

    for (auto& image_point : image_point_list) {
        float x_from_center = image_point.x - camera_parameter_real.cx();
        float y_from_center = image_point.y - camera_parameter_real.cy();
        cv::Point3f object_point;
        object_point.z = focal_length * camera_y / y_from_center;
        object_point.x = x_from_center * camera_y / y_from_center;
        object_point.y = 0;
        object_point_list.push_back(object_point);
    }
}

static void loop_main(const cv::Mat& image_org)
{
    cvui::context(kWindowMain);

    /* Generate object points (3D: world coordinate) using the dash camera (real camera) */
    std::vector<cv::Point3f> object_point_list;
    std::vector<cv::Point2f> roi_image_point_list;
    CreateMapping(image_org.size(), roi_image_point_list, object_point_list);

    /* Convert to image points (2D) using the top view camera (virtual camera) */
    std::vector<cv::Point2f> image_point_list;
    cv::projectPoints(object_point_list, camera_parameter_topview.rvec, camera_parameter_topview.tvec, camera_parameter_topview.camera_matrix, camera_parameter_topview.dist_coeff, image_point_list);

    /* Affine transform */
    cv::Mat mat_affine = cv::getPerspectiveTransform(&roi_image_point_list[0], &image_point_list[0]);
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
    cv::Mat mat = cv::Mat(600, 300, CV_8UC3, cv::Scalar(70, 70, 70));

    cvui::beginColumn(mat, 10, 10, -1, -1, 2);
    {
        if (cvui::button(120, 20, "Reset")) {
            camera_parameter_real.ResetReal();
            camera_parameter_topview.ResetTopView();
        }

        cvui::text("Camera Parameter (internal)");
        MAKE_GUI_SETTING_FLOAT(camera_parameter_real.fx(), "Focal Length", 10.0f, "%.0Lf", 0.0f, 1000.0f);
        camera_parameter_real.fy() = camera_parameter_real.fx();
        camera_parameter_topview.fx() = camera_parameter_real.fx();
        camera_parameter_topview.fy() = camera_parameter_real.fy();

        cvui::text("Camera Parameter (external)");
        float temp_deg = Rad2Deg(camera_parameter_real.pitch());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Pitch", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_parameter_real.pitch() = Deg2Rad(temp_deg);

        temp_deg = Rad2Deg(camera_parameter_real.yaw());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Yaw", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_parameter_real.yaw() = Deg2Rad(temp_deg);

        temp_deg = Rad2Deg(camera_parameter_real.roll());
        MAKE_GUI_SETTING_FLOAT(temp_deg, "Roll", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_parameter_real.roll() = Deg2Rad(temp_deg);

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
            camera_parameter_topview.yaw() += kIncAnglePerPx * (x - s_drag_previous_point.x);
            camera_parameter_topview.pitch() -= kIncAnglePerPx * (y - s_drag_previous_point.y);
            s_drag_previous_point.x = x;
            s_drag_previous_point.y = y;
        }
        camera_parameter_topview.yaw() = (std::min)(Deg2Rad(90.0f), (std::max)(camera_parameter_topview.yaw(), Deg2Rad(-90.0f)));
        camera_parameter_topview.pitch() = (std::min)(Deg2Rad(90.0f), (std::max)(camera_parameter_topview.pitch(), Deg2Rad(-90.0f)));
    }
}


static void TreatKeyInputMain(int32_t key)
{
    static constexpr float kIncPosPerFrame = 0.8f;
    key &= 0xFF;
    switch (key) {
    case 'w':
        camera_parameter_topview.z() -= kIncPosPerFrame;
        break;
    case 'W':
        camera_parameter_topview.z() -= kIncPosPerFrame * 3;
        break;
    case 's':
        camera_parameter_topview.z() += kIncPosPerFrame;
        break;
    case 'S':
        camera_parameter_topview.z() += kIncPosPerFrame * 3;
        break;
    case 'a':
        camera_parameter_topview.x() += kIncPosPerFrame;
        break;
    case 'A':
        camera_parameter_topview.x() += kIncPosPerFrame * 3;
        break;
    case 'd':
        camera_parameter_topview.x() -= kIncPosPerFrame;
        break;
    case 'D':
        camera_parameter_topview.x() -= kIncPosPerFrame * 3;
        break;
    case 'z':
        camera_parameter_topview.y() += kIncPosPerFrame;
        break;
    case 'Z':
        camera_parameter_topview.y() += kIncPosPerFrame * 3;
        break;
    case 'x':
        camera_parameter_topview.y() -= kIncPosPerFrame;
        break;
    case 'X':
        camera_parameter_topview.y() -= kIncPosPerFrame * 3;
        break;
    case 'q':
        camera_parameter_topview.roll() += 0.1f;
        break;
    case 'e':
        camera_parameter_topview.roll() -= 0.1f;
        break;
    }
}

int main(int argc, char* argv[])
{
    cvui::init(kWindowMain);
    cvui::init(kWindowParam);

    cv::setMouseCallback(kWindowMain, CallbackMouseMain);

    camera_parameter_real.ResetReal();
    camera_parameter_topview.ResetTopView();

    static const std::string image_path = RESOURCE_DIR"/dashcam_00.jpg";
    cv::Mat image_org = cv::imread(image_path);

    while (true) {
        loop_main(image_org);
        loop_param();
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
        TreatKeyInputMain(key);
    }

    return 0;
}
