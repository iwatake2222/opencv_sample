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
    camera_real.SetExtrinsic(
        { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
        { 0.0f, -1.0f, 0.0f }, true);   /* tvec (Oc - Ow in world coordinate. X+= Right, Y+ = down, Z+ = far) */
        
    camera_top.SetExtrinsic(
        { 90.0f, 0.0f, 0.0f },    /* rvec [deg] */
        { 0.0f, -5.0f, 7.0f }, true);   /* tvec (Oc - Ow in world coordinate. X+= Right, Y+ = down, Z+ = far) */
}

void ResetCamera(int32_t width, int32_t height)
{
    camera_real.SetIntrinsic(width, height, FocalLength(width, kFovDeg));
    camera_top.SetIntrinsic(width, height, FocalLength(width, kFovDeg));
    camera_real.SetDist({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
    camera_top.SetDist({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
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
    cv::projectPoints(object_point_list, camera_real.rvec, camera_real.tvec, camera_real.K, camera_real.dist_coeff, image_point_real_list);

    /* Convert to image points (2D) using the top view camera (virtual camera) */
    std::vector<cv::Point2f> image_point_top_list;
    cv::projectPoints(object_point_list, camera_top.rvec, camera_top.tvec, camera_top.K, camera_top.dist_coeff, image_point_top_list);

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
        MAKE_GUI_SETTING_FLOAT(camera_real.fx(), "Focal Length", 10.0f, "%.0Lf", 0.0f, 1000.0f);
        camera_real.fy() = camera_real.fx();
        camera_top.fx() = camera_real.fx();
        camera_top.fy() = camera_real.fy();

        camera_real.UpdateNewCameraMatrix();
        camera_top.UpdateNewCameraMatrix();

        cvui::text("Top Camera Parameter (Extrinsic)");
        float pitch_deg = Rad2Deg(camera_top.rx());
        MAKE_GUI_SETTING_FLOAT(pitch_deg, "Pitch", 1.0f, "%.0Lf", -180.0f, 180.0f);
        float yaw_deg = Rad2Deg(camera_top.ry());
        MAKE_GUI_SETTING_FLOAT(yaw_deg, "Yaw", 1.0f, "%.0Lf", -180.0f, 180.0f);
        float roll_deg = Rad2Deg(camera_top.rz());
        MAKE_GUI_SETTING_FLOAT(roll_deg, "Roll", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_top.SetCameraAngle(pitch_deg, yaw_deg, roll_deg);


        cvui::text("Real Camera Parameter (Extrinsic)");
        float x = -camera_real.tx();
        float y = -camera_real.ty();
        float z = -camera_real.tz();
        MAKE_GUI_SETTING_FLOAT(y, "Height", 0.5f, "%.1Lf", 0.0f, -5.0f);
        camera_real.SetCameraPos(x, y, z, false);

        pitch_deg = Rad2Deg(camera_real.rx());
        MAKE_GUI_SETTING_FLOAT(pitch_deg, "Pitch", 1.0f, "%.0Lf", -180.0f, 180.0f);
        yaw_deg = Rad2Deg(camera_real.ry());
        MAKE_GUI_SETTING_FLOAT(yaw_deg, "Yaw", 1.0f, "%.0Lf", -180.0f, 180.0f);
        roll_deg = Rad2Deg(camera_real.rz());
        MAKE_GUI_SETTING_FLOAT(roll_deg, "Roll", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera_real.SetCameraAngle(pitch_deg, yaw_deg, roll_deg);
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
            float delta_yaw = kIncAnglePerPx * (x - s_drag_previous_point.x);
            float pitch_delta = -kIncAnglePerPx * (y - s_drag_previous_point.y);
            camera_top.RotateCameraAngle(pitch_delta, delta_yaw, 0);
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
        camera_top.MoveCameraPos(0, 0, kIncPosPerFrame, false);
        break;
    case 'W':
        camera_top.MoveCameraPos(0, 0, kIncPosPerFrame, true);
        break;
    case 's':
        camera_top.MoveCameraPos(0, 0, -kIncPosPerFrame, false);
        break;
    case 'S':
        camera_top.MoveCameraPos(0, 0, -kIncPosPerFrame, true);
        break;
    case 'a':
        camera_top.MoveCameraPos(-kIncPosPerFrame, 0, 0, false);
        break;
    case 'A':
        camera_top.MoveCameraPos(-kIncPosPerFrame, 0, 0, true);
        break;
    case 'd':
        camera_top.MoveCameraPos(kIncPosPerFrame, 0, 0, false);
        break;
    case 'D':
        camera_top.MoveCameraPos(kIncPosPerFrame, 0, 0, true);
        break;
    case 'z':
        camera_top.MoveCameraPos(0, -kIncPosPerFrame, 0, false);
        break;
    case 'Z':
        camera_top.MoveCameraPos(0, -kIncPosPerFrame, 0, true);
        break;
    case 'x':
        camera_top.MoveCameraPos(0, kIncPosPerFrame, 0, false);
        break;
    case 'X':
        camera_top.MoveCameraPos(0, kIncPosPerFrame, 0, true);
        break;
    case 'q':
        camera_top.RotateCameraAngle(0, 0, 2.0f);
        break;
    case 'e':
        camera_top.RotateCameraAngle(0, 0, -2.0f);
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
