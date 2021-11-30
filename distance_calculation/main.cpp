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
static constexpr char kInputFilename[] = RESOURCE_DIR"/dashcam_00.jpg";
static constexpr char kWindowMain[] = "WindowMain";
static constexpr char kWindowParam[] = "WindowParam";

static constexpr int32_t kWidth = 1280;
static constexpr int32_t kHeight = 720;
static constexpr float kFovDeg = 130.0f;


/*** Global variable ***/
static CameraModel camera;
static std::vector<cv::Point2f> selecting_point_list;

/*** Function ***/
void ResetCameraPose()
{
    camera.parameter.SetExtrinsic(
        { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
        { 0.0f, -1.5f, 0.0f }, true);   /* tvec (Oc - Ow in world coordinate. X+= Right, Y+ = down, Z+ = far) */
}

void ResetCamera(int32_t width, int32_t height)
{
    camera.parameter.SetIntrinsic(width, height, CameraModel::FocalLength(width, kFovDeg));
    camera.parameter.SetDist({ -0.1f, 0.01f, -0.005f, -0.001f, 0.0f });
    ResetCameraPose();
}


static void loop_main(const cv::Mat& image_org)
{
    cv::Mat image;
    cvui::context(kWindowMain);
    if (!image_org.empty()) {
        image = image_org.clone();

        std::vector<cv::Point3f> object_point_list;
        camera.ProjectImage2GroundPlane(selecting_point_list, object_point_list);
        for (int32_t i = 0; i < selecting_point_list.size(); i++) {
            const auto& image_point = selecting_point_list[i];
            cv::circle(image, image_point, 5, cv::Scalar(255, 0, 0), -1);
            char text[64];
            snprintf(text, sizeof(text), "%.1f, %.1f[m]", object_point_list[i].x, object_point_list[i].z);
            cv::putText(image, text, image_point, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
        }
    } else {
        std::vector<cv::Point3f> original_object_point_list;
        for (float x = -10; x <= 10; x += 1) {
            for (float z = 0; z <= 20; z += 1) {
                original_object_point_list.push_back(cv::Point3f(x, 0, z));
            }
        }
        std::vector<cv::Point2f> image_point_list;
        cv::projectPoints(original_object_point_list, camera.parameter.rvec, camera.parameter.tvec, camera.parameter.K, camera.parameter.dist_coeff, image_point_list);
        image = cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(70, 70, 70));

        /* Re-convert image point to object poitn(world) */
        std::vector<cv::Point3f> object_point_list;
        camera.ProjectImage2GroundPlane(image_point_list, object_point_list);
        for (int32_t i = 0; i < image_point_list.size(); i++) {
            const auto& image_point = image_point_list[i];
            /* Draw calculated imate point */
            cv::circle(image, image_point_list[i], 2, cv::Scalar(220, 0, 0), -1);

            /* draw to check */
            char text[64];
            snprintf(text, sizeof(text), "%.1f, %.1f", object_point_list[i].x, object_point_list[i].z);
            cv::putText(image, text, image_point_list[i] + cv::Point2f(0, 0), 0, 0.4, cv::Scalar(0, 255, 0));
        }
    }

    cv::line(image, cv::Point(0, camera.EstimateVanishmentY()), cv::Point(image.cols, camera.EstimateVanishmentY()), cv::Scalar(0, 0, 0), 1);
    cvui::imshow(kWindowMain, image);
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
    cv::Mat mat = cv::Mat(500, 300, CV_8UC3, cv::Scalar(70, 70, 70));

    cvui::beginColumn(mat, 10, 10, -1, -1, 2);
    {
        if (cvui::button(120, 20, "Reset")) {
            ResetCameraPose();
        }
        if (cvui::button(120, 20, "ResetImage")) {
            selecting_point_list.clear();
        }

        cvui::text("Camera Parameter (Intrinsic)");
        MAKE_GUI_SETTING_FLOAT(camera.parameter.fx(), "Focal Length", 10.0f, "%.0Lf", 0.0f, 1000.0f);
        camera.parameter.fy() = camera.parameter.fx();
        camera.parameter.UpdateNewCameraMatrix();

        cvui::text("Camera Parameter (Extrinsic)");
        float x = -camera.parameter.x();
        float y = -camera.parameter.y();
        float z = -camera.parameter.z();
        MAKE_GUI_SETTING_FLOAT(y, "Height", 0.5f, "%.1Lf", 0.0f, -5.0f);
        camera.parameter.SetCameraPos(x, y, z, false);

        cvui::text("Camera Parameter (Extrinsic)");
        float pitch_deg = Rad2Deg(camera.parameter.pitch());
        MAKE_GUI_SETTING_FLOAT(pitch_deg, "Pitch", 1.0f, "%.0Lf", -90.0f, 90.0f);
        float yaw_deg = Rad2Deg(camera.parameter.yaw());
        MAKE_GUI_SETTING_FLOAT(yaw_deg, "Yaw", 1.0f, "%.0Lf", -90.0f, 90.0f);
        float roll_deg = Rad2Deg(camera.parameter.roll());
        MAKE_GUI_SETTING_FLOAT(roll_deg, "Roll", 1.0f, "%.0Lf", -90.0f, 90.0f);
        camera.parameter.SetCameraAngle(pitch_deg, yaw_deg, roll_deg);
    }
    cvui::endColumn();

    cvui::imshow(kWindowParam, mat);
}

static void CallbackMouseMain(int32_t event, int32_t x, int32_t y, int32_t flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONUP) {
    } else if (event == cv::EVENT_LBUTTONDOWN) {
        cv::Point2f point(static_cast<float>(x), static_cast<float>(y));
        selecting_point_list.push_back(point);
    } else {
    }
}



static void TreatKeyInputMain(int32_t key)
{
    static constexpr float kIncPosPerFrame = 0.8f;
    key &= 0xFF;
    switch (key) {
    case 'w':
        camera.parameter.MoveCameraPos(0, 0, kIncPosPerFrame, false);
        break;
    case 'W':
        camera.parameter.MoveCameraPos(0, 0, kIncPosPerFrame, true);
        break;
    case 's':
        camera.parameter.MoveCameraPos(0, 0, -kIncPosPerFrame, false);
        break;
    case 'S':
        camera.parameter.MoveCameraPos(0, 0, -kIncPosPerFrame, true);
        break;
    case 'a':
        camera.parameter.MoveCameraPos(-kIncPosPerFrame, 0, 0, false);
        break;
    case 'A':
        camera.parameter.MoveCameraPos(-kIncPosPerFrame, 0, 0, true);
        break;
    case 'd':
        camera.parameter.MoveCameraPos(kIncPosPerFrame, 0, 0, false);
        break;
    case 'D':
        camera.parameter.MoveCameraPos(kIncPosPerFrame, 0, 0, true);
        break;
    case 'z':
        camera.parameter.MoveCameraPos(0, -kIncPosPerFrame, 0, false);
        break;
    case 'Z':
        camera.parameter.MoveCameraPos(0, -kIncPosPerFrame, 0, true);
        break;
    case 'x':
        camera.parameter.MoveCameraPos(0, kIncPosPerFrame, 0, false);
        break;
    case 'X':
        camera.parameter.MoveCameraPos(0, kIncPosPerFrame, 0, true);
        break;
    case 'q':
        camera.parameter.RotateCameraAngle(0, 0, 2.0f);
        break;
    case 'e':
        camera.parameter.RotateCameraAngle(0, 0, -2.0f);
        break;
    }
}

int main(int argc, char* argv[])
{
    cvui::init(kWindowMain);
    cvui::init(kWindowParam);

    cv::setMouseCallback(kWindowMain, CallbackMouseMain);

    cv::Mat image_org = cv::imread(kInputFilename);
    if (!image_org.empty()) {
        ResetCamera(image_org.cols, image_org.rows);
    } else {
        ResetCamera(kWidth, kHeight);
    }

    while (true) {
        loop_main(image_org);
        loop_param();
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
        TreatKeyInputMain(key);
    }

    return 0;
}
