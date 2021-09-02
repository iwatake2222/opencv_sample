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
#include <numeric>

#include <opencv2/opencv.hpp>

#define CVUI_IMPLEMENTATION
#include "cvui.h"


/*** Macro ***/
static constexpr char kWindowMain[] = "WindowMain";
static constexpr char kWindowParam[] = "WindowParam";

typedef struct CameraParameter_ {
    float xi;
    float focal_length;

    void Reset()
    {
        xi = 1.0f;
        focal_length = 500.0f;
    }
    CameraParameter_() { Reset(); }
} CameraParameter;


/*** Global variable ***/
static CameraParameter camera_parameter;
static int32_t new_image_size_scale = 3;   /* this value should be adjusted according to distortion level */
static bool update_camera_parameter = true;

/*** Function ***/
static inline float Deg2Rad(float deg) { return static_cast<float>(deg * M_PI / 180.0f); }
static inline float Rad2Deg(float rad) { return static_cast<float>(rad * 180.0f / M_PI); }
static void CreateUndistortMap(cv::Size undist_image_size, float f_undist, float xi, float u0_undist, float v0_undist, float f_dist, float u0_dist, float v0_dist, cv::Mat& mapx, cv::Mat& mapy);


static void loop_main(const cv::Mat& image_org)
{
    cvui::context(kWindowMain);

    /* Set up parameters */
    cv::Size undist_image_size = image_org.size() * new_image_size_scale;
    float f_undist = camera_parameter.focal_length;
    float u0_undist = undist_image_size.width / 2.0f;
    float v0_undist = undist_image_size.height / 2.0f;
    float f_dist = camera_parameter.focal_length;
    float u0_dist = image_org.cols / 2.0f;
    float v0_dist = image_org.rows / 2.0f;
    
    /* Calculate undistort map */
    static cv::Mat mapx, mapy;
    if (update_camera_parameter) {
        CreateUndistortMap(undist_image_size, f_undist, camera_parameter.xi, u0_undist, v0_undist, f_dist, u0_dist, v0_dist, mapx, mapy);
        update_camera_parameter = false;

        /* Create undistorted image */
        cv::Mat image_undistorted;
        cv::remap(image_org, image_undistorted, mapx, mapy, cv::INTER_LINEAR);
        cv::resize(image_undistorted, image_undistorted, cv::Size(), 1.0 / new_image_size_scale, 1.0 / new_image_size_scale);

        cvui::imshow(kWindowMain, image_undistorted);
    }
}

#define MAKE_GUI_SETTING_FLOAT(VAL, LABEL, STEP, FORMAT, RANGE) {\
cvui::beginColumn(-1, -1, 2);\
double temp_double_current = static_cast<double>(VAL);\
double temp_double_new = temp_double_current;\
float temp_float_current = VAL;\
float temp_float_new = temp_float_current;\
cvui::text(LABEL);\
cvui::counter(&temp_double_new, STEP, FORMAT);\
cvui::trackbar<float>(200, &temp_float_new, 0, RANGE);\
if (temp_double_new != temp_double_current) VAL = static_cast<float>(temp_double_new);\
if (temp_float_new != temp_float_current) VAL = temp_float_new;\
cvui::endColumn();\
}

static void loop_param()
{
    cvui::context(kWindowParam);
    cv::Mat mat = cv::Mat(400, 300, CV_8UC3, cv::Scalar(70, 70, 70));

    cvui::beginColumn(mat, 10, 10, -1, -1, 10);
    {
        if (cvui::button(200, 20, "Reset")) {
            camera_parameter.Reset();
            new_image_size_scale = 3;
        }

        MAKE_GUI_SETTING_FLOAT(new_image_size_scale, "Scale", 1.0f, "%.0Lf", 10.0f);

        cvui::text("Camera Parameter (Unified projection model)");
        MAKE_GUI_SETTING_FLOAT(camera_parameter.focal_length, "Focal Length", 10.0f, "%.0Lf", 1000.0f);
        MAKE_GUI_SETTING_FLOAT(camera_parameter.xi, "xi", 0.001f, "%.03Lf", 1.2f);

        if (cvui::button(200, 20, "Update")) {
            update_camera_parameter = true;
        }
    }
    cvui::endColumn();

    cvui::imshow(kWindowParam, mat);
}

int main(int argc, char* argv[])
{
    cvui::init(kWindowMain);
    cvui::init(kWindowParam);

    static const std::string image_path = RESOURCE_DIR"/fisheye_00.jpg";
    cv::Mat image_org = cv::imread(image_path);

    while (true) {
        loop_main(image_org);
        loop_param();
        int32_t key = cv::waitKey(1);
        if (key == 27) break;   /* ESC to quit */
    }

    return 0;
}


/* reference: https://github.com/alexvbogdan/DeepCalib/blob/master/undistortion/undistSphIm.m */
/* Unified projection model */
static void CreateUndistortMap(cv::Size undist_image_size, float f_undist, float xi, float u0_undist, float v0_undist, float f_dist, float u0_dist, float v0_dist, cv::Mat& mapx, cv::Mat& mapy)
{
    cv::Mat grid_x(undist_image_size, CV_32F);
    cv::Mat grid_y(undist_image_size, CV_32F);
    for (int32_t y = 0; y < undist_image_size.height; y++) {
        for (int32_t x = 0; x < undist_image_size.width; x++) {
            grid_x.at<float>(y, x) = x + 0.0f;
            grid_y.at<float>(y, x) = y + 0.0f;
        }
    }

    cv::Mat X_Cam(undist_image_size, CV_32F);
    cv::Mat Y_Cam(undist_image_size, CV_32F);
    cv::Mat Z_Cam(undist_image_size, CV_32F, 1.0f);
    X_Cam = (grid_x - u0_undist) / f_undist;
    Y_Cam = (grid_y - v0_undist) / f_undist;
    //for (int32_t y = 0; y < undist_image_size.height; y++) {
    //    for (int32_t x = 0; x < undist_image_size.width; x++) {
    //        X_Cam.at<float>(y, x) = (grid_x.at<float>(y, x) - u0_undist) / f_undist;
    //        Y_Cam.at<float>(y, x) = (grid_y.at<float>(y, x) - v0_undist) / f_undist;
    //        Z_Cam.at<float>(y, x) = 1.0f;
    //    }
    //}

    cv::Mat Alpha_Cam(undist_image_size, CV_32F);
    for (int32_t y = 0; y < undist_image_size.height; y++) {
        for (int32_t x = 0; x < undist_image_size.width; x++) {
            Alpha_Cam.at<float>(y, x) = 1 / sqrtf(
                X_Cam.at<float>(y, x) * X_Cam.at<float>(y, x)
                + Y_Cam.at<float>(y, x) * Y_Cam.at<float>(y, x)
                + Z_Cam.at<float>(y, x) * Z_Cam.at<float>(y, x)
            );
        }
    }

    cv::Mat X_Sph(undist_image_size, CV_32F);
    cv::Mat Y_Sph(undist_image_size, CV_32F);
    cv::Mat Z_Sph(undist_image_size, CV_32F);
    cv::multiply(X_Cam, Alpha_Cam, X_Sph);
    cv::multiply(Y_Cam, Alpha_Cam, Y_Sph);
    cv::multiply(Z_Cam, Alpha_Cam, Z_Sph);
    //for (int32_t y = 0; y < undist_image_size.height; y++) {
    //    for (int32_t x = 0; x < undist_image_size.width; x++) {
    //        X_Sph.at<float>(y, x) = X_Cam.at<float>(y, x) * Alpha_Cam.at<float>(y, x);
    //        Y_Sph.at<float>(y, x) = Y_Cam.at<float>(y, x) * Alpha_Cam.at<float>(y, x);
    //        Z_Sph.at<float>(y, x) = Z_Cam.at<float>(y, x) * Alpha_Cam.at<float>(y, x);
    //    }
    //}

    cv::Mat den(undist_image_size, CV_32F);
    for (int32_t y = 0; y < undist_image_size.height; y++) {
        for (int32_t x = 0; x < undist_image_size.width; x++) {
            den.at<float>(y, x) = xi * sqrtf(
                X_Sph.at<float>(y, x) * X_Sph.at<float>(y, x)
                + Y_Sph.at<float>(y, x) * Y_Sph.at<float>(y, x)
                + Z_Sph.at<float>(y, x) * Z_Sph.at<float>(y, x)
            ) + Z_Sph.at<float>(y, x);
        }
    }

    mapx = cv::Mat(undist_image_size, CV_32F);
    mapy = cv::Mat(undist_image_size, CV_32F);
    mapx = X_Sph * f_dist / den + u0_dist;
    mapy = Y_Sph * f_dist / den + v0_dist;
    //for (int32_t y = 0; y < undist_image_size.height; y++) {
    //    for (int32_t x = 0; x < undist_image_size.width; x++) {
    //        mapx.at<float>(y, x) = (X_Sph.at<float>(y, x) * f_dist) / den.at<float>(y, x) + u0_dist;
    //        mapy.at<float>(y, x) = (Y_Sph.at<float>(y, x) * f_dist) / den.at<float>(y, x) + v0_dist;
    //    }
    //}
}
