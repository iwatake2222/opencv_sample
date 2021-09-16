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

#include <opencv2/opencv.hpp>

#include "common_helper_cv.h"
#include "face_detection.h"
#include "camera_model.h"

/*** Macro ***/
static constexpr char kInputImageFilename[] = RESOURCE_DIR"/lena.jpg";
static constexpr char kModelFilename[] = RESOURCE_DIR"/face_detection_yunet.onnx";
static constexpr float kFovDeg = 60.0f;

/*** Global variable ***/
static CameraModel camera;


/*** Function ***/
void EstimateHeadPose(cv::Mat& image, const FaceDetection::Landmark& landmark)
{
    /* reference: https://qiita.com/TaroYamada/items/e3f3d0ea4ecc0a832fac */
    /* reference: https://github.com/spmallick/learnopencv/blob/master/HeadPose/headPose.cpp */
    static const std::vector<cv::Point3f> face_object_point_list = {
        { 0.0f, 0.0f, 0.0f },           /* nose */
        { 0.0f, 0.0f, 0.0f },           /* nose */
        { -225.0f, 170.0f, -135.0f },   /* left eye */
        { -225.0f, 170.0f, -135.0f },   /* left eye */
        { 225.0f, 170.0f, -135.0f },    /* right eye */
        { 225.0f, 170.0f, -135.0f },    /* right eye */
        { -150.0f, -150.0f, -125.0f },  /* left lip */
        { -150.0f, -150.0f, -125.0f },  /* left lip */
        { 150.0f, -150.0f, -125.0f },   /* right lip */
        { 150.0f, -150.0f, -125.0f },   /* right lip */
    };

    std::vector<cv::Point2f> face_image_point_list;
    face_image_point_list.push_back(landmark[2]);
    face_image_point_list.push_back(landmark[2]);
    face_image_point_list.push_back(landmark[0]);
    face_image_point_list.push_back(landmark[0]);
    face_image_point_list.push_back(landmark[1]);
    face_image_point_list.push_back(landmark[1]);
    face_image_point_list.push_back(landmark[3]);
    face_image_point_list.push_back(landmark[3]);
    face_image_point_list.push_back(landmark[4]);
    face_image_point_list.push_back(landmark[4]);

    cv::Mat rvec = cv::Mat_<float>(3, 1);
    cv::Mat tvec = cv::Mat_<float>(3, 1);
    cv::solvePnP(face_object_point_list, face_image_point_list, camera.parameter.K, camera.parameter.dist_coeff, rvec, tvec);
    char text[128];
    snprintf(text, sizeof(text), "Pitch = %-+4.0f, Yaw = %-+4.0f, Roll = %-+4.0f", Rad2Deg(rvec.at<float>(0, 0)), Rad2Deg(rvec.at<float>(1, 0)), Rad2Deg(rvec.at<float>(2, 0)));
    CommonHelper::DrawText(image, text, cv::Point(10, 10), 0.7, 3, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255), false);
    
    std::vector<cv::Point3f> nose_end_point3D = { { 0.0f, 0.0f, 200.0f } };
    std::vector<cv::Point2f> nose_end_point2D;
    cv::projectPoints(nose_end_point3D, rvec, tvec, camera.parameter.K, camera.parameter.dist_coeff, nose_end_point2D);
    cv::arrowedLine(image, face_image_point_list[0], nose_end_point2D[0], cv::Scalar(255, 0, 0), 2);

    /* Calculate Euler Angle */
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat projMat = (cv::Mat_<double>(3, 4) <<
        R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2), 0,
        R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2), 0,
        R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2), 0);

    cv::Mat K, R2, tvec2, R_x, R_y, R_z, euler_angle_deg_list;
    cv::decomposeProjectionMatrix(projMat, K, R, tvec2, R_x, R_y, R_z, euler_angle_deg_list);
    double pitch = euler_angle_deg_list.at<double>(0, 0);
    double yaw = euler_angle_deg_list.at<double>(1, 0);
    double roll = euler_angle_deg_list.at<double>(2, 0);
    snprintf(text, sizeof(text), "X = %-+4.0f, Y = %-+4.0f, Z = %-+4.0f", pitch, yaw, roll);
    CommonHelper::DrawText(image, text, cv::Point(10, 40), 0.7, 3, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255), false);
}

int main(int argc, char *argv[])
{
    /* Initialize Model */
    FaceDetection face_detection;
    face_detection.Initialize(kModelFilename);

    /* Find source image */
    std::string input_name = (argc > 1) ? argv[1] : kInputImageFilename;
    cv::VideoCapture cap;   /* if cap is not opened, src is still image */
    if (!CommonHelper::FindSourceImage(input_name, cap)) {
        return -1;
    }

    /* Process for each frame */
    int32_t frame_cnt = 0;
    for (frame_cnt = 0; cap.isOpened() || frame_cnt < 1; frame_cnt++) {
        /* Read image */
        cv::Mat image_input;
        if (cap.isOpened()) {
            cap.read(image_input);
        } else {
            image_input = cv::imread(input_name);
        }
        if (image_input.empty()) break;

        if (frame_cnt == 0) {
            camera.parameter.SetIntrinsic(image_input.cols, image_input.rows, CameraModel::FocalLength(image_input.cols, kFovDeg));
            camera.parameter.SetDist({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
            camera.parameter.SetExtrinsic(
                { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
                { 0.0f, 0.0f, 0.0f }, true);   /* tvec (in world coordinate) */
        }

        /* Detect face */
        std::vector<cv::Rect> bbox_list;
        std::vector<FaceDetection::Landmark> landmark_list;
        face_detection.Process(image_input, bbox_list, landmark_list);

        /* Draw Result */
        for (int32_t i = 0; i < static_cast<int32_t>(bbox_list.size()); i++) {
            const auto& bbox = bbox_list[i];
            const auto& landmark = landmark_list[i];
            cv::rectangle(image_input, bbox, cv::Scalar(255, 0, 0));
            int32_t num = 0;
            for (const auto& p : landmark) {
                cv::circle(image_input, p, 3, cv::Scalar(255, 0, 0), 2);
                cv::putText(image_input, std::to_string(num++), p, 1, 1.0, cv::Scalar(255, 0, 0));
            }
        }

        /* Draw HeadPose */
        for (int32_t i = 0; i < static_cast<int32_t>(landmark_list.size()); i++) {
            EstimateHeadPose(image_input, landmark_list[i]);
        }

        cv::imshow("Result", image_input);
        int32_t key = cv::waitKey(1);
        if (key == 'q') break;
    }

    face_detection.Finalize();
    cv::waitKey(-1);

    return 0;
}
