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
#ifndef CAMERA_MODEL_
#define CAMERA_MODEL_

/*** Include ***/
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <vector>
#include <array>

#include <opencv2/opencv.hpp>


static inline float Deg2Rad(float deg) { return static_cast<float>(deg * M_PI / 180.0); }
static inline float Rad2Deg(float rad) { return static_cast<float>(rad * 180.0 / M_PI); }
static inline float FocalLength(int32_t image_size, float fov)
{
    /* (w/2) / f = tan(fov/2) */
    return (image_size / 2) / std::tan(Deg2Rad(fov / 2));
}

class CameraModel {
    /***
    * s[u, v, 1] = K * [R t] * [Mw, 1]
    *     K: カメラの内部パラメータ
    *     [R t]: カメラの外部パラメータ
    *     R: ワールド座標上でのカメラの回転行列 (カメラの姿勢)
    *     t: カメラ座標上での、カメラ位置(Oc)からワールド座標原点(Ow)へのベクトル= Ow - Oc
    *         = -RT (T = ワールド座標上でのOwからOcへのベクトル)
    *     Mw: ワールド座標上での対象物体の座標 (Xw, Yw, Zw)
    * s[u, v, 1] = K * Mc
    *     Mc: カメラ座標上での対象物体の座標 (Xc, Yc, Zc)
    *         = [R t] * [Mw, 1]
    * 
    * 理由: Mc = R(Mw - T) = RMw - RT = RMw + t   (t = -RT)
    * 
    * 注意1: tはカメラ座標上でのベクトルである。そのため、Rを変更した場合はtを再計算する必要がある
    * 
    * 注意2: 座標系は右手系。X+ = 右、Y+ = 下、Z+ = 奥  (例. カメラから見て物体が上にある場合、Ycは負値)
    ***/


public:
    /* Intrinsic parameters */
    /* float, 3 x 3 */
    cv::Mat K;

    int32_t width;
    int32_t height;

    /* Extrinsic parameters */
    /* float, 3 x 1, pitch,  yaw, roll [rad] */
    cv::Mat rvec;

    /* float, 3 x 1, (X, Y, Z): horizontal, vertical, depth (Camera location: Ow - Oc in camera coordinate) */
    cv::Mat tvec;


    /* Default Parameters */
    CameraModel() {
        SetIntrinsic(1280, 720, 500.0f);
        SetExtrinsic({ 0, 0, 0 }, { 0, 0, 0 });
    }

    void SetIntrinsic(int32_t _width, int32_t _height, float focal_length) {
        width = _width;
        height = _height;
        K = (cv::Mat_<float>(3, 3) <<
            focal_length,            0,  width / 2.f,
                        0, focal_length, height / 2.f,
                        0,            0,            1);
    }

    void SetExtrinsic(const std::array<float, 3>& r_deg, const std::array<float, 3>& t, bool is_t_on_world = true)
    {
        /*
        is_t_on_world == true: tvec = T (Oc - Ow in world coordinate)
        is_t_on_world == false: tvec = tvec (Ow - Oc in camera coordinate)
        */
        rvec = (cv::Mat_<float>(3, 1) << Deg2Rad(r_deg[0]), Deg2Rad(r_deg[1]), Deg2Rad(r_deg[2]));
        tvec = (cv::Mat_<float>(3, 1) << t[0], t[1], t[2]);

        if (is_t_on_world) {
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            tvec = -R * tvec;   /* t = -RT */
        }
    }

    void SetCameraPos(float x, float y, float z, bool is_on_world = true)    /* Oc - Ow */
    {
        tvec = (cv::Mat_<float>(3, 1) << x, y, z);
        if (is_on_world) {
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            tvec = -R * tvec;   /* t = -RT */
        } else {
            /* Oc - Ow -> Ow - Oc */
            tvec *= -1;
        }
    }

    void MoveCameraPos(float dx, float dy, float dz, bool is_on_world = true)    /* Oc - Ow */
    {
        cv::Mat tvec_delta = (cv::Mat_<float>(3, 1) << dx, dy, dz);
        if (is_on_world) {
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            tvec_delta = -R * tvec_delta;
        } else {
            /* Oc - Ow -> Ow - Oc */
            tvec_delta *= -1;
        }
        tvec += tvec_delta;
    }

    void SetCameraAngle(float pitch_deg, float yaw_deg, float roll_deg)
    {
        /* t vec is vector in camera coordinate, so need to re-calculate it when rvec is updated */
        cv::Mat R_old;
        cv::Rodrigues(rvec, R_old);
        cv::Mat T = -R_old.inv() * tvec;      /* T is tvec in world coordinate.  t = -RT */
        rvec = (cv::Mat_<float>(3, 1) << Deg2Rad(pitch_deg), Deg2Rad(yaw_deg), Deg2Rad(roll_deg));
        cv::Mat R_new;
        cv::Rodrigues(rvec, R_new);
        tvec = -R_new * T;   /* t = -RT */
    }

    void RotateCameraAngle(float dpitch_deg, float dyaw_deg, float droll_deg)
    {
        /* t vec is vector in camera coordinate, so need to re-calculate it when rvec is updated */
        cv::Mat R_old;
        cv::Rodrigues(rvec, R_old);
        cv::Mat T = -R_old.inv() * tvec;      /* T is tvec in world coordinate.  t = -RT */
        cv::Mat rvec_delta = (cv::Mat_<float>(3, 1) << Deg2Rad(dpitch_deg), Deg2Rad(dyaw_deg), Deg2Rad(droll_deg));
        cv::Mat R_delta;
        cv::Rodrigues(rvec_delta, R_delta);
        cv::Mat R_new = R_delta * R_old;
        tvec = -R_new * T;   /* t = -RT */
        cv::Rodrigues(R_new, rvec);
    }


    void ProjectWorld2Image(const std::vector<cv::Point3f>& object_point_list, std::vector<cv::Point2f>& image_point_list)
    {
        /* the followings get exactly the same result */
#if 1
        /*** Projection ***/
        /* s[x, y, 1] = K * [R t] * [M, 1] = K * M_from_cam */
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        cv::Mat Rt = (cv::Mat_<float>(3, 4) <<
            R.at<float>(0), R.at<float>(1), R.at<float>(2), tvec.at<float>(0),
            R.at<float>(3), R.at<float>(4), R.at<float>(5), tvec.at<float>(1),
            R.at<float>(6), R.at<float>(7), R.at<float>(8), tvec.at<float>(2));

        image_point_list.resize(object_point_list.size());
        for (int32_t i = 0; i < object_point_list.size(); i++) {
            const auto& object_point = object_point_list[i];
            auto& image_point = image_point_list[i];
            cv::Mat Mw = (cv::Mat_<float>(4, 1) << object_point.x, object_point.y, object_point.z, 1);
            cv::Mat Mc = Rt * Mw;
            float Zc = Mc.at<float>(2);
            if (Zc <= 0) {
                /* Do not project points behind the camera */
                image_point = cv::Point2f(-1, -1);
                continue;
            }

            cv::Mat xy = K * Mc;
            float x = xy.at<float>(0);
            float y = xy.at<float>(1);
            float s = xy.at<float>(2);
            x /= s;
            y /= s;

            image_point.x = x;
            image_point.y = y;
        }
#else
        cv::projectPoints(object_point_list, rvec, tvec, K, cv::Mat(), image_point_list);
#endif
    }

};

#endif
