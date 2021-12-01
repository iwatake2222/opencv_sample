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
#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef M_PI
#define M_PI 3.141592653f
#endif

static inline float Deg2Rad(float deg) { return static_cast<float>(deg * M_PI / 180.0); }
static inline float Rad2Deg(float rad) { return static_cast<float>(rad * 180.0 / M_PI); }

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
    class Parameter {
    public:
        float& pitch() { return rvec.at<float>(0); }
        float& yaw() { return rvec.at<float>(1); }
        float& roll() { return rvec.at<float>(2); }
        float& x() { return tvec.at<float>(0); }
        float& y() { return tvec.at<float>(1); }
        float& z() { return tvec.at<float>(2); }
        float& fx() { return K.at<float>(0); }
        float& cx() { return K.at<float>(2); }
        float& fy() { return K.at<float>(4); }
        float& cy() { return K.at<float>(5); }

        /* float, 3 x 1, pitch,  yaw, roll [rad] */
        cv::Mat rvec;

        /* float, 3 x 1, (X, Y, Z): horizontal, vertical, depth (Camera location: Ow - Oc in camera coordinate) */
        cv::Mat tvec;

        /* float, 3 x 3 */
        cv::Mat K;
        cv::Mat K_new;

        /* float, 5 x 1 */
        cv::Mat dist_coeff;
        
        int32_t width;
        int32_t height;

        /* Default Parameters */
        Parameter() {
            SetIntrinsic(1280, 720, 500.0f);
            SetDist({ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f });
            //SetDist({ -0.1f, 0.01f, -0.005f, -0.001f, 0.0f });
            SetExtrinsic({ 0, 0, 0 }, { 0, 0, 0 });
        }

        void SetIntrinsic(int32_t _width, int32_t _height, float focal_length) {
            width = _width;
            height = _height;
            K = (cv::Mat_<float>(3, 3) <<
                focal_length,            0,  width / 2.f,
                           0, focal_length, height / 2.f,
                           0,            0,            1);
            UpdateNewCameraMatrix();
        }

        void SetDist(const std::array<float, 5>& dist) {
            dist_coeff = (cv::Mat_<float>(5, 1) << dist[0], dist[1], dist[2], dist[3], dist[4]);
            UpdateNewCameraMatrix();
        }

        void UpdateNewCameraMatrix()
        {
            if (!K.empty() && !dist_coeff.empty()) {
                K_new = cv::getOptimalNewCameraMatrix(K, dist_coeff, cv::Size(width, height), 0.0);
            }
        }

        /* 
        is_t_on_world == true: tvec = T (Oc - Ow in world coordinate)
        is_t_on_world == false: tvec = tvec (Ow - Oc in camera coordinate) 
        */
        void SetExtrinsic(const std::array<float, 3>& r_deg, const std::array<float, 3>& t, bool is_t_on_world = true)
        {
            rvec = (cv::Mat_<float>(3, 1) << Deg2Rad(r_deg[0]), Deg2Rad(r_deg[1]), Deg2Rad(r_deg[2]));
            tvec = (cv::Mat_<float>(3, 1) << t[0], t[1], t[2]);

            if (is_t_on_world) {
                auto R = MakeRotateMat(r_deg[0], r_deg[1], r_deg[2]);
                tvec = -R * tvec;   /* t = -RT */
            }
        }

        void GetExtrinsic(std::array<float, 3>& r_deg, std::array<float, 3>& t)
        {
            r_deg = { Rad2Deg(rvec.at<float>(0)), Rad2Deg(rvec.at<float>(1)) , Rad2Deg(rvec.at<float>(2)) };
            t = { tvec.at<float>(0), tvec.at<float>(1), tvec.at<float>(2) };
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
    };

    Parameter parameter;

    static float FocalLength(int32_t image_size, float fov)
    {
        /* (w/2) / f = tan(fov/2) */
        return (image_size / 2) / std::tan(Deg2Rad(fov / 2));
    }

    template <typename T = float>
    static cv::Mat MakeRotateMat(T x_deg, T y_deg, T z_deg)
    {
        T x_rad = Deg2Rad(x_deg);
        T y_rad = Deg2Rad(y_deg);
        T z_rad = Deg2Rad(z_deg);
#if 0
        /* Rotation Matrix with Euler Angle */
        cv::Mat R_x = (cv::Mat_<T>(3, 3) <<
            1, 0, 0,
            0, std::cos(x_rad), -std::sin(x_rad),
            0, std::sin(x_rad), std::cos(x_rad));

        cv::Mat R_y = (cv::Mat_<T>(3, 3) <<
            std::cos(y_rad), 0, std::sin(y_rad),
            0, 1, 0,
            -std::sin(y_rad), 0, std::cos(y_rad));

        cv::Mat R_z = (cv::Mat_<T>(3, 3) <<
            std::cos(z_rad), -std::sin(z_rad), 0,
            std::sin(z_rad), std::cos(z_rad), 0,
            0, 0, 1);
        
        cv::Mat R = R_z * R_x * R_y;
#else
        /* Rodrigues */
        cv::Mat r = (cv::Mat_<T>(3, 1) << x_rad, y_rad, z_rad);
        cv::Mat R;
        cv::Rodrigues(r, R);
#endif
        return R;
    }

    static void RotateObject(float x_deg, float y_deg, float z_deg, std::vector<cv::Point3f>& object_point_list)
    {
        cv::Mat R = MakeRotateMat(x_deg, y_deg, z_deg);
        for (auto& object_point : object_point_list) {
            cv::Mat p = (cv::Mat_<float>(3, 1) << object_point.x, object_point.y, object_point.z);
            p = R * p;
            object_point.x = p.at<float>(0);
            object_point.y = p.at<float>(1);
            object_point.z = p.at<float>(2);
        }
    }

    static void MoveObject(float x, float y, float z, std::vector<cv::Point3f>& object_point_list)
    {
        for (auto& object_point : object_point_list) {
            object_point.x += x;
            object_point.y += y;
            object_point.z += z;
        }
    }

    template <typename T = float>
    static void PRINT_MAT_FLOAT(const cv::Mat& mat, int32_t size)
    {
        for (int32_t i = 0; i < size; i++) {
            printf("%d: %.3f\n", i, mat.at<T>(i));
        }
    }


    void ProjectWorld2Image(const std::vector<cv::Point3f>& object_point_list, std::vector<cv::Point2f>& image_point_list)
    {
        /* the followings get exactly the same result */
#if 1
        /*** Projection ***/
        /* s[u, v, 1] = K * [R t] * [M, 1] = K * M_from_cam */
        cv::Mat K = parameter.K;
        cv::Mat R = MakeRotateMat(Rad2Deg(parameter.pitch()), Rad2Deg(parameter.yaw()), Rad2Deg(parameter.roll()));
        cv::Mat Rt = (cv::Mat_<float>(3, 4) <<
            R.at<float>(0), R.at<float>(1), R.at<float>(2), parameter.x(),
            R.at<float>(3), R.at<float>(4), R.at<float>(5), parameter.y(),
            R.at<float>(6), R.at<float>(7), R.at<float>(8), parameter.z());

        image_point_list.resize(object_point_list.size());

#ifdef _OPENMP
#pragma omp parallel for
#endif
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

            cv::Mat UV = K * Mc;
            float u = UV.at<float>(0);
            float v = UV.at<float>(1);
            float s = UV.at<float>(2);
            u /= s;
            v /= s;

            if (parameter.dist_coeff.empty() || parameter.dist_coeff.at<float>(0) == 0) {
                image_point.x = u;
                image_point.y = v;
            } else {
                /*** Distort ***/
                float uu = (u - parameter.cx()) / parameter.fx();  /* from optical center*/
                float vv = (v - parameter.cy()) / parameter.fy();  /* from optical center*/
                float r2 = uu * uu + vv * vv;
                float r4 = r2 * r2;
                float k1 = parameter.dist_coeff.at<float>(0);
                float k2 = parameter.dist_coeff.at<float>(1);
                float p1 = parameter.dist_coeff.at<float>(3);
                float p2 = parameter.dist_coeff.at<float>(4);
                uu = uu + uu * (k1 * r2 + k2 * r4 /*+ k3 * r6 */) + (2 * p1 * uu * vv) + p2 * (r2 + 2 * uu * uu);
                vv = vv + vv * (k1 * r2 + k2 * r4 /*+ k3 * r6 */) + (2 * p2 * uu * vv) + p1 * (r2 + 2 * vv * vv);
                image_point.x = uu * parameter.fx() + parameter.cx();
                image_point.y = vv * parameter.fy() + parameter.cy();
            }
        }
#else
        cv::projectPoints(object_point_list, parameter.rvec, parameter.tvec, parameter.K, parameter.dist_coeff, image_point_list);
#endif
    }


    void ProjectImage2GroundPlane(const std::vector<cv::Point2f>& image_point_list, std::vector<cv::Point3f>& object_point_list)
    {
        /*** Calculate point in ground plane (in world coordinate) ***/
        /* Main idea:*/
        /*   s * [u, v, 1] = K * [R t] * [M, 1]  */
        /*   s * [u, v, 1] = K * R * M + K * t */
        /*   s * Kinv * [u, v, 1] = R * M + t */
        /*   s * Kinv * [u, v, 1] - t = R * M */
        /*   Rinv * (s * Kinv * [u, v, 1] - t) = M */
        /* calculate s */
        /*   s * Rinv * Kinv * [u, v, 1] = M + R_inv * t */
        /*      where, M = (X, Y, Z), and we assume Y = 0(ground_plane) */
        /*      so , we can solve left[1] = R_inv * t[1](camera_height) */

        if (image_point_list.size() == 0) return;

        cv::Mat K = parameter.K;
        cv::Mat R = MakeRotateMat(Rad2Deg(parameter.pitch()), Rad2Deg(parameter.yaw()), Rad2Deg(parameter.roll()));
        cv::Mat K_inv;
        cv::invert(K, K_inv);
        cv::Mat R_inv;
        cv::invert(R, R_inv);
        cv::Mat t = parameter.tvec;

        /*** Undistort image point ***/
        std::vector<cv::Point2f> image_point_undistort;
        if (parameter.dist_coeff.empty() || parameter.dist_coeff.at<float>(0) == 0) {
            image_point_undistort = image_point_list;
        } else {
            cv::undistortPoints(image_point_list, image_point_undistort, parameter.K, parameter.dist_coeff, parameter.K);    /* don't use K_new */
        }

        object_point_list.resize(image_point_list.size());
        for (int32_t i = 0; i < object_point_list.size(); i++) {
            const auto& image_point = image_point_list[i];
            auto& object_point = object_point_list[i];

            float u = image_point_undistort[i].x;
            float v = image_point_undistort[i].y;
            if (v < EstimateVanishmentY()) {
                object_point.x = 999;
                object_point.y = 999;
                object_point.z = 999;
                continue;
            }

            cv::Mat UV = (cv::Mat_<float>(3, 1) << u, v, 1);

            /* calculate s */
            cv::Mat LEFT_WO_S = R_inv * K_inv * UV;
            cv::Mat RIGHT_WO_M = R_inv * t;         /* no need to add M because M[1] = 0 (ground plane)*/
            float s = RIGHT_WO_M.at<float>(1) / LEFT_WO_S.at<float>(1);

            /* calculate M */
            cv::Mat TEMP = R_inv * (s * K_inv * UV - t);

            object_point.x = TEMP.at<float>(0);
            object_point.y = TEMP.at<float>(1);
            object_point.z = TEMP.at<float>(2);
            if (object_point.z < 0) object_point.z = 999;
        }
    }

    void ProjectImage2PosInCamera(const std::vector<cv::Point2f>& image_point_list, const std::vector<float>& z_list, std::vector<cv::Point3f>& object_point_list)
    {
        if (image_point_list.size() == 0) return;

        /*** Undistort image point ***/
        std::vector<cv::Point2f> image_point_undistort;
        if (parameter.dist_coeff.empty() || parameter.dist_coeff.at<float>(0) == 0) {
            image_point_undistort = image_point_list;
        } else {
            cv::undistortPoints(image_point_list, image_point_undistort, parameter.K, parameter.dist_coeff, parameter.K);    /* don't use K_new */
        }

        object_point_list.resize(image_point_list.size());
        for (int32_t i = 0; i < object_point_list.size(); i++) {
            const auto& image_point = image_point_list[i];
            const auto& Zc = z_list[i];
            auto& object_point = object_point_list[i];

            float u = image_point_undistort[i].x;
            float v = image_point_undistort[i].y;

            float x_from_center = u - parameter.cx();
            float y_from_center = v - parameter.cy();
            float Xc = Zc * x_from_center / parameter.fx();
            float Yc = Zc * y_from_center / parameter.fy();
            object_point.x = Xc;
            object_point.y = Yc;
            object_point.z = Zc;
        }
    }


    /* tan(theta) = delta / f */
    float EstimatePitch(float vanishment_y)
    {
        float pitch = std::atan2(parameter.cy() - vanishment_y, parameter.fy());
        return Rad2Deg(pitch);
    }

    float EstimateYaw(float vanishment_x)
    {
        float yaw = std::atan2(parameter.cx() - vanishment_x, parameter.fx());
        return Rad2Deg(yaw);
    }

    int32_t EstimateVanishmentY()
    {
        float fy = parameter.fy();
        float cy = parameter.cy();
        //float fy = parameter.K_new.at<float>(4);
        //float cy = parameter.K_new.at<float>(5);
        float px_from_center = std::tan(parameter.pitch()) * fy;
        float vanishment_y = cy - px_from_center;
        return static_cast<int32_t>(vanishment_y);
    }

    int32_t EstimateVanishmentX()
    {
        float px_from_center = std::tan(parameter.yaw()) * parameter.fx();
        float vanishment_x = parameter.cx() - px_from_center;
        return static_cast<int32_t>(vanishment_x);
    }

};

#endif
