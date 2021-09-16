/* Copyright 2020 iwatake2222

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
#include <cstdint>
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


/*** Function ***/
/* reference: https://github.com/opencv/opencv_zoo/blob/dev/models/face_detection_yunet/yunet.py */
bool FaceDetection::Initialize(const std::string& model_filename)
{
    /*  Read Model */
    net_ = cv::dnn::readNetFromONNX(model_filename);
    if (net_.empty() == true) {
        printf("Failed to create inference engine (%s)\n", model_filename.c_str());
        return false;
    }

    /*  Set backend */
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    /* Display model information */
    for (const auto& layer_name : net_.getUnconnectedOutLayersNames()) {
        printf("Output layer: %s\n", layer_name.c_str());
    }

    return true;
}

bool FaceDetection::Finalize()
{
    return true;
}

bool FaceDetection::Process(const cv::Mat& image_input, std::vector<cv::Rect>& bbox_list, std::vector<Landmark>& landmark_list)
{
    /* Initialize for image size */
    if (prior_list_.empty() || model_input_size_.width == 0) {
        model_input_size_.width = kModelInputWidth;
        model_input_size_.height = kModelInputWidth * image_input.rows / image_input.cols;
        model_input_size_.height = (model_input_size_.height / 32) * 32;    /* just in case */
        GeneratePriors(model_input_size_);
    }

    /* PreProcess */
    cv::Mat blob_input;
    PreProcess(image_input, blob_input);

    /* Inference */
    std::vector<cv::Mat> output_mat_list;
    Inference(blob_input, { "loc", "conf", "iou" }, output_mat_list);

    /* Post Process */
    PostProcess(output_mat_list[0], output_mat_list[1], output_mat_list[2], image_input.size(), bbox_list, landmark_list);

    return true;
}

void FaceDetection::GeneratePriors(const cv::Size& model_input_size)
{
    std::vector<std::pair<int32_t, int32_t>> feature_map_list;
    std::pair<int32_t, int32_t> feature_map_2th = { (model_input_size.height + 1) / 2 / 2, (model_input_size.width + 1) / 2 / 2 };
    feature_map_list.push_back({ (feature_map_2th.first + 1) / 2 , (feature_map_2th.second + 1) / 2 });
    for (int32_t i = 0; i < 3; i++) {
        const auto& previous = feature_map_list.back();
        feature_map_list.push_back({ (previous.first + 1) / 2 , (previous.second + 1) / 2 });
    }

    prior_list_.clear();
    for (int32_t i = 0; i < static_cast<int32_t>(feature_map_list.size()); i++) {
        const auto& min_sizes = min_size_list[i];
        const auto& feature_map = feature_map_list[i];
        for (int32_t y = 0; y < feature_map.first; y++) {
            for (int32_t x = 0; x < feature_map.second; x++) {
                for (const auto& min_size : min_sizes) {
                    float s_kx = static_cast<float>(min_size) / model_input_size.width;
                    float s_ky = static_cast<float>(min_size) / model_input_size.height;
                    float cx = (x + 0.5f) * step_list[i] / model_input_size.width;
                    float cy = (y + 0.5f) * step_list[i] / model_input_size.height;
                    prior_list_.push_back({ cx, cy, s_kx, s_ky });
                }
            }
        }

    }
}

void FaceDetection::PreProcess(const cv::Mat& image_input, cv::Mat& blob_input)
{
    cv::resize(image_input, blob_input, model_input_size_);
    blob_input = cv::dnn::blobFromImage(blob_input);
}

void FaceDetection::Inference(const cv::Mat& blob_input, const std::vector<cv::String> output_name_list, std::vector<cv::Mat>& output_mat_list)
{
    net_.setInput(blob_input);
    net_.forward(output_mat_list, output_name_list);
}

void FaceDetection::PostProcess(const cv::Mat& mat_loc, const cv::Mat& mat_conf, const cv::Mat& mat_iou, const cv::Size image_size, std::vector<cv::Rect>& bbox_list, std::vector<Landmark>& landmark_list)
{
    /* Get score list */
    std::vector<float> cls_score_list;
    for (int32_t row = 0; row < mat_conf.rows; row++) {
        float val = mat_conf.at<float>(cv::Point(1, row));
        val = (std::min)((std::max)(0.0f, val), 1.0f);
        cls_score_list.push_back(val);
    }

    std::vector<float> iou_score_list;
    for (int32_t row = 0; row < mat_iou.rows; row++) {
        float val = mat_conf.at<float>(cv::Point(0, row));
        val = (std::min)((std::max)(0.0f, val), 1.0f);
        iou_score_list.push_back(val);
    }

    std::vector<float> score_list;
    for (int32_t row = 0; row < mat_conf.rows; row++) {
        score_list.push_back(std::sqrt(cls_score_list[row] * iou_score_list[row]));
    }

    /* Get all bbox list */
    std::vector<cv::Rect> bbox_all_list;
    for (int32_t row = 0; row < mat_loc.rows; row++) {
        float cx = mat_loc.at<float>(cv::Point(0, row));
        float cy = mat_loc.at<float>(cv::Point(1, row));
        float w = mat_loc.at<float>(cv::Point(2, row));
        float h = mat_loc.at<float>(cv::Point(3, row));
        cx = prior_list_[row][0] + cx * variance_list[0] * prior_list_[row][2];
        cy = prior_list_[row][1] + cy * variance_list[0] * prior_list_[row][3];
        w = prior_list_[row][2] * std::exp(w * variance_list[0]);
        h = prior_list_[row][3] * std::exp(h * variance_list[1]);
        bbox_all_list.push_back({ static_cast<int32_t>((cx - w / 2) * image_size.width), static_cast<int32_t>((cy - h / 2) * image_size.height), static_cast<int32_t>(w * image_size.width), static_cast<int32_t>(h * image_size.height) });
    }

    /* NMS */
    std::vector<int32_t> indices;
    cv::dnn::NMSBoxes(bbox_all_list, score_list, kThresholdConf, kThresholdNms, indices);

    /* Get valid bbox and land mark list */
    bbox_list.clear();
    landmark_list.clear();
    for (int32_t row : indices) {
        bbox_list.push_back(bbox_all_list[row]);

        Landmark landmark;
        for (int32_t landmark_index = 0; landmark_index < static_cast<int32_t>(landmark.size()); landmark_index++) {
            auto& p = landmark[landmark_index];
            float x = mat_loc.at<float>(cv::Point(4 + landmark_index * 2, row));
            float y = mat_loc.at<float>(cv::Point(4 + landmark_index * 2 + 1, row));
            p.x = static_cast<int32_t>((prior_list_[row][0] + x * variance_list[0] * prior_list_[row][2]) * image_size.width);
            p.y = static_cast<int32_t>((prior_list_[row][1] + y * variance_list[0] * prior_list_[row][3]) * image_size.height);
        }
        landmark_list.push_back(landmark);
    }
}
