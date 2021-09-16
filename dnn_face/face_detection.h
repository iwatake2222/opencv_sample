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
#ifndef FACE_DETECTION_
#define FACE_DETECTION_

/*** Include ***/
#include <cstdint>
#include <string>
#include <vector>
#include <array>

#include <opencv2/opencv.hpp>


class FaceDetection
{
public:
    typedef std::array<cv::Point, 5> Landmark;

private:
    static constexpr int32_t kModelInputWidth = 512;
    static constexpr float kThresholdConf = 0.4f;
    static constexpr float kThresholdNms = 0.3f;
    const std::vector<float> variance_list = { 0.1f, 0.2f };
    const std::vector<std::vector<int32_t>> min_size_list = { { 10, 16, 24 }, { 32, 48 }, { 64, 96 }, { 128, 192, 256 } };
    const std::vector<int32_t> step_list = { 8, 16, 32, 64 };

public:
    FaceDetection() {}
    ~FaceDetection() {}
    bool Initialize(const std::string& model_filename);
    bool Finalize();
    bool Process(const cv::Mat& image_input, std::vector<cv::Rect>& bbox_list, std::vector<Landmark>& landmark_list);

private:
    void GeneratePriors(const cv::Size& model_input_size);
    void PreProcess(const cv::Mat& image_input, cv::Mat& blob_input);
    void Inference(const cv::Mat& blob_input, const std::vector<cv::String> output_name_list, std::vector<cv::Mat>& output_mat_list);
    void PostProcess(const cv::Mat& mat_loc, const cv::Mat& mat_conf, const cv::Mat& mat_iou, const cv::Size image_size, std::vector<cv::Rect>& bbox_list, std::vector<Landmark>& landmark_list);

private:
    cv::dnn::Net net_;
    cv::Size model_input_size_;
    std::vector<std::vector<float>> prior_list_;
};

#endif
