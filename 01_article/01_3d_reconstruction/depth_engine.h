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
#ifndef DEPTH_ENGINE_
#define DEPTH_ENGINE_

/*** Include ***/
#include <cstdint>
#include <string>
#include <vector>
#include <array>

#include <opencv2/opencv.hpp>


class DepthEngine
{
public:
    typedef std::array<cv::Point, 5> Landmark;

private:
    static constexpr char kModelFilename[] = RESOURCE_DIR"/model/midasv2_small_256x256.onnx";
    static constexpr int32_t kModelInputWidth = 256;
    static constexpr int32_t kModelInputHeight = 256;
    const std::array<float, 3> kMeanList = { 0.485f, 0.456f, 0.406f };
    const std::array<float, 3> kNormList = { 0.229f, 0.224f, 0.225f };

public:
    DepthEngine() {}
    ~DepthEngine() {}
    bool Initialize(const std::string& model_filename);
    bool Finalize();
    bool Process(const cv::Mat& image_input, cv::Mat& mat_depth);
    bool NormalizeMinMax(const cv::Mat& mat_depth, cv::Mat& mat_depth_normalized);
    bool NormalizeScaleShift(const cv::Mat& mat_depth, cv::Mat& mat_depth_normalized, float scale, float shift);

private:
    void PreProcess(const cv::Mat& image_input, cv::Mat& blob_input);
    void Inference(const cv::Mat& blob_input, const std::vector<cv::String> output_name_list, std::vector<cv::Mat>& output_mat_list);

private:
    cv::dnn::Net net_;

};

#endif
