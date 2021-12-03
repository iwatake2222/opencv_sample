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
#include "depth_engine.h"


/*** Function ***/
/* reference: https://github.com/opencv/opencv_zoo/blob/dev/models/face_detection_yunet/yunet.py */
bool DepthEngine::Initialize(const std::string& model_filename)
{
    /*  Read Model */
    try {
        net_ = cv::dnn::readNetFromONNX(kModelFilename);
    } catch (std::exception &e) {
        printf("%s\n", e.what());
        exit(-1);
    }
    
    if (net_.empty() == true) {
        printf("Failed to create inference engine (%s)\n", kModelFilename);
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

bool DepthEngine::Finalize()
{
    return true;
}

bool DepthEngine::Process(const cv::Mat& image_input, cv::Mat& mat_depth)
{
    /* PreProcess */
    cv::Mat blob_input;
    PreProcess(image_input, blob_input);

    /* Inference */
    std::vector<cv::Mat> output_mat_list;
    Inference(blob_input, { "797" }, output_mat_list);

    /* Post Process */
    /* Inverse relative depth (Far = small Value, Near = huge value) */
    mat_depth = cv::Mat(kModelInputHeight, kModelInputWidth, CV_32FC1, output_mat_list[0].data);
    mat_depth = mat_depth.clone();  /* I need to clone it because output_mat_list is destroyed */

    return true;
}


bool DepthEngine::NormalizeMinMax(const cv::Mat& mat_depth, cv::Mat& mat_depth_normalized)
{
    /***
    * Normalize to uint8_t(0-255) (Far = 255, Neat = 0)
    * Normalized Value  = 255 * (value - min) / (max - min)
    ***/
    mat_depth_normalized = cv::Mat(kModelInputHeight, kModelInputWidth, CV_8UC1);
    double depth_min, depth_max;
    cv::minMaxLoc(mat_depth, &depth_min, &depth_max);
    double range = depth_max - depth_min;
    if (range > 0) {
        mat_depth.convertTo(mat_depth_normalized, CV_8UC1, 255. / range, (-255. * depth_min) / range);
        mat_depth_normalized = 255 - mat_depth_normalized;
        return true;
    } else {
        return false;
    }
}

bool DepthEngine::NormalizeScaleShift(const cv::Mat& mat_depth, cv::Mat& mat_depth_normalized, float scale, float shift)
{
    /***
    * Normalize to float (Far = huge value, Near = small value)
    * 1 / Normalized Value = Estimated Depth(inverse relative depth) * scale + shift
    ***/
    mat_depth_normalized = cv::Mat(kModelInputHeight, kModelInputWidth, CV_32FC1);
    mat_depth.convertTo(mat_depth_normalized, CV_32FC1, scale, shift);
    mat_depth_normalized = 1.0 / mat_depth_normalized;
    return true;
}

void DepthEngine::PreProcess(const cv::Mat& image_input, cv::Mat& blob_input)
{
    cv::Mat image_normalize;
    cv::resize(image_input, image_normalize, cv::Size(kModelInputWidth, kModelInputHeight));
    cv::cvtColor(image_normalize, image_normalize, cv::COLOR_BGR2RGB);
    image_normalize.convertTo(image_normalize, CV_32FC3);
    cv::subtract(image_normalize, cv::Scalar(cv::Vec<float, 3>(kMeanList[0], kMeanList[1], kMeanList[2])), image_normalize);
    cv::multiply(image_normalize, cv::Scalar(cv::Vec<float, 3>(kNormList[0], kNormList[1], kNormList[2])), image_normalize);

    /*
    const std::vector<int> sizes = { 1, image_normalize.rows, image_normalize.cols, image_normalize.channels() };
    cv::Mat mat(sizes, CV_32F, image_normalize.data);
    blob_input = mat.clone();
    printf(" % d, \n", mat.size().width);
    */

    /* NHWC(image) -> NCHW */
    blob_input = cv::dnn::blobFromImage(image_normalize);

}

void DepthEngine::Inference(const cv::Mat& blob_input, const std::vector<cv::String> output_name_list, std::vector<cv::Mat>& output_mat_list)
{
    net_.setInput(blob_input);
    net_.forward(output_mat_list, output_name_list);
}
