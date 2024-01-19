//
// Created by xinyang on 2021/4/8.
//
#pragma once
#ifndef _ONNXTRTMODULE_HPP_
#define _ONNXTRTMODULE_HPP_

#include <fstream>
#include <iostream>
#include <sstream>
#include <numeric>
#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include "NvInfer.h"
#include "cuda_runtime_api.h"
#include "../common/preprocess.h"
#include "../common/common.h"

#define MAX_IMAGE_INPUT_SIZE_THRESH 5000 * 5000

#define DEVICE 0 // GPU id
#define INPUT_H 416
#define INPUT_W 416
static const int NUM_CLASSES = 8;
#define DEVICE 0 // GPU id

enum class COLOR
{
    RED,
    BLUE,
    GRAY
};

struct Object
{   

    cv::Rect_<float> rect; // rect
    float landmarks[10];   // 四点
    int label;             // class
    float prob;            // confidence
    cv::Point2f pts[4];
    Eigen::Vector3d coord;
    Eigen::Vector3d pyr;
};
struct bbox
{
    float x1, x2, y1, y2;
    float landmarks[10];
    float score;
};

/*
 * 四点模型
 */
class TRTModule
{
    static constexpr int TOPK_NUM = 128;
    static constexpr float KEEP_THRES = 0.1f;
    
public:
    explicit TRTModule(const std::string &trt_file);

    ~TRTModule();

    TRTModule(const TRTModule &) = delete;

    TRTModule operator=(const TRTModule &) = delete;

    std::vector<Object> operator()(const cv::Mat &src,float confidence_threshold ,float nms_threshold);

   

private:
    float intersection_area(const Object &a, const Object &b);
    void qsort_descent_inplace(std::vector<Object> &faceobjects, int left, int right);
    void qsort_descent_inplace(std::vector<Object> &objects);
    void nms_sorted_bboxes(const std::vector<Object> &faceobjects,
                           std::vector<int> &picked,
                           float nms_threshold);
    int find_max(float *prob, int num);
    void generate_yolox_proposals(float *feat_blob,
                                  float prob_threshold,
                                  std::vector<Object> &objects,
                                  int OUTPUT_CANDIDATES);
    void decode_outputs(float *prob,
                        std::vector<Object> &objects,
                        float scale,
                        const int img_w,
                        const int img_h,
                        int OUTPUT_CANDIDATES,
                        int top,
                        int left,
                       const cv::Mat &src,
                       float confidence_threshold ,
                       float nms_threshold);
    void doInference_cu(nvinfer1::IExecutionContext &context,
                        cudaStream_t &stream,
                        void **buffers,
                        float *output,
                        int batchSize,
                        int OUTPUT_SIZE);
    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;

    int OUTPUT_CANDIDATES;
    cudaStream_t stream;
    float *buffers[2];
    int inputIndex;
    int outputIndex;
    uint8_t *img_host = nullptr;
    uint8_t *img_device = nullptr;
    int output_size = 1;
public:
    COLOR color_id;
    bool State_outpost=false;
};
#endif
 /* _ONNXTRTMODULE_HPP_ */
