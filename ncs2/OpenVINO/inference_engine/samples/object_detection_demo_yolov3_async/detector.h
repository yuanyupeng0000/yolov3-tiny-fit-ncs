#ifndef DETECTOR_H
#define DETECTOR_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <samples/ocv_common.hpp>
#include <inference_engine.hpp>
#include "Common.h"
using namespace InferenceEngine;

class Detector
{
public:
    Detector(std::string& inputXml,
             std::string& inputBin,
             std::string& inputDevice,
             float thresh=0.3,
             int nireq=2);
    void Detect(const cv::Mat frame);
    void Detect(const cv::Mat frame, std::vector<DetectionObject>& objects);
    InferRequest::Ptr async_infer_request_next;
    InferRequest::Ptr async_infer_request_curr;
    CNNNetReader netReader;
    InputsDataMap inputInfo;
    OutputsDataMap outputInfo;
    std::vector<std::string> labels;
    InferRequest::Ptr IfReqs[4*24];
    float thresh;
    int nireq;
    int current_request_id;

    /**
     * @brief A destructor
     */
    virtual ~Detector() {}
};

#endif // DETECTOR_H
