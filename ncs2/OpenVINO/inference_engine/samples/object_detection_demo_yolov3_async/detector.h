#ifndef DETECTOR_H
#define DETECTOR_H

#include <stdio.h>
#include <string>
#include <iostream>
#include <samples/ocv_common.hpp>
#include <inference_engine.hpp>
using namespace InferenceEngine;

class Detector
{
public:
    Detector(std::string& inputXml, std::string& inputBin, std::string& inputDevice, int nireq=2);
    void Detect(const cv::Mat frame);
    InferRequest::Ptr async_infer_request_next;
    InferRequest::Ptr async_infer_request_curr;
    CNNNetReader netReader;
    InputsDataMap inputInfo;
    OutputsDataMap outputInfo;
    std::vector<std::string> labels;
    float thresh = 0.3;

    /**
     * @brief A destructor
     */
    virtual ~Detector() {}
};

#endif // DETECTOR_H
