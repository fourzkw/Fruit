#ifndef YOLO_INFERENCE_H_
#define YOLO_INFERENCE_H_

#include <string>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <openvino/openvino.hpp>

namespace yolo {

struct Detection {
    short class_id;
    float confidence;
    cv::Rect box;
};

class Inference {
public:
    // 默认构造函数
    Inference() = default;
    
    // 使用默认输入尺寸(640x640)初始化模型
    Inference(const std::string &model_path, 
              const float &model_confidence_threshold = 0.5f,
              const float &model_NMS_threshold = 0.45f);
              
    // 使用指定输入尺寸初始化模型
    Inference(const std::string &model_path, 
              const cv::Size model_input_shape,
              const float &model_confidence_threshold = 0.5f, 
              const float &model_NMS_threshold = 0.45f);

    // 执行推理
    void RunInference(cv::Mat &frame);

    // 获取检测结果轮廓
    const std::vector<std::vector<cv::Point2f>>& GetContours() const { return contours; }

private:
    void InitializeModel(const std::string &model_path);
    void Preprocessing(const cv::Mat &frame);
    void PostProcessing(cv::Mat &frame);
    cv::Rect GetBoundingBox(const cv::Rect &src) const;

    void DrawDetectedObject(cv::Mat &frame, const Detection &detection) const;
    
    cv::Point2f scale_factor_;           // 缩放因子
    cv::Size2f model_input_shape_;       // 模型输入尺寸
    cv::Size model_output_shape_;        // 模型输出尺寸

    ov::InferRequest inference_request_;  // OpenVINO推理请求
    ov::CompiledModel compiled_model_;    // OpenVINO编译模型

    float model_confidence_threshold_ = 0.5f;  // 置信度阈值
    float model_NMS_threshold_ = 0.45f;         // NMS阈值

    std::vector<std::vector<cv::Point2f>> contours;  // 检测结果轮廓
    
    // 类别名称:
    //   0: 
    //   1:
    //   2: 
    //   3: 
    const std::vector<std::string> classes_ {
        "Chili_ripe", "Chili_unripe", "apple_ripe", "apple_unripe", "pumpking_ripe", "pumpking_unripe"
    };
};

} // namespace yolo

#endif // YOLO_INFERENCE_H_