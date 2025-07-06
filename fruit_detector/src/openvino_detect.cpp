#include "fruit_detector/openvino_detect.hpp"
#include <memory>
#include <opencv2/dnn.hpp>
#include <random>

namespace yolo {

// 构造函数1: 使用默认输入尺寸(640x640)初始化模型
Inference::Inference(const std::string &model_path, 
                    const float &model_confidence_threshold,
                    const float &model_NMS_threshold) {
    model_input_shape_ = cv::Size(640, 640);   // 设置默认输入尺寸
    model_confidence_threshold_ = model_confidence_threshold;
    model_NMS_threshold_ = model_NMS_threshold;
    InitializeModel(model_path);
}

// 构造函数2: 使用指定输入尺寸初始化模型
Inference::Inference(const std::string &model_path,
                    const cv::Size model_input_shape,
                    const float &model_confidence_threshold,
                    const float &model_NMS_threshold) {
    model_input_shape_ = model_input_shape;
    model_confidence_threshold_ = model_confidence_threshold;
    model_NMS_threshold_ = model_NMS_threshold;
    InitializeModel(model_path);
}

// 模型初始化方法
void Inference::InitializeModel(const std::string &model_path) {
    try {
        ov::Core core;
        // 读取模型文件
        std::shared_ptr<ov::Model> model = core.read_model(model_path);

            // 处理动态形状模型: 将输入形状固定为指定尺寸
        if (model->is_dynamic()) {
            model->reshape({1, 3, 
                            static_cast<long int>(model_input_shape_.height),
                            static_cast<long int>(model_input_shape_.width)});
        }

        // 创建预处理管道
        ov::preprocess::PrePostProcessor ppp(model);
        // 配置输入预处理
        ppp.input().tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        ppp.input().preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale({255, 255, 255});
        ppp.input().model().set_layout("NCHW");
        ppp.output().tensor().set_element_type(ov::element::f32);
        model = ppp.build();

        // 编译模型并创建推理请求
        compiled_model_ = core.compile_model(model, "AUTO");
        inference_request_ = compiled_model_.create_infer_request();

        // 获取模型输入尺寸
        const std::vector<ov::Output<ov::Node>> inputs = model->inputs();
        const ov::Shape input_shape = inputs[0].get_shape();
        model_input_shape_ = cv::Size(input_shape[2], input_shape[1]); // 宽x高

        // 获取模型输出尺寸
        const std::vector<ov::Output<ov::Node>> outputs = model->outputs();
        const ov::Shape output_shape = outputs[0].get_shape();
        model_output_shape_ = cv::Size(output_shape[2], output_shape[1]);
    }
    catch (const std::exception& e) {
        throw std::runtime_error("模型初始化失败: " + std::string(e.what()));
    }
}

// 执行推理流程
void Inference::RunInference(cv::Mat &frame) {
    if (frame.empty()) {
        return;
    }
    
    contours.clear();
    Preprocessing(frame);
    inference_request_.infer();
    PostProcessing(frame);
}

// 图像预处理方法
void Inference::Preprocessing(const cv::Mat &frame) {
    cv::Mat resized_frame;
    // 将输入图像缩放到模型输入尺寸(双线性插值)
    cv::resize(frame, resized_frame, model_input_shape_, 0, 0, cv::INTER_LINEAR);

    // 计算缩放比例(用于后续坐标还原)
    scale_factor_.x = static_cast<float>(frame.cols) / model_input_shape_.width;
    scale_factor_.y = static_cast<float>(frame.rows) / model_input_shape_.height;

    // 创建OpenVINO输入张量
    const ov::Tensor input_tensor = ov::Tensor(
        compiled_model_.input().get_element_type(),  // 获取模型输入数据类型
        compiled_model_.input().get_shape(),         // 获取模型输入形状
        (float*)resized_frame.data                   // 图像数据指针
    );
    inference_request_.set_input_tensor(input_tensor);
}

// 推理后处理方法
void Inference::PostProcessing(cv::Mat &frame) {
    std::vector<int> class_list;    // 存储类别ID
    std::vector<float> confidence_list; // 存储置信度
    std::vector<cv::Rect> box_list; // 存储边界框
    
    // 获取输出张量数据
    const float* detections = inference_request_.get_output_tensor().data<const float>();
    cv::Mat detection_outputs(model_output_shape_, CV_32F, (float*)detections);

    // 遍历所有检测结果
    for (int i = 0; i < detection_outputs.cols; ++i) {
        // 提取类别概率
        cv::Mat classes_scores = detection_outputs.col(i).rowRange(4, classes_.size()+4);
        
        cv::Point class_id;
        double score;
        cv::minMaxLoc(classes_scores, nullptr, &score, nullptr, &class_id);

        // 过滤低置信度检测结果
        if (score > model_confidence_threshold_) {
            class_list.push_back(class_id.y);
            confidence_list.push_back(score);
            
            // 提取边界框坐标
            const float x = detection_outputs.at<float>(0, i);
            const float y = detection_outputs.at<float>(1, i);
            const float w = detection_outputs.at<float>(2, i);
            const float h = detection_outputs.at<float>(3, i);



            // 计算边界框
            cv::Rect box(
                static_cast<int>(x - w/2),
                static_cast<int>(y - h/2),
                static_cast<int>(w),
                static_cast<int>(h)
            );
            box_list.push_back(box);
        }
    }
    
    // 执行NMS过滤重叠框
    std::vector<int> NMS_result;
    cv::dnn::NMSBoxes(box_list, confidence_list, 
                    model_confidence_threshold_, 
                    model_NMS_threshold_,
                    NMS_result);

    // 处理NMS后的检测结果
    for (size_t i = 0; i < NMS_result.size(); ++i) {
        Detection result;
        const int id = NMS_result[i]; // 获取保留结果的索引
        // 记录检测结果
        result.class_id = class_list[id];     // 类别ID
        result.confidence = confidence_list[id]; // 置信度
        result.box = GetBoundingBox(box_list[id]); // 缩放后的边界框

        contours.emplace_back();
        contours.back().push_back(cv::Point2f(result.class_id, result.confidence)); // 存储类别和置信度
        contours.back().push_back(cv::Point2f(result.box.x, result.box.y)); // 左上角坐标
        contours.back().push_back(cv::Point2f(result.box.x + result.box.width, result.box.y + result.box.height)); // 右下角坐标

        DrawDetectedObject(frame, result); // 绘制检测结果
    }
}

// 边界框坐标缩放方法
cv::Rect Inference::GetBoundingBox(const cv::Rect &src) const {
    cv::Rect box = src;
    // 将相对坐标转换为原始图像坐标
    box.x = static_cast<int>(box.x * scale_factor_.x);
    box.y = static_cast<int>(box.y * scale_factor_.y);
    box.width = static_cast<int>(box.width * scale_factor_.x);
    box.height = static_cast<int>(box.height * scale_factor_.y);
    return box;
}

// 绘制检测结果方法
void Inference::DrawDetectedObject(cv::Mat &frame, const Detection &detection) const {
    const cv::Rect &box = detection.box;       // 边界框
    const float &confidence = detection.confidence; // 置信度
    const int &class_id = detection.class_id; // 类别ID
    
    // 检查类别ID是否在有效范围内
    if (class_id >= 0 && class_id < static_cast<int>(classes_.size())) {
        // 生成随机颜色(120-255范围避免深色)
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(120, 255);
        const cv::Scalar color = cv::Scalar(dis(gen), dis(gen), dis(gen));
        
        // 绘制边界框(线宽3像素)
        cv::rectangle(frame, box, color, 3);
        
        // 创建标签文本: 类别名 + 置信度(保留4位)
        std::string confidence_str = std::to_string(confidence);
        if (confidence_str.length() > 4) {
            confidence_str = confidence_str.substr(0, 4);
        }
        std::string classString = classes_[class_id] + " " + confidence_str;
        
        // 计算文本尺寸
        cv::Size textSize = cv::getTextSize(classString, 
                                          cv::FONT_HERSHEY_DUPLEX, 0.75, 2, 0);
        // 创建文本背景框
        cv::Rect textBox(box.x, box.y - textSize.height - 20, 
                        textSize.width + 10, textSize.height + 20);
        
        // 确保文本框在图像边界内
        textBox.x = std::max(0, textBox.x);
        textBox.y = std::max(0, textBox.y);
        textBox.width = std::min(textBox.width, frame.cols - textBox.x);
        textBox.height = std::min(textBox.height, frame.rows - textBox.y);
        
        // 绘制文本背景框(填充)
        cv::rectangle(frame, textBox, color, cv::FILLED);
        
        // 绘制文本(黑色字体，字号0.75，线宽2)
        cv::putText(frame, classString, 
                   cv::Point(box.x + 5, box.y - 10), 
                   cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(0, 0, 0), 2);
    }
}

} // namespace yolo