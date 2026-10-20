#ifndef _LQ_NCNN_HPP_
#define _LQ_NCNN_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "net.h"
#include <opencv2/core.hpp>

#define LQ_NCNN_OUTPUT_CLASS_MAX_NUM 16

typedef struct
{
    int predict_index;                                // predicted class index
    float predict_score;                              // predicted class score
    int class_count;                                  // valid class count
    float class_scores[LQ_NCNN_OUTPUT_CLASS_MAX_NUM]; // output scores (truncated if overflow)
    int64_t invoke_time_us;                           // ncnn inference cost (us)
} LqNcnnInferResult;

class LQ_NCNN
{
public:
    LQ_NCNN();

    bool Init();

    std::string Infer(const cv::Mat& bgr_image);

    int InferClass(const cv::Mat& bgr_image,
                   float* predict_score,
                   std::vector<float>* class_scores,
                   int64_t* invoke_time_us);

    void SetModelPath(const std::string& param_path, const std::string& bin_path);

    void SetInputSize(int width, int height);

    void SetLabels(const std::vector<std::string>& labels);

    void SetNormalize(const float mean_vals[3], const float norm_vals[3]);

    void Deinit();

    ~LQ_NCNN();

private:
    int Argmax(const ncnn::Mat& logits, float* best_score, std::vector<float>* class_scores);

private:
    ncnn::Net m_net;
    std::vector<std::string> m_labels;
    bool m_initialized;

    std::string m_param_path;
    std::string m_bin_path;

    int m_input_width;
    int m_input_height;
    float m_mean_vals[3];
    float m_norm_vals[3];

    std::string m_input_name;
    std::string m_output_name;
};

int lq_ncnn_deinit(void);
int lq_ncnn_init(void);
int lq_ncnn_infer(const cv::Mat& src_bgr, LqNcnnInferResult* result);
int lq_ncnn_batch_infer_picture_dir(const char* picture_dir);
int lq_ncnn_detect_and_infer_guarded(const cv::Mat& src_img,int roi_size,LqNcnnInferResult* infer_result);
void lq_ncnn_reset_control_state(void);
void model_use(const cv::Mat& src_img);
void model_use_bypass(const cv::Mat& src_img);
//debug
bool lq_ncnn_get_debug_roi_patch(cv::Mat* roi_img);
//debug

//模型推理成功
extern bool is_model_infer_complete;
//模型控制状态
extern char model_control_access;
extern bool model_red_found;

#endif
