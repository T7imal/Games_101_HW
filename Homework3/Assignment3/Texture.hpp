//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture {
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name) {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v) {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color00 = image_data.at<cv::Vec3b>(v_img, u_img);
        auto color01 = image_data.at<cv::Vec3b>(v_img, u_img + 1);
        auto color10 = image_data.at<cv::Vec3b>(v_img + 1, u_img);
        auto color11 = image_data.at<cv::Vec3b>(v_img + 1, u_img + 1);
        auto color0 = color00 * ((float)((int)(u_img + 1)) - u_img) + color01 * (u_img - (float)((int)u_img));
        auto color1 = color10 * ((float)((int)(u_img + 1)) - u_img) + color11 * (u_img - (float)((int)u_img));
        auto color = color0 * ((float)((int)(v_img + 1)) - v_img) + color1 * (v_img - (float)((int)v_img));
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
