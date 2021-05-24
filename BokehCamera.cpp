#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <librealsense2/rs.hpp>

#define VID_WIDTH  1280
#define VID_HEIGHT 720
#define VIDEO_IN   "/dev/video0"
#define VIDEO_OUT  "/dev/video6"

#include <iostream>

void clamp(cv::Mat& mat, cv::Point2f lowerBound, cv::Point2f upperBound) {
    std::vector<cv::Mat> matc;
    cv::split(mat, matc);
    cv::min(cv::max(matc[0], lowerBound.x), upperBound.x, matc[0]);
    cv::min(cv::max(matc[1], lowerBound.y), upperBound.y, matc[1]);
    cv::merge(matc, mat);   
}

class BokehCamera {
    public:
        BokehCamera();
	void start();
    private:
        rs2::pipeline rs2_pipe;
        float flength; // focal length
	float dof;     // depth of focus
};

BokehCamera::BokehCamera() {
  rs2_pipe.start();
  flength = 600.0f;
  dof = 300.0f;
}

void BokehCamera::start() {
  for(;;) {
    rs2::frameset data = rs2_pipe.wait_for_frames();
    rs2::depth_frame depth = data.get_depth_frame();
    rs2::video_frame color = data.get_color_frame();

    std::cout << "received depth frame: " << depth.get_width() << "x" << depth.get_height() << std::endl;
    std::cout << "received color frame: " << color.get_width() << "x" << color.get_height() << std::endl;

    int depth_w = depth.get_width();
    int depth_h = depth.get_height();
    int color_w = color.get_width();
    int color_h = color.get_height();

    cv::Mat img_depth(cv::Size(depth_w, depth_h), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat img_color(cv::Size(color_w, color_h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

    cv::Mat img_color_small;
    cv::Mat img_color_small_blur_1;
    cv::Mat img_color_small_blur_2;
    cv::Mat img_color_blur_1;
    cv::Mat img_color_blur_2;

    cv::resize(img_color, img_color_small, cv::Size(640, 360), 0, 0, cv::INTER_NEAREST);
    cv::blur(img_color_small, img_color_small_blur_2, cv::Size(10, 10));
    cv::blur(img_color_small, img_color_small_blur_1, cv::Size(5, 5));
    cv::resize(img_color_small_blur_1, img_color_blur_1, cv::Size(color_w, color_h), 0, 0, cv::INTER_NEAREST);
    cv::resize(img_color_small_blur_2, img_color_blur_2, cv::Size(color_w, color_h), 0, 0, cv::INTER_NEAREST);

    auto b = (img_depth - flength) / dof;
    cv::Mat cx = 1.0f / (1.0f + b.mul(b));
    cx = cv::max(cv::min(cx, 1.0f), 0.0f);

    auto c0 = (cx > 0.5f).mul((cx - 0.5f) * 2.0f);
    auto c1 = (cx > 0.5f).mul((1.0f - c0)) + (cx <= 0.5f).mul(cx * 2.0f);
    auto c2 = (cx <= 0.5f).mul((1.0f - c1));

    // cv::Mat output_image = c0 * img_color;

    // cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    // cv::imshow("Display Image", output_image);

    auto key = cv::waitKey(1);
    if (key == 32) break;
  }
}

int main(void) {
  BokehCamera b;
  b.start();
}
