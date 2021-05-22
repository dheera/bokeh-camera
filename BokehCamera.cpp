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
        float focal_length;
	float depth_of_focus;
};

BokehCamera::BokehCamera() {
  rs2_pipe.start();
  focal_length = 600;
  depth_of_focus = 300;
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

    cv::Mat depth_cv(cv::Size(depth_w, depth_h), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat color_cv(cv::Size(color_w, color_h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

    cv::Mat color_cv_small;
    cv::Mat color_cv_small_blur_1;
    cv::Mat color_cv_small_blur_2;
    cv::Mat color_cv_blur_1;
    cv::Mat color_cv_blur_2;

    cv::resize(color_cv, color_cv_small, cv::Size(640, 360), 0, 0, cv::INTER_NEAREST);
    cv::blur(color_cv_small, color_cv_small_blur_2, cv::Size(10, 10));
    cv::blur(color_cv_small, color_cv_small_blur_1, cv::Size(5, 5));
    cv::resize(color_cv_small_blur_1, color_cv_blur_1, cv::Size(color_w, color_h), 0, 0, cv::INTER_NEAREST);
    cv::resize(color_cv_small_blur_2, color_cv_blur_2, cv::Size(color_w, color_h), 0, 0, cv::INTER_NEAREST);

    auto b = (depth_cv - focal_length) / depth_of_focus;
    cv::Mat cx = 1.0 / (1.0 + b.mul(b));
    cx = cv::max(cv::min(cx, 1.0), 0.0);

    auto c0 = (cx > 0.5).mul((cx - 0.5) * 2, CV_32F);
    auto c1 = (cx > 0.5).mul((1 - c0), CV_32F) + (cx <= 0.5).mul(cx * 2, CV_32F);
    auto c2 = (cx <= 0.5).mul((1 - c1), CV_32F);

    // cv::Mat output_image = c0 * color_cv;

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
