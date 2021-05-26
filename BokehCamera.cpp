#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <librealsense2/rs.hpp>

#define VID_WIDTH  1280
#define VID_HEIGHT 720

#include <iostream>

void clamp(cv::Mat& mat, cv::Point2f lowerBound, cv::Point2f upperBound) {
    std::vector<cv::Mat> matc;
    cv::split(mat, matc);
    cv::min(cv::max(matc[0], lowerBound.x), upperBound.x, matc[0]);
    cv::min(cv::max(matc[1], lowerBound.y), upperBound.y, matc[1]);
    cv::merge(matc, mat);   
}

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

class BokehCamera {
    public:
        BokehCamera();
        ~BokehCamera();
	void start();
    private:
        rs2::pipeline rs2_pipe;
        float flength; // focal length
	float dof;     // depth of focus
        std::string output_device;
	std::vector<cv::Mat> img_out_buffer;
	int vid_out;
	size_t framesize;
};

BokehCamera::BokehCamera() {
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    rs2_pipe.start(cfg);
    flength = 600.0f;
    dof = 300.0f;
    output_device = "/dev/video20";

    vid_out = open(output_device.c_str(), O_RDWR);
    if(vid_out < 0) {
        std::cerr << "ERROR: could not open output device!\n" <<
        strerror(errno);
	//return -2;
    }

    struct v4l2_format vid_format;
    memset(&vid_format, 0, sizeof(vid_format));
    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

    if (ioctl(vid_out, VIDIOC_G_FMT, &vid_format) < 0) {
        std::cerr << "ERROR: unable to get video format!\n" <<
        strerror(errno); // return -1;
    }   

    // configure desired video format on device
    framesize = 1280 * 720 * 3;
    vid_format.fmt.pix.width = 1280;
    vid_format.fmt.pix.height = 720;
    vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    vid_format.fmt.pix.sizeimage = framesize;
    vid_format.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(vid_out, VIDIOC_S_FMT, &vid_format) < 0) {
        std::cerr << "ERROR: unable to set video format!\n" <<
        strerror(errno);
       	//return -1;
    }
}

BokehCamera::~BokehCamera() {
    close(vid_out);
}

void BokehCamera::start() {
  rs2::align align_to_color(RS2_STREAM_COLOR);

  for(;;) {
    // fetch synchronized depth and color frames from RealSense
    rs2::frameset data = rs2_pipe.wait_for_frames();
    rs2::depth_frame depth = data.get_depth_frame();
    rs2::video_frame color = data.get_color_frame();

    data = align_to_color.process(data);

    std::cout << "received depth frame: " << depth.get_width() << "x" << depth.get_height() << std::endl;
    std::cout << "received color frame: " << color.get_width() << "x" << color.get_height() << std::endl;

    int depth_w = depth.get_width();
    int depth_h = depth.get_height();
    int color_w = color.get_width();
    int color_h = color.get_height();

    // convert to OpenCV matrices
    cv::Mat img_depth(cv::Size(depth_w, depth_h), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat img_color(cv::Size(color_w, color_h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

    // compute blurred images for a small kernel and large kernel on scaled-down versions to reduce CPU usage
    // they will be scaled back up and interpolated to simulate other blur kernels in-between
    cv::Mat img_color_small;
    cv::Mat img_color_small_blur_1;
    cv::Mat img_color_small_blur_2;
    cv::resize(img_color, img_color_small, cv::Size(640, 360), 0, 0, cv::INTER_NEAREST);
    cv::blur(img_color_small, img_color_small_blur_2, cv::Size(10, 10));
    cv::blur(img_color_small, img_color_small_blur_1, cv::Size(5, 5));

    // scale back up after blurring
    cv::Mat img_color_blur_1;
    cv::Mat img_color_blur_2;
    cv::resize(img_color_small_blur_1, img_color_blur_1, cv::Size(color_w, color_h), 0, 0, cv::INTER_NEAREST);
    cv::resize(img_color_small_blur_2, img_color_blur_2, cv::Size(color_w, color_h), 0, 0, cv::INTER_NEAREST);

    // compute amount of blur at each pixel
    cv::Mat1f b = (img_depth - flength);
    b /= dof;
    cv::Mat1f cx = 1.0f / (1.0f + b.mul(b));
    cx = cv::max(cv::min(cx, 1.0f), 0.0f);

    CV_Assert(cx.depth() == CV_32F);

    // compute coefficients and interpolate
    //
    cv::Mat output(cx.rows, cx.cols, CV_8UC3);

    cv::MatIterator_<float> it, end;
    cv::MatIterator_<cv::Vec3b> iti0 = img_color.begin<cv::Vec3b>();
    cv::MatIterator_<cv::Vec3b> iti1 = img_color_blur_1.begin<cv::Vec3b>();
    cv::MatIterator_<cv::Vec3b> iti2 = img_color_blur_2.begin<cv::Vec3b>();
    cv::MatIterator_<cv::Vec3b> ito = output.begin<cv::Vec3b>();

    float q0, q1, q2;

    for(it = cx.begin(), end = cx.end(); it != end; ++it) {
	q0 = (*it > 0.5f) * (*it - 0.5f) * 2.0f;
	q1 = (*it > 0.5f) * (1.0f - q0) + (*it <= 0.5f) * (*it * 2.0f);
	q2 = (*it < 0.5f) * (1.0f - q1);

	(*ito)[0] = q0 * (*iti0)[0] + q1 * (*iti1)[0] + q2 * (*iti2)[0];
	(*ito)[1] = q0 * (*iti0)[1] + q1 * (*iti1)[1] + q2 * (*iti2)[1];
	(*ito)[2] = q0 * (*iti0)[2] + q1 * (*iti1)[2] + q2 * (*iti2)[2];
	++ito;++iti0;++iti1;++iti2;
    }
   

    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Image", output);

    size_t written = write(vid_out, output.data, framesize);
    std::cout << written << std::endl;
    if (written < 0) {
        std::cerr << "ERROR: could not write to output device!\n";
        close(vid_out);
        break;
    }

    auto key = cv::waitKey(1);
    if (key == 32) break;
  }
}

int main(void) {
  assert(CHAR_BIT * sizeof (float) == 32);
  BokehCamera b;
  b.start();
}
