#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <librealsense2/rs.hpp>

#define VID_WIDTH  1280
#define VID_HEIGHT 720

#include <iostream>

#define OUTPUT_MODE_BOKEH 0
#define OUTPUT_MODE_CX 1
#define OUTPUT_MODE_DEPTH 2

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
	int output_mode;
};

BokehCamera::BokehCamera() {
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    rs2_pipe.start(cfg);
    flength = 600.0f;
    dof = 300.0f;
    output_device = "/dev/video20";
    output_mode = OUTPUT_MODE_BOKEH;

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


    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;



  rs2::align align_to_color(RS2_STREAM_COLOR);
  rs2::hole_filling_filter hole_filter;
  rs2::spatial_filter spatial_filter;
  rs2::temporal_filter temporal_filter;

  rs2::disparity_transform depth_to_disparity(true);
  rs2::disparity_transform disparity_to_depth(false);

  hole_filter.set_option(RS2_OPTION_HOLES_FILL,1);

    cv::Mat img_color_small;
    cv::Mat img_color_small_blur_1;
    cv::Mat img_color_small_blur_2;
    cv::Mat img_color_blur_1;
    cv::Mat img_color_blur_2;

  for(;;) {
    // fetch synchronized depth and color frames from RealSense
    rs2::frameset data = rs2_pipe.wait_for_frames();
    data = align_to_color.process(data);

    rs2::depth_frame depth = data.get_depth_frame();
    rs2::video_frame color = data.get_color_frame();

    //depth = depth_to_disparity.process(depth);
    depth = spatial_filter.process(depth);
    depth = temporal_filter.process(depth);
    //depth = disparity_to_depth.process(depth);
    depth = hole_filter.process(depth);

    //std::cout << "received depth frame: " << depth.get_width() << "x" << depth.get_height() << std::endl;
    //std::cout << "received color frame: " << color.get_width() << "x" << color.get_height() << std::endl;

    int depth_w = depth.get_width();
    int depth_h = depth.get_height();
    int color_w = color.get_width();
    int color_h = color.get_height();

    // convert to OpenCV matrices
    cv::Mat img_depth(cv::Size(depth_w, depth_h), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat img_color(cv::Size(color_w, color_h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

    // compute blurred images for a small kernel and large kernel on scaled-down versions to reduce CPU usage
    // they will be scaled back up and interpolated to simulate other blur kernels in-between
    cv::resize(img_color, img_color_small, cv::Size(640, 360), 0, 0, cv::INTER_NEAREST);
    cv::blur(img_color_small, img_color_small_blur_2, cv::Size(10, 10));
    cv::blur(img_color_small, img_color_small_blur_1, cv::Size(5, 5));

    // scale back up after blurring
    cv::resize(img_color_small_blur_1, img_color_blur_1, cv::Size(color_w, color_h), 0, 0, cv::INTER_NEAREST);
    cv::resize(img_color_small_blur_2, img_color_blur_2, cv::Size(color_w, color_h), 0, 0, cv::INTER_NEAREST);

    // compute amount of blur at each pixel
    // TODO 5ms !!
    cv::Mat1f b = cv::abs(img_depth - flength);
    b /= dof;

    cv::Mat1f cx = 1.0f / (1.0f + b.mul(b));
    cx = cv::max(cv::min(cx, 1.0f), 0.0f);

    cv::blur(cx, cx, cv::Size(10, 10));

    CV_Assert(cx.depth() == CV_32F);

    auto t1 = high_resolution_clock::now();

    // compute coefficients and interpolate
    // TODO 24ms
    cv::Mat output(cx.rows, cx.cols, CV_8UC3);

    float q0, q1, q2;
    float* it;
    cv::Vec3b* iti0;
    cv::Vec3b* iti1;
    cv::Vec3b* iti2;
    cv::Vec3b* ito;

    // for(it = cx.begin(), end = cx.end(); it != end; ++it) {
    for(int r = 0; r < cx.rows; r++) {
	it = cx.ptr<float>(r);
	iti0 = img_color.ptr<cv::Vec3b>(r);
	iti1 = img_color_blur_1.ptr<cv::Vec3b>(r);
	iti2 = img_color_blur_2.ptr<cv::Vec3b>(r);
	ito = output.ptr<cv::Vec3b>(r);

        for(int c = 0; c < cx.cols; c++) {

	q0 = (it[c] > 0.5f) ? (it[c] - 0.5f) * 2.0f : 0.0f;
	q1 = ((it[c] > 0.5f) ? (1.0f - q0) : 0.0f ) + ((it[c] <= 0.5f) ? (it[c] * 2.0f) : 0.0f);
	q2 = (it[c] <= 0.5f) ? (1.0f - q1) : 0.0f;

	ito[c][0] = q0 * iti0[c][0] + q1 * iti1[c][0] + q2 * iti2[c][0];
	ito[c][1] = q0 * iti0[c][1] + q1 * iti1[c][1] + q2 * iti2[c][1];
	ito[c][2] = q0 * iti0[c][2] + q1 * iti1[c][2] + q2 * iti2[c][2];

	}
    }
   
    auto t2 = high_resolution_clock::now();
    auto ms_int = duration_cast<milliseconds>(t2 - t1);
    /* Getting number of milliseconds as a double. */
    duration<double, std::milli> ms_double = t2 - t1;
    std::cout << ms_int.count() << "ms\n";
    std::cout << ms_double.count() << "ms";


    if(output_mode == OUTPUT_MODE_BOKEH) {
	    std::cout << "BOKEH" << std::endl;
	    size_t written = write(vid_out, output.data, framesize);
	    std::cout << "bytes written: " << written << std::endl;
	    if (written < 0) {
	        std::cerr << "ERROR: could not write to output device!\n";
	        close(vid_out);
	        break;
	    }
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", output);
    } else if(output_mode == OUTPUT_MODE_CX) {
        std::cout << "CX" << std::endl;
	cv::Mat output(img_depth.rows, img_depth.cols, CV_8UC3);
        for(int r = 0; r < cx.rows; r++) {
	  float* it = cx.ptr<float>(r);
	  cv::Vec3b* ito = output.ptr<cv::Vec3b>(r);
          for(int c = 0; c < cx.cols; c++) {
            ito[c][0] = it[c]*255;
            ito[c][1] = it[c]*255;
            ito[c][2] = it[c]*255;
	  }
	}
	    size_t written = write(vid_out, output.data, framesize);
	    std::cout << "bytes written: " << written << std::endl;
	    if (written < 0) {
	        std::cerr << "ERROR: could not write to output device!\n";
	        close(vid_out);
	        break;
	    }
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", output);
    } else if(output_mode == OUTPUT_MODE_DEPTH) {
        std::cout << "DEPTH" << std::endl;
	cv::Mat output(img_depth.rows, img_depth.cols, CV_8UC3);
        for(int r = 0; r < img_depth.rows; r++) {
	  uint16_t* it = img_depth.ptr<uint16_t>(r);
	  cv::Vec3b* ito = output.ptr<cv::Vec3b>(r);
          for(int c = 0; c < img_depth.cols; c++) {
            ito[c][0] = it[c] >> 4;
            ito[c][1] = it[c] >> 4;
            ito[c][2] = it[c] >> 4;
	  }
	}
	    size_t written = write(vid_out, output.data, framesize);
	    std::cout << "bytes written: " << written << std::endl;
	    if (written < 0) {
	        std::cerr << "ERROR: could not write to output device!\n";
	        close(vid_out);
	        break;
	    }
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", output);
    }

    auto key = cv::waitKey(1);
    if (key == 32) { break; }
    else if (key == 49) { output_mode = OUTPUT_MODE_BOKEH; }
    else if (key == 50) { output_mode = OUTPUT_MODE_CX; }
    else if (key == 51) { output_mode = OUTPUT_MODE_DEPTH; }
    else { std::cout << "key press: " << key << std::endl; }
  }
}

int main(void) {
  assert(CHAR_BIT * sizeof (float) == 32);
  BokehCamera b;
  b.start();
}
