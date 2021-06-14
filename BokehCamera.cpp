#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <librealsense2/rs.hpp>
#include <iostream>

#define OUTPUT_MODE_BOKEH 0
#define OUTPUT_MODE_CX 1
#define OUTPUT_MODE_WHITEBOARD 2
#define OUTPUT_MODE_DEPTH 3
#define OUTPUT_MODE_RGB 4
#define OUTPUT_MODE_IR 5
#define DEBUG 0

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
        uint8_t output_mode;
	bool whiteboard_enabled;
        size_t vid_width;
        size_t vid_height;
        rs2::config cfg;
	rs2::pipeline_profile selection;
	rs2::device selected_device;
};

BokehCamera::BokehCamera() {
    vid_width = 1280;
    vid_height = 720;
    cfg.enable_stream(RS2_STREAM_DEPTH, vid_width, vid_height, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, vid_width, vid_height, RS2_FORMAT_RGB8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, vid_width, vid_height, RS2_FORMAT_Y8, 30);
    rs2::pipeline_profile selection = rs2_pipe.start(cfg);
    flength = 600.0f;
    dof = 300.0f;
    output_device = "/dev/video20";
    output_mode = OUTPUT_MODE_BOKEH;
    whiteboard_enabled = false;

    selected_device = selection.get_device();

    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);

    auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
    depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max/20);

    vid_out = open(output_device.c_str(), O_RDWR);
    if(vid_out < 0) {
        std::cerr << "ERROR: could not open output device!\n" <<
        strerror(errno);
        exit(1);
    }

    struct v4l2_format vid_format;
    memset(&vid_format, 0, sizeof(vid_format));
    vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

    if (ioctl(vid_out, VIDIOC_G_FMT, &vid_format) < 0) {
        std::cerr << "ERROR: unable to get video format!\n" <<
        strerror(errno);
        exit(2);
    }   

    // configure desired video format on device
    framesize = vid_width * vid_height * 3;
    vid_format.fmt.pix.width = vid_width;
    vid_format.fmt.pix.height = vid_height;
    vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    vid_format.fmt.pix.sizeimage = framesize;
    vid_format.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(vid_out, VIDIOC_S_FMT, &vid_format) < 0) {
        std::cerr << "ERROR: unable to set video format!\n" <<
        strerror(errno);
        exit(3);
    }
}

BokehCamera::~BokehCamera() {
    close(vid_out);
}

void BokehCamera::start() {
  rs2::align align_to_color(RS2_STREAM_COLOR);
  rs2::spatial_filter spatial_filter;
  rs2::temporal_filter temporal_filter;
  rs2::disparity_transform depth_to_disparity(true);
  rs2::disparity_transform disparity_to_depth(false);
  rs2::hole_filling_filter hole_filter;
  hole_filter.set_option(RS2_OPTION_HOLES_FILL,1);
  rs2::frameset data;

  // downscaled version of image for faster blur operation, it's blurry anyway
  cv::Mat img_color_small;

  // slightly blurred version of img_color_small
  cv::Mat img_color_small_blur_1;

  // massively blurred version of img_color_small
  cv::Mat img_color_small_blur_2;

  // upscaled version of img_color_small_blur_1
  cv::Mat img_color_blur_1;

  // upscaled version of img_color_small_blur_2
  cv::Mat img_color_blur_2;

  // the amount that each pixel wants to be blurred (1.0=no blur, 0.0=full blur)
  // because blurring is expensive, we interpolate between
  // img_color (blur_amount = 0.0), img_color_blur_1 (blur_amount = 0.5), and img_color_blur_2 (blur_amount = 1.0)
  // to fake other values of blur_amount
  cv::Mat1f blur_amount;

  // intermediate value needed to process blur_amount
  cv::Mat1f defocus_amount;

  cv::Mat img_whiteboard(cv::Size(vid_width, vid_height), CV_8UC3);

  int clear_whiteboard_counts = 0;
  uint64_t seq = 0;

  for(;;++seq) {

    // fetch synchronized depth and color frames from RealSense
    data = rs2_pipe.wait_for_frames();
    data = align_to_color.process(data);

    rs2::depth_frame depth = data.get_depth_frame();
    rs2::video_frame color = data.get_color_frame();
    rs2::video_frame ir1 = data.get_infrared_frame(1);

    // depth = depth_to_disparity.process(depth); // not much impact on images by omitting this
    depth = spatial_filter.process(depth);
    depth = temporal_filter.process(depth);
    // depth = disparity_to_depth.process(depth); // not much impact on images by omitting this
    depth = hole_filter.process(depth);

    if(DEBUG) {
        std::cout << "received depth frame: " << depth.get_width() << "x" << depth.get_height() << std::endl;
        std::cout << "received color frame: " << color.get_width() << "x" << color.get_height() << std::endl;
    }

    int depth_w = depth.get_width();
    int depth_h = depth.get_height();
    int color_w = color.get_width();
    int color_h = color.get_height();
    int ir1_w = ir1.get_width();
    int ir1_h = ir1.get_height();

    // convert to OpenCV matrices
    cv::Mat img_depth_o(cv::Size(depth_w, depth_h), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat img_color_o(cv::Size(color_w, color_h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat img_ir1_o(cv::Size(ir1_w, ir1_h), CV_8UC1, (void*)ir1.get_data(), cv::Mat::AUTO_STEP);

    cv::Mat img_depth, img_color, img_ir1;
    cv::flip(img_depth_o, img_depth, 1);
    cv::flip(img_color_o, img_color, 1);
    cv::flip(img_ir1_o, img_ir1, 1);

    if(DEBUG) {
        // checking to make sure the cv::Mat didn't do a memory copy from the rs2::frame, it shouldn't
        std::cout << "POINTERS" << std::endl;
        std::cout << "rs2 depth ptr = " << (uint64_t)(void*)depth.get_data() << std::endl;
        std::cout << "rs2 color ptr = " << (uint64_t)(void*)color.get_data() << std::endl;
        std::cout << "cv depth ptr = " << (uint64_t)(void*)img_depth.ptr<uchar>(0) << std::endl;
        std::cout << "cv color ptr = " << (uint64_t)(void*)img_color.ptr<uchar>(0) << std::endl;
    }

    // compute blurred images for a small kernel and large kernel on scaled-down versions to reduce CPU usage
    // they will be scaled back up and interpolated to simulate other blur kernels in-between
    cv::resize(img_color, img_color_small, cv::Size(640, 360), 0, 0, cv::INTER_NEAREST);
    cv::blur(img_color_small, img_color_small_blur_1, cv::Size(5, 5));
    cv::blur(img_color_small_blur_1, img_color_small_blur_2, cv::Size(10, 10));

    // scale back up after blurring
    cv::resize(img_color_small_blur_1, img_color_blur_1, cv::Size(color_w, color_h), 0, 0, cv::INTER_NEAREST);
    cv::resize(img_color_small_blur_2, img_color_blur_2, cv::Size(color_w, color_h), 0, 0, cv::INTER_NEAREST);

    // compute amount of blur at each pixel
    defocus_amount = cv::abs(img_depth - flength);
    defocus_amount /= dof;

    blur_amount = 1.0f / (1.0f + defocus_amount.mul(defocus_amount));
    blur_amount = cv::max(cv::min(blur_amount, 1.0f), 0.0f);

    cv::blur(blur_amount, blur_amount, cv::Size(10, 10));

    CV_Assert(blur_amount.depth() == CV_32F);

    // compute coefficients and interpolate
    cv::Mat output(blur_amount.rows, blur_amount.cols, CV_8UC3);

    float q0, q1, q2;
    float* it;
    uchar* iti0;
    uchar* iti1;
    uchar* iti2;
    uchar* ito;

    assert(blur_amount.isContinuous());
    assert(output.isContinuous());
    assert(img_color.isContinuous());
    assert(img_color_blur_1.isContinuous());
    assert(img_color_blur_2.isContinuous());

    it = blur_amount.ptr<float>(0);
    iti0 = img_color.ptr<uchar>(0);
    iti1 = img_color_blur_1.ptr<uchar>(0);
    iti2 = img_color_blur_2.ptr<uchar>(0);
    ito = output.ptr<uchar>(0);

    if(whiteboard_enabled) {
      for(int idx = 0; idx < img_ir1.rows; idx++) {
        for(int jdx = 0; jdx < img_ir1.cols; jdx++) {
          uint8_t* p = img_ir1.ptr<uint8_t>(idx);
           if(p[jdx] == 255) {
             float x = ((float)jdx - ir1_w/2) * 0.9 + ir1_w/2 + ir1_w / 20;
	     float y = (float)idx;
             img_whiteboard.at<cv::Vec3b>(cv::Point(int(x), int(y)))[1] = 191;
             img_whiteboard.at<cv::Vec3b>(cv::Point(int(x), int(y)))[2] = 255;
     	  }
        }
      }
    }

    for(int idx = 0; idx < blur_amount.rows*blur_amount.cols; idx++) {
        int idx3 = idx*3; // cache this value for minor speedup

        if(it[idx] > 0.5f) {
            // blur radius is low, interpolate between img_color and img_color_blur_1
            q0 = (it[idx] - 0.5f) * 2.0f;
            q1 = (1.0f - q0);
            ito[idx3+0] = q0 * iti0[idx3+0] + q1 * iti1[idx3+0];
            ito[idx3+1] = q0 * iti0[idx3+1] + q1 * iti1[idx3+1];
            ito[idx3+2] = q0 * iti0[idx3+2] + q1 * iti1[idx3+2];
        } else {
            // blur radius is high, interpolate between img_color_blur_1 and img_color_blur_2
            q1 = (it[idx] * 2.0f);
            q2 = (1.0f - q1);
            ito[idx3+0] = q1 * iti1[idx3+0] + q2 * iti2[idx3+0];
            ito[idx3+1] = q1 * iti1[idx3+1] + q2 * iti2[idx3+1];
            ito[idx3+2] = q1 * iti1[idx3+2] + q2 * iti2[idx3+2];
        }
    }

    if(whiteboard_enabled) {
	output = cv::max(img_color_blur_2, img_whiteboard);
	// 0.5 * img_color_blur_1 + 0.5 * img_whiteboard;
    }
   
    if(output_mode == OUTPUT_MODE_BOKEH) {
        if(DEBUG) std::cout << "BOKEH" << std::endl;
        int written = write(vid_out, output.data, framesize);
        if(DEBUG) std::cout << "bytes written: " << written << std::endl;
        if (written < 0) {
            std::cerr << "ERROR: could not write to output device!\n";
            close(vid_out);
            break;
        }
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", output);
    } else if(output_mode == OUTPUT_MODE_CX) {
        if(DEBUG) std::cout << "CX" << std::endl;
        cv::Mat output(img_depth.rows, img_depth.cols, CV_8UC3);
        for(int r = 0; r < blur_amount.rows; r++) {
            float* it = blur_amount.ptr<float>(r);
            cv::Vec3b* ito = output.ptr<cv::Vec3b>(r);
            for(int c = 0; c < blur_amount.cols; c++) {
                ito[c][0] = it[c]*255;
                ito[c][1] = it[c]*255;
                ito[c][2] = it[c]*255;
            }
        }
        int written = write(vid_out, output.data, framesize);
        if(DEBUG) std::cout << "bytes written: " << written << std::endl;
        if (written < 0) {
            std::cerr << "ERROR: could not write to output device!\n";
            close(vid_out);
            break;
        }
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", output);
    } else if(output_mode == OUTPUT_MODE_WHITEBOARD) {
        if(DEBUG) std::cout << "WHTIEBOARD" << std::endl;
        int written = write(vid_out, output.data, framesize); // TODO FIX THIS
        if(DEBUG) std::cout << "bytes written: " << written << std::endl;
        if (written < 0) {
            std::cerr << "ERROR: could not write to output device!\n";
            close(vid_out);
            break;
        }
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", img_whiteboard);
    } else if(output_mode == OUTPUT_MODE_DEPTH) {
        if(DEBUG) std::cout << "DEPTH" << std::endl;
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
        int written = write(vid_out, output.data, framesize);
        if(DEBUG) std::cout << "bytes written: " << written << std::endl;
        if (written < 0) {
            std::cerr << "ERROR: could not write to output device!\n";
            close(vid_out);
            break;
        }
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", output);
    } else if(output_mode == OUTPUT_MODE_RGB) {
        int written = write(vid_out, img_color.data, framesize);
        if(DEBUG) std::cout << "bytes written: " << written << std::endl;
        if (written < 0) {
            std::cerr << "ERROR: could not write to output device!\n";
            close(vid_out);
            break;
        }
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", img_color);
    } else if(output_mode == OUTPUT_MODE_IR) {
        int written = write(vid_out, img_color.data, framesize); // TODO FIX THIS
        if(DEBUG) std::cout << "bytes written: " << written << std::endl;
        if (written < 0) {
            std::cerr << "ERROR: could not write to output device!\n";
            close(vid_out);
            break;
        }
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", img_ir1);
    }

    auto key = cv::waitKey(1);
    if (key == 32) { break; }
    else if (key == 49) { output_mode = OUTPUT_MODE_BOKEH; }
    else if (key == 50) { output_mode = OUTPUT_MODE_CX; }
    else if (key == 51) { output_mode = OUTPUT_MODE_WHITEBOARD; }
    else if (key == 52) { output_mode = OUTPUT_MODE_DEPTH; }
    else if (key == 53) { output_mode = OUTPUT_MODE_RGB; }
    else if (key == 54) { output_mode = OUTPUT_MODE_IR; }
    else if (key == 91) { flength += 40; if(DEBUG) std::cout << "flength = " << flength << std::endl; }
    else if (key == 93) { flength -= 40; if(DEBUG) std::cout << "flength = " << flength << std::endl; }
    else if (key == 96) {
	whiteboard_enabled = !whiteboard_enabled;
        auto depth_sensor = selected_device.first<rs2::depth_sensor>();
        if(whiteboard_enabled) {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
	    clear_whiteboard_counts = 10;
	} else {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
	}
    }
    else if (key != -1) { std::cout << "key press: " << key << std::endl; }

    if(clear_whiteboard_counts > 0) { img_whiteboard *= 0.0; clear_whiteboard_counts--; }
  }
}

int main(void) {
  assert(CHAR_BIT * sizeof (float) == 32);
  BokehCamera b;
  b.start();
}
