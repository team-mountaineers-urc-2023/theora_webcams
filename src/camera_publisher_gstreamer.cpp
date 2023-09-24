#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <libv4l2.h>
#include <vector>
#include <string>
#include <thread>
#include <iostream>
#include <future>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include "theora_webcams/Framerate.h"
#include "theora_webcams/Resolution.h"
#include "theora_webcams/Status.h"
#include "theora_webcams/VideoFormat.h"
#include "theora_webcams/VideoFormatList.h"
#include "theora_webcams/ChangeVideo.h"
#include "theora_webcams/GetResolutions.h"


class CameraNode {
public:
	std::string device;
	std::string name;
	cv::VideoCapture cam;
	theora_webcams::VideoFormat chosen_format;
	theora_webcams::Status status;
	int current_width, current_height, current_fps_num, current_fps_den;
	bool can_run;

	CameraNode(std::string path);
	int run();

	// service handlers
	bool change_video_handler(theora_webcams::ChangeVideo::Request& req, theora_webcams::ChangeVideo::Response& resp);
	bool get_resolutions_handler(theora_webcams::GetResolutions::Request& req, theora_webcams::GetResolutions::Response& resp);
	bool try_open_cam(int width, int height, int fps_num, int fps_den);
	std::string create_gstreamer_pipeline(uint32_t width, uint32_t height, uint32_t fps_num, uint32_t fps_den);
};

bool video_format_exists(const theora_webcams::VideoFormat& format, int width, int height, int fps_num, int fps_den);
theora_webcams::VideoFormatList get_video_formats(const std::string& device);
std::vector<theora_webcams::Resolution> get_resolutions(int fd, unsigned int fmt);
std::vector<theora_webcams::Framerate> get_framerates(int fd, unsigned int fmt, unsigned int width, unsigned int height);
std::vector<std::string> get_file_list(const std::string& path_str);

CameraNode::CameraNode(std::string path): device(path), cam() {
	// Let the name of this device be the /dev/urc/cam/"camera_name"
	this->name = this->device.substr(this->device.rfind('/') + 1);

	// First we get all the supported formats of this camera
	theora_webcams::VideoFormatList video_formats = get_video_formats(this->device);

	if (video_formats.formats.size() == 0) {
		ROS_ERROR("No video formats available for camera %s.", this->device.c_str());
		can_run = false;
		return;
	}

	size_t mjpg_index = video_formats.formats.size();
	size_t yuyv_index = video_formats.formats.size();

	// Then we look for the two formats we support
	for (size_t i = 0; i < video_formats.formats.size(); i++) {
		if (video_formats.formats[i].format == "MJPG") {
			mjpg_index = i;
		} else if (video_formats.formats[i].format == "YUYV") {
			yuyv_index = i;
		}
	}

	// Ideally we want to use MJPG, but we'll fall back to YUYV if its not there
	if (mjpg_index != video_formats.formats.size()) {
		this->chosen_format = video_formats.formats[mjpg_index];
	} else if (yuyv_index != video_formats.formats.size()) {
		this->chosen_format = video_formats.formats[yuyv_index];
	} else {
		this->chosen_format = video_formats.formats[0];
		ROS_WARN("Could not find MJPG or YUYV, selecting first format for camera %s. This may fail.", this->device.c_str());
	}

	ROS_INFO("Chosen format for %s: %s", this->device.c_str(), this->chosen_format.format.c_str());

	can_run = true;
}

int CameraNode::run() {
	if (!can_run) {
		return 1;
	}

	ros::NodeHandle nh("/cameras/" + this->name);
	image_transport::ImageTransport it(nh); // creating image transporter

	camera_info_manager::CameraInfoManager camera_info_man(nh, this->name);
	sensor_msgs::CameraInfo camera_info = camera_info_man.getCameraInfo();
	camera_info.header.frame_id = this->name;
	camera_info.header.stamp = ros::Time::now();

	image_transport::Publisher cam_pub = it.advertise("image_raw", 1);
	ros::Publisher info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, true);
	info_pub.publish(camera_info);

	ros::Publisher status_pub = nh.advertise<theora_webcams::Status>("status", 1, true);
	this->status.height = 0;
	this->status.width = 0;
	this->status.framerate_numerator = 0;
	this->status.framerate_denominator = 1;
	this->status.exposure = -1;
	status_pub.publish(this->status);

	ros::ServiceServer change_service = nh.advertiseService("change_video", &CameraNode::change_video_handler, this);
	ros::ServiceServer get_res_service = nh.advertiseService("get_resolutions", &CameraNode::get_resolutions_handler, this);

	cv::Mat frame;
	int error_count = 0;

	while (ros::ok() && error_count < 6) {
		// Die when we disconnect
		if (!boost::filesystem::exists(device)) {
			ROS_ERROR_THROTTLE(1, "Camera %s could not be found.", this->device.c_str());
			error_count += 1;
		}

		if (cam.isOpened()) {
			// wait for a new frame from camera and store it into 'frame'
			// this wont return until a new frame is available
			cam.read(frame);

			// Making sure there is a real frame
			if (!frame.empty()) {
				// Reset the error count
				error_count = 0;

				// Publishing image
				sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
				image->header.frame_id = this->name;
				image->header.stamp = ros::Time::now();
				cam_pub.publish(image);

				// Copy over the image header to camera info header and publish info
				camera_info.header = image->header;
				info_pub.publish(camera_info);
			} else {
				ROS_ERROR_THROTTLE(1, "Could not get image from %s.", this->device.c_str());
				error_count += 1;
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
			}
		} else {
			// Limit to 10Hz if we're not getting images
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		status_pub.publish(this->status);
		ros::spinOnce(); // getting message callback
	}

	this->status.height = 0;
	this->status.width = 0;
	this->status.framerate_numerator = 0;
	this->status.framerate_denominator = 1;
	this->status.exposure = -1;
	status_pub.publish(this->status);

	cam.release();

	if (error_count > 0) {
		ROS_ERROR("Camera node for %s finished with errors.", this->device.c_str());
	}

	return error_count;
}

bool CameraNode::change_video_handler(theora_webcams::ChangeVideo::Request& req, theora_webcams::ChangeVideo::Response& resp) {
	// If we want to stop, just return early
	if (!req.start) {
		this->cam.release();
		this->status.width = 0;
		this->status.height = 0;
		this->status.framerate_numerator = 0;
		this->status.framerate_denominator = 1;
		this->status.exposure = -1;
		resp.success = true;
		return true;
	}

	if (cam.isOpened()) {
		// check to see if we're trying to change to the resolution we're already at
		// don't check if force_restart is set
		if (
			!req.force_restart
			&& this->status.width == req.width
			&& this->status.height == req.height
			&& this->status.framerate_numerator == req.fps_num &&
			this->status.framerate_denominator == req.fps_den
		) {
			resp.success = true;
			return true;
		}

		// make sure to release before trying to open
		this->cam.release();
	}

	resp.success = try_open_cam(req.width, req.height, req.fps_num, req.fps_den);
	return true;
}

bool CameraNode::get_resolutions_handler(theora_webcams::GetResolutions::Request& req, theora_webcams::GetResolutions::Response& resp) {
	(void) req; // get rid of unused warning
	resp.resolutions = this->chosen_format.resolutions;
	return true;
}

bool CameraNode::try_open_cam(int width, int height, int fps_num, int fps_den) {
	// Lets verify the resolution we want works
	if (!video_format_exists(this->chosen_format, width, height, fps_num, fps_den)) {
		ROS_ERROR("Could not find support for resolution %dx%d @ %d/%dfps for camera %s.", width, height, fps_num, fps_den, this->device.c_str());
		return false;
	}

	// Finally we can create the video stream
	std::string pipeline_str = this->create_gstreamer_pipeline(width, height, fps_num, fps_den);

	ROS_INFO("GStreamer Pipeline: %s", pipeline_str.c_str());
	this->cam.open(pipeline_str);

	int tries = 0;
	while (!this->cam.isOpened()) {
		tries += 1;
		if (tries < 3 && ros::ok()) {
			ROS_ERROR("Could not open camera %s. Will try again.", this->device.c_str());
		} else {
			ROS_ERROR("Failed to open camera %s.", this->device.c_str());
			return false;
		}

		std::this_thread::sleep_for(std::chrono::seconds(1));
		this->cam.open(pipeline_str);
	}

	this->status.width = width;
	this->status.height = height;
	this->status.framerate_numerator = fps_num;
	this->status.framerate_denominator = fps_den;
	this->status.exposure = -1;
	return true;
}

std::string CameraNode::create_gstreamer_pipeline(uint32_t width, uint32_t height, uint32_t fps_num, uint32_t fps_den) {
	// We need to create a gstreamer pipeline to open the webcam and fetch a video stream.

	// Here we specify the camera path. v4l2src is Video 4 linux which is the driver for webcams.
	std::string pipeline_str = "v4l2src device=\"" + this->device + "\"";

	if (chosen_format.format == "MJPG") {
		// Specify we want jpegs
		pipeline_str += " ! image/jpeg";
	} else if (chosen_format.format == "YUYV") {
		// Specify we want raw YUYV 4:2:2
		pipeline_str += " ! video/x-raw,format=YUY2";
	} else {
		// Hope for the best
		pipeline_str += " ! video/x-raw";
	}

	// Set the width, height, and framerate
	pipeline_str += ",width=" + std::to_string(width)
		+ ",height=" + std::to_string(height)
		+ ",framerate=" + std::to_string(fps_num) + "/" + std::to_string(fps_den);

	if (chosen_format.format == "MJPG") {
		// Decode the jpeg frames into pixel data
		pipeline_str += " ! jpegdec";
	}

	// Convert the frames into BGR so opencv can parse it correctly
	// Tell gstreamer we are the sink for this data, and to drop any frames that we dont read in time
	pipeline_str += " ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1";

	return pipeline_str;
}

// utility functions
bool video_format_exists(const theora_webcams::VideoFormat& format, int width, int height, int fps_num, int fps_den) {
	// Lets verify the resolution we want works
	bool found_res = false;
	theora_webcams::Resolution chosen_res;

	for (const theora_webcams::Resolution& res : format.resolutions) {
		if (res.height == height && res.width == width) {
			found_res = true;
			chosen_res = res;
			break;
		}
	}

	if (!found_res) {
		return false;
	}

	// And now we find the fps in the resolution
	bool found_fps = false;
	theora_webcams::Framerate chosen_fps;

	for (const theora_webcams::Framerate& framerate : chosen_res.framerates) {
		if (framerate.numerator == fps_num && framerate.denominator == fps_den) {
			found_fps = true;
			break;
		}
	}

	return found_fps;
}

theora_webcams::VideoFormatList get_video_formats(const std::string& device) {
	theora_webcams::VideoFormatList formats;

	int fd = v4l2_open(device.c_str(), O_RDWR);

	int tries = 0;
	while (fd == -1) {
		tries += 1;
		if (tries < 3 && ros::ok()) {
			ROS_ERROR("Could not open camera %s. Will try again.", device.c_str());
		} else {
			ROS_ERROR("Failed to open camera %s.", device.c_str());
			return formats;
		}

		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	struct v4l2_fmtdesc fmtdesc;
	memset(&fmtdesc, 0, sizeof(fmtdesc));
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
		char format_str[5] = {
				(char) ((fmtdesc.pixelformat >> 0) & 0xff),
				(char) ((fmtdesc.pixelformat >> 8) & 0xff),
				(char) ((fmtdesc.pixelformat >> 16) & 0xff),
				(char) ((fmtdesc.pixelformat >> 24) & 0xff),
				0
		};

		theora_webcams::VideoFormat video_format;
		video_format.format = format_str;
		video_format.resolutions = get_resolutions(fd, fmtdesc.pixelformat);
		formats.formats.push_back(video_format);

		fmtdesc.index++;
	}

	v4l2_close(fd);

	return formats;
}

std::vector<theora_webcams::Resolution> get_resolutions(int fd, unsigned int fmt) {
	std::vector<theora_webcams::Resolution> resolutions;

	struct v4l2_frmsizeenum frmsize;
	memset(&frmsize, 0, sizeof(frmsize));
	frmsize.pixel_format = fmt;

	while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0) {
		if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
			theora_webcams::Resolution resolution;
			resolution.width = frmsize.discrete.width;
			resolution.height = frmsize.discrete.height;
			resolution.framerates = get_framerates(fd, fmt, resolution.width, resolution.height);

			resolutions.push_back(resolution);
		}

		frmsize.index++;
	}

	return resolutions;
}

std::vector<theora_webcams::Framerate> get_framerates(int fd, unsigned int fmt, unsigned int width, unsigned int height) {
	std::vector<theora_webcams::Framerate> framerates;

	struct v4l2_frmivalenum frame_interval;
	memset(&frame_interval, 0, sizeof(frame_interval));
	frame_interval.pixel_format = fmt;
	frame_interval.width = width;
	frame_interval.height = height;

	while (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frame_interval) == 0) {
		if (frame_interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
			theora_webcams::Framerate framerate;
			// The frame rate is the inverse of the frame interval
			framerate.numerator = frame_interval.discrete.denominator;
			framerate.denominator = frame_interval.discrete.numerator;

			framerates.push_back(framerate);
		}

		frame_interval.index += 1;
	}

	return framerates;
}

std::vector<std::string> get_file_list(const std::string& path_str) {
	namespace fs = boost::filesystem;

	std::vector<std::string> file_list;
	fs::path path(path_str);

	if (fs::exists(path) && fs::is_directory(path)) {
		for (fs::directory_entry& entry : boost::make_iterator_range(fs::directory_iterator(path), {})) {
			file_list.push_back(std::string(entry.path().string()));
		}
	}

	return file_list;
}

int launch_node(std::string device) {
	CameraNode node = CameraNode(device);
	if (node.can_run) {
		return node.run();
	} else {
		return 1;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "camera_publisher");
	ros::NodeHandle nh;

	std::map<std::string, std::future<int>> cam_nodes;

	while (ros::ok()) {
		// Remove any nodes from the map that finished or crashed
		for (auto it = cam_nodes.cbegin(); it != cam_nodes.cend(); /* no increment */) {
			auto status = it->second.wait_for(std::chrono::milliseconds(0));
			// Our thread died for some reason. Lets remove it from the map so we can remake it later
			if (status == std::future_status::ready) {
				ROS_INFO("Camera node for %s finished", it->first.c_str());
				it = cam_nodes.erase(it);
				ROS_INFO("Map size %ld", cam_nodes.size());
			} else {
				++it;
			}
		}

		// Add new nodes to the map that aren't already in it
		for (auto path : get_file_list("/dev/urc/cam/")) {
			if (cam_nodes.find(path) == cam_nodes.end()) {
				ROS_INFO("Launching camera node for %s", path.c_str());
				cam_nodes.emplace(path, std::move(std::async(&launch_node, path)));
				ROS_INFO("Map size %ld", cam_nodes.size());
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	for (auto& entry : cam_nodes) {
		entry.second.get();
	}

	return 0;
}
