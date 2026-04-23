/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <unistd.h>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"


#include <System.h>

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;

}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
std::string buildRuntimeSettingsFile(const std::string &settings_file,
                                     const rs2_intrinsics &intrinsics,
                                     const float depth_scale);
float get_depth_scale(const rs2::device &device);
void set_sensor_option_if_supported(rs2::sensor &sensor, rs2_option option, float value);

static rs2_option get_sensor_option(const rs2::sensor& sensor)
{
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n" << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        //SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;

        // To control an option, use the following api:

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << std::endl;

            // Get a human readable description of the option
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            //To change the value of an option, please follow the change_sensor_option() function
        }
        else
        {
            std::cout << " is not supported" << std::endl;
        }
    }

    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

int main(int argc, char **argv) {

    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: " << argv[0] << " path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return 0;
    }
    else
    {
        selected_device = devices[0];
        for (rs2::device device : devices)
        {
            const string device_name = device.supports(RS2_CAMERA_INFO_NAME) ?
                                       device.get_info(RS2_CAMERA_INFO_NAME) : "";
            if(device_name.find("D455") != string::npos)
            {
                selected_device = device;
                break;
            }
        }

        const string selected_name = selected_device.supports(RS2_CAMERA_INFO_NAME) ?
                                     selected_device.get_info(RS2_CAMERA_INFO_NAME) : "Unknown RealSense";
        const string selected_serial = selected_device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) ?
                                       selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : "unknown";
        std::cout << "Selected RealSense device: " << selected_name
                  << " (serial: " << selected_serial << ")" << std::endl;
    }

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    int index = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors)
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
            ++index;
            if (index == 1) {
                set_sensor_option_if_supported(sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                set_sensor_option_if_supported(sensor, RS2_OPTION_AUTO_EXPOSURE_LIMIT, 50000);
                set_sensor_option_if_supported(sensor, RS2_OPTION_EMITTER_ENABLED, 1); // emitter on for depth information
            }
            // std::cout << "  " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
            get_sensor_option(sensor);
            if (index == 2){
                // RGB camera
                set_sensor_option_if_supported(sensor, RS2_OPTION_EXPOSURE, 80.f);

            }

        }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // RGB stream
    cfg.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_RGB8, 30);

    // Depth stream
    // cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH,640, 480, RS2_FORMAT_Z16, 30);

    int width_img, height_img;

    // start and stop just to get necessary profile
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);
    pipe.stop();

    // Align depth and RGB frames
    //Pipeline could choose a device that does not have a color stream
    //If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(pipe_profile.get_streams());

    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    //The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);
    pipe_profile = pipe.start(cfg);

    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);




    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();
    width_img = intrinsics_cam.width;
    height_img = intrinsics_cam.height;
    std::cout << " fx = " << intrinsics_cam.fx << std::endl;
    std::cout << " fy = " << intrinsics_cam.fy << std::endl;
    std::cout << " cx = " << intrinsics_cam.ppx << std::endl;
    std::cout << " cy = " << intrinsics_cam.ppy << std::endl;
    std::cout << " height = " << intrinsics_cam.height << std::endl;
    std::cout << " width = " << intrinsics_cam.width << std::endl;
    std::cout << " Coeff = " << intrinsics_cam.coeffs[0] << ", " << intrinsics_cam.coeffs[1] << ", " <<
    intrinsics_cam.coeffs[2] << ", " << intrinsics_cam.coeffs[3] << ", " << intrinsics_cam.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_cam.model << std::endl;

    const float depth_scale = get_depth_scale(pipe_profile.get_device());
    std::cout << " depth scale = " << depth_scale << std::endl;
    const string settings_file = buildRuntimeSettingsFile(argv[2], intrinsics_cam, depth_scale);
    std::cout << " Using settings file = " << settings_file << std::endl;


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],settings_file,ORB_SLAM3::System::RGBD, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    double timestamp;
    cv::Mat im, depth;

#ifdef REGISTER_TIMES
    double t_resize = 0.f;
    double t_track = 0.f;
#endif
    rs2::frameset fs;

    while (!SLAM.isShutDown() && b_continue_session)
    {
        fs = pipe.wait_for_frames();
        timestamp = fs.get_timestamp()*1e-3;

        if (profile_changed(pipe.get_active_profile().get_streams(), pipe_profile.get_streams()))
        {
            // If the profile was changed, update the align object.
            pipe_profile = pipe.get_active_profile();
            align_to = find_stream_to_align(pipe_profile.get_streams());
            align = rs2::align(align_to);
        }

        // Perform alignment here
        auto processed = align.process(fs);

        // Trying to get both other and aligned depth frames
        rs2::video_frame color_frame = processed.first(align_to);
        rs2::depth_frame depth_frame = processed.get_depth_frame();
        if (!depth_frame || !color_frame)
        {
            cout << "Not synchronized depth and image\n";
            continue;
        }

        im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
        depth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);

        /*cv::Mat depthCV_8U;
        depthCV.convertTo(depthCV_8U,CV_8U,0.01);
        cv::imshow("depth image", depthCV_8U);*/

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(depth, depth, cv::Size(width, height));

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_Start_Track = std::chrono::monotonic_clock::now();
    #endif
#endif
        // Pass the image to the SLAM system
        SLAM.TrackRGBD(im, depth, timestamp);

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_End_Track = std::chrono::monotonic_clock::now();
    #endif
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Track - t_Start_Track).count();
        SLAM.InsertTrackTime(t_track);
#endif
    }
    cout << "System shutdown!\n";
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}


bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
        if (itr == std::end(current)) //If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}

float get_depth_scale(const rs2::device &device)
{
    for (rs2::sensor sensor : device.query_sensors())
    {
        if (rs2::depth_sensor depth_sensor = sensor.as<rs2::depth_sensor>())
            return depth_sensor.get_depth_scale();
    }

    return 0.001f;
}

void set_sensor_option_if_supported(rs2::sensor &sensor, rs2_option option, float value)
{
    try
    {
        if(sensor.supports(option))
            sensor.set_option(option, value);
    }
    catch(const rs2::error &e)
    {
        std::cerr << "Could not set " << option << ": " << e.what() << std::endl;
    }
}

static bool starts_with_key(const std::string &line, const std::string &key)
{
    return line.compare(0, key.size(), key) == 0;
}

static void write_setting_line(std::ostream &out, const std::string &key, const double value)
{
    out << key << " " << std::fixed << std::setprecision(9) << value << std::endl;
}

std::string buildRuntimeSettingsFile(const std::string &settings_file,
                                     const rs2_intrinsics &intrinsics,
                                     const float depth_scale)
{
    std::ifstream input(settings_file);
    if(!input.is_open())
    {
        std::cerr << "Could not open settings file for runtime calibration: " << settings_file << std::endl;
        return settings_file;
    }

    const std::string runtime_file = "/tmp/ngd_slam_realsense_" + std::to_string(getpid()) + ".yaml";
    std::ofstream output(runtime_file);
    if(!output.is_open())
    {
        std::cerr << "Could not create runtime settings file: " << runtime_file << std::endl;
        return settings_file;
    }

    const double depth_factor = depth_scale > 0.f ? 1.0 / static_cast<double>(depth_scale) : 1000.0;
    std::string line;
    while(std::getline(input, line))
    {
        if(starts_with_key(line, "Camera1.fx:"))
            write_setting_line(output, "Camera1.fx:", intrinsics.fx);
        else if(starts_with_key(line, "Camera1.fy:"))
            write_setting_line(output, "Camera1.fy:", intrinsics.fy);
        else if(starts_with_key(line, "Camera1.cx:"))
            write_setting_line(output, "Camera1.cx:", intrinsics.ppx);
        else if(starts_with_key(line, "Camera1.cy:"))
            write_setting_line(output, "Camera1.cy:", intrinsics.ppy);
        else if(starts_with_key(line, "Camera.width:"))
            output << "Camera.width: " << intrinsics.width << std::endl;
        else if(starts_with_key(line, "Camera.height:"))
            output << "Camera.height: " << intrinsics.height << std::endl;
        else if(starts_with_key(line, "RGBD.DepthMapFactor:"))
            write_setting_line(output, "RGBD.DepthMapFactor:", depth_factor);
        else
            output << line << std::endl;
    }

    return runtime_file;
}
