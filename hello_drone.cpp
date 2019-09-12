#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <vector>
#include <memory>
#include <thread> 

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#include <math.h>

enum DroneStates{stopping = 0,
                moving};
DroneStates mDroneState = stopping;

#define FIFO_NAME "/mnt/hdd/ORB_SLAM2/american_maid"

static int image_0_numerator = 0;
static int image_1_numerator = 0;
static int image_count_max = 150000;
static bool end_simulation = false;

template<typename ... Args>
string string_format(const string& format, Args ... args){
    size_t size = 1 + snprintf(nullptr, 0, format.c_str(), args ...);
    unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args ...);
    return string(buf.get(), buf.get() + size);
}

void rotateDroneByAngle(msr::airlib::MultirotorRpcLibClient& client, const float speed, const float angle) 
{
    auto position = client.getMultirotorState().getPosition();
    float z = position.z(); // current position (NED coordinate system).  
    DrivetrainType driveTrain = DrivetrainType::MaxDegreeOfFreedom;
    YawMode yaw_mode(true, 0);
    yaw_mode.yaw_or_rate = speed;
    const float total_angle_of_rotation = angle;
    const float total_angle_of_rotation_duration = total_angle_of_rotation / yaw_mode.yaw_or_rate;
    client.moveByVelocityZAsync(0, 0, z, total_angle_of_rotation_duration, driveTrain, yaw_mode);
    std::this_thread::sleep_for(std::chrono::duration<double>(total_angle_of_rotation_duration));
    yaw_mode.yaw_or_rate = 0.0f;
}

int getStereoAndDepthImages(msr::airlib::MultirotorRpcLibClient &client) 
{
    using namespace msr::airlib;

    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;
    
    while(image_count_max > 0) {
        if(end_simulation == true)
            break;
        
        image_count_max--;
        try {
            client.simPause(true);
            //get right, left and depth images. First two as png, second as float16.
            const vector<ImageRequest> request = { 
                //png format
                ImageRequest("0", ImageType::Scene),
                //uncompressed RGBA array bytes
                ImageRequest("1", ImageType::Scene),       
                //floating point uncompressed image  
                ImageRequest("1", ImageType::DepthPlanner, true) 
            };

            const vector<ImageResponse>& response = client.simGetImages(request);
            //do something with response which contains image data, pose, timestamp etc
            if (response.size() > 0) {
                //std::cout << "Image taken..." << std::endl; 
                std::string path_L = "/home/okanb/airsim/dataset/sequences/00/image_0";
                std::string path_R = "/home/okanb/airsim/dataset/sequences/00/image_1";
                // Removed user input for getting path            
                //std::getline(std::cin, path);

                const ImageResponse& image_info_left_cam = response[0];
                const ImageResponse& image_info_right_cam = response[1];

                const std::string file_path_L = FileSystem::combine(path_L, string_format("%06d.png", image_0_numerator++));              
                std::ofstream file_L(file_path_L.c_str(), std::ios::binary);
                file_L.write(reinterpret_cast<const char*>(image_info_left_cam.image_data_uint8.data()), image_info_left_cam.image_data_uint8.size());
                file_L.close();

                const std::string file_path_R = FileSystem::combine(path_R, string_format("%06d.png", image_1_numerator++));
                std::ofstream file_R(file_path_R.c_str(), std::ios::binary);
                file_R.write(reinterpret_cast<const char*>(image_info_right_cam.image_data_uint8.data()), image_info_right_cam.image_data_uint8.size());
                file_R.close();

                /*
                for (const ImageResponse& image_info : response) {            
                    std::cout << "Image uint8 size: " << image_info.image_data_uint8.size() << std::endl;
                    std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;

                    if (path != "") {
                        static int file_path_numerator = 0;
                        file_path_numerator++;
                        std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp)) + std::to_string(file_path_numerator);
                        if (image_info.pixels_as_float) {
                            Utils::writePfmFile(image_info.image_data_float.data(), image_info.width, image_info.height,
                                file_path + ".pfm");
                        }
                        else {
                            std::ofstream file(file_path + ".png", std::ios::binary);
                            file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
                            file.close();
                        }
                    }
                }
                 */
            }
            client.simPause(false);
        }
        catch (rpc::rpc_error&  e) {
            std::string msg = e.get_error().as<std::string>();
            std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
        }
        // Sleep this thread for 200 MilliSeconds
	std::this_thread::sleep_for(std::chrono::milliseconds(101));
    }
}

std::vector<float> pos_x;
std::vector<float> pos_y;
std::vector<float> pos_z;
float pos_x_cur = 0.0f;
float pos_y_cur = 0.0f;
float pos_z_cur = 0.0f;
    
size_t FindClosestPoint(float &x, float &y, float &z) {
    size_t size_x = pos_x.size();
    size_t size_y = pos_y.size();
    size_t size_z = pos_z.size();
    size_t smallest_size = 999999999;
    if(size_x < smallest_size) smallest_size = size_x;
    if(size_y < smallest_size) smallest_size = size_y;
    if(size_z < smallest_size) smallest_size = size_z;
    
    float cubic_dist_max = 999999999.0f;
    size_t saved_i = smallest_size - 1;
    //int break_ctr = 0;
    for(size_t i = smallest_size - 1; i > 0; i--) {
        float cubic_dist =  (pos_x[i] - x)*(pos_x[i] - x) + \
                            (pos_y[i] - y)*(pos_y[i] - y) + \
                            (pos_z[i] - z)*(pos_z[i] - z);
        if(cubic_dist < cubic_dist_max) {
            cubic_dist_max = cubic_dist;
            saved_i = i;
        }
        
        //break_ctr++;
        
        //if(break_ctr > 5)
        //    break;
    }
    
    return saved_i;
}

int main() 
{
    using namespace msr::airlib;

    msr::airlib::MultirotorRpcLibClient client;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;
    
    try {    
        static int fd = -999;
        static float x_ = 0.0f;
        static float y_ = 0.0f;
        static float z_ = 0.0f;
        
        if(fd == -999) {
            printf("waiting for writers...\n");
            fd = open(FIFO_NAME, O_RDONLY); // fd has a new number now
        }
        
        printf("Starting drone command...\n");
            
        client.confirmConnection();

        /*
        std::cout << "Press Enter to get FPV image" << std::endl; std::cin.get();
        vector<ImageRequest> request = { ImageRequest("0", ImageType::Scene), ImageRequest("1", ImageType::DepthPlanner, true) };
        const vector<ImageResponse>& response = client.simGetImages(request);
        std::cout << "# of images received: " << response.size() << std::endl;

        if (response.size() > 0) {
            std::cout << "Enter path with ending separator to save images (leave empty for no save)" << std::endl; 
            std::string path;
            std::getline(std::cin, path);

            for (const ImageResponse& image_info : response) {
                std::cout << "Image uint8 size: " << image_info.image_data_uint8.size() << std::endl;
                std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;

                if (path != "") {
                    std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
                    if (image_info.pixels_as_float) {
                        Utils::writePfmFile(image_info.image_data_float.data(), image_info.width, image_info.height,
                            file_path + ".pfm");
                    }
                    else {
                        std::ofstream file(file_path + ".png", std::ios::binary);
                        file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
                        file.close();
                    }
                }
            }
        }
        */
        
        //std::cout << "Press Enter to get stereo and depth images" << std::endl; std::cin.get();
        //getStereoAndDepthImages();
        
        //std::cout << "Press Enter to start image generation thread" << std::endl; std::cin.get();
        // Start thread t1 
        std::thread t1(getStereoAndDepthImages, std::ref(client)); 
	
	
	//std::cout << "Press Enter to arm the drone" << std::endl; std::cin.get();
        client.enableApiControl(true);
        client.armDisarm(true);

        //std::cout << "Press Enter to takeoff" << std::endl; std::cin.get();
        float takeoffTimeout = 5; 
        client.takeoffAsync(takeoffTimeout)->waitOnLastTask();

        // switch to explicit hover mode so that this is the fall back when 
        // move* commands are finished.
        //std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hoverAsync()->waitOnLastTask();

        //std::cout << "Press Enter to fly in a 10m box pattern at 3 m/s velocity" << std::endl; std::cin.get();
        // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        client.enableApiControl(true); 
        auto position = client.getMultirotorState().getPosition();
        float z = position.z(); // current position (NED coordinate system).  
        const float speed = 7.65f;
        const float size = 900.0f; 
        const float duration = size / speed;
        DrivetrainType driveTrain = DrivetrainType::ForwardOnly;
        YawMode yaw_mode(true, 0);
        
        std::cout << "Move to position..." << std::endl;        
        std::vector<Vector3r> drone_path;
        
        /*
        z = -200.0;
        drone_path.emplace_back(10.0f,  0.0, z);
        drone_path.emplace_back(100.0f,  0.0, z);
        drone_path.emplace_back(100.0f,  500.0, z);
        drone_path.emplace_back(100.0f,  500.0, z-300);
        drone_path.emplace_back(-400.0f,  500.0, z-300);
        drone_path.emplace_back(-400.0f,  -500.0, z-300);
        drone_path.emplace_back(100.0f,  -500.0, z-300);
        drone_path.emplace_back(100.0f,  0.0, z-300);
        drone_path.emplace_back(100.0f,  0.0, z);
        drone_path.emplace_back(10.0f,  0.0, z);
        drone_path.emplace_back(0.0f,  0.0, 2.0f);
        auto last_command = client.moveOnPathAsync(drone_path, 0.7)->waitOnLastTask();
         * */
        z = -200.0;
        float square = 150.0f;
        // SLAM mode
        std::cout << "MP: 1" << std::endl;
        drone_path.emplace_back(0.0f,  0.0, z);
        //client.moveOnPathAsync(drone_path, 0.7)->waitOnLastTask();
        std::cout << "MP: 2" << std::endl;
        drone_path.emplace_back(square,  0.0, z);  
        drone_path.emplace_back(square,  100.0, z); 
        //client.moveOnPathAsync(drone_path, 5.0)->waitOnLastTask();  
        client.moveOnPathAsync(drone_path, 5.0);
        
        static bool start_auto_landing = false;
        static bool keep_pos_vals = true;
        while(1) {            
            /*
               x:  294,891144
               y: -448,559967 
               z: -455,417542
             */
            /*
            if(fd == -999) {
                printf("waiting for writers...\n");
                fd = open(FIFO_NAME, O_RDONLY); // fd has a new number now
            }
             * */
            
            if(fd > 0) {                  
                char _vals_[200] = {0};
                //char y_val[200] = {0};
                //char z_val[200] = {0};
                size_t readbytes = read(fd, _vals_, 200);
                //read(fd, y_val, 200);
                //read(fd, z_val, 200);    
                
                for(int i = 0; i < readbytes + 5; i++) {
                    if(_vals_[i] == ',') {
                        _vals_[i] = '.';                        
                    }
                }
                //for(int i = 0; i < 200; i++) if(y_val[i] == ',') y_val[i] = '.';
                //for(int i = 0; i < 200; i++) if(z_val[i] == ',') z_val[i] = '.';
                //printf("{");printf("bytes: %d", readbytes);printf("}\n");
                static int push_back_ctr = 0;
                push_back_ctr++;
                push_back_ctr %= 5;
                for(int i = 0; i < readbytes + 5; i++) {
                    if(_vals_[i] == 'x') {
                        char *s = _vals_ + i + 2;
                        x_ = atof(s);
                        pos_x_cur = x_;
                        if(keep_pos_vals) {
                            if(push_back_ctr == 0)
                                pos_x.push_back(x_);
                            printf("x: %f\n", x_); 
                        }
                    }
                    else if(_vals_[i] == 'y') {
                        char *s = _vals_ + i + 2;
                        y_ = atof(s);
                        pos_y_cur = y_;
                        if(keep_pos_vals) {
                            if(push_back_ctr == 0)
                                pos_y.push_back(y_);
                            printf("y: %f\n", y_); 
                        }
                    }
                    else if(_vals_[i] == 'z') {
                        char *s = _vals_ + i + 2;
                        z_ = atof(s);
                        pos_z_cur = z_;
                        if(keep_pos_vals) {
                            if(push_back_ctr == 0)
                                pos_z.push_back(z_);
                            printf("z: %f\n", z_);   
                        }
                    }                       
                }
            } else {
                fd = -999; // If it wasnt opened successfully, reset fd.
            }
            
            //if(start_auto_landing)
            //    std::cout << "Auto landing is running!" << std::endl;
            
            if(!start_auto_landing && (x_ > 90.0f && y_ > 300.0f) && client.getMultirotorState().getPosition().z() < -100.0) {
                start_auto_landing = true;
            }
            
            static int thread_ctr = 0;
            //thread_ctr++;
            thread_ctr %= 4;
            if(start_auto_landing && thread_ctr == 0) {
                size_t cp = 0;
                std::cout << "Here are calculations: \n";
                std::cout << "pos_x size: " << pos_x.size() << std::endl;
                cp = FindClosestPoint(pos_x_cur, pos_y_cur, pos_z_cur);
                size_t which_point = cp - 5;
                std::cout << "Closest Point: " << cp << std::endl;
                std::cout << "Differences: dx: "    << pos_x_cur - pos_x[which_point] \
                          << "\ndy: "               << pos_y_cur - pos_y[which_point] \
                          << "\ndz: "               << pos_z_cur - pos_z[which_point] << std::endl;
                
                //client.moveByVelocityAsync(speed_vx, speed_vy, speed_vz, 2.0f, driveTrain, yaw_mode)->waitOnLastTask();     
                //client.moveByVelocityAsync(speed_vx, speed_vy, speed_vz, 2.0f, driveTrain, yaw_mode)->waitOnLastTask();  
                
                //drone_path.clear();
                //drone_path.emplace_back(pos_x[which_point],  pos_y[which_point], pos_z[which_point]);
                //client.moveOnPathAsync(drone_path, 5.0);
                switch(mDroneState) {
                    case stopping: {
                        client.cancelLastTask();
                        client.cancelLastTask();
                        client.moveByVelocityAsync(0.0f, 0.0f, 0.0f, 5.0f, driveTrain, yaw_mode);
                        float cur_x_val = client.getMultirotorState().getPosition().x();
                        float cur_y_val = client.getMultirotorState().getPosition().y();
                        static float old_x_val = cur_x_val;
                        static float old_y_val = cur_y_val;
                        static int pos_ctr = 0;
                        pos_ctr++;
                        if(pos_ctr > 10) {
                            pos_ctr = 0;
                            
                            if((abs(old_x_val - cur_x_val) < 1.0f) && (abs(old_y_val - cur_y_val) < 1.0f)) {
                                std::cout << "Drone is stopped!\n";
                                mDroneState = moving;
                                keep_pos_vals = false;
                            } else {
                                old_x_val = cur_x_val;
                                old_y_val = cur_y_val;
                            }
                        }
                    } break;
                    case moving: {
                        float speed_vx = (pos_x_cur - pos_x[which_point]) * 0.05f;
                        float speed_vy = -(pos_y_cur - pos_y[which_point]) * 0.05f;
                        float speed_vz = 0.0f;
                        client.cancelLastTask();
                        client.cancelLastTask();
                        client.moveByVelocityAsync(speed_vy, speed_vx, speed_vz, 500.0f, driveTrain, yaw_mode);
                    } break;
                    default: {
                        std::cout << "Drone is default movement problem!!!!\n";
                    } break;
                }
            }
            /*
            else if (!doonce && y_ < 0.0f) {
                client.cancelLastTask();
                client.cancelLastTask();
                
                std::cout << "Rock n roll! ended" << std::endl;
                return 0;    
            }
             * */
        }
        
        drone_path.emplace_back(square,  square, z);
        drone_path.emplace_back(-square,  square, z);
        drone_path.emplace_back(-square,  -square, z);
        drone_path.emplace_back(square,  -square, z);
        
        drone_path.emplace_back(square,  0.0, z);
        //client.moveOnPathAsync(drone_path, 1.1)->waitOnLastTask();
        std::cout << "MP: 3" << std::endl;
        drone_path.emplace_back(0.0f,  0.0, z);
        //client.moveOnPathAsync(drone_path, 1.1)->waitOnLastTask();
        std::cout << "MP: 4" << std::endl;
        drone_path.emplace_back(0.0f,  0.0, -2.0);
        
        client.moveOnPathAsync(drone_path, 11.0)->waitOnLastTask();     
        
        // Wait for t1 to finish 
        t1.join(); 
        
        //std::this_thread::sleep_for(std::chrono::duration<double>(duration*3));
        
        std::cout << "moveByVelocityZ(" << speed << ", 0, " << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(speed, 0, z, duration, driveTrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        
        rotateDroneByAngle(client, 4.0f, 90.0f);
        /*
        yaw_mode.yaw_or_rate = 20.0f;
        const float total_angle_of_rotation = 110.0f;
        const float total_angle_of_rotation_duration = total_angle_of_rotation / yaw_mode.yaw_or_rate;
        client.moveByVelocityZAsync(0, 0, z, total_angle_of_rotation_duration, driveTrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(total_angle_of_rotation_duration));
         * */
        //yaw_mode.yaw_or_rate = 0.0f;

        std::cout << "moveByVelocityZ(0, " << speed << "," << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(0, speed, z, duration, driveTrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        
        rotateDroneByAngle(client, 4.0f, 0.0f);
        
        std::cout << "moveByVelocityZ(" << -speed << ", 0, " << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(-speed, 0, z, duration, driveTrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        
        
        std::cout << "moveByVelocityZ(0, " << -speed << "," << z << "," << duration << ")" << std::endl;
        client.moveByVelocityZAsync(0, -speed, z, duration, driveTrain, yaw_mode);
        std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        client.hoverAsync()->waitOnLastTask();

        //std::cout << "Press Enter to land" << std::endl; std::cin.get();
        client.landAsync()->waitOnLastTask();

        //std::cout << "Press Enter to disarm" << std::endl; std::cin.get();
        client.armDisarm(false);

        end_simulation = true;
        
    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    return 0;
}
