/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>

#ifdef _WIN32
   #include <io.h> 
   #define access    _access_s
#else
   #include <unistd.h>
#endif

#include<opencv2/core/core.hpp>

#include<System.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "../../include/Map.h"

#define FIFO_NAME "american_maid"

bool FileExists( const std::string &Filename )
{
    return access( Filename.c_str(), 0 ) == 0;
}

template<typename ... Args>
string string_format(const string& format, Args ... args){
    size_t size = 1 + snprintf(nullptr, 0, format.c_str(), args ...);
    unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args ...);
    return string(buf.get(), buf.get() + size);
}

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    argc = 4;
    argv[1] = "Vocabulary/ORBvoc.bin";
    argv[2] = "Examples/Stereo/KITTI04-12.yaml";
    //argv[3] = "/mnt/hdd/Downloads/gray/airsim_recs/dataset/sequences/00";
    argv[3] = "/home/okanb/airsim/dataset/sequences/00";
    
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    
    printf("waiting for readers...\n");
    if(ORB_SLAM2::Map::fd == -999) {
        ORB_SLAM2::Map::fd = open(FIFO_NAME, O_WRONLY); // fd has a new number now
    }

    // Retrieve paths to images
    //vector<string> vstrImageLeft;
    //vector<string> vstrImageRight;
    //vector<double> vTimestamps;
    //LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    //const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    //vector<float> vTimesTrack;
    //vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    //cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    //for(int ni=0; ni<nImages; ni++)
    while(1)
    {
        static int ni = 0;
        ++ni;
        // Read left and right images from file
        //imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_GRAYSCALE);
        //imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_GRAYSCALE);
        std::string img_no_left = std::string("/home/okanb/airsim/dataset/sequences/00/image_0/") + string_format("%06d.png", ni);
        std::string img_no_right = std::string("/home/okanb/airsim/dataset/sequences/00/image_1/") + string_format("%06d.png", ni);
        
        imLeft = cv::imread(img_no_left, CV_LOAD_IMAGE_GRAYSCALE);
        imRight = cv::imread(img_no_right, CV_LOAD_IMAGE_GRAYSCALE);
        //double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load left image at: "
                 << img_no_left << endl;
            if(FileExists(std::string("/home/okanb/airsim/dataset/sequences/00/image_0/") + string_format("%06d.png", ni + 1))) {
                
            } else {
                ni--; // TODO skip olayını ekleeee!!!
            }            
            usleep(100*1e3);
            continue;
        }
        else if(imRight.empty())
        {
            cerr << endl << "Failed to load right image at: "
                 << img_no_right << endl;
            if(FileExists(std::string("/home/okanb/airsim/dataset/sequences/00/image_1/") + string_format("%06d.png", ni + 1))) {
                
            } else {
                ni--; // TODO skip olayını ekleeee!!!
            }
            usleep(100*1e3);
            continue;
        }

        //std::cout << "Loaded images: " << string_format("%06d.png", ni) << " & " <<  string_format("%06d.png", ni) << std::endl;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight, 0.0d);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        //vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
//usleep(43000);
//break;

        /*
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];
        */
        
        /*
        if(ttrack<T) {
            usleep((T-ttrack)*1e6);
            std::cout << "usleep time: " << (T-ttrack)*1e6 << std::endl;
        }
         * */

        //std::cout << "T: " << T << std::endl;
        //std::cout << "vTimestamps[ni+1]-tframe: " << vTimestamps[ni+1]-tframe << std::endl;
        //std::cout << "tframe: " << tframe << std::endl;
        
        //usleep(43*1e3);
    }

    // Stop all threads
    SLAM.Shutdown();

    /*
    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
    */
    
    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    cout << "Trying to load files ..." << endl;
    
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            //cout << "Timestamp received: " << t << endl;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
    
    cout << "Files are loaded ..." << endl;
}
