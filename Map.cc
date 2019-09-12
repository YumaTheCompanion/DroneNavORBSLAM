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
#include <iostream>
#include "Map.h"

#include <mutex>
#include <cstdio>
#include <iostream>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#define FIFO_NAME "american_maid"

template<typename ... Args>
string string_format(const string& format, Args ... args){
    size_t size = 1 + snprintf(nullptr, 0, format.c_str(), args ...);
    unique_ptr<char[]> buf(new char[size]);
    snprintf(buf.get(), size, format.c_str(), args ...);
    return string(buf.get(), buf.get() + size);
}

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

int Map::fd = -999;

void Map::AddKeyFrame(KeyFrame *pKF)
{
    
    char s[300];
    int num;
    /*
    static int fd = -999;

    //mknod(FIFO_NAME, S_IFIFO | 0666, 0);

    printf("waiting for readers...\n");
    if(fd == -999) {
        fd = open(FIFO_NAME, O_WRONLY); // fd has a new number now
    }
     * */
    //printf("got a reader--type some stuff\n");
    ////////////////////////////// open this
    /*
    cv::Mat test(pKF->GetTranslation());
    printf("rows: %d, cols: %d\n", test.rows, test.cols);
    if(fd > 0 && test.rows >= 3 && test.rows >= 1) {        
        std::string result_x = string_format("x: %f\n", test.at<float>(0,0));
        std::string result_y = string_format("y: %f\n", test.at<float>(1,0));
        std::string result_z = string_format("z: %f\n", test.at<float>(2,0));
        //std::string result_t = string_format("t: %f\n", test.at<float>(3,0));
        
        write(fd, result_x.c_str(), result_x.length() + 1);
        write(fd, result_y.c_str(), result_y.length() + 1);
        write(fd, result_z.c_str(), result_z.length() + 1);
         
        std::string concat((result_x + result_y + result_z));
        write(fd, concat.c_str(), concat.length() + 1);
        //write(fd, result_t.c_str(), result_t.length() + 1);
    } else {
        fd = -999; // If it wasnt opened successfully, reset fd.
    }
    */
    /*
    static int i = 0;
    cv::FileStorage file("log_keyframes.yml", cv::FileStorage::APPEND);
    std::string str = string("KeyFrame_") + std::to_string(i);
    i++;
    file << str << pKF->GetTranslation();
    std::cout << "Saved Mat to log_keyframes.yml!" << std::endl;
     * */
//    file.close();
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    //std::cout << "AddMapPoint Called\n";
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

} //namespace ORB_SLAM
