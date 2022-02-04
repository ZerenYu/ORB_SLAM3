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

#include <condition_variable>

#include <opencv2/core/core.hpp>


#include <System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

void savePly(const string &path, const std::vector<cv::Mat> points);

double ttrack_tot = 0;
int main(int argc, char **argv) {

    std::cout << argc << std::endl;

    if (argc !=6) {
        cerr << endl
            //  << "Usage: ./rgbd_inertial_progresslabeler path_to_vocabulary path_to_settings path_to_rgbd path_to_imu_filename path_to_association_filename"
             << "Usage: ./rgbd_inertial_progresslabeler path_to_vocabulary path_to_settings path_to_rgbd path_to_association_filename"
             << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestampsRGBD;


    vector<cv::Point3f> vAcc;
    vector<cv::Point3f> vGyro;
    vector<double> vTimestampsIMU;
    string strAssociationFilename = string(argv[5]);
    string pathRgbd = string(argv[3]);

    // string pathImu = string(argv[4]);
    int first_imu = 0;
    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD, true, 0);
    float imageScale = SLAM.GetImageScale();

    double timestamp;
    cv::Mat im;
    cv::Mat depth;


    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestampsRGBD);
    // LoadIMU(pathImu, vTimestampsIMU, vAcc, vGyro);

    int proccIm = 0;
    int nImage = vstrImageFilenamesRGB.size();
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImage);

    for(int ni=1; ni<nImage; ni++, proccIm++)
    {
        // Read image from file
        im = cv::imread(pathRgbd + vstrImageFilenamesRGB[ni],cv::IMREAD_UNCHANGED); //CV_LOAD_IMAGE_UNCHANGED);
        depth = cv::imread(pathRgbd + vstrImageFilenamesD[ni],cv::IMREAD_UNCHANGED);

        double tframe = vTimestampsRGBD[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                    <<  vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

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

        // Load imu measurements from previous frame
        vImuMeas.clear();

        // if(ni>0)
        // {
        //     // cout << "t_cam " << tframe << endl;

        //     while((vTimestampsIMU[first_imu])<=vTimestampsRGBD[ni])
        //     {
        //         vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[first_imu].x,vAcc[first_imu].y,vAcc[first_imu].z,
        //                                                     vGyro[first_imu].x,vGyro[first_imu].y,vGyro[first_imu].z,
        //                                                     vTimestampsIMU[first_imu]));
        //         first_imu++;
        //     }
        // }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        cout << "tframe = " << tframe << endl;
        
        SLAM.TrackRGBD(im, depth, tframe); // TODO change to monocular_inertial

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        ttrack_tot += ttrack;
        // std::cout << "ttrack: " << ttrack << std::endl;

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImage-1)
            T = vTimestampsRGBD[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestampsRGBD[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6); // 1e6
    }

    cout << "Changing the dataset" << endl;

    SLAM.ChangeDataset();
    

    // im = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
    // depth = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);

    // for(int i=0; i<vGyro.size(); ++i)
    // {
    //     ORB_SLAM3::IMU::Point lastPoint(vAccel[i].x, vAccel[i].y, vAccel[i].z,
    //                                     vGyro[i].x, vGyro[i].y, vGyro[i].z,
    //                                     vGyro_times[i]);
    //     vImuMeas.push_back(lastPoint);
    // }

    // if(imageScale != 1.f)
    // {

    //     int width = im.cols * imageScale;
    //     int height = im.rows * imageScale;
    //     cv::resize(im, im, cv::Size(width, height));
    //     cv::resize(depth, depth, cv::Size(width, height));

    // // Pass the image to the SLAM system
    // SLAM.TrackRGBD(im, depth, timestamp, vImuMeas);


    // vImuMeas.clear();
    SLAM.Shutdown();
    
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    // Save camera trajectory
    cout << "System shutdown!\n";
}

void savePly(const string &path, const std::vector<cv::Mat> points) {
    ofstream f(path + "/fused.ply");
    f << "ply\n"
        << "format ascii 1.0\n"
        << "element vertex " << points.size() << "\n"
        << "property float x\n"
        << "property float y\n"
        << "property float z\n"
        << "end_header\n";
    for (auto p : points)
    {
        f << p.at<float>(0) << " " << p.at<float>(1) << " " << p.at<float>(2) << "\n";
    }
    f.close();
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
    
}


void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(' ')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos-1);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
