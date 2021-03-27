/**
 * This is the Euroc stereo visual odometry program
 * Please specify the dataset directory in the config file
*/
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "ygz/System.h"
#include "ygz/EurocReader.h"
#include <sensor_msgs/image_encodings.h> 
#include <image_transport/image_transport.h>
#include "std_msgs/Int32MultiArray.h"
#include <cv_bridge/cv_bridge.h> 
#include <time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

using namespace std;
using namespace ygz;

class RosKCF
{
private:
    // ros::NodeHandle nodeHandle;
    // ros::Publisher targetPosePub;
    image_transport::Subscriber leftImg;
    image_transport::Subscriber rightImg;
    std::string _left_topic = "/cam0/image_raw";
    std::string _right_topic = "/cam1/image_raw";    
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    cv::Mat M1l, M2l, M1r, M2r;
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimeStamp;
    cv::Mat imLeft, imRight, imLeftRect, imRightRect;

public:
    RosKCF(const string &configFile) {
        cv::FileStorage fsSettings(configFile, cv::FileStorage::READ);
        if (fsSettings.isOpened() == false) {
            LOG(FATAL) << "Cannot load the config file from " << configFile << endl;
        }
        System system(configFile);
        // rectification parameters
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() ||
            D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0) {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return;
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l,
                                    M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r,
                                    M2r);
        // subscribe  
        // image_transport::ImageTransport it(nodeHandle);
        // this->leftImg = it.subscribe(_left_topic, 100, &RosKCF::leftCallback, this);
        // this->rightImg = it.subscribe(_right_topic, 100, &RosKCF::rightCallback, this);
        ros::NodeHandle nh;
        message_filters::Subscriber<sensor_msgs::Image> image_sub_L(nh, _left_topic, 10000);
        message_filters::Subscriber<sensor_msgs::Image> image_sub_R(nh, _right_topic, 10000);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), image_sub_L, image_sub_R);
        sync.registerCallback(boost::bind(&RosKCF::callback, this, _1, _2));

        ros::Rate loop_rate(20);
        while (ros::ok())
        {
            if (!imLeft.empty() && !imRight.empty()) {
                cv::remap(imLeft, imLeftRect, M1l, M2l, cv::INTER_LINEAR);
                cv::remap(imRight, imRightRect, M1r, M2r, cv::INTER_LINEAR);
                ros::Time begin = ros::Time::now();
                double timeStamp_ = begin.sec + begin.nsec/1e9;
                // std::cout << "Time: " << begin.sec << "-" << begin.nsec/1e9 << std::endl;
                // imshow("b", imRightRect);
                // imshow("a", imLeftRect);
                // cv::waitKey(3); 
                // printf("d1 = %.9lf\n", timeStamp_);
                // double timeStamp_ = clock();
                system.AddStereo(imLeftRect, imRightRect, timeStamp_);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void callback(const sensor_msgs::Image::ConstPtr &image_L, const sensor_msgs::Image::ConstPtr &image_R)
    {
        cv_bridge::CvImagePtr cv_ptrL, cv_ptrR;
        try
        {
            cv_ptrL = cv_bridge::toCvCopy(*image_L, sensor_msgs::image_encodings::MONO8);
            cv_ptrR = cv_bridge::toCvCopy(*image_R, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }    
        imLeft = cv_ptrL->image;
        imRight = cv_ptrR->image;  
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_kcf");
    ros::Time::init();
    if (argc != 2) {
        LOG(INFO) << "Usage: EurocStereo path_to_config" << endl;
        return 1;
    }
    FLAGS_logtostderr = false;
    google::InitGoogleLogging(argv[0]);
    string configFile(argv[1]);
    RosKCF rosKCF(configFile);

    return 0;

}