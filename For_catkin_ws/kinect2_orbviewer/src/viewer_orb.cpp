/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

#include "System.h"
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <depthimage_to_laserscan/DepthImageToLaserScan.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "SerialPort.h"
#include <iomanip>
#include <ctime>
//may not use
#include <Eigen/Core>
#include <Eigen/Geometry>
//for grid map
#include "geometry_msgs/PoseArray.h"
#include <Converter.h>

//for convenience, we use global variables
int all_pts_pub_gap = 0;
bool pub_all_pts = false;
int pub_count = 0;

class Receiver
{
public:
  enum Mode
  {
    IMAGE = 0,
    CLOUD,
    BOTH,
    ORB,
    ODOM
  };

private:
  std::mutex lock;

  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;

  bool updateImage, updateCloud,updateOrb;
  bool save,bRecord;//if we record images
  bool running;
  size_t frame;
  const size_t queueSize;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  std::vector<int> params;

	//zzh variables
	ORB_SLAM2::System* orbslam=nullptr;
	std::string camera_ns;
	ros::Time img_time;
	sensor_msgs::Image::ConstPtr imgdepthptr_ros;
	sensor_msgs::CameraInfo::ConstPtr caminfoptr_ros;
	CSerialPort sport_imu,sport_enc;
	volatile int last_cmd_time;
	
	std::mutex lock_cmd;
	bool mb_speed_unlock;
	bool mbContinuous;
	
	ros::Publisher pub_pts_and_pose,pub_all_kf_and_pts;

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed,const int useOrbviewer,const std::string& ns)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),camera_ns(ns),
      updateImage(false), updateCloud(false), updateOrb(false),
      save(false), running(false), frame(0), queueSize(5),mb_speed_unlock(false),
      nh("~"), spinner(0), it(nh), mode(CLOUD),sport_imu(nh),sport_enc(nh),
      last_cmd_time(-1),mbContinuous(false)
  {
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);

	string orbVocFile=gstrORBFile+"/catkin_ws/src/kinect2_orbviewer/src/ORBvoc.bin";
	string orbSettingsFile=gstrORBFile+"/catkin_ws/src/kinect2_orbviewer/src/kinect2_qhd.yaml";
	orbslam=new ORB_SLAM2::System(orbVocFile,orbSettingsFile,ORB_SLAM2::System::RGBD,useOrbviewer==1);
	if (useOrbviewer==4){
	  bRecord=false;mbContinuous=true;
	  pub_pts_and_pose = nh.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
	  pub_all_kf_and_pts = nh.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
	}else if (useOrbviewer==3){ bRecord=true;mbContinuous=true;}
	else if (useOrbviewer==2) bRecord=true;
	else bRecord=false;
  }

  ~Receiver()
  {
	if (orbslam){
		orbslam->Shutdown();
		delete orbslam;
	}
  }

  void run(const Mode mode)
  {
    start(mode);
    stop();
  }

private:
  void start(const Mode mode)
  {
    this->mode = mode;
    running = true;

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

    if(useExact)
    {
      syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }
    else
    {
      syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }

    spinner.start();

    std::chrono::milliseconds duration(1);
    while(!updateImage || !updateCloud||!updateOrb)
    {
      if(!ros::ok())
      {
        return;
      }
      std::this_thread::sleep_for(duration);
    }
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(this->color.cols, this->color.rows);

    switch(mode)
    {
    case CLOUD:
      cloudViewer();
      break;
    case IMAGE:
      imageViewer();
      break;
    case BOTH:
      imageViewerThread = std::thread(&Receiver::imageViewer, this);
      cloudViewer();
      break;
    case ORB:case ODOM:
      orbViewer(); 
      break;
    }
  }

  void stop()
  {
    spinner.stop();

    if(useExact)
    {
      delete syncExact;
    }
    else
    {
      delete syncApproximate;
    }

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;

    running = false;
    if(mode == BOTH)
    {
      imageViewerThread.join();
    }
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
    cv::Mat color, depth;

    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    readImage(imageColor, color);
    readImage(imageDepth, depth);
    img_time=imageDepth->header.stamp;
//	  cout<<fixed<<redSTR<<setprecision(6)<<img_time.toSec()<<" "<<ros::Time::now().toSec()<<whiteSTR<<endl;
    caminfoptr_ros=cameraInfoDepth;
    imgdepthptr_ros=imageDepth;

    // IR image input
    if(color.type() == CV_16U)
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = color;
    this->depth = depth;
    updateImage = true;
    updateCloud = true;
    updateOrb=true;
    lock.unlock();
  }
  
  void baseContrlCallback(const geometry_msgs::TwistConstPtr& msg){
    //ROS_INFO("I heard: [%d %d]",msg->linear.x,msg->angular.z);
    //cout<<msg->linear.x<<" "<<msg->angular.z<<endl;
    using std::sprintf;
    int x1=msg->linear.x*1200;//advance speed
    int x2=msg->angular.z*600;//left rotational speed
    last_cmd_time=time(NULL);
    sport_enc.GetBytesInCOM(2);//clear the output error
    lock_cmd.lock();
    gdCmdVel[0]=x1+x2,gdCmdVel[1]=x1-x2;//y1=x1+x2,y2=x1-x2
    mb_speed_unlock=true;
    gbCmdChanged=true;
    lock_cmd.unlock();
  }
  
  void publish(ORB_SLAM2::System &SLAM, ros::Publisher &pub_pts_and_pose,
	ros::Publisher &pub_all_kf_and_pts, int frame_id){
    if (all_pts_pub_gap > 0 && pub_count >= all_pts_pub_gap) {
	    pub_all_pts = true;
	    pub_count = 0;
    }
    if (pub_all_pts || SLAM.GetLoopDetected()) {
	    pub_all_pts = false;SLAM.SetLoopDetected(false);
	    geometry_msgs::PoseArray kf_pt_array;
	    vector<ORB_SLAM2::KeyFrame*> key_frames = SLAM.GetAllKeyFrames();
	    //! placeholder for number of keyframes
	    kf_pt_array.poses.push_back(geometry_msgs::Pose());
// 	    std::sort(key_frames.begin(), key_frames.end(), ORB_SLAM2::KeyFrame::lId);
	    unsigned int n_kf = 0;
	    unsigned int n_pts_id = 0;
	    for (auto key_frame : key_frames) {
		    // pKF->SetPose(pKF->GetPose()*Two);

		    if (!key_frame || key_frame->isBad()) {
			    continue;
		    }

		    cv::Mat R = key_frame->GetRotation().t();
		    vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
		    cv::Mat twc = key_frame->GetCameraCenter();
		    geometry_msgs::Pose kf_pose;

		    kf_pose.position.x = twc.at<float>(0);
		    kf_pose.position.y = twc.at<float>(1);
		    kf_pose.position.z = twc.at<float>(2);
		    kf_pose.orientation.x = q[0];
		    kf_pose.orientation.y = q[1];
		    kf_pose.orientation.z = q[2];
		    kf_pose.orientation.w = q[3];
		    kf_pt_array.poses.push_back(kf_pose);

		    n_pts_id = kf_pt_array.poses.size();
		    //! placeholder for number of points
		    kf_pt_array.poses.push_back(geometry_msgs::Pose());
		    std::set<ORB_SLAM2::MapPoint*> map_points = key_frame->GetMapPoints();
		    unsigned int n_pts = 0;
		    for (auto map_pt : map_points) {
			    if (!map_pt || map_pt->isBad()) {
				    //printf("Point %d is bad\n", pt_id);
				    continue;
			    }
			    cv::Mat pt_pose = map_pt->GetWorldPos();
			    if (pt_pose.empty()) {
				    //printf("World position for point %d is empty\n", pt_id);
				    continue;
			    }
			    geometry_msgs::Pose curr_pt;
			    //printf("wp size: %d, %d\n", wp.rows, wp.cols);
			    //pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
			    curr_pt.position.x = pt_pose.at<float>(0);
			    curr_pt.position.y = pt_pose.at<float>(1);
			    curr_pt.position.z = pt_pose.at<float>(2);
			    kf_pt_array.poses.push_back(curr_pt);
			    ++n_pts;
		    }
		    kf_pt_array.poses[n_pts_id].position.x = (double)n_pts;
		    kf_pt_array.poses[n_pts_id].position.y = (double)n_pts;
		    kf_pt_array.poses[n_pts_id].position.z = (double)n_pts;
		    ++n_kf;
	    }
	    kf_pt_array.poses[0].position.x = (double)n_kf;
	    kf_pt_array.poses[0].position.y = (double)n_kf;
	    kf_pt_array.poses[0].position.z = (double)n_kf;
	    kf_pt_array.header.frame_id = "1";
	    kf_pt_array.header.seq = frame_id + 1;
	    std::printf("Publishing data for %u keyfranmes\n", n_kf);
	    pub_all_kf_and_pts.publish(kf_pt_array);
    }
    else if (SLAM.GetKeyFrameCreated()) {
	    ++pub_count;
	    SLAM.SetKeyFrameCreated(false);

	    cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

	    // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
	    //while (pKF->isBad())
	    //{
	    //	Trw = Trw*pKF->mTcp;
	    //	pKF = pKF->GetParent();
	    //}

  // 		vector<ORB_SLAM2::KeyFrame*> vpKFs = SLAM.GetAllKeyFrames();
  // 		sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

	    // Transform all keyframes so that the first keyframe is at the origin.
	    // After a loop closure the first keyframe might not be at the origin. but it this is rectified, we should rectify Maps' position as well!
  // 		cv::Mat Two = vpKFs[0]->GetPoseInverse();

  // 		Trw = Trw*pKF->GetPose()*Two;
  // 		cv::Mat lit = SLAM.getTracker()->mlRelativeFramePoses.back();
  // 		cv::Mat Tcw = lit*Trw;
	    cv::Mat Tcw=SLAM.GetKeyFramePose();
	    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
	    cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

	    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
	    //geometry_msgs::Pose camera_pose;
	    //std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.getMap()->GetAllMapPoints();
	    std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
	    int n_map_pts = map_points.size();

	    //printf("n_map_pts: %d\n", n_map_pts);

	    //pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	    geometry_msgs::PoseArray pt_array;
	    //pt_array.poses.resize(n_map_pts + 1);

	    geometry_msgs::Pose camera_pose;

	    camera_pose.position.x = twc.at<float>(0);
	    camera_pose.position.y = twc.at<float>(1);
	    camera_pose.position.z = twc.at<float>(2);

	    camera_pose.orientation.x = q[0];
	    camera_pose.orientation.y = q[1];
	    camera_pose.orientation.z = q[2];
	    camera_pose.orientation.w = q[3];

	    pt_array.poses.push_back(camera_pose);

	    //printf("Done getting camera pose\n");

	    for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){

		    if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
			    //printf("Point %d is bad\n", pt_id);
			    continue;
		    }
		    cv::Mat wp = map_points[pt_id - 1]->GetWorldPos();

		    if (wp.empty()) {
			    //printf("World position for point %d is empty\n", pt_id);
			    continue;
		    }
		    geometry_msgs::Pose curr_pt;
		    //printf("wp size: %d, %d\n", wp.rows, wp.cols);
		    //pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
		    curr_pt.position.x = wp.at<float>(0);
		    curr_pt.position.y = wp.at<float>(1);
		    curr_pt.position.z = wp.at<float>(2);
		    pt_array.poses.push_back(curr_pt);
		    //printf("Done getting map point %d\n", pt_id);
	    }
	    //sensor_msgs::PointCloud2 ros_cloud;
	    //pcl::toROSMsg(*pcl_cloud, ros_cloud);
	    //ros_cloud.header.frame_id = "1";
	    //ros_cloud.header.seq = ni;

	    //printf("valid map pts: %lu\n", pt_array.poses.size()-1);

	    //printf("ros_cloud size: %d x %d\n", ros_cloud.height, ros_cloud.width);
	    //pub_cloud.publish(ros_cloud);
	    pt_array.header.frame_id = "1";
	    pt_array.header.seq = frame_id + 1;
	    pub_pts_and_pose.publish(pt_array);
	    //pub_kf.publish(camera_pose);
    }
  }
  
  void orbViewer(){
    cv::Mat color, depth;

    //(usb to)serial port init
    gpSLAM=orbslam;
    sport_imu.InitPort("/dev/ttyUSB0");
    sport_enc.InitPort("/dev/ttyACM0");
    if (!sport_imu.OpenListenThread())
      cerr<<redSTR"Error in creating the thread_imu!"<<whiteSTR<<endl;
    if (!sport_enc.OpenListenThread())
      cerr<<redSTR"Error in creating the thread_enc!"<<whiteSTR<<endl; 
    sport_imu.nFlag=1;//enter IMU reading state1
    sport_enc.nFlag=2;//enter hall sensor reading state2
    sport_imu.nSizeOfBytes=31;//though we can read only ahead 9 bytes to speed up
    sport_imu.lpBytes=new BYTE[sport_imu.nSizeOfBytes];
    sport_enc.nSizeOfBytes=50;//return at most "!VAR 1 1200\r+\r"*2=>28,please add the '\0'
      //at most "?BS \r" and return "?BS \r""response""\r"=>21
      //use total size for the the reply of the cmd_vel at the time of the reply for the next encoder!
    sport_enc.lpBytes=new BYTE[sport_enc.nSizeOfBytes];
    //Continuous mode!
    cout<<fixed<<setprecision(6);//for tmstamp's output
    BYTE utmp[3]={0x10,0x00,0x00};
    if (mbContinuous){
	if (CSerialPort::mbstartRecord==3){
	  //set Recording files
	  sport_enc.Openfout(gstrORBFile+"/test_save/EncSensor.txt");
	  sport_imu.Openfout(gstrORBFile+"/test_save/IMUSensor.txt");
	}
	//close continuous mode & clear buff
	sport_imu.nSizeOfBytes=7;
	sport_imu.lpNo=0;
	sport_imu.WriteData(utmp,3);
	sport_enc.lpNo=0;
	sport_enc.WriteData((BYTE*)"# C\r",strlen("# C\r"));//enc
	sleep(1);
	//set continuous mode for IMU
	utmp[2]=COMMAND;
	sport_imu.lpNo=0;
	sport_imu.nSizeOfBytes=7;
	//set encoder continuous mode(stop & add command to the buffer)
	sport_enc.lpNo=0;
	sport_enc.WriteData((BYTE*)"#\r",strlen("#\r"));
	sport_enc.WriteData((BYTE*)"?BS\r",strlen("?BS\r"));
	//start continuous mode
	sport_imu.WriteData(utmp,3);//imu
	sport_enc.WriteData((BYTE*)ENC_AUTO_START"\r",strlen(ENC_AUTO_START"\r"));//enc
    }else{
	if (CSerialPort::mbstartRecord==1){
	  //set Recording files
	  sport_enc.Openfout(gstrORBFile+"/test_save/odometrysensor.txt");
	}
    }
    //base_control part
    ros::Subscriber sub_cmd=nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel",10,boost::bind(&Receiver::baseContrlCallback, this, _1));//_manual
    
    //send static transform by tf2_ros
    //Trgbcameraframe_scanframe/Trs
    static tf2_ros::StaticTransformBroadcaster sttf_broadcaster;
    geometry_msgs::TransformStamped sttrans_rgb_scan,sttrans_base_rgb;
    sttrans_rgb_scan.header.stamp=ros::Time::now();
    sttrans_rgb_scan.header.frame_id=camera_ns+"_link";
    sttrans_rgb_scan.child_frame_id=camera_ns+"_depth_frame";
    sttrans_rgb_scan.transform.translation.x=0;
    sttrans_rgb_scan.transform.translation.y=0;
    sttrans_rgb_scan.transform.translation.z=0;
    tf::Matrix3x3 tf_mat2;
    tf_mat2.setEulerYPR(1.57,-1.57,0);
    tf::Quaternion tf_q2;
    tf_mat2.getRotation(tf_q2);
    sttrans_rgb_scan.transform.rotation.x=tf_q2.x();
    sttrans_rgb_scan.transform.rotation.y=tf_q2.y();
    sttrans_rgb_scan.transform.rotation.z=tf_q2.z();
    sttrans_rgb_scan.transform.rotation.w=tf_q2.w();
    sttf_broadcaster.sendTransform(sttrans_rgb_scan);
    //Tbaselink_rgbcameraframe/Tbr
    sttrans_base_rgb.header.stamp=ros::Time::now();
    sttrans_base_rgb.header.frame_id="base_link";
    sttrans_base_rgb.child_frame_id=camera_ns+"_link";
    sttrans_base_rgb.transform.translation.x=0.293+0.225;//m
    sttrans_base_rgb.transform.translation.y=-0.015;
    sttrans_base_rgb.transform.translation.z=0.578-0.105;//use the first centre of the wheel as the odom frame
    tf_mat2.setEulerYPR(-1.57,0,-1.57);
    tf_mat2.getRotation(tf_q2);
    sttrans_base_rgb.transform.rotation.x=tf_q2.x();
    sttrans_base_rgb.transform.rotation.y=tf_q2.y();
    sttrans_base_rgb.transform.rotation.z=tf_q2.z();
    sttrans_base_rgb.transform.rotation.w=tf_q2.w();
    sttf_broadcaster.sendTransform(sttrans_base_rgb);
    

    ros::Time current_time,last_time;
    cv::Mat cam_pose_last=cv::Mat::eye(4,4,CV_32F),cam_pose_last_inv=cv::Mat::eye(4,4,CV_32F);
    cv::Mat cam_pose=cv::Mat::eye(4,4,CV_32F),cam_pose_init;
    //about the publisher of the odom topic "/odom"
    ros::Publisher odom_pub=nh.advertise<nav_msgs::Odometry>("/odom",50);
    tf::TransformBroadcaster odom_broadcaster;
    nav_msgs::Odometry odom;
    for (int i=0;i<36;++i) 
	odom.pose.covariance[i]=odom.twist.covariance[i]=0.01;
    
    //get the Trb from the /tf(static) and transfrom it into cv::Mat
    tf2_ros::Buffer orb_buffer(ros::Duration(10.0));
    tf2_ros::TransformListener orb_listener(orb_buffer);
    geometry_msgs::TransformStamped trans_rgb_base=orb_buffer.lookupTransform(camera_ns+"_link","base_link",ros::Time(),ros::Duration(10.0));//Trb & Tboro=Tbr
    cv::Mat mat_Trb(4,4,CV_32F);
    mat_Trb.at<float>(3,0)=mat_Trb.at<float>(3,1)=mat_Trb.at<float>(3,2)=0;mat_Trb.ptr<float>(3)[3]=1;
    mat_Trb.ptr<float>(0)[3]=(float)trans_rgb_base.transform.translation.x;
    mat_Trb.ptr<float>(1)[3]=(float)trans_rgb_base.transform.translation.y;
    mat_Trb.ptr<float>(2)[3]=(float)trans_rgb_base.transform.translation.z;
    tf::Quaternion tf_q(trans_rgb_base.transform.rotation.x,trans_rgb_base.transform.rotation.y,trans_rgb_base.transform.rotation.z,trans_rgb_base.transform.rotation.w);
    tf::Matrix3x3 tf_mat(tf_q);
    for (int i=0;i<3;++i)
	for (int j=0;j<3;++j)
	  mat_Trb.ptr<float>(i)[j]=(float)tf_mat[i].m_floats[j];
    //get Tbr=Trb.inverse()
    cv::Mat twc(3,1,CV_32F),Rwc(3,3,CV_32F);
    cv::Mat mat_Tbr=cv::Mat::eye(4,4,CV_32F);
    Rwc=mat_Trb.rowRange(0,3).colRange(0,3).t();
    twc=-Rwc*mat_Trb.rowRange(0,3).col(3);
    Rwc.copyTo(mat_Tbr(cv::Rect(0,0,3,3)));
    twc.copyTo(mat_Tbr(cv::Rect(3,0,1,3)));
    //mat_Tbr=mat_Trb.inv();
    /*cv::Mat mat_Trorb=cv::Mat::eye(4,4,CV_32F);
    tf_mat.setEulerYPR(3.1415,0,0);
    for (int i=0;i<3;++i)
	for (int j=0;j<3;++j)
	  mat_Trorb.ptr<float>(i)[j]=(float)tf_mat[i].m_floats[j];
    cv::Mat mat_Trhlh=cv::Mat::eye(4,4,CV_32F);//Toriginalorb_orb=Tlh_Trh, here is wrong, orb is right
    mat_Trhlh.ptr<float>(2)[2]=-1;
    mat_Trorb=mat_Trorb*mat_Trhlh;
    cv::Mat mat_Torbr=mat_Trorb.inv();

    tf2_ros::StaticTransformBroadcaster st_bc1;//yaw180d
    geometry_msgs::TransformStamped st_transStamped;
    st_transStamped.header.stamp=ros::Time::now();
    st_transStamped.header.frame_id=""*/

    //about the publisher of the LaserScan topic "/scan"
    ros::Publisher scan_pub(nh.advertise<sensor_msgs::LaserScan>("/scan",50));
    depthimage_to_laserscan::DepthImageToLaserScan dep2scan;
    dep2scan.set_output_frame(camera_ns+"_depth_frame");//"/kinect2_depth_frame"
    dep2scan.set_range_limits(0.45,5.00);
    dep2scan.set_scan_height(50);//50 rows
    dep2scan.set_scan_time(0.033);//0.033s
    
    int frame_id=0;
    ros::Time img_time_process;
    for(; running && ros::ok();)
    {
      if(updateOrb)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
	img_time_process=img_time;
        updateOrb = false;
        lock.unlock();

	if (orbslam){
	  ++frame_id;
	  
	  current_time=img_time_process;//ros::Time::now();//
	  double dt=(current_time-last_time).toSec();
	  if (dt<0.02){
	    cerr<<redSTR"sth wrong between photos reading!"<<whiteSTR<<endl;
	    last_time=current_time;
	    continue;//at least 0.033s
	  }
	  
	  //rectify the time of SerialPort for polling mode & image's recording tmstamp
	  CSerialPort::m_tm_stamp=current_time.toSec();
	  if (!mbContinuous){
	    //read the odometry data(asking/polling mode)
	    sport_imu.lpNo=0;
	    int utmp=COMMAND;//0x0C/0x03
	    sport_imu.WriteData((BYTE*)&utmp,1);
	    sport_enc.lpNo=0;
	    sport_enc.WriteData((BYTE*)"?BS\r",4);//strlen("?BS\r")
	  }
	  
	  if (mb_speed_unlock&&(time(NULL)-last_cmd_time)>1){
	    //cout<<lasttime<<" "<<time(0)<<endl;
	    lock_cmd.lock();
	    gdCmdVel[0]=gdCmdVel[1]=0;mb_speed_unlock=false;
	    gbCmdChanged=true;
	    lock_cmd.unlock();
	  }
	  if (mode==ODOM){
	    //publish LasserScan topic "/scan"
	    sensor_msgs::LaserScanPtr pscan=dep2scan.convert_msg(imgdepthptr_ros,caminfoptr_ros);//the same info as color;maybe unsafe!!!
	    scan_pub.publish(*pscan);
	  }
	  static bool bReduceFreq=false;
	  if (bRecord){//if use "record" option
//		cout<<"Write Pic...";
	    if (bReduceFreq&&gstrHome!="")//when using "" we record images by rosbag!
	      orbslam->SaveFrame(gstrHome+"/test_save/",color,depth,CSerialPort::m_tm_stamp);
	    bReduceFreq=!bReduceFreq;
//		cout<<"Over!"<<endl;
	    last_time=current_time;continue;
	  }else if (mode==ODOM){
	    last_time=current_time;//ros::spinOnce();
	    continue;
	  }
	  //mode == ORB
          cam_pose_init=orbslam->TrackRGBD(color,depth,current_time.toSec());//ros::Time::now();//
	  if (cam_pose_init.empty()) continue;
	  //publish LasserScan topic "/scan"
	  sensor_msgs::LaserScanPtr pscan=dep2scan.convert_msg(imgdepthptr_ros,caminfoptr_ros);//the same info as color;maybe unsafe!!!
	  scan_pub.publish(*pscan); 
	  
	  //publish the info needed by the grid map topic
	  publish(*orbslam,pub_pts_and_pose, pub_all_kf_and_pts,frame_id);
// 	  ros::spinOnce();
	  
	  //cam_pose is originally Tcw, get the inverse() of it;finally it's Twc,here w means c0
	  Rwc=cam_pose_init.rowRange(0,3).colRange(0,3).t();
	  twc=-Rwc*cam_pose_init.rowRange(0,3).col(3);
	  Rwc.copyTo(cam_pose(cv::Rect(0,0,3,3)));//notice initial cam_pose is Tcw not Twc! we should transform it! && cannot use clone() here!
	  twc.copyTo(cam_pose(cv::Rect(3,0,1,3)));
	  //printf("%d %d type: %d \n",cam_pose.rows,cam_pose.cols,cam_pose.type());
	  //trans_lefthand(CCW)_to_righthand(CCW) and trans_to_rgb_link(it's hard to understand but you can use the same vector of translation and axis angle in this two simple cases)
	  /*cam_pose.ptr<float>(2)[3]=-cam_pose.ptr<float>(2)[3];//z->-z
	  cam_pose.ptr<float>(0)[3]=-cam_pose.ptr<float>(0)[3];//x,y->-x,-y
	  cam_pose.ptr<float>(1)[3]=-cam_pose.ptr<float>(1)[3];
	  tf_mat.setValue(*cam_pose.ptr<float>(0,0),*cam_pose.ptr<float>(0,1),*cam_pose.ptr<float>(0,2),*cam_pose.ptr<float>(1,0),*cam_pose.ptr<float>(1,1),*cam_pose.ptr<float>(1,2),*cam_pose.ptr<float>(2,0),*cam_pose.ptr<float>(2,1),*cam_pose.ptr<float>(2,2));
	  tf_mat.getRotation(tf_q);
	  tf_mat.setRotation(tf::Quaternion(-tf_q.x(),-tf_q.y(),-tf_q.z(),tf_q.w()));//qz->-qz and qx,qy->-qx,-qy
	  for (int i=0;i<3;++i)
	    for (int j=0;j<3;++j)
	      cam_pose.ptr<float>(i)[j]=(float)tf_mat[i].m_floats[j];//-qz->Rrh  
	  //transform the Trgbodom_rgb(RH) to Tbaseodom_base(RH) by Tbob=Tboro*Tror*Trb=Tbr*Tror*Trb
	  //cam_pose=mat_Tbr*cam_pose*mat_Trb;
	  cam_pose=mat_Tbr*mat_Trorb*cam_pose*mat_Torbr*mat_Trb;//get the Tbob or Todom_baselink*/
	  //Twc is righthand and at right direction as rgb!!!
	  //get the Tb0_b/Todom_baselink
	  cam_pose=mat_Tbr*cam_pose*mat_Trb;
	  
	  //publish the To_b to the "/tf" topic
	  geometry_msgs::TransformStamped odom_trans;
	  odom_trans.header.stamp=current_time;
	  odom_trans.header.frame_id="odom";
	  odom_trans.child_frame_id="base_link";
	  odom_trans.transform.translation.x=(double)cam_pose.ptr<float>(0)[3];
	  odom_trans.transform.translation.y=(double)cam_pose.ptr<float>(1)[3];
	  odom_trans.transform.translation.z=(double)cam_pose.ptr<float>(2)[3];//0;//
	  tf_mat.setValue(*cam_pose.ptr<float>(0,0),*cam_pose.ptr<float>(0,1),*cam_pose.ptr<float>(0,2),*cam_pose.ptr<float>(1,0),*cam_pose.ptr<float>(1,1),*cam_pose.ptr<float>(1,2),*cam_pose.ptr<float>(2,0),*cam_pose.ptr<float>(2,1),*cam_pose.ptr<float>(2,2));
	  //double yaw,tmp;
	  //tf_mat.getEulerYPR(yaw,tmp,tmp);
	  //tf_mat.setEulerYPR(yaw,0,0);
	  tf_mat.getRotation(tf_q);
	  odom_trans.transform.rotation.x=tf_q.x();//qx=sin(t/2)*nx;
	  odom_trans.transform.rotation.y=tf_q.y();
	  odom_trans.transform.rotation.z=tf_q.z();
	  odom_trans.transform.rotation.w=tf_q.w();
	  
	  odom_broadcaster.sendTransform(odom_trans);
	  
	  //publish the To_b && deltaTo_b/deltat to the "/odom" topic
	  odom.header.stamp=current_time;
	  //set the position
	  odom.header.frame_id="odom";
	  odom.child_frame_id="base_link";
	  odom.pose.pose.position.x=odom_trans.transform.translation.x;
	  odom.pose.pose.position.y=odom_trans.transform.translation.y;
	  odom.pose.pose.position.z=odom_trans.transform.translation.z;
	  odom.pose.pose.orientation=odom_trans.transform.rotation;
	  //set the velocity
	  //get the Tbt-1_o=To_bt-1.inverse()=>deltaTo_b=Tbt-1_bt=Tbt-1_o*To_bt
	  Rwc=cam_pose_last.rowRange(0,3).colRange(0,3).t();
	  twc=-Rwc*cam_pose_last.rowRange(0,3).col(3);
	  Rwc.copyTo(cam_pose_last_inv(cv::Rect(0,0,3,3)));
	  twc.copyTo(cam_pose_last_inv(cv::Rect(3,0,1,3)));
	  cv::Mat cam_pose_motion=cam_pose_last_inv*cam_pose;
	  //printf("%f\n",dt);
	  odom.twist.twist.linear.x=(double)cam_pose_motion.ptr<float>(0)[3]/dt;
	  odom.twist.twist.linear.y=(double)cam_pose_motion.ptr<float>(1)[3]/dt;
	  odom.twist.twist.linear.z=(double)cam_pose_motion.ptr<float>(2)[3]/dt;//0;//
	  tf_mat.setValue(*cam_pose_motion.ptr<float>(0,0),*cam_pose_motion.ptr<float>(0,1),*cam_pose_motion.ptr<float>(0,2),*cam_pose_motion.ptr<float>(1,0),*cam_pose_motion.ptr<float>(1,1),*cam_pose_motion.ptr<float>(1,2),*cam_pose_motion.ptr<float>(2,0),*cam_pose_motion.ptr<float>(2,1),*cam_pose_motion.ptr<float>(2,2));
	  tf_mat.getRotation(tf_q);
	  tf::Vector3 tf_vec3=tf_q.getAxis();
	  tfScalar tf_ang=tf_q.getAngle();
	  odom.twist.twist.angular.x=tf_ang*tf_vec3.x()/dt;//0;//
	  odom.twist.twist.angular.y=tf_ang*tf_vec3.y()/dt;//0;//
	  odom.twist.twist.angular.z=tf_ang*tf_vec3.z()/dt;//!notice R cannot /dt!

	  odom_pub.publish(odom);
	  
	  last_time=current_time;
	  cam_pose.copyTo(cam_pose_last);
        }
      }else{
	usleep(1000);//allow 1ms error
      }
    }

    if (mbContinuous){
	//reset output of dCmdVel
	lock_cmd.lock();
	gdCmdVel[0]=gdCmdVel[1]=0;mb_speed_unlock=false;
	gbCmdChanged=true;
	lock_cmd.unlock();
	cout<<azureSTR"Reset Output of CmdVel:"<<gdCmdVel[0]<<":"<<gdCmdVel[1]<<whiteSTR<<endl;
	sleep(1);
	//close continuous mode
	utmp[2]=0x00;
	sport_imu.WriteData(utmp,3);//imu
	sport_enc.WriteData((BYTE*)"# C\r",strlen("# C\r"));//enc
	sleep(1);
	cout<<"orbViewer End."<<endl;
    }

    int odomsize=gvecEnc.size();
    int size=2;if (odomsize>0) size=gvecEnc[0].size;
    double* podomdata=new double[size];
    for (int i=0;i<odomsize;++i){
	for (int j=0;j<size;++j) podomdata[j]=gvecEnc[i].data[j];
	sport_enc.writeToFile(podomdata,size,gvecEnc[i].tmstamp);
	if (i>0&&gvecEnc[i].tmstamp==gvecEnc[i-1].tmstamp){cout<<"Wrong!"<<gvecEnc[i].tmstamp<<endl;}
    }
    delete[] podomdata;
    odomsize=gvecIMU.size();
    for (int i=0;i<odomsize;++i){
	sport_imu.writeToFile(gvecIMU[i].data,gvecIMU[i].size,gvecIMU[i].tmstamp);
	if (i>0&&gvecIMU[i].tmstamp==gvecIMU[i-1].tmstamp){cout<<"Wrong!"<<gvecIMU[i].tmstamp<<endl;}
    }
    
    //save Map.bin
    if (mode==ORB){
      orbslam->SaveMap("Map.bin",false);//for Reused Sparse Map
      cout<<"Map Saved!"<<endl;
    }

//    delete []sport_imu.lpBytes;
//    delete []sport_enc.lpBytes;
  }

  
  void imageViewer()
  {
    cv::Mat color, depth, depthDisp, combined;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double fps = 0;
    size_t frameCount = 0;
    std::ostringstream oss;
    const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;
    //cv::namedWindow("Image Viewer");
    oss << "starting...";

    start = std::chrono::high_resolution_clock::now();
    for(; running && ros::ok();)
    {
      if(updateImage)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateImage = false;
        lock.unlock();

        ++frameCount;
        now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
        if(elapsed >= 1.0)
        {
          fps = frameCount / elapsed;
          oss.str("");
          oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
          start = now;
          frameCount = 0;
        }

        dispDepth(depth, depthDisp, 12000.0f);
        combine(color, depthDisp, combined);
        //combined = color;

        cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
        //cv::imshow("Image Viewer", combined);

	if (orbslam){
                orbslam->TrackRGBD(color,depth,ros::Time::now().toSec());
        }
     }

      /*int key = cv::waitKey(1);
      switch(key & 0xFF)
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        if(mode == IMAGE)
        {
          createCloud(depth, color, cloud);
          saveCloudAndImages(cloud, color, depth, depthDisp);
        }
        else
        {
          save = true;
        }
        break;
      }*/
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
  }

  void cloudViewer()
  {
    cv::Mat color, depth;
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";

    lock.lock();
    color = this->color;
    depth = this->depth;
    updateCloud = false;
    lock.unlock();

    createCloud(depth, color, cloud);

    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

    for(; running && ros::ok();)
    {
      if(updateCloud)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        visualizer->updatePointCloud(cloud, cloudName);

	if (orbslam){
                orbslam->TrackRGBD(color,depth,ros::Time::now().toSec());
        }
      }
      if(save)
      {
        save = false;
        cv::Mat depthDisp;
        dispDepth(depth, depthDisp, 12000.0f);
        saveCloudAndImages(cloud, color, depth, depthDisp);
      }
      visualizer->spinOnce(10);
    }
    visualizer->close();
  }

  void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
  {
    if(event.keyUp())
    {
      switch(event.getKeyCode())
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        save = true;
        break;
      }
    }
  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
  }

  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
  {
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

    #pragma omp parallel for
    for(int r = 0; r < in.rows; ++r)
    {
      const uint16_t *itI = in.ptr<uint16_t>(r);
      uint8_t *itO = tmp.ptr<uint8_t>(r);

      for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
      {
        *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
      }
    }

    cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
  }

  void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
  {
    out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

    #pragma omp parallel for
    for(int r = 0; r < inC.rows; ++r)
    {
      const cv::Vec3b
      *itC = inC.ptr<cv::Vec3b>(r),
       *itD = inD.ptr<cv::Vec3b>(r);
      cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

      for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
      {
        itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
        itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
        itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
      }
    }
  }

  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
      pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
      const uint16_t *itD = depth.ptr<uint16_t>(r);
      const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
      const float y = lookupY.at<float>(0, r);
      const float *itX = lookupX.ptr<float>();

      for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
      {
        register const float depthValue = *itD / 1000.0f;
        // Check for invalid measurements
        if(*itD == 0)
        {
          // not valid
          itP->x = itP->y = itP->z = badPoint;
          itP->rgba = 0;
          continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 255;
      }
    }
  }

  void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
  {
    oss.str("");
    oss << "./" << std::setfill('0') << std::setw(4) << frame;
    const std::string baseName = oss.str();
    const std::string cloudName = baseName + "_cloud.pcd";
    const std::string colorName = baseName + "_color.jpg";
    const std::string depthName = baseName + "_depth.png";
    const std::string depthColoredName = baseName + "_depth_colored.png";

    OUT_INFO("saving cloud: " << cloudName);
    writer.writeBinary(cloudName, *cloud);
    OUT_INFO("saving color: " << colorName);
    cv::imwrite(colorName, color, params);
    OUT_INFO("saving depth: " << depthName);
    cv::imwrite(depthName, depth, params);
    OUT_INFO("saving depth: " << depthColoredName);
    cv::imwrite(depthColoredName, depthColored, params);
    OUT_INFO("saving complete!");
    ++frame;
  }

  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }
};

void help(const std::string &path)
{
  std::cout << path << FG_BLUE " [options]" << std::endl
            << FG_GREEN "  name" NO_COLOR ": " FG_YELLOW "'any string'" NO_COLOR " equals to the kinect2_bridge topic base name" << std::endl
            << FG_GREEN "  mode" NO_COLOR ": " FG_YELLOW "'qhd'" NO_COLOR ", " FG_YELLOW "'hd'" NO_COLOR ", " FG_YELLOW "'sd'" NO_COLOR " or " FG_YELLOW "'ir'" << std::endl
            << FG_GREEN "  visualization" NO_COLOR ": " FG_YELLOW "'image'" NO_COLOR ", " FG_YELLOW "'cloud'" NO_COLOR " or " FG_YELLOW "'both'" << std::endl
            << FG_GREEN "  options" NO_COLOR ":" << std::endl
            << FG_YELLOW "    'compressed'" NO_COLOR " use compressed instead of raw topics" << std::endl
            << FG_YELLOW "    'approx'" NO_COLOR " use approximate time synchronization" << std::endl;
}

int main(int argc, char **argv)
{
#if EXTENDED_OUTPUT
  ROSCONSOLE_AUTOINIT;
  if(!getenv("ROSCONSOLE_FORMAT"))
  {
    ros::console::g_formatter.tokens_.clear();
    ros::console::g_formatter.init("[${severity}] ${message}");
  }
#endif

  ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
  bool useExact = true;
  bool useCompressed = false;
  int useOrbviewer=0;//0 means not use, 1 means use viewer, 2 means record imgs with no viewer
  Receiver::Mode mode = Receiver::ODOM;

  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);

    if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
    else if(param == "qhd")
    {
      topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "hd")
    {
      topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "ir")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "sd")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "approx")
    {
      useExact = false;
    }

    else if(param == "compressed")
    {
      useCompressed = true;
    }
    else if(param == "image")
    {
      mode = Receiver::IMAGE;
    }
    else if(param == "cloud")
    {
      mode = Receiver::CLOUD;
    }
    else if(param == "orb")
    {
      mode = Receiver::ORB;
    }
    else if(param == "both")
    {
      mode = Receiver::BOTH;
    }else if (param=="viewer"){
      useOrbviewer=1;
    }else if (param=="recordPoll"){
      useOrbviewer=2;
      CSerialPort::mbstartRecord=1;
    }else if (param=="record"){
      useOrbviewer=3;
      CSerialPort::mbstartRecord=3;
    }else if (param=="VIEO"){
      mode = Receiver::ORB;
      useOrbviewer=4;//VIEORBSLAM2 provides map.bin and map.pgm, it also can be used as AMCL
      CSerialPort::mbstartRecord=2;//VIEORBSLAM2 provides map.bin and map.pgm, it also can be used as AMCL
    }
    else
    {
      ns = param;
    }
  }

  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
  OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

  Receiver receiver(topicColor, topicDepth, useExact, useCompressed,useOrbviewer,ns);

  OUT_INFO("starting receiver...");
  receiver.run(mode);

  ros::shutdown();
  return 0;
}
