#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_msgs/Time.h>

#include "Thirdparty/DLib/FileFunctions.h"

//#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/Image.h>

#include <boost/foreach.hpp>

using namespace std;

typedef enum ID_MODE {
  MODE_FREQUENCY = 1,
  MODE_IMU_DATA_INFO = 2,
  //<path to imagei directory> is input when bit4 not on, output when it's on
  MODE_IN_IMAGES_BAG = 4
} eID_MODE;

typedef enum SIZE_ARG {
  SIZE_ID_MODE = 1,
  SIZE_FREQUENCY = 1,
  SIZE_IMU_DATA_INFO = 6,
  SIZE_IMAGES_BAG_PATH = 1,
  SIZE_IMAGE_FOLDER_INFO = 4
} eSIZE_ARG;

void printusage() {
  cerr << "Usage: [rosrun BagFromImages or ./]BagFromImages <id_mode> "
          "[{id_mode&1:}<frequency> or {!(id_mode&1):<scale_name_time>}] "
          "[{id_mode&4:}<path to input bag> else <path to output bag>]"
          "({id_mode&2:}<num of IMU> <start_line> <path to imu file> <imu "
          "topic name> <num of IMU data:13/7/6> <scale_acc> <scale_gyro> "
          "<scale_time>) <path to imu1 file>..."
          "(<path to image0 directory> <image0 extension e.g. bmp> <image0 "
          "topic name> <image0 channels num> [{id_mode&4:} <> else <num "
          "divided>] (<path to depth/image1 directory>...)"
       << endl;
  cerr << "<number of IMU data> doesn't include timestamp(default 0th data):"
          " 13 means qw qx qy qz mx my mz ax ay az gx gy gz>; 6 means ax ay az "
          "gx gy gz; 7 means id_imu ax ay az gx gy gz{so 7 could allow to "
          "merge all imus in one file}"
       << endl;
  cerr << "<start_line> means first n lines to be skipped in such file" << endl;
  cerr << "<imagei channels num>: 3 means Color, 1 means Depth" << endl;
  cerr << "Current version unsupported imu output to a file from input bag"
       << endl;
  cerr << "<num divided> means the number of images with same width shared in "
          "one image, standard is 1";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "BagFromImages");

  if (argc < 6) {
    printusage();
    return -1;
  }

  ros::start();

  int pos = 1;
  int id_mode = atoi(argv[pos]);
  pos += SIZE_ID_MODE;

  // Frequency
  double freq = 30;
  double scale_tm = 1.;
  if (id_mode & MODE_FREQUENCY) {
    freq = atof(argv[pos]);
  } else {
    scale_tm = atof(argv[pos]);
  }
  pos += SIZE_FREQUENCY;

  // Bag Path
  string path_in_image_bag = "";
  rosbag::Bag *pbag_out = nullptr; // Output bag
  if (id_mode & MODE_IN_IMAGES_BAG) {
    path_in_image_bag = argv[pos];
  } else {
    pbag_out = new rosbag::Bag(argv[pos], rosbag::bagmode::Write);
  }
  pos += SIZE_IMAGES_BAG_PATH;

  // Imu Path
  vector<string> paths_imu_file;
  vector<string> topics_imu; // e.g. "/imu" "/imu0"
  vector<int> nums_imu_data; // not including timestamp but including id_imu
  vector<int> start_lines;
  vector<vector<double>> scales_imu; // a, g, tm
  if (id_mode & MODE_IMU_DATA_INFO) {
    int num = atoi(argv[pos]);
    start_lines.push_back(atoi(argv[pos + 1]));
    pos += 2;

    for (int j = 0; j < num; ++j) {
      paths_imu_file.push_back(argv[pos]);
      topics_imu.push_back(argv[pos + 1]);
      nums_imu_data.push_back(atoi(argv[pos + 2]));
      scales_imu.push_back(vector<double>());
      for (int i = 0; i < 3; ++i)
        scales_imu.back().push_back(atof(argv[pos + 3 + i]));
      pos += SIZE_IMU_DATA_INFO;
    }
  }

  // Vector of paths to image
  vector<string> foldernames_image, exts_image;
  vector<vector<string>> filenames_images;
  vector<int> nums_channels;
  fstream finIMU;
  string strtmp;
  int num_params_left = argc - pos;
  int i = 0;
  vector<string> topics_image;
  vector<size_t> vnum_divided;
  while (num_params_left > 0) {
    foldernames_image.push_back(argv[pos]);
    exts_image.push_back(argv[pos + 1]);
    filenames_images.push_back(
        DUtils::FileFunctions::Dir(argv[pos], argv[pos + 1], true));
    cout << "Image" << i << "'s num: " << filenames_images.back().size();
    // like "kinect2/qhd/image_color_rect" "/kinect2/qhd/image_color"
    topics_image.push_back(argv[pos + 2]);
    cout << " Topic: " << topics_image.back() << endl;
    nums_channels.push_back(atoi(argv[pos + 3]));
    if (!(id_mode & MODE_IN_IMAGES_BAG)) {
      vnum_divided.push_back(atoi(argv[pos + 4]));
      cout << " Divided Num: " << vnum_divided.back() << endl;
      ++pos;
      --num_params_left;
    }
    pos += SIZE_IMAGE_FOLDER_INFO;
    num_params_left -= SIZE_IMAGE_FOLDER_INFO;
    ++i;
  }

  // path to input bag
  if (path_in_image_bag != "") { // save images from referenced rosbag
    rosbag::Bag bagin;
    bagin.open(path_in_image_bag, rosbag::bagmode::Read);
    for (size_t j = 0; j < filenames_images.size(); ++j) {
      vector<string> topics(1, topics_image[j]);
      rosbag::View view(bagin, rosbag::TopicQuery(topics));
      i = 0;
      BOOST_FOREACH (rosbag::MessageInstance const m, view) {
        sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
        if (simage != NULL) {
          const sensor_msgs::Image &img = *simage;
          double tm_stamp = img.header.stamp.toSec();
          char ch[25]; // at least 10+1+6+4+1=22
          // mpTracker->mCurrentFrame.mTimeStamp;
          sprintf(ch, "%.6f.", tm_stamp);
          string imgname = foldernames_image[j] + "/" + ch + exts_image[j];

          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(simage);
          cv::imwrite(imgname, cv_ptr->image);
          cout << "Saved " << ++i << " images~" << endl;
        }
      }
    }
    cout << "Saved Num Images Type: " << filenames_images.size() << endl;
  } else {
    assert(pbag_out);
    rosbag::Bag &bag_out = *pbag_out;

    ros::Time t, t_old = ros::Time::now();
    t = t_old;

    const float T = 1.0f / freq;
    ros::Duration d(T);

    // write depth/color images to rosbag::Bag
    for (size_t j = 0; j < filenames_images.size(); ++j) {
      t = t_old;
      for (size_t i = 0; i < filenames_images[j].size(); ++i) {
        if (!ros::ok())
          break;

        string &filename = filenames_images[j][i];
        for (size_t k = 0; k < vnum_divided[j]; ++k) {
          cv_bridge::CvImage cvImage;
          cv::Mat im;
          if (1 == nums_channels[j]) {
            im = cv::imread(filename, cv::IMREAD_UNCHANGED);
            im = im.colRange(im.cols / vnum_divided[j] * k,
                             im.cols / vnum_divided[j] * (k + 1));
            cvImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
          } else {
            im = cv::imread(filename, cv::IMREAD_COLOR);
            im = im.colRange(im.cols / vnum_divided[j] * k,
                             im.cols / vnum_divided[j] * (k + 1));
            cvImage.encoding = sensor_msgs::image_encodings::RGB8;
          }
          if (!(id_mode & MODE_FREQUENCY)) {
            int pos = filename.rfind('/') + 1;
            t = ros::Time(
                scale_tm *
                atof(filename.substr(pos, filename.rfind(exts_image[j]) - pos)
                         .c_str()));
          }
          cvImage.image = im;
          cvImage.header.stamp = t;
          // e.g. "/camera/image_raw" "/cam0/rgb/image"
          bag_out.write(topics_image[j] +
                            (vnum_divided[j] > 1 ? to_string(k) : ""),
                        ros::Time(t), cvImage.toImageMsg());
        }
        t += d;
        if (!(i % (int)freq) || i == filenames_images[j].size() - 1)
          cout << i + 1 << " / " << filenames_images[j].size() << " ";
      }
      cout << endl;
    }

    // write IMU data to rosbag
    for (size_t j = 0; j < paths_imu_file.size(); ++j) {
      finIMU.open(paths_imu_file[j], std::ios::in);
      for (int i = 0; i < start_lines[0]; ++i)
        getline(finIMU, strtmp);

      double tmstamp, data[9], quat[4]; // max9+4=13
      int id_imu = -1, pos_gx = 3, pos_ax = 0;
      int line_id = 0;
      do {
        finIMU >> tmstamp;
        if (finIMU.fail() || finIMU.eof())
          break;
        tmstamp *= scales_imu[j][2];
        sensor_msgs::ImuPtr pdataIMU(new sensor_msgs::Imu());
        sensor_msgs::Imu &dataIMU = *pdataIMU;
        if (nums_imu_data[j] == 13) {
          // wxyz while q3 is w
          finIMU >> quat[3] >> quat[0] >> quat[1] >> quat[2];
          dataIMU.orientation.w = quat[3];
          dataIMU.orientation.x = quat[0];
          dataIMU.orientation.y = quat[1];
          dataIMU.orientation.z = quat[2];
          for (int i = 0; i < 3; ++i)
            finIMU >> data[i]; // magxyz
          pos_gx = 6;
          pos_ax = 3;
        } else if (nums_imu_data[j] == 7) {
          finIMU >> id_imu;
        }
        for (int i = 0; i < 6; ++i)
          finIMU >> data[i]; // axyz wxyz
        getline(finIMU, strtmp);

        for (int i = pos_gx; i < pos_gx + 3; ++i)
          data[i] *= scales_imu[j][1];
        for (int i = pos_ax; i < pos_ax + 3; ++i)
          data[i] *= scales_imu[j][0];
        dataIMU.angular_velocity.x = data[pos_gx];
        dataIMU.angular_velocity.y = data[pos_gx + 1];
        dataIMU.angular_velocity.z = data[pos_gx + 2];
        dataIMU.linear_acceleration.x = data[pos_ax];
        dataIMU.linear_acceleration.y = data[pos_ax + 1];
        dataIMU.linear_acceleration.z = data[pos_ax + 2];
        if (!(line_id++))
          cout << fixed << setprecision(9) << "tmstemp_start=" << tmstamp
               << endl;
        t = ros::Time(tmstamp);
        dataIMU.header.stamp = t;
        bag_out.write(topics_imu[j] + (id_imu >= 0 ? to_string(id_imu) : ""), t,
                      pdataIMU);
      } while (!finIMU.eof());
      cout << "tmstemp_end=" << tmstamp << ", imu_size=" << line_id << endl;
    }

    bag_out.close();
  }

  ros::shutdown();

  if (pbag_out)
    delete pbag_out;

  return 0;
}
