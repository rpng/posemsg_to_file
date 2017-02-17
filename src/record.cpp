/*
 * record.cpp
 *
 *  Created on: Jan 15, 2013
 *      Author: cforster
 *  Edited on: Feb 16, 2017
 *      Author: pgeneva
 */

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "GPSConversion.h"

using namespace std;

class Recorder {
public:
    std::string name;
    std::ofstream ofs_;
    int n_msgs_received_;
    bool invert_pose_;
    tf::TransformListener tf_listener_;
    Eigen::Quaterniond q_;
    Eigen::Vector3d p_;
    Eigen::Vector3d Pp_;
    Eigen::Vector3d Pr_;
    bool gps_ref_got;
    Eigen::Vector3d gps_ref_;
    double stamp_;

    Recorder(std::string topic_name, std::string filename, bool invert_pose) :
            name(topic_name),
            n_msgs_received_(0),
            invert_pose_(invert_pose),
            gps_ref_got(false),
            tf_listener_(ros::Duration(100)) {
        // Set defaults to zero
        Pp_ = Eigen::Vector3d::Zero();
        Pr_ = Eigen::Vector3d::Zero();
        // Setup file
        ofs_.open(filename.c_str());
        if (ofs_.fail())
            throw std::runtime_error("Could not create tracefile. Does folder exist?");
        ofs_ << "# format: timestamp tx ty tz qx qy qz qw Ptx Pty Ptz Prx Pry Prz" << std::endl;
    }

    ~Recorder() {}

    void write() {
        if (invert_pose_) {
            Eigen::Matrix3d R = q_.toRotationMatrix().transpose();
            p_ = -R * p_;
            q_ = Eigen::Quaterniond(R);
        }

        ofs_.precision(15);
        ofs_.setf(std::ios::fixed, std::ios::floatfield);
        ofs_ << stamp_ << " ";
        ofs_.precision(10);
        ofs_ << p_.x() << " " << p_.y() << " " << p_.z() << " "
             << q_.x() << " " << q_.y() << " " << q_.z() << " " << q_.w() << " "
                << Pp_.x() << " " << Pp_.y() << " " << Pp_.z() << " "
                << Pr_.x() << " " << Pr_.y() << " " << Pr_.z() << " " << std::endl;

        if (++n_msgs_received_ % 50 == 0)
            printf("[%s]: Received %i pose messages\n", name.c_str(), n_msgs_received_);
    }

    void poseCallback(const geometry_msgs::PoseStampedPtr &msg) {
        q_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                msg->pose.orientation.y, msg->pose.orientation.z);
        p_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        stamp_ = msg->header.stamp.toSec();
        write();
    }

    void poseCovCallback(const geometry_msgs::PoseWithCovarianceStampedPtr &msg) {
        q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        p_ = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        Pp_ = Eigen::Vector3d(msg->pose.covariance.at(0),msg->pose.covariance.at(7),msg->pose.covariance.at(14));
        Pr_ = Eigen::Vector3d(msg->pose.covariance.at(21),msg->pose.covariance.at(28),msg->pose.covariance.at(35));
        stamp_ = msg->header.stamp.toSec();
        write();
    }

    void transformStampedCallback(const geometry_msgs::TransformStampedPtr &msg) {
        q_ = Eigen::Quaterniond(msg->transform.rotation.w, msg->transform.rotation.x,
                                msg->transform.rotation.y, msg->transform.rotation.z);
        p_ = Eigen::Vector3d(msg->transform.translation.x, msg->transform.translation.y,
                             msg->transform.translation.z);
        stamp_ = msg->header.stamp.toSec();
        write();
    }

    void tfCallback(const std::string &topic, const std::string &topic_ref) {
        tf::StampedTransform tf_transform;
        ros::Time now(ros::Time::now());
        try {
            tf_listener_.waitForTransform(topic, topic_ref, now, ros::Duration(2.0));
            tf_listener_.lookupTransform(topic, topic_ref, now, tf_transform);
        }
        catch (tf::TransformException ex) {
            ROS_WARN("tfCallback: %s", ex.what());
        }

        Eigen::Affine3d eigen_transform;
        tf::transformTFToEigen(tf_transform, eigen_transform);
        q_ = Eigen::Quaterniond(eigen_transform.rotation());
        p_ = eigen_transform.translation();
        stamp_ = now.toSec();
        write();
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        // Return if we do not have a reference yet
        if(!gps_ref_got)
            return;
        // Convert into ENU frame from the Lat, Lon frame
        double xEast, yNorth, zUp;
        GPSConversion::GeodeticToEnu(msg->latitude, msg->longitude, msg->altitude, gps_ref_(0), gps_ref_(1), gps_ref_(2), xEast, yNorth, zUp);
        // Set our values
        q_ = Eigen::Quaterniond(0,0,0,1);
        p_ = Eigen::Vector3d(xEast, yNorth, zUp);
        stamp_ = msg->header.stamp.toSec();
        write();
    }

    void gpsRefCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        gps_ref_ = Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude);
        gps_ref_got = true;
    }

};

int main(int argc, char **argv) {
    // create ros node
    ros::init(argc, argv, "trajectory_recorder");
    ros::NodeHandle nh("~");

    // get parameters to subscribe
    std::string topic;
    nh.getParam("topic", topic);
    std::string topic_type;
    nh.getParam("topic_type", topic_type);
    std::string topic_ref;
    nh.getParam("topic_ref", topic_ref);
    bool invert_pose;
    nh.getParam("invert_pose", invert_pose);

    // Get current time/date
    // http://stackoverflow.com/a/997803
    char buf[16];
    snprintf(buf, 16, "%lu", time(NULL));

    // Generate filename
    std::string topic_time(buf);
    std::string topic_name(topic);
    std::replace(topic_name.begin(), topic_name.end(), '/', '_');
    std::string filename(ros::package::getPath("posemsg_to_file") + "/logs/" + topic_time + topic_name + ".txt");

    // Debug
    cout << "Done reading config values" << endl;
    cout << " - topic = " << topic << endl;
    cout << " - topic_type = " << topic_type << endl;
    cout << " - topic_ref = " << topic_ref << endl;
    cout << " - invert_pose = " << invert_pose << endl;
    cout << " - file = " << topic_time+topic_name << ".txt" << endl;

    // start recorder
    Recorder recorder(topic_name, filename, invert_pose);

    // subscribe to topic
    ros::Subscriber sub, sub_ref;
    if (topic_type == std::string("PoseWithCovarianceStamped")) {
        sub = nh.subscribe(topic, 10, &Recorder::poseCovCallback, &recorder);
    } else if (topic_type == std::string("PoseStamped")) {
        sub = nh.subscribe(topic, 10, &Recorder::poseCallback, &recorder);
    } else if (topic_type == std::string("TransformStamped")) {
        sub = nh.subscribe(topic, 10, &Recorder::transformStampedCallback, &recorder);
    } else if (topic_type == std::string("tf")) {
        if (topic_ref.empty())
            throw std::runtime_error("no tf reference topic specified.");
    } else if(topic_type == std::string("NavSatFix")) {
        // Check that we have a gps datum
        if (topic_ref.empty())
            throw std::runtime_error("no GPS reference topic specified.");
        // Sub to the two message topics
        sub = nh.subscribe(topic, 10, &Recorder::gpsCallback, &recorder);
        sub_ref = nh.subscribe(topic_ref, 10, &Recorder::gpsRefCallback, &recorder);
    } else {
        throw std::runtime_error("specified topic_type is not supported.");
    }

    // spin
    ros::Rate r(500);
    while (ros::ok()) {
        ros::spinOnce();
        if (topic_type == std::string("tf"))
            recorder.tfCallback(topic, topic_ref);
        r.sleep();
    }
    return 0;
}
