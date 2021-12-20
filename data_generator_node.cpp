#include "ros/ros.h"

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

#include "data_generator.h"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

#define ROW 480
#define COL 752


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_generator");
    ros::NodeHandle n("~");

    ros::Publisher pub_imu      = n.advertise<sensor_msgs::Imu>("imu", 1000);
    ros::Publisher pub_image    = n.advertise<sensor_msgs::PointCloud>("image", 1000);
    ros::Publisher pub_wifi     = n.advertise<sensor_msgs::PointCloud>("wifi", 1000);

    ros::Publisher pub_path     = n.advertise<nav_msgs::Path>("path", 1000);
    ros::Publisher pub_odometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    ros::Publisher pub_pose     = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
    ros::Publisher pub_cloud    = n.advertise<sensor_msgs::PointCloud>("cloud", 1000);
    //ros::Publisher pub_ap       = n.advertise<sensor_msgs::PointCloud>("ap", 1000);
    ros::Publisher pub_line     = n.advertise<visualization_msgs::Marker>("direction", 1000);

    ros::Publisher pub_disp     = n.advertise<nav_msgs::Path>("disp", 1000);

    ros::Duration(1).sleep();

    DataGenerator generator;
    ros::Rate loop_rate(generator.FREQ);

    //if (argc == 1)
    //    while (pub_imu.getNumSubscribers() == 0)
    //        loop_rate.sleep();

    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.frame_id = "world";
    point_cloud.header.stamp = ros::Time::now();
    for (auto & it : generator.getCloud())
    {
        geometry_msgs::Point32 p;
        p.x = it(0);
        p.y = it(1);
        p.z = it(2);
        point_cloud.points.push_back(p);
    }
    pub_cloud.publish(point_cloud);

    point_cloud.points.clear();

    //visualization_msgs::Marker line_ap[DataGenerator::NUMBER_OF_AP];
    /*
    for (int i = 0; i < DataGenerator::NUMBER_OF_AP; i++)
    {
        geometry_msgs::Point32 p;
        Vector3d p_ap = generator.getAP(i);
        p.x = p_ap(0);
        p.y = p_ap(1);
        p.z = p_ap(2);
        point_cloud.points.push_back(p);

        line_ap[i].id = i;
        line_ap[i].header.frame_id = "world";
        line_ap[i].ns = "line";
        line_ap[i].action = visualization_msgs::Marker::ADD;
        line_ap[i].pose.orientation.w = 1.0;
        line_ap[i].type = visualization_msgs::Marker::LINE_STRIP;
        line_ap[i].scale.x = 0.1;
        line_ap[i].color.r = 1.0;
        line_ap[i].color.a = 1.0;

        geometry_msgs::Point p2;
        p2.x = p.x;
        p2.y = p.y;
        p2.z = p.z;
        line_ap[i].points.push_back(p2);
        line_ap[i].points.push_back(p2);
    }
    pub_ap.publish(point_cloud);
    */

    int publish_count = 0;

    nav_msgs::Path path;
    path.header.frame_id = "world";
	default_random_engine random_generator;
	std::normal_distribution<double> distribution (0.0, 0.1);

    nav_msgs::Path disp;
    disp.header.frame_id = "world";

    Vector3d pre_p;
    double fst_del_d = -1;

    while (ros::ok())
    {
        double current_time = generator.getTime();
        ROS_INFO("time: %lf", current_time);

        Vector3d position     = generator.getPosition();
        Vector3d velocity     = generator.getVelocity();
        Matrix3d rotation     = generator.getRotation();
        Quaterniond q(rotation);

        Vector3d linear_acceleration = generator.getLinearAcceleration();
        Vector3d angular_velocity = generator.getAngularVelocity();

        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "world";
        odometry.header.stamp = ros::Time(current_time);
        odometry.pose.pose.position.x = position(0);
        odometry.pose.pose.position.y = position(1);
        odometry.pose.pose.position.z = position(2);
        odometry.pose.pose.orientation.x = q.x();
        odometry.pose.pose.orientation.y = q.y();
        odometry.pose.pose.orientation.z = q.z();
        odometry.pose.pose.orientation.w = q.w();
        odometry.twist.twist.linear.x = velocity(0);
        odometry.twist.twist.linear.y = velocity(1);
        odometry.twist.twist.linear.z = velocity(2);
        pub_odometry.publish(odometry);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "world";
        pose_stamped.header.stamp = ros::Time(current_time);
        pose_stamped.pose = odometry.pose.pose;
        path.poses.push_back(pose_stamped);
        pub_path.publish(path);
        pub_pose.publish(pose_stamped);
        /*
        for (int i = 0; i < DataGenerator::NUMBER_OF_AP; i++)
        {
            line_ap[i].header.stamp = ros::Time(current_time);
            line_ap[i].points.back().x = position(0);
            line_ap[i].points.back().y = position(1);
            line_ap[i].points.back().z = position(2);
            pub_line.publish(line_ap[i]);
        }
        */
        sensor_msgs::Imu imu;
        imu.header.frame_id = "world";
        imu.header.stamp = ros::Time(current_time);
        imu.linear_acceleration.x = linear_acceleration(0);
        imu.linear_acceleration.y = linear_acceleration(1);
        imu.linear_acceleration.z = linear_acceleration(2);
        imu.angular_velocity.x = angular_velocity(0);
        imu.angular_velocity.y = angular_velocity(1);
        imu.angular_velocity.z = angular_velocity(2);
        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.orientation.w = q.w();

        pub_imu.publish(imu);
        ROS_INFO("publish imu data with stamp %lf", imu.header.stamp.toSec());

        //publish wifi data
        if (publish_count % generator.IMU_PER_WIFI == 0)
        {
            sensor_msgs::PointCloud wifi;
            sensor_msgs::ChannelFloat32 id_ap;
            wifi.header.stamp = ros::Time(current_time);
            if (disp.poses.empty())
            {
                geometry_msgs::PoseStamped disp_pose_stamped;
                disp_pose_stamped.header.frame_id = "world";
                disp_pose_stamped.header.stamp = ros::Time(current_time);
                disp_pose_stamped.pose = odometry.pose.pose;
                disp.poses.push_back(disp_pose_stamped);
                pre_p(0) = odometry.pose.pose.position.x;
                pre_p(1) = odometry.pose.pose.position.y;
                pre_p(2) = odometry.pose.pose.position.z;
            }
            else
            {
                //noise
                cout << "Pre p:" << pre_p(0) << ", " << pre_p(1) << ", " << pre_p(2) << endl;

                Vector3d cur_p;
                cur_p(0) = odometry.pose.pose.position.x;
                cur_p(1) = odometry.pose.pose.position.y;
                cur_p(2) = odometry.pose.pose.position.z;
                cout << "Cur p:" << cur_p(0) << ", " << cur_p(1) << ", " << cur_p(2) << endl;
                Vector3d delta_p = cur_p - pre_p;
                cout << "Delta p:" << delta_p(0) << ", " << delta_p(1) << ", " << delta_p(2) << endl;
                double delta_d = delta_p.norm();
                if (fst_del_d < 0)
                {
                    fst_del_d = delta_d;
                }
                if (delta_d == 0 && fst_del_d > 0)
                {
                    delta_d = fst_del_d;
                }
                cout << "Delta dist: " << delta_d << endl;
                cout << "Disturb before:" << cur_p(0) << ", " << cur_p(1) << ", " << cur_p(2) << endl;
                double disturb = distribution(random_generator);
                cur_p(0) += delta_d * disturb;
                disturb = distribution(random_generator);
                cur_p(1) += delta_d * disturb;
                disturb = distribution(random_generator);
                cur_p(2) += delta_d * disturb;
                cout << "Disturb after:" << cur_p(0) << ", " << cur_p(1) << ", " << cur_p(2) << endl;

                geometry_msgs::PoseStamped disp_pose_stamped;
                disp_pose_stamped.header.frame_id = "world";
                disp_pose_stamped.header.stamp = ros::Time(current_time);
                disp_pose_stamped.pose.position.x = cur_p(0);
                disp_pose_stamped.pose.position.y = cur_p(1);
                disp_pose_stamped.pose.position.z = cur_p(2);
                disp_pose_stamped.pose.orientation = odometry.pose.pose.orientation;
                disp.poses.push_back(disp_pose_stamped);
                
                geometry_msgs::Point32 disp_p;
                disp_p.x = cur_p(0) - pre_p(0);
                disp_p.y = cur_p(1) - pre_p(1);
                disp_p.z = cur_p(2) - pre_p(2);

                pre_p(0) = odometry.pose.pose.position.x;
                pre_p(1) = odometry.pose.pose.position.y;
                pre_p(2) = odometry.pose.pose.position.z;

                wifi.points.push_back(disp_p);
                id_ap.values.push_back(0);
                wifi.channels.push_back(id_ap);
                pub_wifi.publish(wifi);
            }
            pub_disp.publish(disp);
            
            /*
            for (int i = 0; i < DataGenerator::NUMBER_OF_AP; i++)
            {
                Vector3d direction;
                direction(0) = line_ap[i].points[0].x - line_ap[i].points[1].x;
                direction(1) = line_ap[i].points[0].y - line_ap[i].points[1].y;
                direction(2) = line_ap[i].points[0].z - line_ap[i].points[1].z;
                direction.normalize();
                geometry_msgs::Point32 p;
				//noise
				double disturb = distribution(random_generator);
				cout << "Disturb: " << disturb << endl;
                p.x = direction.dot(rotation.col(0));
				cout << "disturb before:" << p.x << endl;
                p.y = 0.0;
                p.z = 0.0;
				double angle = -2*M_PI*p.x*0.04/0.0514/M_PI*180 + disturb;
				p.x = -1*angle/180.0*M_PI/2/M_PI/0.04*0.0514;
				cout << "disturb after: " << p.x << endl;
                wifi.points.push_back(p);
                id_ap.values.push_back(i);
            }
            wifi.channels.push_back(id_ap);
            pub_wifi.publish(wifi);
            */
        }

        //publish image data
        if (publish_count % generator.IMU_PER_IMG == 0)
        {
            ROS_INFO("feature count: %lu", generator.getImage().size());
            sensor_msgs::PointCloud feature;
            sensor_msgs::ChannelFloat32 ids;
            sensor_msgs::ChannelFloat32 pixel;
            for (int i = 0; i < ROW; i++)
                for (int j = 0; j < COL; j++)
                    pixel.values.push_back(255);
            feature.header.stamp = ros::Time(current_time);

            cv::Mat simu_img[DataGenerator::NUMBER_OF_CAMERA];
            for (int i = 0; i < DataGenerator::NUMBER_OF_CAMERA; i++)
                simu_img[i] = cv::Mat(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));

            for (auto & id_pts : generator.getImage())
            {
                int id = id_pts.first;
                geometry_msgs::Point32 p;
                p.x = id_pts.second(0);
                p.y = id_pts.second(1);
                p.z = id_pts.second(2);

                feature.points.push_back(p);
                ids.values.push_back(id);


                char label[10];
                sprintf(label, "%d", id / DataGenerator::NUMBER_OF_CAMERA);
                cv::putText(simu_img[id % DataGenerator::NUMBER_OF_CAMERA], label, cv::Point2d(p.x + 1, p.y + 1) * 0.5 * 600, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
            }
            feature.channels.push_back(ids);
            feature.channels.push_back(pixel);
            pub_image.publish(feature);
            ROS_INFO("publish image data with stamp %lf", feature.header.stamp.toSec());
            for (int k = 0; k < DataGenerator::NUMBER_OF_CAMERA; k++)
            {
                char name[] = "camera 1";
                name[7] += k;
                cv::imshow(name, simu_img[k]);
            }
            cv::waitKey(1);
            if (generator.getTime() > 3 * DataGenerator::MAX_TIME)
                break;
        }

        //update work
        generator.update();
        publish_count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
