/*
File: OccupancyGridMapNode.cpp
Author: 최시웅 (Si Woong Choi) with siwon
Date: 2023-07-18
Version: v1.0
Description: 해당 노드는 Point Cloud, IMU 데이터를 받고 2D 점유그리드맵을 생성한다.
*/ 

#include <ros/ros.h>
#include <deque>
#include <vector>
#include <iostream>
#include <string> 
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <XmlRpcValue.h>
#include <Eigen/Dense>
#include <cmath>
#include <pcl/filters/voxel_grid.h>
class OccupancyGridMapNode
{
public:
    OccupancyGridMapNode(){   
        //* ros 노드핸들 선언
        nh = ros::NodeHandle();

        //* 토픽 불러오기
        std::string cocloudTopic= "/ouster/points";

        //* Subscription 생성

        subcoCloud = nh.subscribe(cocloudTopic, 10, &OccupancyGridMapNode::coCloudCallback, this);

        //* Publisher 생성
        pubOctomap = nh.advertise<octomap_msgs::Octomap>("/navi/octomap", 1);
        pubOccupancyGrid = nh.advertise<nav_msgs::OccupancyGrid>("/navi/grid_map", 1);
        pubFilteredCloud = nh.advertise<sensor_msgs::PointCloud2>("/navi/ray_cloud", 1);
    }


    void process(){   
        ros::Rate rate(20); // 주기: 0.1 sec 

        while (ros::ok()){
            //* Parameter 불러오기
            if (correctionCloud){     

        
                pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*correctionCloud, *filterCloud);

                //* Crop Box filter 적용 (관심영역 필터)
                pcl::CropBox<pcl::PointXYZ> crop;
                crop.setInputCloud(filterCloud);

                //* 관심영역 선언
                Eigen::Vector4f boatMinPoint(boatXMin, boatYMin, boatZMin, 1.0);
                Eigen::Vector4f boatMaxPoint(boatXMax, boatZMax, boatZMax, 1.0);
                Eigen::Vector4f mapMinPoint(mapXMin, mapYMin, mapZMin, 1.0);
                Eigen::Vector4f mapMaxPoint(mapXMAX, mapYMAX, mapZMAX, 1.0);

                //* filter 적용
                crop.setMin(boatMinPoint);
                crop.setMax(boatMaxPoint);
                crop.setNegative(true); // 설정영역 데이터 삭제
                crop.filter(*filterCloud);
                crop.setMin(mapMinPoint);
                crop.setMax(mapMaxPoint);
                crop.setNegative(false); // 설정영역 외부 데이터 삭제
                crop.filter(*filterCloud);
                

                // //* SOR 필터링 적용
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorFilter;
                sorFilter.setInputCloud(filterCloud);
                sorFilter.setMeanK(sorMeanK); // 각 점에 대해 고려할 이웃 점 수
                sorFilter.setStddevMulThresh(sorStddevMulThresh); // 표준 편차의 임계값 설정
                sorFilter.filter(*filterCloud);
                
                //* Octomap parameter 설정
                octomap::OcTree octree(mapResolution); // map 해상도
                octree.setOccupancyThres(0.7);
                octree.setProbHit(0.6);
                octree.setProbMiss(0.34);
                octree.setClampingThresMin(0.12);
                octree.setClampingThresMax(0.97);

                //* OctoMap 업데이트
                for (const auto& point : filterCloud->points){
                    octomap::point3d endpoint(point.x, point.y, point.z);
                    octree.updateNode(endpoint, true);
                }
                octree.updateInnerOccupancy();
                octree.prune();

                //* 2D 점유 그리드 맵 설정
                nav_msgs::OccupancyGrid gridMapMsg;
                gridMapMsg.header = correctionCloud->header; 
                gridMapMsg.info.resolution = mapResolution; // map 해상도 설정
                gridMapMsg.info.width = (mapXMAX - mapXMin) / mapResolution; // map width 계산
                gridMapMsg.info.height = (mapYMAX - mapYMin) / mapResolution; // map height 계산
                gridMapMsg.info.origin.position.x = mapXMin; // map 원점 x 좌표 설정
                gridMapMsg.info.origin.position.y = mapYMin; // map 원점 y 좌표 설정
                gridMapMsg.info.origin.position.z = 0.0; // map 원점 z 좌표 설정
                gridMapMsg.data.resize(gridMapMsg.info.width * gridMapMsg.info.height, -1); // 맵 데이터 배열 초기화 (default: 미확인구간(-1))



                pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
                for (octomap::OcTree::leaf_iterator it = octree.begin_leafs(); it != octree.end_leafs(); ++it){
                    if (octree.isNodeOccupied(*it)){
                        double x = it.getX();
                        double y = it.getY();


                        int col = static_cast<int>((x - mapXMin) / mapResolution);
                        int row = static_cast<int>((y - mapYMin) / mapResolution);

                        // 점유된 공간 저장
                        gridMapMsg.data[col + row * gridMapMsg.info.width] = 100;

                    }
                }

                pcl::PointCloud<pcl::PointXYZ>::Ptr rayCloud(new pcl::PointCloud<pcl::PointXYZ>);
                //* 2D 점유 그리드 맵 이동 가능 공간 저장 (Ray Casting)
                double MaxViewRange = std::min(gridMapMsg.info.width / 2.0, gridMapMsg.info.height / 2.0); // 최대 시야 거리
                double fov = 2*M_PI;  // 시야각
                int rayNum = 360 * rayDensity; 
                double stepAngle = fov/rayNum;

                // Map 중심 위치
                double centerX = gridMapMsg.info.width / 2.0;
                double centerY = gridMapMsg.info.height / 2.0;

                double startAngle = 0.0 - fov/2.0;
                for (size_t i = 0; i < rayNum; i++) {
                    for (size_t range = 0; range < (MaxViewRange); range++) {

                        // ray 좌표 계산
                        double targetX = centerX + sin(startAngle) * range;
                        double targetY = centerY - cos(startAngle) * range;

                        // ray가 속하는 grid 계산
                        int col = static_cast<int>(targetX);
                        int row = static_cast<int>(targetY);

                        // 해당 grid 인덱스 계산
                        int rayIndex = static_cast<int>(col + row * gridMapMsg.info.width);

                        // 이동 가능 grid 계산
                        if (gridMapMsg.data[rayIndex] ==100 ) {
                            pcl::PointXYZ point;
                            point.x = sin(startAngle) * range*mapResolution;
                            point.y = - cos(startAngle) * range*mapResolution;
                            point.z = 0.0;
                            rayCloud->push_back(point);
                            break;
                        }
                        else if (gridMapMsg.data[rayIndex] == -1){
                            gridMapMsg.data[rayIndex] = 0;
                        }
                    }
                    startAngle = startAngle + stepAngle;
                }

                std_msgs::Header localHeader;
                localHeader.frame_id = "body";
                localHeader.stamp = ros::Time::now();

                //* OccupancyGrid 맵 퍼블리시
                gridMapMsg.header = localHeader;
                pubOccupancyGrid.publish(gridMapMsg);

                //* Octomap msg 퍼블리시
                octomap_msgs::Octomap octomapMsg;
                octomap_msgs::fullMapToMsg(octree, octomapMsg);
                octomapMsg.header = localHeader;
                pubOctomap.publish(octomapMsg);

                //* 보정된 Point Cloud msg 퍼블리시
                sensor_msgs::PointCloud2 rayCloudMsg;
                pcl::toROSMsg(*rayCloud, rayCloudMsg);
                rayCloudMsg.header = localHeader;
                pubFilteredCloud.publish(rayCloudMsg);
                
            }

            //* Callback Data 초기화
            correctionCloud.reset();

            //* Rate
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;

    ros::Subscriber subcoCloud;
    ros::Publisher pubFilteredCloud;
    ros::Publisher pubOctomap;
    ros::Publisher pubOccupancyGrid;

    sensor_msgs::PointCloud2::ConstPtr correctionCloud;
    nav_msgs::Odometry::ConstPtr odomData; 

    //* parameter
    // map 정보
    double mapXMin = -5.0 ;
    double mapYMin = -5.0 ;
    double mapZMin = -0.2;
    double mapXMAX = 5.0;
    double mapYMAX = 5.0;
    double mapZMAX = 1.0;
    double mapResolution = 0.1  ;
    double rayDensity  = 4 ;
    
    // 관심 영역
    double boatXMin =-0.9;
    double boatYMin =-0.35;
    double boatZMin =-0.5;
    double boatXMax =0.3;
    double boatYMax =0.35;
    double boatZMax =0.5;

    // 노이즈 제거 
    double sorMeanK =10;
    double sorStddevMulThresh = 30 ;

    void coCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

        correctionCloud = msg;
    }
    ///////////////////////////

};

int main(int argc, char** argv){
    ros::init(argc, argv, "scan_tool");
    OccupancyGridMapNode node;
    try {
    node.process();
    } catch (const std::exception& e) {
        ROS_ERROR("An exception occurred: %s", e.what());
    } catch (...) {
        ROS_ERROR("An unknown exception occurred.");
    }
    return 0;
}