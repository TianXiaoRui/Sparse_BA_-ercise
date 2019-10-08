#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "sophus/se3.h"

using namespace std;
using namespace Eigen;
using namespace cv;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;
/**
 *　给定的相机内参
 */
double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;

void getdata (
        VecVector2d& p2d,
        VecVector3d& p3d
);


int main ( int argc, char** argv ) {
    //生成3D点和像素坐标点
    VecVector2d p2d;
    VecVector3d p3d;
    Sophus::SE3 T_esti; // estimated pose   相机位姿R,t李代数形式
/**
 * 开始代码
 */





/**
 * 结束代码
 */
}

void getdata (
        VecVector2d& p2d,
        VecVector3d& p3d
){
    //-- 读取图像
    Mat img_1 = imread ( "", CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( "", CV_LOAD_IMAGE_COLOR );

    //初始化
    vector<KeyPoint> keypoints_1;
    vector<KeyPoint> keypoints_2;
    vector< DMatch > matches;
    /**
     *两帧之间的匹配，可以使用ＯＲＢ　ＳＩＦＴ等
     */
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;
    Mat d1 = imread ( "./data/1_depth.png", CV_LOAD_IMAGE_UNCHANGED );
    for ( DMatch m:matches )
    {
        ushort d = d1.ptr<unsigned short> (int ( keypoints_1[m.queryIdx].pt.y )) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        if ( d == 0 )   // bad depth
            continue;
        float z = d/5000.0;
        float x = ( keypoints_1[m.queryIdx].pt.x - cx )* z / fx;
        float y = ( keypoints_1[m.queryIdx].pt.y - cx ) * z / fy;
        p3d.push_back ( Vector3d( x, y, z) );
        p2d.push_back ( Vector2d( keypoints_2[m.trainIdx].pt.x, keypoints_2[m.trainIdx].pt.y) );
    }

}

