//
// Created by lamb97 on 23-11-20.
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <eigen3/Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
using namespace cv;
using namespace std;
cv::Point2f calculateCenter(const std::vector<cv::Point2f>& points) {
    cv::Point2f center1(0, 0);
    for (const auto& point : points) {
        center1 += point;
    }
    center1 /= (float)points.size();
    return center1;
}

typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;

//pnp ba优化
void bundleAdjustmentGaussNewton(
        const VecVector3d &points_3d,
        const VecVector2d &points_2d,
        const Mat &K,
        Sophus::SE3d &pose)
{  //传入空间点点，空间点像素坐标，内参; 初始pose,此程序中初始pose为0;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    const int iterations = 10;
    double cost = 0, lastCost = 0;
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    for (int iter = 0; iter < iterations; iter++) {  //进行迭代
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        cost = 0;
        // compute cost
        for (int i = 0; i < points_3d.size(); i++) {
            Eigen::Vector3d pc = pose * points_3d[i];//得到空间点在相机坐标系下的坐标
            // Vector6d se3 = pose.log();
            // cout<<"se3 = "<<se3.transpose()<<endl;一开始se3为0,三四次迭代后趋于稳定
            double inv_z = 1.0 / pc[2];
            double inv_z2 = inv_z * inv_z;
            Eigen::Vector2d proj (fx * pc[0] / pc[2] + cx, fy * pc[1] / pc[2] + cy);//P点像素坐标
            //的投影计算值
            Eigen::Vector2d e = points_2d[i] - proj;//作差，得到差值;是观测值-预测值！
            cost += e.squaredNorm();//误差的二范数的平方
            Eigen::Matrix<double, 2, 6> J;//2*6的雅克比矩阵，即误差相对于位姿求导，通过导数可以知道有了误
            //差以后我们应该往哪个方向去优化
            J << -fx * inv_z,
                    0,
                    fx * pc[0] * inv_z2,
                    fx * pc[0] * pc[1] * inv_z2,
                    -fx - fx * pc[0] * pc[0] * inv_z2,
                    fx * pc[1] * inv_z,
                    0,
                    -fy * inv_z,
                    fy * pc[1] * inv_z,
                    fy + fy * pc[1] * pc[1] * inv_z2,
                    -fy * pc[0] * pc[1] * inv_z2,
                    -fy * pc[0] * inv_z;
            H += J.transpose() * J;//高斯牛顿方法，H*dx=b
            b += -J.transpose() * e;
        }
        Vector6d dx;
        dx = H.ldlt().solve(b);//对H做LDLT分解，并求解dx
        cout <<"dx:" << endl << dx <<endl;
        cout <<"dx.norm():" << endl << dx.norm() <<endl;
        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }
        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good//发散的情况
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }
        // update your estimation
        pose = Sophus::SE3d::exp(dx) * pose;
        lastCost = cost;
        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
        if (dx.norm() < 1e-6) {
            // converge 当dx<1e-6时停止迭代
            break;
        }
    }
    cout << "pose by g-n: \n" << pose.matrix() << endl;
}

//基于最小二乘法求相机位姿
Eigen::Affine3d estimatePose(const std::vector<Eigen::Vector3d>& objectPoints,
                             const std::vector<Eigen::Vector2d>& imagePoints,
                             const Eigen::Matrix3d& cameraMatrix) {
    // 最小二乘法求解相机姿态
    assert(objectPoints.size() == imagePoints.size());
    int nPoints = objectPoints.size();
    assert(nPoints >= 3);
    Eigen::Matrix<double, Eigen::Dynamic, 6> A(nPoints, 6);
    Eigen::Matrix<double, Eigen::Dynamic, 1> B(nPoints);
    for (int i = 0; i < nPoints; i++) {
        const Eigen::Vector3d& P = objectPoints[i];
        const Eigen::Vector2d& p = imagePoints[i];
        double u = p.x();
        double v = p.y();
        A(i, 0) = P.x() / P.z(); // fx*rx1 (fx is the camera focal length)
        A(i, 1) = P.y() / P.z(); // fy*rx2 (fy is the camera focal length)
        A(i, 2) = 1 / P.z();     // rx3
        A(i, 3) = 0;             // tx1
        A(i, 4) = 0;             // tx2
        A(i, 5) = 0;             // tx3
        B(i) = u;
        A(i + nPoints, 0) = 0;
        A(i + nPoints, 1) = 0;
        A(i + nPoints, 2) = 1 / P.z();
        A(i + nPoints, 3) = -P.x() / P.z() * u; // (-fx*rx1)*u (fx is the camera focal length)
        A(i + nPoints, 4) = -P.y() / P.z() * u; // (-fy*rx2)*u (fy is the camera focal length)
        A(i + nPoints, 5) = -u;                 // -u
        B(i + nPoints) = v;
    }
    Eigen::Matrix<double, 6, 1> X = A.colPivHouseholderQr().solve(B);
    Eigen::Matrix<double, 3, 3> R;
    R(0, 0) = X(0);
    R(1, 1) = X(1);
    R(2, 2) = 1.0;
    R(0, 1) = X(2);
    R(1, 0) = -X(2);
    R(0, 2) = -X(1) * X(5);
    R(1, 2) = X(0) * X(5);
    R(2, 0) = X(1);
    R(2, 1) = -X(0);
    Eigen::Affine3d pose;
    pose.linear() = R;
    pose.translation() = Eigen::Vector3d(X(3), X(4), X(5));
    return pose;
}

class param{
public:
    //计算目标兑换站顶点的世界坐标
    std::vector<cv::Point3f>worldPoints;

    //定义目标兑换站顶点的像素坐标
    std::vector<cv::Point2f> imagePoints;
   //定义相机参数
    cv::Mat cameraMatrix ;
    cv::Mat disCoeffs ;
};
//兑换槽姿态解算函数
int main(int argc, char *argv[]){


    cv::VideoCapture cap(2); // 打开默认相机

    // 检查相机是否成功打开
    if (!cap.isOpened()) {
        std::cout << "无法打开相机" << std::endl;
        return -1;
    }

    // 设置视频编码器和输出文件名
    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    std::string filename = "output.avi";

    // 获取相机的帧宽度和帧高度
    int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

    // 创建一个 VideoWriter 对象，用于写入输出视频文件
    cv::VideoWriter writer(filename, codec, 30, cv::Size(frame_width, frame_height));

    // 检查 VideoWriter 是否成功创建
    if (!writer.isOpened()) {
        std::cout << "无法创建输出视频文件" << std::endl;
        return -1;
    }
    cv::Mat src;
    // Default exposure value
    int exposure = 10;
    // 开始录制视频
    while (1) {

        cap.read(src);
        cap >> src;
        // Set exposure
        cap.set(cv::CAP_PROP_EXPOSURE, exposure);
        std::vector<Mat> channels;
        Mat channel_merge, binary;
        // 分离通道
        split(src, channels);
        // 通道叠加
        add(channels[0], channels[1], channel_merge);
        // 二值化处理
        threshold(channel_merge, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
        cv::imshow("1",binary);
//        cv::Mat dilateElement = cv::getStructuringElement(cv::MORPH_CROSS,
//                                                      cv::Size(3, 3),
//                                                      cv::Point(-1, -1));
//        cv::dilate(binary,
//               binary,
//               dilateElement,
//               cv::Point(-1, -1),
//               1);
//        cv::imshow("2", binary);

        // 轮廓检测
        std::vector<std::vector<Point>> contours;
        std::vector<Vec4i> hierarchy;
        cv::findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);



        // 设定初步筛选条件
        double minArea = 310;
        double maxArea = 2500;
        double minAspectRatio = 3;
        double maxAspectRatio = 1 / 4.5;
        int minEdges = 5;
        int maxEdges = 8;
        // 初步筛选3
        std::vector<std::vector<Point>> selectedContours;
        for (const auto &contour: contours)
        {
            double area = contourArea(contour);
            Rect bounding_Rect = boundingRect(contour);
            double aspectRatio = static_cast<double>(bounding_Rect.height) / bounding_Rect.width;
            double invAspectRatio = static_cast<double>(bounding_Rect.width) / bounding_Rect.height;

            std::vector<cv::Point> approxCurve;
            approxPolyDP(contour, approxCurve, cv::arcLength(contour, true) * 0.02, true);
            int edges = approxCurve.size();
            if (area > minArea && area < maxArea
                && aspectRatio < minAspectRatio && invAspectRatio < minAspectRatio
                && edges > minEdges && edges < maxEdges
                    )
            {
                selectedContours.push_back(contour);
            }
        }
        // 设定二次筛选条件
        int numContours = 4;
        double maxMinAreaRatio = 10.0;
        // 二次筛选
        std::vector<std::vector<Point>> finalContours;
        double minSize = std::numeric_limits<double>::max();
        double maxSize = 0;
        for (const auto &contour: selectedContours) {
            double area = contourArea(contour);
            if (area < minSize)
                minSize = area;
            if (area > maxSize)
                maxSize = area;
            finalContours.push_back(contour);
        }
        if (finalContours.size() == numContours && maxSize < maxMinAreaRatio * minSize) {
            // 符合筛选条件，进行后续操作

        }
        drawContours(src, finalContours, -1, Scalar(0, 255, 0), 2);
        cv::imshow("相机1", src);

        Point2f center;
        std::vector<cv::Point2f> centers;
        float radius;
        // 遍历轮廓
        vector<Point2f> finalVertices;




        // 如果没有找到符合角点顶点条件的轮廓 计算各顶点到中点的距离并找到最远者作为轮廓顶点
        for (const auto &finalContour: finalContours) {
            // 对轮廓进行三角形拟合
            vector<Point> approx;
            approxPolyDP(finalContour, approx, arcLength(finalContour, true) * 0.02, true);
            minEnclosingCircle(approx, center, radius);
            centers.push_back(center);
        }
        //定义中心点
        Point finalcenter = calculateCenter(centers);
        cv::circle(src,finalcenter,5,Scalar(255,0,0));


        Point farthestVertex;
        for (const auto &finalContour: finalContours)
        {
            double maxDist = 0;
            // 对轮廓进行三角形拟合
            vector<Point> approx;
            approxPolyDP(finalContour, approx, arcLength(finalContour, true) * 0.02, true);
            for (const auto &approx1: approx)
            {
                double dist = norm(approx1 - finalcenter);
                if (dist > maxDist)
                {
                    maxDist = dist;
                    farthestVertex = approx1;
                }
            }
            finalVertices.push_back(farthestVertex);
        }
        for (const auto &finalContour: finalContours)
        {
            // 对轮廓进行三角形拟合
            vector<Point> approx;
            approxPolyDP(finalContour, approx, arcLength(finalContour, true) * 0.02, true);

            // 判断顶点数量并计算最大角度
            if (approx.size() == 3)
            {
                Point vertex1 = approx[0];
                Point vertex2 = approx[1];
                Point vertex3 = approx[2];
                double dist1 = norm(vertex2 - vertex3);
                double dist2 = norm(vertex1 - vertex3);
                double dist3 = norm(vertex1 - vertex2);
                double angle = acos((dist1 * dist1 + dist2 * dist2 - dist3 * dist3) / (2 * dist1 * dist2));  // 根据余弦定理计算夹角
                angle = angle * 180 / CV_PI;  // 弧度转换为角度
                // 判断是否符合角点顶点条件
                if (angle > 130)
                {
                    //如果三角形最大角大于120度，则最大角顶点设为最后顶点
                    double maxDist = max(dist1, max(dist2, dist3));
                    Point vertex = dist1 == maxDist ? vertex1 : (dist2 == maxDist ? vertex2 : vertex3);
                    finalVertices.push_back(vertex);
                    break;
                }
            }
        }
        for (int i = 0; i < finalVertices.size(); i++)
        {
            circle(src, finalVertices[i], 5, Scalar(255, 0, 0), -1);

        }
        int minArea1 = 40;
        int maxArea1 = 300 ;
        int aspectRatio1 = 2;
        //寻找0号角点
        Point2f zeroVertex;
        std::vector<std::vector<Point>> selecteRectdContours ;
        for (const auto &contour: contours)
        {
            double area = contourArea(contour);
            Rect bounding_Rect1 = boundingRect(contour);

            double aspectRatio = static_cast<double>(bounding_Rect1.height) / bounding_Rect1.width;
            double invAspectRatio = static_cast<double>(bounding_Rect1.width) / bounding_Rect1.height;
            if (
                    area > minArea1 && area < maxArea1
                    && aspectRatio < aspectRatio1 && invAspectRatio < aspectRatio1
                )
            {
                selecteRectdContours.push_back(contour);
            }
        }
        drawContours(src, selecteRectdContours, -1, Scalar(0, 255, 0), 2);
        Point2f centerRectPoint;
        if (selecteRectdContours.size() == 1)
        {
            for (const auto &selecteRectdContour:selecteRectdContours)
            {
                // 计算小方块中点坐标
                Rect bounding_Rect2 = boundingRect(selecteRectdContour);
                float centerRectX = bounding_Rect2.x + bounding_Rect2.width / 2.0;
                float centerRectY = bounding_Rect2.y + bounding_Rect2.height / 2.0;
                Point centerRectPoint(centerRectX, centerRectY);
            }
            for (const auto &finalVertex: finalVertices)
            {

                double minDist = 1000000;
                double dist = norm(finalVertex - centerRectPoint);
                if (dist < minDist)
                {
                    minDist = dist;
                    zeroVertex = finalVertex;
                }
            }
        }
        if (selecteRectdContours.size() == 2)
        {
            for (const auto &selecteRectdContour:selecteRectdContours)
            {
                // 计算小方块中点坐标
                Rect bounding_Rect2 = boundingRect(selecteRectdContour);
                float centerRectX = bounding_Rect2.x + bounding_Rect2.width / 2.0;
                float centerRectY = bounding_Rect2.y + bounding_Rect2.height / 2.0;
                centerRectPoint.x += centerRectX / 2.0;
                centerRectPoint.y += centerRectY / 2.0;
            }
            double mindist = numeric_limits<double>::max();
            for (const auto &finalVertex: finalVertices)
            {


                double dist = norm(finalVertex - centerRectPoint);
                if (dist < mindist)
                {
                    mindist = dist;
                    zeroVertex = finalVertex;
                }
            }
        }
        if( selecteRectdContours.size() == 0)
        {
            double minArea = 1000000;
            for (const auto &finalContour: finalContours)
            {
                double area = contourArea(finalContour);
                if (minArea > area)
                {
                    minArea = area;
//                    zeroVertex =
                }
            }
        }

        circle(src, zeroVertex, 2, Scalar(0, 0, 255), -1);
        //重新排序角点

            int n = 0;
            for (int i = 0; i < finalVertices.size(); i++) {
                n++;
                if (finalVertices[i] == zeroVertex) {
                    break;
                }
            }

        //寻找二号角点
        Point2f twoVertex ;
        int max = 0;
        for (const auto &finalVertex: finalVertices)
        {
            double dist = norm(finalVertex - zeroVertex);
            if(dist > max)
            {
                max = dist;
                twoVertex = finalVertex;
            }
        }
        int m = 0;
        for (int i = 0; i < finalVertices.size(); i++) {
            m++;
            if (finalVertices[i] == zeroVertex) {
                break;
            }
        }

        Point2f oneVertex;
        Point2f threeVertex;
        if(zeroVertex.x > 280){

            for (const auto &finalVertex: finalVertices) {
                if (finalVertex !=zeroVertex && finalVertex != twoVertex){
                    if(finalVertex.y <= 200){
                        oneVertex = finalVertex;
                    }
                    if(finalVertex.y >= 200 ){
                        threeVertex = finalVertex;
                    }
                }

            }
        }
        if(zeroVertex.x < 280){

            for (const auto &finalVertex: finalVertices) {
                if (finalVertex !=zeroVertex && finalVertex != twoVertex){
                    if(finalVertex.y >= 200){
                        oneVertex = finalVertex;
                    }
                    if(finalVertex.y <= 200){
                        threeVertex = finalVertex;
                    }
                }

            }
        }





        // 创建 Param 并进行初始化
        param param;
        param.imagePoints ={zeroVertex,
                            oneVertex,
                            twoVertex,
                            threeVertex};
        param.cameraMatrix = (cv::Mat_<double>(3, 3) << 742.916347167730, 0	,979.856079304276, 0	, 742.664623576404	,542.610372269964,0,0 ,1);
        param.disCoeffs = (cv::Mat_<double>(1, 5) <<0.00832137649841396	,-0.0390820889767429	,0.00385697116730112, -0.000560432716814933,	-0.00148596384799744);
        param.worldPoints ={
                cv::Point3f(138, -138, 0),
                cv::Point3f(-138, -138, 0),
                cv::Point3f(-138, 138, 0),
                cv::Point3f(138, 138, 0)
        };


        std::cout<<"角点坐标0"<<zeroVertex<<std::endl;
        std::cout<<"顶点坐标"<<finalVertices<<std::endl;



        std::cout<<"角点坐标0="<<param.imagePoints[0]<<std::endl;
        std::cout<<"角点坐标1="<<param.imagePoints[1]<<std::endl;
        std::cout<<"角点坐标2="<<param.imagePoints[2]<<std::endl;
        std::cout<<"角点坐标3="<<param.imagePoints[3]<<std::endl;

        std::cout<<"0="<<param.worldPoints[0]<<std::endl;
        std::cout<<"1="<<param.worldPoints[1]<<std::endl;
        std::cout<<"2="<<param.worldPoints[2]<<std::endl;
        std::cout<<"3="<<param.worldPoints[3]<<std::endl;


//        //ba求位姿
//
//
//        // 将输入数据转换为 Eigen 格式
//        typedef Eigen::Vector2d Vector2d;
//        typedef Eigen::Vector3d Vector3d;
//        typedef std::vector<Vector2d, Eigen::aligned_allocator<Vector2d>> VecVector2d;
//        typedef std::vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
//
//        VecVector3d points_3d;
//        VecVector2d points_2d;
//
//        for (size_t i = 0; i < param.worldPoints.size(); ++i) {
//            points_3d.push_back(Vector3d(param.worldPoints[i].x, param.worldPoints[i].y, param.worldPoints[i].z));
//            points_2d.push_back(Vector2d(param.imagePoints[i].x, param.imagePoints[i].y));
//        }
//
//        // 调用函数进行优化
//        Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity(); // 初始化单位旋转矩阵
//        Eigen::Vector3d t = Eigen::Vector3d::Zero(); // 初始化零平移向量
//        Sophus::SE3d pose(R1, t); // 初始化位姿
//
//        bundleAdjustmentGaussNewton(points_3d, points_2d, param.cameraMatrix, pose);
//
//        // 输出优化之后的相机姿态
//        std::cout << "Optimized pose:\n" << pose.matrix() << std::endl;
//
//
//        //最小二乘求位姿
//
//
//        // 将物体点集转换为Mat类型
//        cv::Mat objectPointsMat(param.worldPoints.size(), 3, CV_64FC1);
//        for (int i = 0; i < param.worldPoints.size(); ++i) {
//            objectPointsMat.at<double>(i, 0) = param.worldPoints[i].x;
//            objectPointsMat.at<double>(i, 1) = param.worldPoints[i].y;
//            objectPointsMat.at<double>(i, 2) = param.worldPoints[i].z;
//        }
//
//        // 将图像点集转换为Mat类型
//        cv::Mat imagePointsMat(param.imagePoints.size(), 2, CV_64FC1);
//        for (int i = 0; i < param.imagePoints.size(); ++i) {
//            imagePointsMat.at<double>(i, 0) = param.imagePoints[i].x;
//            imagePointsMat.at<double>(i, 1) = param.imagePoints[i].y;
//        }

//        // 调用函数求解相机位姿
//        cv::Mat pose = estimatePose(objectPointsMat, imagePointsMat, param.cameraMatrix);


        //solvepnp求位姿


        // 计算旋转向量和平移向量
        cv::Mat rotation(3, 1, CV_32F);
        cv::Mat translation(3, 1, CV_32F);
        cv::solvePnP(param.worldPoints,
                             param.imagePoints,
                             param.cameraMatrix,//相机内参矩阵
                             param.disCoeffs,//畸变矩阵
                             rotation,
                             translation,
                             false,
                             SOLVEPNP_EPNP
                             );
        //eigen库转欧拉角
        cv::Mat rotationMat(3,3,CV_32F);
        cv::Rodrigues(rotation, rotationMat);
        Eigen::Matrix<double, 3, 3> rotationMatrix;
        cv::cv2eigen(rotationMat,rotationMatrix);
        Eigen::Vector3d euler_t = rotationMatrix.eulerAngles(0,1,2);

        // 从旋转矩阵中提取欧拉角
        Vec3d euler_angles;
        Mat rotationMatInv = rotationMat.inv();
        euler_angles[1] = asin(rotationMatInv.at<double>(0, 2)); // pitch
        euler_angles[0] = atan2(-rotationMatInv.at<double>(1, 2), rotationMatInv.at<double>(2, 2)); // yaw
        euler_angles[2] = atan2(-rotationMatInv.at<double>(0, 1), rotationMatInv.at<double>(0, 0)); // roll


        float distance = std::sqrt(translation.at<float>(0, 0) * translation.at<float>(0, 0)
                                   + translation.at<float>(1, 0) * translation.at<float>(1, 0)
                                   + translation.at<float>(2, 0) * translation.at<float>(2, 0));

        double distance1 = norm(translation);
        //四元数
        Eigen::Quaterniond quaternion(rotationMatrix);
//       std::cout<<"Yaw = "<<euler_t(0)*57.3<<"Pitch = "<<euler_t(1)*57.3<<"Roll = "<<euler_t(2)*57.3<<std::endl;
        // 将旋转向量转换为旋转矩阵
        cv::Mat R;
        //R = (cv::Mat_<double>(3, 3) << 0.866,0.433,0.25,0,0.5,-0.866,-0.5,0.75,0.433);
        R = (cv::Mat_<double>(3, 3) << 1,0,0,0,1,0,0,0,1);
        cv::Rodrigues(rotation,R);
//        Eigen::Vector3d euler_t = R.eulerAngles(2,1,0);


       // 以下是手动计算欧拉角
        double sy = std::sqrt(R.at<double>(0,0) * R.at<double>(0,0) + R.at<double>(1,0) * R.at<double>(1,0));
        bool singular = sy < 1e-6; // 如果这个值很小，可以认为是奇异的

        double x, y, z;
        if (!singular)
        {
            y = std::atan2(-R.at<double>(2,0), sy);
            x = std::atan2(R.at<double>(2,1), R.at<double>(2,2));

            z = std::atan2(R.at<double>(1,0), R.at<double>(0,0));
        }
        else
        {
            x = std::atan2(-R.at<double>(1,2), R.at<double>(1,1));
            y = std::atan2(-R.at<double>(2,0), sy);
            z = 0;
        }

        // 将弧度转换为度
        x = x * 180.0 / CV_PI;
        y = y * 180.0 / CV_PI;
        z = z * 180.0 / CV_PI;

//        // 输出欧拉角
//        std::cout << "Euler angles : "
//                  << "x = " << x << ", "<<std::endl
//                  << "y = " << y << ", "<<std::endl
//                  << "z = " << z << std::endl
//                  <<"distance = "<<distance<<std::endl;

        // 将旋转和平移变量转换为字符串
        stringstream ss_yaw, ss_pitch ,ss_roll,ss_dis;
        stringstream ss_x, ss_y ,ss_z;

        //输出手动计算欧拉角
        ss_x << "x1 = " << x;
        ss_y << "y1 = " << y;
        ss_z <<"z1 = "<<z;

        //输出eigen库转欧拉角
        ss_yaw << "x = " << euler_t(0)*57.3;
        ss_pitch << "y = " <<euler_t(1)*57.3;
        ss_roll<<"z = "<<euler_t(2)*57.3;
//        ss_yaw << "yaw = " << euler_angles[0];
//        ss_pitch << "pitch = " <<euler_angles[1];
//        ss_roll<<"roll = "<<euler_angles[2];
        ss_dis<<"dis= "<<distance;

        string text_yaw = ss_yaw.str();
        string text_pitch = ss_pitch.str();
        string text_roll = ss_roll.str();
        string text_x = ss_x.str();
        string text_y = ss_y.str();
        string text_z = ss_z.str();
        string text_dis = ss_dis.str();
        // 设置文字参数
        Point textOrg_yaw(25, 50);
        Point textOrg_pitch(25, 100);
        Point textOrg_roll(25, 150);
        Point textOrg_dis(25, 200);
        Point textOrg_x(450, 50);
        Point textOrg_y(450, 100);
        Point textOrg_z(450, 150);


        int fontFace = FONT_HERSHEY_SIMPLEX; // 字体样式
        double fontScale = 0.6; // 字体缩放因子
        Scalar color(255, 255, 255); // 字体颜色 (白色)
        int thickness = 2; // 线宽
        // 在图像上绘制文字
        putText(src, text_yaw, textOrg_yaw, fontFace, fontScale, color, thickness);
        putText(src, text_pitch, textOrg_pitch, fontFace, fontScale, color, thickness);
        putText(src, text_roll, textOrg_roll, fontFace, fontScale, color, thickness);
        putText(src, text_x, textOrg_x, fontFace, fontScale, color, thickness);
        putText(src, text_y, textOrg_y, fontFace, fontScale, color, thickness);
        putText(src, text_z, textOrg_z, fontFace, fontScale, color, thickness);
        putText(src, text_dis, textOrg_dis, fontFace, fontScale, color, thickness);
        // 写入帧到输出视频文件
        writer.write(src);
        // 在窗口中显示实时视频
        cv::imshow("相机", src);
//        std::cout<<"rotationMatrix = "<<rotationMatrix<<std::endl;
//        std::cout<<"rotation = "<<rotation<<std::endl;
//        std::cout<<"translation = "<<translation<<std::endl;
//        //输出四元数
//        std::cout << "quaternion Real part: " << quaternion.w() << std::endl;
//        std::cout << "quaternion Imaginary part: [" << quaternion.x() << ", " << quaternion.y() << ", " << quaternion.z() << "]" << std::endl;

        char key = cv::waitKey(1);
        if (key == 27) { // Press ESC to exit the loop
            break;
        }
        if (key == 103)
        {
            if (exposure == 200)
            {
                exposure = 10;
                std::cout << "曝光值设置为 10" << std::endl;
            }
            else
            {
                exposure = 200;
                std::cout << "曝光值设置为 200" << std::endl;
            }
        }


    }

    // 释放资源
    cap.release();
    writer.release();
    cv::destroyAllWindows();





     return 0;
     }
