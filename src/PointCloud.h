#ifndef LAB0_POINTCLOUD_H
#define LAB0_POINTCLOUD_H

#include "ImageRGBD.h"
#include "ImageRGB.h"
#include "KDTree.h"
#include "KDTree2.h"
#include <VVRScene/mesh.h>
#include <VVRScene/utils.h>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <random>

using namespace vvr;

struct CameraConstants {
    int topLeft[2]{1,1}; // the position of the top-left corner of depth in the original depth image
    int center[2]{320, 240};
    int image_height = 480;
    int image_width = 640;

//    only for the test folder
//    int image_height = 234;
//    int image_width = 260;
//    only for the test folder

//    float mm_per_m{50.0f};
    float mm_per_m{50.0f};
    float constant{570.3f};
};

class PointCloud {
    public:
        PointCloud() {};

    public:
        void create(const Mat &image, const Mat &depth_image);
        void clearPoints();
        Mat rotationMatrix(const Vec3d &degree);
        void rotate(const Mat &rotation_mat);
        pair<Eigen::Matrix3f, Eigen::Vector3f> computeRigidTransform(const VecArray &l_points, const VecArray &r_points);
//        pair<Eigen::Matrix3f, Eigen::Vector3f> computeRigidTransform(const VecArray4 &src4d, const VecArray4 &dst4d);
        void transformPoints(pair<Eigen::Matrix3f, Eigen::Vector3f> &R_t, VecArray &points);
//        void transformPoints(pair<Eigen::Matrix3f, Eigen::Vector3f> &R_t, VecArray4 &points);
        void kNearest(const VecArray &src, VecArray &nearestPoints, vector<float> &dist, int kn);
//        void kNearest(const VecArray4 &src, VecArray4 &nearestPoints, vector<float> &dist, int kn, float &weight);
        vec getCentroid(const VecArray &src);

        void getPoints(vector< pair <vec,Vec3b>> &points);
        int getRgbdId(const ImageRGBD &image);

        void sobel(const Mat &img, Mat &new_img);
        void laplacian(const Mat &img, Mat &new_img);
        void bilateral(const Mat &img, Mat &new_img);
        void cannyThreshold(const Mat &img, Mat &new_img);
        void thresholding(const Mat &img, Mat &new_img);
        void getPixels(const Mat &img, VecArray &edges, const int &value);
        pair<Eigen::Matrix3f, Eigen::Vector3f> icp(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations);

        float vectorSum(const vector<float> &v);
        void normalize(vector<float> &values);
        float min(const vector<float> &values);
        float max(const vector<float> &values);

        void edgeDetection(const Mat &img, const Mat &depth_img);
        void computeNormals(const Mat &img, const Mat &depth_img, Mat &normals);

//        pair<Point3d,Vec3b> convertTo3d(const Mat &image, const Mat &depth_image, Point2d &point);
//        void findCorrespondingPoints(const Mat &l_frame_rgb, const Mat &r_frame_rgb, const Mat &l_frame_rgbd, const Mat &r_frame_rgbd, vector< pair <Point3d,Vec3b>> &l_points, vector< pair <Point3d,Vec3b>> &r_points);
//        void findAdjacentPoints(const Mat &l_frame_rgb, const Mat &l_frame_rgbd);
//        void validate(int &top_col, int &top_row, int &widht, int &height);

    public:
        vector< pair <vec,Vec3b>> m_points;
        KDTree *m_dst_KDTree;
//        KDTree2 *m_dst_KDTree;
};

#endif //LAB0_POINTCLOUD_H