#ifndef LAB0_POINTCLOUD_H
#define LAB0_POINTCLOUD_H

#include "ImageRGBD.h"
#include "kdTree.h"
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
//    double mm_per_m{1000};
    double mm_per_m{50.0};
    double constant{570.3};
};

class PointCloud {
    private:
        vector< pair <Point3d,Vec3b>> m_points;

    public:
        PointCloud() {};

    private:
        void create(const Mat &image, const Mat &depth_image);
        void clearPoints();
        Mat rotationMatrix(const Vec3d &degree);
        void rotate(const Mat &rotation_mat);
        pair<Eigen::Matrix3d, Eigen::Vector3d> computeRigidTransform(const vector<Point3d> &l_points, const vector<Point3d> &r_points);
        void tranformPoints(pair<Eigen::Matrix3d, Eigen::Vector3d> &R_t, vector<Point3d> &points);
        void kNearest(const vector<Point3d> &src, const vector<Point3d> &dst, vector<Point3d> &nearestPoints, int kn);
        double computeError(const vector<Point3d> &src, const vector<Point3d> &dst);

        void getPoints(vector< pair <Point3d,Vec3b>> &points);
        int getImageId(const ImageRGBD &image);

        pair<Point3d,Vec3b> convertTo3d(const Mat &image, const Mat &depth_image, Point2d &point);
        void convertToEigenMat(const vector<Point3d> &l_points, const vector<Point3d> &r_points, Eigen::MatrixXd &l_mat, Eigen::MatrixXd &r_mat);
        void convertToEigenMat(const vector<Point3d> &points, Eigen::MatrixXd &mat);
        void convertToVector(const Eigen::MatrixXd &mat, vector<Point3d> &points);
        Eigen::Vector3d convertToEigenVector3d(const Point3d &point);
        Point3d convertToPoint3d(const vec &point);

//        void findCorrespondingPoints(const Mat &l_frame_rgb, const Mat &r_frame_rgb, const Mat &l_frame_rgbd, const Mat &r_frame_rgbd, vector< pair <Point3d,Vec3b>> &l_points, vector< pair <Point3d,Vec3b>> &r_points);
//        void findAdjacentPoints(const Mat &l_frame_rgb, const Mat &l_frame_rgbd);
//        void validate(int &top_col, int &top_row, int &widht, int &height);

    friend class surfaceReconstruction;
};

#endif //LAB0_POINTCLOUD_H