#ifndef LAB0_POINTCLOUD_H
#define LAB0_POINTCLOUD_H

#include "ImageRGBD.h"
#include <Eigen/SVD>

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
        void create(const Mat &image, const Mat &depth_image);
        void getPoints(vector< pair <Point3d,Vec3b>> &points);
        void clearPoints();
        int getImageId(const ImageRGBD &image);
        Mat rotationMatrix(const Vec3d &degree);
        void rotate(const Mat &rotation_mat);
        void findCorrespondingPoints(const Mat &l_frame_rgb, const Mat &r_frame_rgb, const Mat &l_frame_rgbd, const Mat &r_frame_rgbd, vector< pair <Point3d,Vec3b>> &l_points, vector< pair <Point3d,Vec3b>> &r_points);
//        void findAdjacentPoints(const Mat &l_frame_rgb, const Mat &l_frame_rgbd);
        pair<Point3d,Vec3b> convertTo3d(const Mat &image, const Mat &depth_image, Point2d &point);
        void validate(int &top_col, int &top_row, int &widht, int &height);
//        void zeroPad(const Mat &image, const double size, Mat &new_image);
        pair<Eigen::Matrix3d, Eigen::Vector3d> computeRigidTransform(const vector<Point3d> &l_points, const vector<Point3d> &r_points);
        void convertToEigenMat(const vector<Point3d> &l_points, const vector<Point3d> &r_points, Eigen::MatrixXd &l_mat, Eigen::MatrixXd &r_mat);
        void convertToEigenMat(const vector<Point3d> points, Eigen::MatrixXd &mat);
        void convertToVector(const Eigen::MatrixXd &mat, vector<Point3d> &points);
        Eigen::Vector3d convertToEigenVector3d(const Point3d &point);
        void tranformPoints(pair<Eigen::Matrix3d, Eigen::Vector3d> &R_t, vector<Point3d> &points);

    friend class surfaceReconstruction;
};

#endif //LAB0_POINTCLOUD_H