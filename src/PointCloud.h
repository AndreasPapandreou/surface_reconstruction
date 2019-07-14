#ifndef LAB0_POINTCLOUD_H
#define LAB0_POINTCLOUD_H

#include "ImageRGBD.h"
#include "ImageRGB.h"
#include "KDTree.h"
#include <VVRScene/mesh.h>
#include <VVRScene/utils.h>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <random>
#include <bits/stdc++.h>

using namespace vvr;

/// define that constant variables about the images of the dataset
struct CameraConstants {
    int topLeft[2]{1,1}; // the position of the top-left corner of depth in the original depth image
    int center[2]{320, 240};
    int image_height = 480;
    int image_width = 640;

    float mm_per_m{50.0f};
//    float mm_per_m{1000.0f};
    float constant{570.3f};
};

/// create structure in order to store the most useful metric for each vertex of point cloud
struct PointFeatures {
    int vec_index;
    vec coordinates, normal;
    float curvature;
    vector<int> neighborVertices;

    PointFeatures();
    PointFeatures(int index, vec coord, vec norm, float curv, vector<int> neighbors) :
    vec_index(index), coordinates(coord), normal(norm),
    curvature(curv), neighborVertices(neighbors) {}
    PointFeatures(int index, vec coord, vector<int> neighbors) :
    vec_index(index), coordinates(coord), neighborVertices(neighbors) {}
};

class PointCloud {
    public:
        PointCloud() {};

    public:
        void create(const Mat &image, Mat &depth_image);
        void clearPoints();
        void transformPoints(pair<Eigen::Matrix3f, Eigen::Vector3f> &R_t, VecArray &points);
        void kNearest(const VecArray &src, VecArray &nearestPoints, vector<float> &dist, int kn);
        void sobel(const Mat &img, Mat &new_img);
        void thresholding(const Mat &img, Mat &new_img);
        void getEdges(const Mat &img, VecArray &edges, const int &value);
        void getNormals(Mesh*& mesh, vector<int>* tri_indices, VecArray &normals);
        void getCurvature(Mesh*& mesh, vector<int>* tri_indices, vector<float> &curvature);
        void segmentation(const vector<PointFeatures> pointFeatures, vector<int> &region, vector<int> &hash, int start);
        void segmentation2(const vector<PointFeatures> &pointFeatures, const vector<int> &segment, vector<VecArray> &new_segments, math::Plane &plane, Mesh*& mesh);
        void planeFitting(const vector<PointFeatures> pointFeatures, const vector<int> &segment, vec &plane_normal, vec &centroid);
        void getPlaneMetrics(vector<PointFeatures> &features, const vector<int> &segment, math::Plane &plane, int &leftPoints, int &rightPoints, float &perc);
        void rotate(const Mat &rotation_mat);

        float vectorSum(const vector<float> &v);
        float getDistance(const vec p1, const vec p2);
        float getResiduals(const vector<PointFeatures> pointFeatures, const vector<int> &segment, math::Plane &plane);

        double dot_product(const vec &v1, const vec &v2);

        template <typename T>
        bool contains(const vector<T> &values, const T value);
        template <typename T>
        T min(const vector<T> &values);
        template <typename T>
        T max(const vector<T> &values);
        template <typename T>
        void normalize(vector<T> &values);

        bool contains(const vector<vec> &values, const vec value);
        bool zeroPoint(const vec p);

        vec cross_product(const vec &v1, const vec &v2);
        vec getCentroid(const VecArray &src);

        pair<Eigen::Matrix3f, Eigen::Vector3f> computeRigidTransform(const VecArray &l_points, const VecArray &r_points);
        pair<Eigen::Matrix3f, Eigen::Vector3f> icp(VecArray &src_points, vector<float> &dist, float &mean_distance, int &iterations);

        vector<int>* triangulateMesh(const std::vector<vec> &vertices, Mesh*& mesh, const float threshold);
        vector<int>* getNRingNeighborhood(Mesh*& mesh, vector<int>*& tri_indices, int &rings);

        Mat rotationMatrix(const Vec3d &degree);

        friend bool operator== (const vec &v1, const vec &v2);
        friend bool operator!= (const vec &v1, const vec &v2);

    /// not used
    /*
     /// sort PointFeatures comparing their curvature
    struct PointFeaturesComparator {
        virtual inline bool operator() (PointFeatures &p1, PointFeatures &p2) {
            return (p1.curvature > p2.curvature);
        }
    };
    double getAngle(const vec &p1, const vec &p2);
    void test(const vector<PointFeatures> pointFeatures, vector<int> &region, vector<int> &hash, int start);
    void findAdjacentPoints(const Mat &l_frame_rgb, const Mat &l_frame_rgbd);
    void validate(int &top_col, int &top_row, int &widht, int &height);
    void findCorrespondingPoints(const Mat &l_frame_rgb, const Mat &r_frame_rgb);
    void planeFitting2(const vector<PointFeatures> &segment, vec &plane_normal, vec &centroid);
    void segmentation2(const vector<PointFeatures> segment, vector<vector<PointFeatures>> &new_segments);
    void segmentation(const vector<PointFeatures> pointFeatures, vector<PointFeatures> &region, vector<int> &hash, int start);
    void findAdjacentPoints(const Mat &l_frame_rgb, const Mat &l_frame_rgbd);
    void validate(int &top_col, int &top_row, int &widht, int &height);
    void normalsFiltering(Mesh*& mesh, vector<PointFeatures> &features, VecArray &normals);
    void getCurvatureOld(Mesh*& mesh, vector<int>* tri_indices, vector<float> &curvature, VecArray &normals);
    vec convertTo3d(const Mat &image, const Mat &depth_image, Point2d &point);
    void edgeDetection(const Mat &img, const Mat &depth_img);
    void computeNormals(const Mat &img, const Mat &depth_img, VecArray &normals);
    void computeNormal(const Mat &img, const Mat &depth_img, int x, int y, vec &normal);
    double getAngle(const vec &p1, const vec &p2);
    void getNormalAngle(const Mat &original_img, const Mat &thresholded_img, const Mat &depth_img, const int &value, vector<double> &curve_degrees);
    pair<Eigen::Matrix3f, Eigen::Vector3f> icpCurves(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations, vector<double> &curves);
    pair<Eigen::Matrix3f, Eigen::Vector3f> icpWeights(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations);
    pair<Eigen::Matrix3f, Eigen::Vector3f> icpNormals(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations, Mat &l_img, Mat &l_depth, Mat &r_img, Mat &r_depth);
    void laplacian(const Mat &img, Mat &new_img);
    void bilateral(const Mat &img, Mat &new_img);
    void cannyThreshold(const Mat &img, Mat &new_img);
    pair<Eigen::Matrix3f, Eigen::Vector3f> computeRigidTransform(const VecArray &l_points, const VecArray &r_points, vector<double> &weights);
    void getPoints(vector< pair <vec,Vec3b>> &points);
    int getRgbdId(const ImageRGBD &image);
    void kNearest2(const VecArray &src, VecArray &nearestPoints, vector<int> &indices, vector<float> &dist, int kn);
    */

public:
        vector< pair <vec,Vec3b>> m_points;
        KDTree *m_dst_KDTree;
};

#endif //LAB0_POINTCLOUD_H