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
#include <bits/stdc++.h>
#include "dkm.hpp"

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

    float mm_per_m{50.0f};
//    float mm_per_m{1000.0f};
    float constant{570.3f};
};

struct PointFeatures {
    int vec_index;
    vec coordinates, normal;
    float curvature, luminance;
    vector<int> neighborVertices;

    PointFeatures();

    PointFeatures(int index, vec coord, vec norm, float curv, float lum, vector<int> neighbors) :
    vec_index(index), coordinates(coord), normal(norm), curvature(curv),
    luminance(lum), neighborVertices(neighbors) {}

    PointFeatures(int index, vec coord, vector<int> neighbors) :
    vec_index(index), coordinates(coord), neighborVertices(neighbors) {}
};

struct PointFeaturesComparator {
    virtual inline bool operator() (PointFeatures &p1, PointFeatures &p2) {
        return (p1.curvature > p2.curvature);
    }
};

class PointCloud {
    public:
        PointCloud() {};

    public:
        void create(const Mat &image, Mat &depth_image);
        void clearPoints();
        Mat rotationMatrix(const Vec3d &degree);
        void rotate(const Mat &rotation_mat);
        pair<Eigen::Matrix3f, Eigen::Vector3f> computeRigidTransform(const VecArray &l_points, const VecArray &r_points, vector<double> &weights);
        pair<Eigen::Matrix3f, Eigen::Vector3f> computeRigidTransform(const VecArray &l_points, const VecArray &r_points);
        void transformPoints(pair<Eigen::Matrix3f, Eigen::Vector3f> &R_t, VecArray &points);
        void kNearest(const VecArray &src, VecArray &nearestPoints, vector<float> &dist, int kn);
        void kNearest2(const VecArray &src, VecArray &nearestPoints, vector<int> &indices, vector<float> &dist, int kn);
        vec getCentroid(const VecArray &src);

        void getPoints(vector< pair <vec,Vec3b>> &points);
        int getRgbdId(const ImageRGBD &image);

        void sobel(const Mat &img, Mat &new_img);
        void laplacian(const Mat &img, Mat &new_img);
        void bilateral(const Mat &img, Mat &new_img);
        void cannyThreshold(const Mat &img, Mat &new_img);
        void thresholding(const Mat &img, Mat &new_img);
        void getEdges(const Mat &img, VecArray &edges, const int &value);
        pair<Eigen::Matrix3f, Eigen::Vector3f> icpCurves(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations, vector<double> &curves);
        pair<Eigen::Matrix3f, Eigen::Vector3f> icpWeights(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations);
        pair<Eigen::Matrix3f, Eigen::Vector3f> icpNormals(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations, Mat &l_img, Mat &l_depth, Mat &r_img, Mat &r_depth);
        pair<Eigen::Matrix3f, Eigen::Vector3f> icp(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations);

        float vectorSum(const vector<float> &v);
        template <typename T>
        T min(const vector<T> &values);
        template <typename T>
        T max(const vector<T> &values);
        template <typename T>
        void normalize(vector<T> &values);
        template <typename T>
        bool contains(const vector<T> &values, const T value);
        bool contains(const vector<vec> &values, const vec value);

//        void edgeDetection(const Mat &img, const Mat &depth_img);
        void computeNormals(const Mat &img, const Mat &depth_img, VecArray &normals);
        void computeNormal(const Mat &img, const Mat &depth_img, int x, int y, vec &normal);
        double getAngle(const vec &p1, const vec &p2);
        void getNormalAngle(const Mat &original_img, const Mat &thresholded_img, const Mat &depth_img, const int &value, vector<double> &curve_degrees);

        double dot_product(const vec &v1, const vec &v2);
        vec cross_product(const vec &v1, const vec &v2);

        vector<int>* triangulateMesh(const std::vector<vec> &vertices, Mesh*& mesh, const float threshold);
        void getNormals(Mesh*& mesh, vector<int>* tri_indices, VecArray &normals);
        void getCurvatureOld(Mesh*& mesh, vector<int>* tri_indices, vector<float> &curvature, VecArray &normals);
        void getCurvature(Mesh*& mesh, vector<int>* tri_indices, vector<float> &curvature);

        /// pass a copy of tri_indices when calling this function because it will change it
        vector<int>* getNRingNeighborhood(Mesh*& mesh, vector<int>*& tri_indices, int &rings);
        void normalsFiltering(Mesh*& mesh, vector<PointFeatures> &features, VecArray &normals);

        vec convertTo3d(const Mat &image, const Mat &depth_image, Point2d &point);
//        void findAdjacentPoints(const Mat &l_frame_rgb, const Mat &l_frame_rgbd);
//        void validate(int &top_col, int &top_row, int &widht, int &height);

        void kmeans(VecArray &points);

//        void segmentation(const vector<PointFeatures> pointFeatures, vector<PointFeatures> &region, vector<int> &hash, int start);
        void segmentation(const vector<PointFeatures> pointFeatures, vector<int> &region, vector<int> &hash, int start);
        void segmentation2(const vector<PointFeatures> &pointFeatures, const vector<int> &segment, vector<VecArray> &new_segments, math::Plane &plane, Mesh*& mesh);
//        void segmentation2(const vector<PointFeatures> segment, vector<vector<PointFeatures>> &new_segments);
        void segmentation2(const vector<PointFeatures> pointFeatures, vector<PointFeatures> &region, vector<int> &hash, int start);
        void planeFitting(const vector<PointFeatures> pointFeatures, const vector<int> &segment, vec &plane_normal, vec &centroid);
//        void planeFitting2(const vector<PointFeatures> &segment, vec &plane_normal, vec &centroid);
        float getResiduals(const vector<PointFeatures> pointFeatures, const vector<int> &segment, math::Plane &plane);

        void setCurvatureThreshold(float value);
        void setSmoothnessThreshold(float value);
        void setNumberOfNeighbours(int value);

        float getLuminance(Vec3b rgb);
        float getDistance(const vec p1, const vec p2);
        float getNorm(const vec p1, const vec p2, int index);
        bool zeroPoint(const vec p);

    friend bool operator== (const vec &v1, const vec &v2);
        friend bool operator!= (const vec &v1, const vec &v2);


    public:
        vector< pair <vec,Vec3b>> m_points;
        KDTree *m_dst_KDTree;
        float m_curv_th, m_angle_th;
        int m_neighbours;
};

#endif //LAB0_POINTCLOUD_H