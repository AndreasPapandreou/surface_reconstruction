#ifndef LAB0_DATATYPES_H
#define LAB0_DATATYPES_H

#include <iostream>
#include "opencv2/opencv.hpp"
#include <Eigen/Geometry>
#include <VVRScene/mesh.h>

using namespace std;
using namespace cv;

namespace dataTypes {
    void convertToEigenMat(const vector<Point3d> &l_points, const vector<Point3d> &r_points, Eigen::MatrixXd &l_mat, Eigen::MatrixXd &r_mat) {
        int size = static_cast<int>(l_points.size());
        for (int i=0; i<size; i++) {
            l_mat(0,i) = l_points.at(i).x;
            l_mat(1,i) = l_points.at(i).y;
            l_mat(2,i) = l_points.at(i).z;

            r_mat(0,i) = r_points.at(i).x;
            r_mat(1,i) = r_points.at(i).y;
            r_mat(2,i) = r_points.at(i).z;
        }
    }

    void convertToEigenMat(const vector<Point3d> &points, Eigen::MatrixXd &mat) {
        int size = static_cast<int>(points.size());
        for (int i=0; i<size; i++) {
            mat(0,i) = points.at(i).x;
            mat(1,i) = points.at(i).y;
            mat(2,i) = points.at(i).z;
        }
    }

    void convertToVector(const Eigen::MatrixXd &mat, vector<Point3d> &points) {
        int size = static_cast<int>(points.size());
        for (int i=0; i<size; i++) {
            points.at(i).x = mat(0,i);
            points.at(i).y = mat(1,i);
            points.at(i).z = mat(2,i);
        }
    }

    Eigen::Vector3d convertToEigenVector3d(const Point3d &point) {
        Eigen::Vector3d res;
        res(0,0) = point.x;
        res(1,0) = point.y;
        res(2,0) = point.z;
        return res;
    }

    Point3d convertToPoint3d(const vec &point) {
        Point3d point3d;
        point3d.x = point.x;
        point3d.y = point.y;
        point3d.z = point.z;
        return point3d;
    }
};

#endif //LAB0_DATATYPES_H