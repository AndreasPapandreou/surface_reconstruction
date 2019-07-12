#ifndef LAB0_DATATYPES_H
#define LAB0_DATATYPES_H

#include <iostream>
#include "opencv2/opencv.hpp"
#include <Eigen/Geometry>
#include <VVRScene/mesh.h>

using namespace std;
using namespace cv;

namespace dataTypes {
    void convertToEigenMat(const vector<vec> &points, Eigen::MatrixXf &mat) {
        int size = points.size();
        for (int i=0; i<size; i++) {
            mat(0,i) = points.at(i).x;
            mat(1,i) = points.at(i).y;
            mat(2,i) = points.at(i).z;
        }
    }

    void convertToVector(const Eigen::MatrixXf &mat, vector<vec> &points) {
        int size = points.size();
        for (int i=0; i<size; i++) {
            points.at(i).x = mat(0,i);
            points.at(i).y = mat(1,i);
            points.at(i).z = mat(2,i);
        }
    }

    Eigen::Vector3f convertToEigenVector(const vec &point) {
        Eigen::Vector3f res;
        res(0,0) = point.x;
        res(1,0) = point.y;
        res(2,0) = point.z;
        return res;
    }

    vector<array<float, 3>> convertTo3dArray(VecArray &points) {
        vector<array<float, 3>> data;
        array<float, 3> a{};

        for (auto &p : points) {
            a.at(0) = p.x;
            a.at(1) = p.y;
            a.at(2) = p.z;
            data.emplace_back(a);
        }
        return data;
    }

};

#endif //LAB0_DATATYPES_H