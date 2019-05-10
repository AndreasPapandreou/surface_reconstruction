#ifndef LAB0_POINTCLOUD_H
#define LAB0_POINTCLOUD_H

#include "opencv2/opencv.hpp"
#include <VVRScene/canvas.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include "ImageRGBD.h"

using namespace cv;
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
        void create(const Mat &image, const Mat &depth_image);
        void getPoints(vector< pair <Point3d,Vec3b>> &points);
        int getImageId(const ImageRGBD &image);
};

#endif //LAB0_POINTCLOUD_H