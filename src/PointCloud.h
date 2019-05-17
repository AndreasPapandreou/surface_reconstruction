#ifndef LAB0_POINTCLOUD_H
#define LAB0_POINTCLOUD_H

#include "ImageRGBD.h"

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
        Mat rotationMatrix(Vec3d &degree);
        void rotate(Mat &rotation_mat);

    friend class surfaceReconstruction;
};

#endif //LAB0_POINTCLOUD_H