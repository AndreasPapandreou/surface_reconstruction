#include "PointCloud.h"

/* ------------------------------------------------------------------------------
 * Inputs       : An image in Mat and its depth one version in Mat
 * Description  : It creates the point cloud of each image. and stores all points
 *                and their colours in vector< pair <Point3d,Vec3b>>.
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void PointCloud::create(const Mat &image, const Mat &depth_image)
{
    CameraConstants camera;
    Mat xgrid, ygrid;
    xgrid.create(camera.image_height,camera.image_width, CV_16SC1);
    ygrid.create(camera.image_height,camera.image_width, CV_16SC1);


    for (unsigned short i=0; i<camera.image_height; i++) {
        for (unsigned short j=0; j<camera.image_width; j++) {
            xgrid.at<short>(i,j) = static_cast<short>(j + 1 + (camera.topLeft[0] - 1) - camera.center[0]);
            ygrid.at<short>(i,j) = static_cast<short>(i + 1 + (camera.topLeft[1] - 1) - camera.center[1]);
        }
    }

    std::vector<short> x, y, z;
    for (unsigned short i=0; i<camera.image_height; i++) {
        for (unsigned short j=0; j<camera.image_width; j++) {
            Point3d point;
            point.x = xgrid.at<short>(i,j)*depth_image.at<unsigned short>(i,j)/camera.constant/camera.mm_per_m;
            point.y = ygrid.at<short>(i,j)*depth_image.at<unsigned short>(i,j)/camera.constant/camera.mm_per_m;
            point.z = depth_image.at<unsigned short>(i,j)/camera.mm_per_m;
            if (point.x != 0 && point.y != 0 && point.z != 0) {
                m_points.emplace_back(point, image.at<Vec3b>(i,j));
            }
        }
    }
}

/* ------------------------------------------------------------------------------
 * Inputs       : One vector< pair <Point3d,Vec3b>>
 * Description  : Gets the instance's point cloud.
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void PointCloud::getPoints(vector< pair <Point3d,Vec3b>> &points)
{
    points = m_points;
}

/* ------------------------------------------------------------------------------
 * Inputs       : One depth image
 * Description  : Gets the id of instances' depth image.
 * Return       : The id of current depth image.
 * ------------------------------------------------------------------------------
*/
int PointCloud::getImageId(const ImageRGBD &image)
{
    return image.m_id;
}