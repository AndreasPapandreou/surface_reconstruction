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

    for (unsigned short i=0; i<camera.image_height; i++) {
        for (unsigned short j=0; j<camera.image_width; j++) {
            Point3d point;
            point.x = xgrid.at<short>(i,j)*depth_image.at<unsigned short>(i,j)/camera.constant/camera.mm_per_m;
            point.y = ygrid.at<short>(i,j)*depth_image.at<unsigned short>(i,j)/camera.constant/camera.mm_per_m;
            point.z = depth_image.at<unsigned short>(i,j)/camera.mm_per_m;
//            if (point.x != 0 && point.y != 0 && point.z != 0) {
            if (point.z != 0) {
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

/* ------------------------------------------------------------------------------
 * Inputs       : Vec3d with degree per axis in order x-y-z
 * Description  : Calculates rotation matrix given euler angles per axis.
 * Return       : The rotation matrix
 * ------------------------------------------------------------------------------
*/
Mat PointCloud::rotationMatrix(Vec3d &degree)
{
    Vec3d radian;
    radian[0] = degree[0]*M_PI/180.; // x_axis
    radian[1] = degree[1]*M_PI/180.; // y_axis
    radian[2] = degree[2]*M_PI/180.; // z_axis

    // calculate rotation about x axis
    Mat rotation_x = (Mat_<double>(3,3) << 1,       0,                 0,
                                    0,       cos(radian[0]),    -sin(radian[0]),
                                    0,       sin(radian[0]),    cos(radian[0]));

    // calculate rotation about y axis
    Mat rotation_y = (Mat_<double>(3,3) << cos(radian[1]),    0,      sin(radian[1]),
                                    0,                 1,      0,
                                    -sin(radian[1]),   0,      cos(radian[1]));

    // calculate rotation about z axis
    Mat rotation_z = (Mat_<double>(3,3) << cos(radian[2]),    -sin(radian[2]),      0,
                                    sin(radian[2]),    cos(radian[2]),       0,
                                    0,                 0,                    1);

    // get the final rotation matrix
    Mat rotation = rotation_z * rotation_y * rotation_x;
    return rotation;

}

/* ------------------------------------------------------------------------------
 * Inputs       : The rotation matrix
 * Description  : Rotate point cloud given the rotation matrix
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void PointCloud::rotate(Mat &rotation_mat) {
    for (auto &point : m_points) {
        Mat current_point = (Mat_<double>(3,1) << point.first.x, point.first.y, point.first.z);
        Mat result = rotation_mat*current_point;
        point.first.x = result.at<double>(0);
        point.first.y = result.at<double>(1);
        point.first.z = result.at<double>(2);
    }
}