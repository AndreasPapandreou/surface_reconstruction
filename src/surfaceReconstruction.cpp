#include <iostream>
#include <math.h>
#include "opencv2/opencv.hpp"
#include "surfaceReconstruction.h"

//!< If set, return 16-bit/32-bit image when the input has the corresponding depth, otherwise convert it to 8-bit.
#define IMREAD_ANYDEPTH 2

using namespace std;
using namespace vvr;
using namespace cv;

surfaceReconstruction::surfaceReconstruction() {
    cout << "surfaceReconstruction constructor" << endl;

    //! Load settings.
    vvr::Shape::DEF_LINE_WIDTH = 4;
    vvr::Shape::DEF_POINT_SIZE = 10;
    m_perspective_proj = true;
    m_bg_col = Colour("768E77");
    m_obj_col = Colour("454545");
    reset();
    run();
}

void surfaceReconstruction::reset()
{
    cout << "reset" << endl;

    Scene::reset();

    //! Define plane
    m_plane_d = 0;
    m_plane = Plane(vec(0, 1, 1).Normalized(), m_plane_d);
}

void surfaceReconstruction::resize()
{
    cout << "resize" << endl;

    //! By Making `first_pass` static and initializing it to true,
    //! we make sure that the if block will be executed only once.

    static bool first_pass = true;

    if (first_pass)
    {
        first_pass = false;
    }
}

void surfaceReconstruction::run() {
    cout << "run" << endl;
    ImageRGBD depth_image("../data/depth_image.png");
    depth_image.convertToMat();

    Mat depth_mat;
    depth_image.getMat(depth_mat);

    ImageRGB image("../data/image.png");
    image.convertToMat();

    Mat image_mat;
    image.getMat(image_mat);

    PointCloud pcloud;
    pcloud.create(image_mat, depth_mat);

    pcloud.getPoints(m_points);
    createMesh(m_points, m_mesh);
}


/*
void surfaceReconstruction::arrowEvent(ArrowDir dir, int modif)
{
    math::vec n = m_plane.normal;
    if (dir == UP) m_plane_d += 1;
    if (dir == DOWN) m_plane_d -= 1;
    else if (dir == LEFT) n = math::float3x3::RotateY(DegToRad(1)).Transform(n);
    else if (dir == RIGHT) n = math::float3x3::RotateY(DegToRad(-1)).Transform(n);
    m_plane = Plane(n.Normalized(), m_plane_d);
}

void surfaceReconstruction::keyEvent(unsigned char key, bool up, int modif)
{
    Scene::keyEvent(key, up, modif);
    key = tolower(key);

    switch (key)
    {
        case 's': m_style_flag ^= FLAG_SHOW_SOLID; break;
        case 'w': m_style_flag ^= FLAG_SHOW_WIRE; break;
        case 'n': m_style_flag ^= FLAG_SHOW_NORMALS; break;
        case 'a': m_style_flag ^= FLAG_SHOW_AXES; break;
        case 'p': m_style_flag ^= FLAG_SHOW_PLANE; break;
        case 'b': m_style_flag ^= FLAG_SHOW_AABB; break;
    }
}
*/

void surfaceReconstruction::draw() {
//    for (auto &point : m_pClouds->points) {
//        Point3D(point.x, point.y, point.z, Colour::black).draw();
//    }

    double max_depth{0.0};
    for (auto &point : m_points) {
        if (point.first.z > max_depth)    max_depth = point.first.z;
    }

    int counter{0};
    for (auto &i : m_mesh.getVertices()) {
//        Point3D(i.x, i.y, i.z, Colour(static_cast<unsigned char>(i.z * 255 / max_depth), 0, 0)).draw();
        Point3D(i.x, i.y, i.z, Colour(m_points.at(counter).second[2] , m_points.at(counter).second[1], m_points.at(counter).second[0])).draw();
        counter++;
    }
}

void surfaceReconstruction::drawMesh(const vector<Point3d> &points) {
    double max_depth{0.0};
    for (auto &point : points) {
        if (point.z > max_depth)    max_depth = point.z;
    }

    for (auto &i : m_mesh.getVertices()) {
        Point3D(i.x, i.y, i.z, Colour(static_cast<unsigned char>(i.z * 255 / max_depth), 0, 0)).draw();
    }
}

//void surfaceReconstruction::matToVector(Mat &matrix_image, vector<float> &vector_image) {
//    if (matrix_image.isContinuous()) {
//        vector_image.assign((float*)matrix_image.datastart, (float*)matrix_image.dataend);
//    }
//    else {
//        for (int i = 0; i < matrix_image.rows; ++i) {
//            vector_image.insert(vector_image.end(), matrix_image.ptr<float>(i), matrix_image.ptr<float>(i)+matrix_image.cols);
//        }
//    }
//}


void surfaceReconstruction::createMesh(const vector< pair <Point3d,Vec3b>> &image_points, Mesh &mesh) {
    vector<vec> &mesh_vertices = m_mesh.getVertices();

    for (auto &point : image_points) {
        mesh_vertices.emplace_back(point.first.x, point.first.y, point.first.z);
    }
}

/*
void surfaceReconstruction::rotateX3D(PointCloud &pcloud, const float angle) {
    float radian = angle * pi/180.0f;

    Mat rotation_mat;
    rotation_mat = Mat(2, 2, CV_32F, (cos(radian), sin(radian),
                                      -sin(radian), cos(radian)));

    for (auto &point : pcloud.points) {
        point.y = point.y*rotation_mat.at<float>(0,0) + point.z*rotation_mat.at<float>(0,1);
        point.z = point.y*rotation_mat.at<float>(1,0) + point.z*rotation_mat.at<float>(1,1);
    }
}

void surfaceReconstruction::rotateX3D(Mesh &mesh, const float angle) {
    float radian = angle * pi/180.0f;

    Mat rotation_mat;
    rotation_mat = Mat(2, 2, CV_32F, (cos(radian), sin(radian),
                                      -sin(radian), cos(radian)));

    for (auto &point : mesh.getVertices()) {
        point.y = point.y*rotation_mat.at<float>(0,0) + point.z*rotation_mat.at<float>(0,1);
        point.z = point.y*rotation_mat.at<float>(1,0) + point.z*rotation_mat.at<float>(1,1);
    }
}

void surfaceReconstruction::rotateY3D(PointCloud &pcloud, const float angle) {
    float radian = angle * pi/180.0f;

    Mat rotation_mat;
    rotation_mat = Mat(2, 2, CV_32F, (cos(radian), -sin(radian),
                                      sin(radian), cos(radian)));

    for (auto &point : pcloud.points) {
        point.x = point.x*rotation_mat.at<float>(0,0) + point.z*rotation_mat.at<float>(0,1);
        point.z = point.x*rotation_mat.at<float>(1,0) + point.z*rotation_mat.at<float>(1,1);
    }
}

void surfaceReconstruction::rotateY3D(Mesh &mesh, const float angle) {
    float radian = angle * pi/180.0f;

    Mat rotation_mat;
    rotation_mat = Mat(2, 2, CV_32F, (cos(radian), -sin(radian),
            sin(radian), cos(radian)));

    for (auto &point : mesh.getVertices()) {
        point.x = point.x*rotation_mat.at<float>(0,0) + point.z*rotation_mat.at<float>(0,1);
        point.z = point.x*rotation_mat.at<float>(1,0) + point.z*rotation_mat.at<float>(1,1);
    }
}

void surfaceReconstruction::rotateZ3D(PointCloud &pcloud, const float angle) {
    float radian = angle * pi/180.0f;

    Mat rotation_mat;
    rotation_mat = Mat(2, 2, CV_32F, (cos(radian), sin(radian),
                                      -sin(radian), cos(radian)));

    for (auto &point : pcloud.points) {
        point.x = point.x*rotation_mat.at<float>(0,0) + point.y*rotation_mat.at<float>(0,1);
        point.y = point.x*rotation_mat.at<float>(1,0) + point.y*rotation_mat.at<float>(1,1);
    }
}

*/

