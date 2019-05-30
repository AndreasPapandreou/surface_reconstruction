#include "surfaceReconstruction.h"
#include "dataset.h"

using namespace std;
using namespace vvr;

surfaceReconstruction::surfaceReconstruction(int &index) {
    initialize(index);
    createGui();
}

void surfaceReconstruction::initialize(int &index) {
    image_prefix = convertToStr(generic::convertToDatasetType(index));
    num_images = generic::DatasetSize(generic::convertToDatasetSize(index));
    generic::stereo_dir += image_prefix;
    l_frame_index = index;
    r_frame_index = index+1;
    m_flag = false;
}

//!---------------------------------------------------------------------------------------------------------------------
//! gui functions
//!---------------------------------------------------------------------------------------------------------------------
void surfaceReconstruction::createGui() {
    //! Load settings.
    vvr::Shape::DEF_LINE_WIDTH = 4;
    vvr::Shape::DEF_POINT_SIZE = 10;
    m_perspective_proj = true;
    m_bg_col = Colour("768E77");
    m_obj_col = Colour("454545");

    namedWindow("gui", WINDOW_NORMAL);
    createTrackbar("next frame", "gui", &slider_value, num_images, change_frame, this);

    createButton("Draw left frame", drawLeftFrame, this, QT_PUSH_BUTTON, true);
    createButton("Draw right frame", drawRightFrame, this, QT_PUSH_BUTTON, true);
    createButton("Align frames", alignFrames, this, QT_PUSH_BUTTON, true);
    createButton("Align frames no knn", alignFramesNoKnn, this, QT_PUSH_BUTTON, true);

    // initialize with the two first frames
    showFrames(1);
}

void surfaceReconstruction::showFrames(int index) {
    int size{300}, num_rows{2}, num_cols{1}, max;
    float scale;

    // Create a new 3 channel image
    Mat DispImage = Mat::zeros(Size(100 + size*num_rows, 60 + size*num_cols), CV_8UC3);

    l_frame = getFrame(index);
    l_frame_index = index;
    index++;
    r_frame = getFrame(index);
    r_frame_index = index;

    Mat images[2] = {l_frame, r_frame};
    for (int i = 0, m = 20, n = 20; i < 2; i++, m += (20 + size)) {
        // find whether height or width is greater in order to resize the image
        max = (images[i].cols > images[i].rows)? images[i].cols: images[i].rows;

        // set the scaling factor to resize the image
        scale = ((float) max / size );

        // set the image ROI to display the current image
        cv::Rect ROI(m, n, (int)( images[i].cols/scale ), (int)( images[i].rows/scale ));
        Mat temp;
        cv::resize(images[i], temp, Size(ROI.width, ROI.height));
        temp.copyTo(DispImage(ROI));
    }

    // create a new window and show the new frames
    namedWindow("gui", WINDOW_NORMAL);
    createTrackbar("next frame", "gui", &slider_value, num_images, change_frame, this);
    imshow("gui", DispImage);
    waitKey(1);
}

//void surfaceReconstruction::createMesh(const vector< pair <Point3d,Vec3b>> &image_points, Mesh &mesh) {
//    vector<vec> &mesh_vertices = m_mesh.getVertices();
//    for (auto &point : image_points) {
//        mesh_vertices.emplace_back(point.first.x, point.first.y, point.first.z);
//    }
//}

void surfaceReconstruction::change_frame(int x, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    if (x >=1 && x < myClass->num_images)
        myClass->showFrames(x);
    //TODO delete myClass from everywhere
}

void surfaceReconstruction::drawLeftFrame(int x, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    drawFrame(myClass->l_frame_index, object);
}

void surfaceReconstruction::drawRightFrame(int x, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    drawFrame(myClass->r_frame_index, object);
}

void surfaceReconstruction::drawFrame(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->pcloud.clearPoints();
    myClass->getPointCLoud(myClass->l_points, index);
    myClass->m_flag = false;
}

void surfaceReconstruction::alignFrames(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->pcloud.clearPoints();

    vector< pair <Point3d,Vec3b>> l_initial_points;
    myClass->getPointCLoud(l_initial_points, myClass->l_frame_index);

    myClass->pcloud.clearPoints();

    vector< pair <Point3d,Vec3b>> r_initial_points;
    myClass->getPointCLoud(r_initial_points, myClass->r_frame_index);

    vector<Point3d> l_initial_first_points = myClass->getFirstData(l_initial_points);
    int counter{0};
    while(counter < 5) {
        cout << endl;
        cout << "iteration = " << counter << endl;

        vector<Point3d> nearestPoints;
        vector<Point3d> r_initial_first_points = myClass->getFirstData(r_initial_points);
        myClass->pcloud.kNearest(l_initial_first_points, r_initial_first_points, nearestPoints, 1);

        pair<Eigen::Matrix3d, Eigen::Vector3d> R_t;
        R_t = myClass->pcloud.computeRigidTransform(l_initial_first_points, nearestPoints);

        myClass->pcloud.tranformPoints(R_t, r_initial_first_points);

        myClass->r_points.clear();
        myClass->l_points = l_initial_points;
        for (int i=0; i<r_initial_first_points.size(); i++) {
            myClass->r_points.emplace_back(r_initial_first_points.at(i), r_initial_points.at(i).second);
        }

        r_initial_points = myClass->r_points;

        vector<Point3d> test;
        for (int i=0; i<nearestPoints.size(); i++) {
            test.emplace_back(l_initial_first_points.at(i));
        }
        double error = myClass->pcloud.computeError(test, nearestPoints);
        cout << "error = " << error << endl;
        counter++;
    }
    myClass->m_flag = true;
}

void surfaceReconstruction::alignFramesNoKnn(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->pcloud.clearPoints();
    myClass->getPointCLoud(myClass->l_points, myClass->l_frame_index);

    myClass->pcloud.clearPoints();
    myClass->getPointCLoud(myClass->r_points, myClass->r_frame_index);
    myClass->m_flag = true;
}

void surfaceReconstruction::reset()
{
    Scene::reset();
    m_plane_d = 0;
    m_plane = Plane(vec(0, 1, 1).Normalized(), m_plane_d);
}

void surfaceReconstruction::resize()
{
    static bool first_pass = true;
    if (first_pass)
        first_pass = false;
}
//!---------------------------------------------------------------------------------------------------------------------

//void surfaceReconstruction::alignFrames(int index, void* object) {
//    auto * myClass = (surfaceReconstruction*) object;
//    myClass->pcloud.clearPoints();
//    myClass->m_mesh.getVertices().clear();
//
//    myClass->getImage(myClass->l_frame_index);
//    myClass->getDepthImage(myClass->l_frame_index);
//    const Mat l_frame_rgb = myClass->image_mat;
//    const Mat l_frame_rgbd = myClass->depth_mat;
//    myClass->pcloud.create(myClass->image_mat, myClass->depth_mat);
//
//    Vec3d degree = {180.0, 0.0, 0.0};
//    Mat rotation = myClass->pcloud.rotationMatrix(degree);
//    myClass->pcloud.rotate(rotation);
//    vector< pair <Point3d,Vec3b>> l_initial_points = myClass->pcloud.m_points;
//
//    myClass->pcloud.clearPoints();
//
//    myClass->getImage(myClass->r_frame_index);
//    myClass->getDepthImage(myClass->r_frame_index);
//    const Mat r_frame_rgb = myClass->image_mat;
//    const Mat r_frame_rgbd = myClass->depth_mat;
//    myClass->pcloud.create(myClass->image_mat, myClass->depth_mat);
//    myClass->pcloud.rotate(rotation);
//    vector< pair <Point3d,Vec3b>> r_initial_points = myClass->pcloud.m_points;
//
//    myClass->l_points.clear();
//    myClass->r_points.clear();
//    myClass->pcloud.findCorrespondingPoints(l_frame_rgb, r_frame_rgb, l_frame_rgbd, r_frame_rgbd, myClass->l_points, myClass->r_points);
//
//    // exatrct only the point without its colors
//    vector<Point3d> l_corresponding_points, r_corresponding_points;
//    for (int i=0; i<myClass->l_points.size(); i++) {
//        l_corresponding_points.emplace_back(myClass->l_points.at(i).first);
//        r_corresponding_points.emplace_back(myClass->r_points.at(i).first);
//    }
//
//    pair<Eigen::Matrix3d, Eigen::Vector3d> R_t;
//    R_t = myClass->pcloud.computeRigidTransform(l_corresponding_points, r_corresponding_points);
//
//    vector<Point3d> r_transformed_points;
//    for (auto &r_initial_point : r_initial_points) {
//        r_transformed_points.emplace_back(r_initial_point.first);
//    }
//
//    myClass->pcloud.tranformPoints(R_t, r_transformed_points);
//
//    myClass->r_points.clear();
//    myClass->l_points = l_initial_points;
//    for (int i=0; i<r_transformed_points.size(); i++) {
//        myClass->r_points.emplace_back(r_transformed_points.at(i), r_initial_points.at(i).second);
//    }
//    myClass->m_flag = true;
//}

//!---------------------------------------------------------------------------------------------------------------------
//! getter functions
//!---------------------------------------------------------------------------------------------------------------------
void surfaceReconstruction::getDepthImage(int frame_index) {
    ImageRGBD depth_image(generic::stereo_dir + "/" + image_prefix + "_" + generic::convertToStr(frame_index) + "_depth.png");
    depth_image.convertToMat();
    depth_image.getMat(depth_mat);
}

void surfaceReconstruction::getImage(int frame_index) {
    ImageRGB image(generic::stereo_dir + "/" + image_prefix + "_" + generic::convertToStr(frame_index) + ".png");
    image.convertToMat();
    image.getMat(image_mat);
}

vector<Point3d> surfaceReconstruction::getFirstData(vector< pair <Point3d,Vec3b>> &paired_data) {
    vector<Point3d> data;
    for (auto &i : paired_data) {
        data.emplace_back(i.first);
    }
    return data;
}

void surfaceReconstruction::getPointCLoud(vector< pair <Point3d,Vec3b>> &point_cloud, int &index) {
    getImage(index);
    getDepthImage(index);
    pcloud.create(image_mat, depth_mat);

    Vec3d degree = {180.0, 0.0, 0.0};
    Mat rotation = pcloud.rotationMatrix(degree);
    pcloud.rotate(rotation);
    point_cloud = pcloud.m_points;
}

Mat surfaceReconstruction::getFrame(int index) {
    ostringstream frame_to_stream; // declaring output string stream
    frame_to_stream << index; // frame_to_stream a number as a stream into output
    string frame_to_string = frame_to_stream.str(); // the str() coverts number into string
    return imread(generic::stereo_dir + "/" + image_prefix + "_" + frame_to_string + ".png");
}
//!---------------------------------------------------------------------------------------------------------------------

//!---------------------------------------------------------------------------------------------------------------------
//! draw functions
//!---------------------------------------------------------------------------------------------------------------------
void surfaceReconstruction::draw() {
    if (m_flag) {
        drawAdjacentPoints();
    }
    else {
        for (auto &l_point : l_points) {
            Point3D(l_point.first.x, l_point.first.y, l_point.first.z,
                    Colour(l_point.second[2], l_point.second[1], l_point.second[0])).draw();
        }
    }
}

void surfaceReconstruction::drawAdjacentPoints() {
//    Point3d not_p(0,0,0);
        for (int i=0; i<r_points.size(); i++) {
//        if (l_points[i].first == not_p || r_points[i].first == not_p) {
//        if (l_points[i].first != not_p) {
//            if (r_points[i].first != not_p) {
                Point3D(l_points[i].first.x, l_points[i].first.y, l_points[i].first.z,
                        Colour(l_points.at((i)).second[2], l_points.at((i)).second[1], l_points.at((i)).second[0])).draw();
                Point3D(r_points[i].first.x, r_points[i].first.y, r_points[i].first.z,
                    Colour(r_points.at((i)).second[2], r_points.at((i)).second[1], r_points.at((i)).second[0])).draw();
//            }
        }
}
//!---------------------------------------------------------------------------------------------------------------------

