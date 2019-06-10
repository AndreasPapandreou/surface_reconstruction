#include "surfaceReconstruction.h"
#include "dataset.h"

using namespace std;
using namespace vvr;

surfaceReconstruction::surfaceReconstruction(int &index) {
    initialize(index);
    createGui();
//    pcloud = new PointCloud;
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
    createButton("Align frames using knn", alignFramesKnn, this, QT_PUSH_BUTTON, true);

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
    cout << "l_frame = " << myClass->l_frame_index << endl;
    drawFrame(myClass->l_frame_index, object);
}

void surfaceReconstruction::drawRightFrame(int x, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    cout << "r_frame = " << myClass->r_frame_index << endl;
    drawFrame(myClass->r_frame_index, object);
}

void surfaceReconstruction::drawFrame(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->pcloud.clearPoints();
    myClass->getPointCLoud(myClass->l_points, index);
    myClass->m_flag = false;
}

// without considering the colors
//void surfaceReconstruction::alignFramesKnn(int index, void* object) {
//    auto * myClass = (surfaceReconstruction*) object;
//    myClass->pcloud->clearPoints();
//
//    vector< pair <Point3d,Vec3b>> l_points;
//    myClass->getPointCLoud(l_points, myClass->l_frame_index);
//    vector<Point3d> l_uncolored_points = myClass->getFirstData(l_points);
//
//    myClass->pcloud->clearPoints();
//
//    vector< pair <Point3d,Vec3b>> r_points;
//    myClass->getPointCLoud(r_points, myClass->r_frame_index);
//    vector<Point3d> r_uncolored_points = myClass->getFirstData(r_points);
//
//    int iteration{5};
//    int counter{0};
//    while(counter < iteration) {
//        vector<Point3d> nearestPoints;
//        myClass->pcloud->kNearest(l_uncolored_points, r_uncolored_points, nearestPoints, 1);
//
//        pair<Eigen::Matrix3d, Eigen::Vector3d> R_t;
//        R_t = myClass->pcloud->computeRigidTransform(l_uncolored_points, nearestPoints);
//        myClass->pcloud->transformPoints(R_t, r_uncolored_points);
//        myClass->pcloud->transformPoints(R_t, nearestPoints);
//
//        double error = myClass->pcloud->getError(l_uncolored_points, nearestPoints);
//        cout << "iteration = " << counter << endl;
//        cout << "error = " << error << endl << endl;
//
//        counter++;
//
//        // refresh data in order to draw them
//        if (iteration == counter) {
//            vector<int> indeces = myClass->removePoints(l_uncolored_points, nearestPoints, 1.0f);
//            cout << "indices size = " << indeces.size() << endl;
//
//            myClass->l_points.clear();
//            for (int i=0; i<indeces.size(); i++) {
//                myClass->l_points.emplace_back(l_uncolored_points.at(i), l_points.at(i).second);
//            }
//
//            myClass->r_points.clear();
//            for (int i=0; i<r_uncolored_points.size(); i++) {
//                myClass->r_points.emplace_back(r_uncolored_points.at(i), r_points.at(i).second);
//            }
////            myClass->l_points = l_points;
//
//            cout << "l_points = " << myClass->l_points.size() << endl;
//            cout << "r_points = " << myClass->r_points.size() << endl;
//        }
//    }
//    myClass->m_flag = true;
//}

// without considering the colors
void surfaceReconstruction::alignFramesKnn(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->all_points.clear();
    vector<Point3d> l_uncolored_points, r_uncolored_points, nearestPoints;
    vector< pair <Point3d,Vec3b>> l_points, r_points;
    int numFrames = {1};

    myClass->getPointCLoud(l_points, myClass->l_frame_index);
    cout << "initial frame = " << myClass->l_frame_index << endl;

    for (int i=0; i<numFrames; i++) {
        cout << "next frame = " << myClass->r_frame_index << endl;
        l_uncolored_points = myClass->getFirstData(l_points);
        r_points.clear();
        myClass->getPointCLoud(r_points, myClass->r_frame_index);
        r_uncolored_points = myClass->getFirstData(r_points);

        VecArray dst_pts;
        int size = l_uncolored_points.size();
        for (int j=0; j<size; j++) {
            dst_pts.emplace_back(vec(static_cast<float>(l_uncolored_points.at(j).x),
                                     static_cast<float>(l_uncolored_points.at(j).y),
                                     static_cast<float>(l_uncolored_points.at(j).z)));
        }
        myClass->pcloud.m_dst_KDTree = new KDTree(dst_pts);

        int iteration{3};
        int counter{0};
        while(counter < iteration) {
            nearestPoints.clear();
            myClass->pcloud.kNearest(r_uncolored_points, nearestPoints, 1);

            pair<Eigen::Matrix3d, Eigen::Vector3d> R_t;
            R_t = myClass->pcloud.computeRigidTransform(nearestPoints, r_uncolored_points);

            myClass->pcloud.transformPoints(R_t, r_uncolored_points);

            double error = myClass->pcloud.getError(r_uncolored_points, nearestPoints);
            cout << "iteration = " << counter << endl;
            cout << "error = " << error << endl << endl;

            counter++;

            // refresh data in order to draw them
            if (iteration == counter) {
                vector<int> indices = myClass->removePoints(l_uncolored_points, nearestPoints, 0.005f);
                for (int j = 0; j < indices.size(); j++) {
                    myClass->all_points.emplace_back(l_uncolored_points.at(j), l_points.at(j).second);
                }
                myClass->r_points.clear();
                for (int j=0; j<r_uncolored_points.size(); j++) {
                    myClass->r_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);
                }

                if (numFrames == i+1) {
                    for (int j=0; j<r_uncolored_points.size(); j++) {
                        myClass->all_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);
                    }
                }
            }
        }
        l_points = myClass->r_points;
        myClass->r_frame_index++;
    }
    myClass->m_flag = true;
}

void surfaceReconstruction::alignFrames(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->all_points.clear();

    myClass->pcloud.clearPoints();
    myClass->getPointCLoud(myClass->l_points, myClass->l_frame_index);

    myClass->pcloud.clearPoints();
    myClass->getPointCLoud(myClass->r_points, myClass->r_frame_index);

    for (int i=0; i<myClass->l_points.size(); i++) {
        myClass->all_points.emplace_back(myClass->l_points.at(i));
    }
    for (int i=0; i<myClass->r_points.size(); i++) {
        myClass->all_points.emplace_back(myClass->r_points.at(i));
    }
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

vector<Point3d> surfaceReconstruction::getData(vector<Point3d> points, int num) {
    vector<Point3d> result;
    for (int i=0; i<num; i++) {
        result.emplace_back(points.at(i));
    }
    return result;
}

void surfaceReconstruction::getPointCLoud(vector< pair <Point3d,Vec3b>> &point_cloud, int &index) {
    pcloud.clearPoints();
    getImage(index);
    getDepthImage(index);
    pcloud.create(image_mat, depth_mat);

//    Vec3d degree = {180.0, 0.0, 0.0};
//    Mat rotation = pcloud.rotationMatrix(degree);
//    pcloud.rotate(rotation);
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
    cout << "all_points size = " << all_points.size() << endl;

//    for (auto &l_point : l_points) {
//        Point3D(l_point.first.x, l_point.first.y, l_point.first.z,
//                Colour(l_point.second[2], l_point.second[1], l_point.second[0])).draw();
//        }
//    for (auto &r_point : r_points) {
//        Point3D(r_point.first.x, r_point.first.y, r_point.first.z,
//                Colour(r_point.second[2], r_point.second[1], r_point.second[0])).draw();
//    }
    for (auto &r_point : all_points) {
        Point3D(r_point.first.x, r_point.first.y, r_point.first.z,
                Colour(r_point.second[2], r_point.second[1], r_point.second[0])).draw();
    }
}

//!---------------------------------------------------------------------------------------------------------------------

//!---------------------------------------------------------------------------------------------------------------------
//! other functionalities
//!---------------------------------------------------------------------------------------------------------------------
// if diff <= threshold -> remove them
vector<int> surfaceReconstruction::removePoints(vector<Point3d> &l_points, vector<Point3d> &r_points, float threshold) {
    vector<int> indices;
    float value;
    for (int i=0; i<l_points.size(); i++) {
            Point3d diff_point = l_points.at(i) - r_points.at(i);
            value = abs(pow(diff_point.x,2) + pow(diff_point.y,2) + pow(diff_point.z,2));
//            cout << "value = " << value << endl;
            if (value > threshold)
                indices.emplace_back(i);
    }
    return indices;
}

//!---------------------------------------------------------------------------------------------------------------------