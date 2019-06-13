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
    createButton("Align frames using knn", alignFramesKnn, this, QT_PUSH_BUTTON, true);
    createButton("Align frames using knn-sobel", alignFramesKnnSobel, this, QT_PUSH_BUTTON, true);

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

// without considering the colors
void surfaceReconstruction::alignFramesKnn(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->all_points.clear();

    vector< pair <vec,Vec3b>> l_points, r_points;
    VecArray r_uncolored_points;

    myClass->getPointCLoud(l_points, myClass->l_frame_index);

//    r_points.clear();
    myClass->getPointCLoud(r_points, myClass->r_frame_index);
    r_uncolored_points = myClass->getFirstData(r_points);

    VecArray tree_data = myClass->getFirstData(l_points);
    myClass->pcloud.m_dst_KDTree = new KDTree(tree_data);

    float error;
    int iterations{5};
    float mean_distance;

    cout << "icp started..." << endl;
    myClass->pcloud.icp(r_uncolored_points, mean_distance, error, iterations);

    myClass->all_points = l_points;
    for (int j=0; j<r_uncolored_points.size(); j++) {
        myClass->all_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);
    }
    myClass->m_flag = true;
}

void surfaceReconstruction::alignFramesKnnSobel(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->all_points.clear();

    VecArray l_edges, r_edges, r_uncolored_points;
    vector<pair<vec,Vec3b>> l_points, r_points;
    Mat sobel_img, thresholded_img;

    myClass->getPointCLoud(l_points, myClass->l_frame_index);
    myClass->pcloud.sobel(myClass->image_mat, sobel_img);
    myClass->pcloud.thresholding(sobel_img, thresholded_img);
    myClass->pcloud.getEdges(thresholded_img, l_edges);

//    r_points.clear();
    myClass->getPointCLoud(r_points, myClass->r_frame_index);
    r_uncolored_points = myClass->getFirstData(r_points);

    myClass->pcloud.sobel(myClass->image_mat, sobel_img);
    myClass->pcloud.thresholding(sobel_img, thresholded_img);
    myClass->pcloud.getEdges(thresholded_img, r_edges);

    VecArray tree_data = l_edges;
    myClass->pcloud.m_dst_KDTree = new KDTree(tree_data);

    float error;
    int iterations{10};
    float mean_distance;

    cout << "icp started..." << endl;
    myClass->pcloud.icp(r_edges, r_uncolored_points, mean_distance, error, iterations);

    myClass->all_points = l_points;
    for (int j=0; j<r_uncolored_points.size(); j++) {
        myClass->all_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);
    }
    myClass->m_flag = true;
}

// considering the colors
//void surfaceReconstruction::alignFramesKnn(int index, void* object) {
//    auto * myClass = (surfaceReconstruction*) object;
//
//    vector<float> all_errors;
//    for(float w=0.01f; w<=10.0f; w+=0.1f) {
//
//    myClass->all_points.clear();
//
//    VecArray4 l_colored_points, r_colored_points, nearestPoints, initial_r_colored_points;
//    vector< pair <vec,Vec3b>> l_points, r_points;
//    vector<float> dist;
//    int numFrames = {1};
//
//    myClass->getPointCLoud(l_points, myClass->l_frame_index);
//    myClass->getColorPoints(l_points, l_colored_points);
//    myClass->normalize(l_colored_points);
//
//    cout << "next frame = " << myClass->r_frame_index << endl;
//    r_points.clear();
//    myClass->getPointCLoud(r_points, myClass->r_frame_index);
//    myClass->getColorPoints(r_points, r_colored_points);
//    myClass->normalize(r_colored_points);
//
//    initial_r_colored_points = r_colored_points;
//
//    VecArray4 tree_data = l_colored_points;
//    myClass->pcloud.m_dst_KDTree = new KDTree2(tree_data);
//
//    float error{0.0f};
//    int iteration{1};
////    int counter{0};
////            float w = 0.001f;
////        while(counter < iteration) {
//            nearestPoints.clear();
//            dist.clear();
//            myClass->pcloud.kNearest(r_colored_points, nearestPoints, dist, 1, w);
//
//            pair<Eigen::Matrix3f, Eigen::Vector3f> R_t;
//            R_t = myClass->pcloud.computeRigidTransform(nearestPoints, r_colored_points);
//
//            myClass->pcloud.transformPoints(R_t, r_colored_points);
//
//            cout << "mean dist = " << myClass->vectorSum(dist)/(float)dist.size() << endl;
//
//            myClass->normalize(dist);
//            error = myClass->vectorSum(dist)/(float)dist.size();
////            cout << "iteration = " << counter << endl;
//            cout << "error = " << error << endl << endl;
//            cout << " w = " << w << endl;
//            all_errors.emplace_back(error);
//
////            counter++;
//
//            r_colored_points = initial_r_colored_points;
//
//            // refresh data in order to draw them
////            if (iteration == counter) {
////                vector<int> indices = myClass->removePoints(l_colored_points, nearestPoints, 0.1f);
////                vec p;
////                for (int j = 0; j < indices.size(); j++) {
////                    p.x = l_colored_points.at(j).x; p.y = l_colored_points.at(j).y; p.z = l_colored_points.at(j).z;
////                    myClass->all_points.emplace_back(p, l_points.at(j).second);
////                }
////                if (numFrames == 1) {
////                    vec p;
////                    for (int j=0; j<r_colored_points.size(); j++) {
////                        p.x = r_colored_points.at(j).x; p.y = r_colored_points.at(j).y; p.z = r_colored_points.at(j).z;
////                        myClass->all_points.emplace_back(p, r_points.at(j).second);
////                    }
////                }
////            }
////        }
//    }
//
//    double tmp{0.01f};
//    for (float all_error : all_errors) {
//        cout << "w = " << tmp << " and error = " << all_error << endl;
//        tmp += 0.1f;
//    }
//
//    myClass->m_flag = true;
//}

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

VecArray surfaceReconstruction::getFirstData(vector< pair <vec,Vec3b>> &paired_data) {
    VecArray data;
    for (auto &i : paired_data) {
        data.emplace_back(i.first);
    }
    return data;
}

VecArray surfaceReconstruction::getData(VecArray points, int num) {
    VecArray result;
    for (int i=0; i<num; i++) {
        result.emplace_back(points.at(i));
    }
    return result;
}

void surfaceReconstruction::getPointCLoud(vector< pair <vec,Vec3b>> &point_cloud, int &index) {
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

//void surfaceReconstruction::getColorPoints(const vector< pair <vec,Vec3b>> &p1, VecArray4 &p2) {
//    float H,S,V;
//    vector<float> all_H;
//    for (const auto &i : p1) {
//        RGBtoHSV(i.second,H,S,V);
//        all_H.emplace_back(H);
//    }
//    normalize(all_H);
//
//    for(int i=0; i<p1.size(); i++) {
//        p2.emplace_back(p1.at(i).first.x, p1.at(i).first.y, p1.at(i).first.z, all_H.at(i));
//    }
//}

float surfaceReconstruction::vectorSum(const vector<float> &v) {
    float initial_sum{0.0f};
    return accumulate(v.begin(), v.end(), initial_sum);
}

float surfaceReconstruction::min(const vector<float> &values) {
    float min_value{0.0f};
    for (const auto &i : values) {
        if (i < min_value)
            min_value = i;
    }
    return min_value;
}

//vec surfaceReconstruction::min(const VecArray4 &values) {
//    vec min_p,p;
//    min_p.x = values.at(0).x; min_p.y = values.at(0).y; min_p.z = values.at(0).z;
//    for (int i=1; i<values.size(); i++) {
//        p.x = values.at(i).x; p.y = values.at(i).y; p.z = values.at(i).z;
//        if (p.x < min_p.x)
//            min_p.x = p.x;
//        if (p.y < min_p.y)
//            min_p.y = p.y;
//        if (p.z < min_p.z)
//            min_p.z = p.z;
//    }
//    return min_p;
//}

float surfaceReconstruction::max(const vector<float> &values) {
    float max_value{values.at(0)};
    for(int i=1; i<values.size(); i++) {
        if (i > max_value)
            max_value = i;
    }
    return max_value;
}

//vec surfaceReconstruction::max(const VecArray4 &values) {
//    vec max_p,p;
//    max_p.x = values.at(0).x; max_p.y = values.at(0).y; max_p.z = values.at(0).z;
//    for (int i=1; i<values.size(); i++) {
//        p.x = values.at(i).x; p.y = values.at(i).y; p.z = values.at(i).z;
//        if (p.x > max_p.x)
//            max_p.x = p.x;
//        if (p.y > max_p.y)
//            max_p.y = p.y;
//        if (p.z > max_p.z)
//            max_p.z = p.z;
//    }
//    return max_p;
//}
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
vector<int> surfaceReconstruction::removePoints(VecArray &l_points, VecArray &r_points, float threshold) {
    vector<int> indices;
    float value;
    for (int i=0; i<l_points.size(); i++) {
            vec diff_point = l_points.at(i) - r_points.at(i);
            value = static_cast<float>(abs(pow(diff_point.x, 2) + pow(diff_point.y, 2) + pow(diff_point.z, 2)));
//            if (value > threshold)
                indices.emplace_back(i);
    }
    return indices;
}

//vector<int> surfaceReconstruction::removePoints(VecArray4 &l_points, VecArray4 &r_points, float threshold) {
//    vector<int> indices;
//    float value;
//    vec p1, p2;
//    for (int i=0; i<l_points.size(); i++) {
//        p1.x = l_points.at(i).x; p1.y = l_points.at(i).y; p1.z = l_points.at(i).z;
//        p2.x = r_points.at(i).x; p2.y = r_points.at(i).y; p2.z = r_points.at(i).z;
//        vec diff_point = p1 - p2;
//        value = static_cast<float>(abs(pow(diff_point.x, 2) + pow(diff_point.y, 2) + pow(diff_point.z, 2)));
////        if (value > threshold)
//            indices.emplace_back(i);
//    }
//    return indices;
//}

void surfaceReconstruction::normalize(vector<float> &values) {
    float min_value = min(values);
    float max_value = max(values);
    float diff = max_value - min_value;
    for (float &value : values) {
        value = (value - min_value)/diff;
    }
}

//void surfaceReconstruction::normalize(VecArray4 &values) {
//    vec min_p = min(values);
//    vec max_p = max(values);
//    vec diff = max_p - min_p;
//    for (auto &value : values) {
//        value.x =  (value.x - min_p.x)/diff.x;
//        value.y =  (value.y - min_p.y)/diff.y;
//        value.z =  (value.z - min_p.z)/diff.z;
//    }
//}

///*! \brief Convert RGB to HSV color space
//
//  Converts a given set of RGB values `r', `g', `b' into HSV
//  coordinates. The input RGB values are in the range [0, 1], and the
//  output HSV values are in the ranges h = [0, 360], and s, v = [0,
//  1], respectively.
//
//  \param fR Red component, used as input, range: [0, 1]
//  \param fG Green component, used as input, range: [0, 1]
//  \param fB Blue component, used as input, range: [0, 1]
//  \param fH Hue component, used as output, range: [0, 360]
//  \param fS Hue component, used as output, range: [0, 1]
//  \param fV Hue component, used as output, range: [0, 1]
//
//  Vec3b[0] = blue
//  Vec3b[1] = green
//  Vec3b[2] = red
//*/
//void surfaceReconstruction::RGBtoHSV(const Vec3b color, float& fH, float& fS, float& fV) {
//    float fR = color[2];
//    float fG = color[1];
//    float fB = color[0];
//    float fCMax = std::max(std::max(fR, fG), fB);
//    float fCMin = std::min(std::min(fR, fG), fB);
//    float fDelta = fCMax - fCMin;
//
//    if(fDelta > 0) {
//        if(fCMax == fR) {
//            fH = 60 * (fmod(((fG - fB) / fDelta), 6));
//        } else if(fCMax == fG) {
//            fH = 60 * (((fB - fR) / fDelta) + 2);
//        } else if(fCMax == fB) {
//            fH = 60 * (((fR - fG) / fDelta) + 4);
//        }
//
//        if(fCMax > 0) {
//            fS = fDelta / fCMax;
//        } else {
//            fS = 0;
//        }
//
//        fV = fCMax;
//    } else {
//        fH = 0;
//        fS = 0;
//        fV = fCMax;
//    }
//
//    if(fH < 0) {
//        fH = 360 + fH;
//    }
//}

//!---------------------------------------------------------------------------------------------------------------------