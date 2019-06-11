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
void surfaceReconstruction::alignFramesKnn(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->all_points.clear();
    VecArray l_uncolored_points, r_uncolored_points, nearestPoints;
    vector< pair <vec,Vec3b>> l_points, r_points;
    vector<float> dist;
    int numFrames = {1};

    myClass->getPointCLoud(l_points, myClass->l_frame_index);
    cout << "initial frame = " << myClass->l_frame_index << endl;

    for (int i=0; i<numFrames; i++) {
        cout << "next frame = " << myClass->r_frame_index << endl;
        l_uncolored_points = myClass->getFirstData(l_points);
        r_points.clear();
        myClass->getPointCLoud(r_points, myClass->r_frame_index);
        r_uncolored_points = myClass->getFirstData(r_points);

        VecArray tree_data = l_uncolored_points;
        myClass->pcloud.m_dst_KDTree = new KDTree(tree_data);

        float error{0.0f};
        int iteration{2};
        int counter{0};
        while(counter < iteration) {
            nearestPoints.clear();
            dist.clear();
            myClass->pcloud.kNearest(r_uncolored_points, nearestPoints, dist, 1);

            pair<Eigen::Matrix3f, Eigen::Vector3f> R_t;
            R_t = myClass->pcloud.computeRigidTransform(nearestPoints, r_uncolored_points);

            myClass->pcloud.transformPoints(R_t, r_uncolored_points);

            cout << "mean dist = " << myClass->vectorSum(dist)/(float)dist.size() << endl;

            myClass->normalize(dist);
            error = myClass->vectorSum(dist)/(float)dist.size();
            cout << "iteration = " << counter << endl;
            cout << "error = " << error << endl << endl;

            counter++;

            // refresh data in order to draw them
            if (iteration == counter) {
                vector<int> indices = myClass->removePoints(l_uncolored_points, nearestPoints, 0.1f);
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

float surfaceReconstruction::max(const vector<float> &values) {
    float max_value{values.at(0)};
    for(int i=1; i<values.size(); i++) {
        if (i > max_value)
            max_value = i;
    }
    return max_value;
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
            if (value > threshold)
                indices.emplace_back(i);
    }
    return indices;
}

void surfaceReconstruction::normalize(vector<float> &values) {
    float min_value = min(values);
    float max_value = max(values);
    float diff = max_value - min_value;
    for (float &value : values) {
        value = (value - min_value)/diff;
    }
}

//!---------------------------------------------------------------------------------------------------------------------