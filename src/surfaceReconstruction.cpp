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

    createButton("Test", test, this, QT_PUSH_BUTTON, true);
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

void surfaceReconstruction::alignFrames(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->all_points.clear();

    myClass->pcloud.clearPoints();
    myClass->getPointCLoud(myClass->l_points, myClass->l_frame_index);

//    myClass->r_frame_index += 10;

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
    vector<float> dist;
    float mean_distance;

    cout << "icp started..." << endl;
    myClass->pcloud.icp(r_uncolored_points, dist, mean_distance, error, iterations);

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
    pair<Eigen::Matrix3f, Eigen::Vector3f> R_t, total_R_t;
    Mat sobel_img, thresholded_img, bilateral_img;

    myClass->getPointCLoud(l_points, myClass->l_frame_index);
//    for (auto &l_point : l_points) {
//        myClass->all_points.emplace_back(l_point);
//    }

    myClass->pcloud.sobel(myClass->image_mat, sobel_img);
    myClass->pcloud.thresholding(sobel_img, thresholded_img);
    cout << thresholded_img.size << endl;
    myClass->pcloud.getEdges(thresholded_img, l_edges, 0);

    int frames{1};
    int counter{0};
    while(counter < frames) {
        r_points.clear();
        r_edges.clear();

        myClass->getPointCLoud(r_points, myClass->r_frame_index);
        r_uncolored_points = myClass->getFirstData(r_points);

        myClass->pcloud.sobel(myClass->image_mat, sobel_img);
        myClass->pcloud.thresholding(sobel_img, thresholded_img);
        myClass->pcloud.getEdges(thresholded_img, r_edges, 0);

        if(counter > 0)
            myClass->pcloud.transformPoints(total_R_t, r_edges);

        VecArray tree_data = l_edges;
        myClass->pcloud.m_dst_KDTree = new KDTree(tree_data);

        float error;
        int iterations{8};
        vector<float> dist;
        float mean_distance;

        cout << "icp started..." << endl;
        R_t = myClass->pcloud.icp(r_edges, dist, mean_distance, error, iterations);

        // TODO create total_R_t *= R_t
        if(counter > 0) {
            total_R_t.first *= R_t.first;
            total_R_t.second += R_t.second;
        }
        else
            total_R_t = R_t;

        myClass->pcloud.transformPoints(total_R_t, r_uncolored_points);

        /// test
//        cout << "in test" << endl;
        // remove nearest points with distance less than ε
//        VecArray tree_data2 = r_uncolored_points;
//        cout << "kdtree construction" << endl;
//        myClass->pcloud.m_dst_KDTree = new KDTree(tree_data2);
//
//        VecArray nearestPoints;
//        dist.clear();

//        cout << "kNearest" << endl;
//        myClass->pcloud.kNearest(myClass->getFirstData(l_points), nearestPoints, dist, 1);
//        float threshold = 0.15f;

//        for (int i=0; i<nearestPoints.size(); i++) {
        for (int i=0; i<l_points.size(); i++) {
//            cout << dist[i] << endl;
//            if (dist[i] > threshold)
                myClass->all_points.emplace_back(l_points[i]);
        }

        if (counter == frames-1) {
            for (int j=0; j<r_uncolored_points.size(); j++) {
                myClass->all_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);
            }
        }
        else {
            l_points.clear();
            for (int j=0; j<r_uncolored_points.size(); j++)
                l_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);
        }


        /// test

//        for (int j=0; j<r_uncolored_points.size(); j++)
//            myClass->all_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);

        l_edges = r_edges;
        myClass->r_frame_index++;
        counter++;
    }
    myClass->m_flag = true;
}

void surfaceReconstruction::test(int index, void* object) {
    auto *myClass = (surfaceReconstruction *) object;

    myClass->all_points.clear();
    myClass->m_vertices.clear();
    myClass->segments.clear();
    myClass->m_curvature.clear();
    myClass->m_normals.clear();
    myClass->pointFeatures.clear();
    myClass->segments.clear();
    myClass->test_segments.clear();
    myClass->mesh = new Mesh;

    vector<pair<vec, Vec3b>> l_points;
    myClass->getPointCLoud(l_points, myClass->l_frame_index);
    for (auto &l_point : l_points) {
        myClass->all_points.emplace_back(l_point);
    }

    cout << "frame index = " << myClass->l_frame_index << endl;
    cout << "l points are " << l_points.size() << endl;

    VecArray vertices = myClass->getFirstData(l_points);
    myClass->m_vertices = vertices;

    cout << "mesh tri are " << myClass->mesh->getTriangles().size() << endl;
    cout << "mesh vertices are " << myClass->mesh->getVertices().size() << endl;

    vector<int> *tri_indices = myClass->pcloud.triangulateMesh(vertices, myClass->mesh, 1.5f);
//    myClass->mesh->update();

    cout << "triangles are " << myClass->mesh->getTriangles().size() << endl;

    myClass->pcloud.getNormals(myClass->mesh, tri_indices, myClass->m_normals);

    myClass->pcloud.getCurvature(myClass->mesh, tri_indices, myClass->m_curvature);

    for (int i = 0; i < myClass->all_points.size(); i++) {
        myClass->m_normals[i] += myClass->all_points[i].first;
    }

    vector<int> *tri_indices_tmp = tri_indices;
    int rings{1};
    cout << "finding ringNeighbours with " << rings << " ring(s) ..." << endl;
    vector<int> *ringNeighbours = myClass->pcloud.getNRingNeighborhood(myClass->mesh, tri_indices_tmp, rings);

    int size = myClass->mesh->getVertices().size();
    float luminance;
    for (int i = 0; i < size; i++) {
        luminance = myClass->pcloud.getLuminance(l_points[i].second);
        myClass->pointFeatures.emplace_back(
                PointFeatures(i, myClass->m_vertices[i], myClass->m_normals[i], myClass->m_curvature[i],
                              luminance, ringNeighbours[i]));
    }

    cout << "segmentation ..." << endl;
    vector<int> region;
    vector<int> hash;

    for (int i = 0; i < myClass->pointFeatures.size(); i++) {
        hash.emplace_back(0);
    }

    /// store to myClass->segments the indices of pointFeatures
    for (int i = 0; i < myClass->pointFeatures.size(); i++) {

        if ((myClass->pointFeatures[i].neighborVertices.size() != 0) &&
            !(myClass->pcloud.zeroPoint(myClass->pointFeatures[i].coordinates)) && (hash[i] == 0)) {
            region.clear();
            myClass->pcloud.segmentation(myClass->pointFeatures, region, hash, i);
            if (region.size() > 10) {
                myClass->segments.emplace_back(region);
            }
        }
    }

    cout << "there are " << myClass->segments.size() << " segments" << endl << endl;

    vec plane_normal, centroid;
    vector<VecArray> new_segments;
    VecArray new_segments2;
    vector<float> residuals, percentages;
    vector<int> pointsLeftOfPlane, pointsRightOfPlane;
    int counter1, counter2;
    vec point;

    float residual, curv;
    int my_index;

    for (int i=0; i<myClass->segments.size(); i++) {
        myClass->pcloud.planeFitting(myClass->pointFeatures, myClass->segments[i], plane_normal, centroid);
        myClass->m_plane = Plane(centroid, plane_normal);
        myClass->planes.emplace_back(myClass->m_plane);

        residual = myClass->pcloud.getResiduals(myClass->pointFeatures, myClass->segments[i], myClass->m_plane);
        residuals.emplace_back(residual);

        counter1 = counter2 = 0;
        for (int j=0; j<myClass->segments[i].size(); j++) {
            my_index = myClass->segments[i][j];
            curv += myClass->pointFeatures[my_index].curvature;

            /// Returns the signed distance of this plane to the given point.
            /** If this function returns a negative value, the given point lies in the negative halfspace of this plane.
                Conversely, if a positive value is returned, then the given point lies in the positive halfspace of this plane.
                @see Distance(), IsOnPositiveSide(), AreOnSameSide(). */
            point = myClass->pointFeatures[my_index].coordinates;
            if (myClass->m_plane.SignedDistance(point) > 0)
                counter1+=1;
            else
                counter2+=1;
        }

        pointsLeftOfPlane.emplace_back(counter1);
        pointsRightOfPlane.emplace_back(counter2);
        if (pointsLeftOfPlane[i] > pointsRightOfPlane[i])
            percentages.emplace_back(100.0f*(float(pointsLeftOfPlane[i] - pointsRightOfPlane[i])/pointsLeftOfPlane[i]));
        else
            percentages.emplace_back(100.0f*(float(pointsRightOfPlane[i] - pointsLeftOfPlane[i])/pointsRightOfPlane[i]));

    }

    myClass->normalize(residuals);

    for (int i=0; i<myClass->segments.size(); i++) {
        cout << "residual = " << residuals[i] << endl;
        cout << "pointsLeftOfPlane = " << pointsLeftOfPlane[i] << endl;
        cout << "pointsRightOfPlane = " << pointsRightOfPlane[i] << endl;
        cout << "percentage = " << percentages[i] << endl << endl;

        if ((pointsRightOfPlane[i] != 0) && (pointsLeftOfPlane[i] != 0) ) {

            if ((percentages[i] > 30.0f) && (residuals[i]) > 0.4) {

                new_segments.clear();

                cout << "segmentation 2 ..." << endl;
                myClass->pcloud.segmentation2(myClass->pointFeatures, myClass->segments[i], new_segments, myClass->planes[i],
                                              myClass->mesh);

                myClass->test_segments.emplace_back(new_segments);
            }
            else {
                new_segments.clear();
                new_segments2.clear();

                for (int j=0; j<myClass->segments[i].size(); j++) {
                    my_index = myClass->segments[i][j];
                    new_segments2.emplace_back(myClass->pointFeatures[my_index].coordinates);
                }
                new_segments.emplace_back(new_segments2);
                myClass->test_segments.emplace_back(new_segments);
            }
        }
    }

    cout << "finito" << endl;

    myClass->m_flag = true;
//    delete[] tri_indices;
}

//void surfaceReconstruction::alignFramesKnnSobelNormals(int index, void* object) {
//    auto * myClass = (surfaceReconstruction*) object;
//    myClass->all_points.clear();
//
//    VecArray l_edges, r_edges, r_uncolored_points;
//    vector<pair<vec,Vec3b>> l_points, r_points;
//    pair<Eigen::Matrix3f, Eigen::Vector3f> R_t, total_R_t;
//    Mat sobel_img, thresholded_img;
//
//    myClass->getPointCLoud(l_points, myClass->l_frame_index);
//    for (auto &l_point : l_points) {
//        myClass->all_points.emplace_back(l_point);
//    }
//
//    myClass->pcloud.sobel(myClass->image_mat, sobel_img);
//    myClass->pcloud.thresholding(sobel_img, thresholded_img);
//    myClass->pcloud.getEdges(thresholded_img, l_edges, 0);
//
//    // compute the angle between the data point normal direction and the neighboring points normal direction
//    vector<double> curve_degrees;
//    myClass->pcloud.getNormalAngle(myClass->image_mat, thresholded_img, myClass->depth_mat, 0, curve_degrees);
//    myClass->normalize(curve_degrees);
//
//    int frames{1};
//    int counter{0};
//    while(counter < frames) {
//        r_points.clear();
//        r_edges.clear();
//
//        myClass->getPointCLoud(r_points, myClass->r_frame_index);
//        r_uncolored_points = myClass->getFirstData(r_points);
//
//        myClass->pcloud.sobel(myClass->image_mat, sobel_img);
//        myClass->pcloud.thresholding(sobel_img, thresholded_img);
//        myClass->pcloud.getEdges(thresholded_img, r_edges, 0);
//
//        if(counter > 0)
//            myClass->pcloud.transformPoints(total_R_t, r_edges);
//
//        VecArray tree_data = l_edges;
//        myClass->pcloud.m_dst_KDTree = new KDTree(tree_data);
//
//        float error;
//        int iterations{1};
//        vector<float> dist;
//        float mean_distance;
//
//        cout << "icp started..." << endl;
//        if (counter == 0)
//            R_t = myClass->pcloud.icpCurves(r_edges, dist, mean_distance, error, iterations, curve_degrees);
//        else
//            R_t = myClass->pcloud.icp(r_edges, dist, mean_distance, error, iterations);
//
//        // TODO create total_R_t *= R_t
//        if(counter > 0) {
//            total_R_t.first *= R_t.first;
//            total_R_t.second += R_t.second;
//        }
//        else
//            total_R_t = R_t;
//
//        myClass->pcloud.transformPoints(total_R_t, r_uncolored_points);
//
//        for (int j=0; j<r_uncolored_points.size(); j++)
//            myClass->all_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);
//
//        l_edges = r_edges;
//        myClass->r_frame_index++;
//        counter++;
//    }
//    myClass->m_flag = true;
//}

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

template <typename T>
T surfaceReconstruction::min(const vector<T> &values) {
    T min_value{values.at(0)};
    for(int i=1; i<values.size(); i++) {
        if (values.at(i) < min_value)
            min_value = values.at(i);
    }
    return min_value;
}

template <typename T>
T surfaceReconstruction::max(const vector<T> &values) {
    T max_value{values.at(0)};
    for(int i=1; i<values.size(); i++) {
        if (values.at(i) > max_value)
            max_value = values.at(i);
    }
    return max_value;
}

//!---------------------------------------------------------------------------------------------------------------------

//!---------------------------------------------------------------------------------------------------------------------
//! draw functions
//!---------------------------------------------------------------------------------------------------------------------
void surfaceReconstruction::draw() {
    if (m_flag) {
//        mesh->draw(Colour::blue, SOLID);
        drawAdjacentPoints();
    }
    else {
        for (auto &l_point : l_points) {
//            Point3D(l_point.first.x, l_point.first.y, l_point.first.z,
//                    Colour(l_point.second[2], l_point.second[1], l_point.second[0])).draw();
        }
    }
}

void surfaceReconstruction::drawAdjacentPoints() {
//    cout << "all_points size = " << all_points.size() << endl;
//    mesh->draw(Colour::green, SOLID);
//    mesh->draw(Colour::black, WIRE);
//    mesh->draw(Colour::black, NORMALS);

    for (int i=0; i<all_points.size(); i++) {
//        Point3D(all_points[i].first.x, all_points[i].first.y, all_points[i].first.z,
//                    Colour(all_points[i].second[2], all_points[i].second[1], all_points[i].second[0])).draw();
    }


//    Point3D(testPoint.x, testPoint.y, testPoint.z, Colour::yellow).draw();

    vector<vvr::Colour> cols;
    cols.emplace_back(Colour::yellow);
    cols.emplace_back(Colour::black);
    cols.emplace_back(Colour::blue);
    cols.emplace_back(Colour::darkOrange);
    cols.emplace_back(Colour::darkRed);
    cols.emplace_back(Colour::grey);
    cols.emplace_back(Colour::darkGreen);
    cols.emplace_back(Colour::magenta);
    cols.emplace_back(Colour::red);
    cols.emplace_back(Colour::orange);
    cols.emplace_back(Colour::green);
    cols.emplace_back(Colour::yellowGreen);
    cols.emplace_back(Colour::white);


    int type{0}, index;
//    for (int i=0; i<segments.size(); i++) {
//        if(type > cols.size())
//            type = 0;
//
//        for (int j=0; j<segments[i].size(); j++) {
//            index = segments[i][j];
//            Point3D(pointFeatures[index].coordinates.x,
//                    pointFeatures[index].coordinates.y,
//                    pointFeatures[index].coordinates.z, cols[type]).draw();
//        }
//        type++;
//    }

    for (int i=0; i<test_segments.size(); i++) {

        for (int j=0; j<test_segments[i].size(); j++) {
            if(type > cols.size())
                type = 0;

            for (int m=0; m<test_segments[i][j].size(); m++) {

                Point3D(test_segments[i][j][m].x,
                        test_segments[i][j][m].y,
                        test_segments[i][j][m].z, cols[type]).draw();

            }

            type++;
        }
    }

//    int col_r, col_g, col_b = 0;
//    for (int i=0; i<segments.size(); i++) {
//        for (int j=0; j<segments[i].size(); j++) {
//            Point3D(segments[i][j].coordinates.x, segments[i][j].coordinates.y, segments[i][j].coordinates.z, Colour(col_r, col_g, col_b)).draw();
//        }
//        if (col_r > 255)
//            col_r = 0;
//        else
//            col_r += 5;
//        if (col_g > 255)
//            col_g = 0;
//        else
//            col_g += 5;
//        if (col_b > 255)
//            col_b = 0;
//        else
//            col_b += 5;
//    }

    ///test curvature
//     convert curvature to [0-255]
    double min_v, max_v;
    min_v = min(m_curvature);
    max_v = max(m_curvature);
    for (int i=0; i<m_curvature.size(); i++) {
//        m_curvature[i] = static_cast<float>(255 * ((abs(m_curvature[i]) - min_v) / (max_v - min_v) ));
    }

//    cout << "curv size = " << m_curvature.size() << endl;
//    cout << "curv[0] = " << m_curvature[0] << endl;
//    cout << "curv[1] = " << m_curvature[1] << endl;
//    cout << "curv[2] = " << m_curvature[2] << endl;

    for (int i=0; i<all_points.size(); i++) {
//        Point3D(all_points[i].first.x, all_points[i].first.y, all_points[i].first.z,
//                    Colour(static_cast<unsigned char>(m_curvature[i]), 0, 0)).draw();
//        if (m_curvature[i] > 0.1)
//            Point3D(all_points[i].first.x, all_points[i].first.y, all_points[i].first.z, Colour::white).draw();
    }
    ///test curvature

    double min_x{1}, min_y{1}, min_z{1};
    double max_x{-1}, max_y{-1}, max_z{-1};
    for (int i=1; i<m_normals.size(); i++) {
        if(m_normals[i].x < min_x)
            min_x = m_normals[i].x;
        if(m_normals[i].y < min_y)
            min_y = m_normals[i].y;
        if(m_normals[i].z < min_z)
            min_z = m_normals[i].z;

        if(m_normals[i].x > max_x)
            max_x = m_normals[i].x;
        if(m_normals[i].y > max_y)
            max_y = m_normals[i].y;
        if(m_normals[i].z > max_z)
            max_z = m_normals[i].z;
    }

    LineSeg3D line, line2;
    vec n;
    double x,y,z;
//    for (int i=0; i<m_normals.size(); i++) {
    for (int i=0; i<m_normals.size(); i+=5) {
            n = m_normals[i];
            line.x1 = all_points[i].first.x;
            line.y1 = all_points[i].first.y;
            line.z1 = all_points[i].first.z;

            line.x2 = n.x;
            line.y2 = n.y;
            line.z2 = n.z;

//            line.x2 = all_points[i].first.x + n.x;
//            line.y2 = all_points[i].first.y + n.y;
//            line.z2 = all_points[i].first.z + n.z;

//            line.setColour(Colour::yellow);
            line.setColour(Colour::yellow);
//            line.draw();

            x = 255*(abs(n.x))/(max_x);
            y = 255*(abs(n.y))/(max_y);
            y = 255*(abs(n.z))/(max_z);

//            x = 255*((abs(n.x) + 1)/2.0f);
//            y = 255*((abs(n.y) + 1)/2.0f);
//            z = 127*(n.z);
//            z = (1-(abs(n.z)))*128 - 1;

            line.setColour(Colour(x,y,z));
//            line.draw();

//            LineSeg3D(all_points[i].first.x, all_points[i].first.y, all_points[i].first.z,
//                    all_points[i].first.x+n.x, all_points[i].first.y+n.y, all_points[i].first.z+n.z,
//                    Colour(x,y,z)).draw();

    }

    //! Draw plane

//    vvr::Colour colPlane(0x41, 0x14, 0xB3);
//    float u = 30, v = 30;
//    math::vec p0(planes[used].Point(-u, -v, math::vec(0, 0, 0)));
//    math::vec p1(planes[used].Point(-u, v, math::vec(0, 0, 0)));
//    math::vec p2(planes[used].Point(u, -v, math::vec(0, 0, 0)));
//    math::vec p3(planes[used].Point(u, v, math::vec(0, 0, 0)));
//    math2vvr(math::Triangle(p0, p1, p2), colPlane).draw();
//    math2vvr(math::Triangle(p2, p1, p3), colPlane).draw();

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

template <typename T>
void surfaceReconstruction::normalize(vector<T> &values) {
    T min_value = min(values);
    T max_value = max(values);
    T diff = max_value - min_value;
    for (T &value : values) {
        value = (value - min_value)/diff;
    }
}
//!---------------------------------------------------------------------------------------------------------------------