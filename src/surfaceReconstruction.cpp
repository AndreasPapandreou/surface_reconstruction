#include "surfaceReconstruction.h"
#include "dataset.h"

using namespace std;
using namespace vvr;

/// contructor
surfaceReconstruction::surfaceReconstruction(int &index) {
    initialize(index);
    createGui();
}

/// initialize some class parameters
void surfaceReconstruction::initialize(int &index) {
    image_prefix = convertToStr(generic::convertToDatasetType(index));
    num_images = generic::DatasetSize(generic::convertToDatasetSize(index));
    generic::stereo_dir += image_prefix;
    l_frame_index = index;
    r_frame_index = index+1;
    lFrame = rFrame = false;
}

//!*********************************************************************************************************************
//! gui functions
//!*********************************************************************************************************************
void surfaceReconstruction::createGui() {
    //! Load settings.
    vvr::Shape::DEF_LINE_WIDTH = 4;
    vvr::Shape::DEF_POINT_SIZE = 10;
    m_perspective_proj = true;
    m_bg_col = Colour("768E77");
    m_obj_col = Colour("454545");

    namedWindow("gui", WINDOW_NORMAL);
    moveWindow("gui", 200, 600);
    createTrackbar("next frame", "gui", &slider_value, num_images, changeFrame, this);
    setTrackbarMin("next frame", "gui", 1);
    createTrackbar("frames", "gui", &slider_value2, 30, setNumOfFrames, this);
    setTrackbarMin("frames", "gui", 2);
    createTrackbar("threshold", "gui", &slider_value3, 100, setThreshold, this);
    setTrackbarMin("threshold", "gui", 10);
    setTrackbarPos("threshold", "gui", 50);

    createButton("Draw left frame", drawLeftFrame, this, QT_PUSH_BUTTON|QT_NEW_BUTTONBAR, true);
    createButton("Draw right frame", drawRightFrame, this, QT_PUSH_BUTTON|QT_NEW_BUTTONBAR, true);
    createButton("Show lines between two frames", alignTwoFrames, this, QT_PUSH_BUTTON|QT_NEW_BUTTONBAR, true);
    createButton("Align frames using knn", alignFramesKnn, this, QT_PUSH_BUTTON|QT_NEW_BUTTONBAR, true);
    createButton("Align frames using knn-sobel", alignFramesKnnSobel, this, QT_PUSH_BUTTON|QT_NEW_BUTTONBAR, true);
    createButton("Segmentation", segmentation, this, QT_PUSH_BUTTON|QT_NEW_BUTTONBAR, true);
    createButton("Left frame", setLeftFrame, this, QT_CHECKBOX, true);
    createButton("Right frame", setRightFrame, this, QT_CHECKBOX, false);

    /// initialize with the two first frames
    showFrames(1);
}

void surfaceReconstruction::showFrames(int index) {
    int size{300}, num_rows{2}, num_cols{1}, max;
    float scale;
    /// Create a new 3 channel image
    Mat DispImage = Mat::zeros(Size(100 + size*num_rows, 60 + size*num_cols), CV_8UC3);

    l_frame = getFrame(index);
    l_frame_index = index;
    index++;
    r_frame = getFrame(index);
    r_frame_index = index;

    Mat images[2] = {l_frame, r_frame};
    for (int i = 0, m = 20, n = 20; i < 2; i++, m += (20 + size)) {
        /// find whether height or width is greater in order to resize the image
        max = (images[i].cols > images[i].rows)? images[i].cols: images[i].rows;

        /// set the scaling factor to resize the image
        scale = ((float) max / size );

        /// set the image ROI to display the current image
        cv::Rect ROI(m, n, (int)( images[i].cols/scale ), (int)( images[i].rows/scale ));
        Mat temp;
        cv::resize(images[i], temp, Size(ROI.width, ROI.height));
        temp.copyTo(DispImage(ROI));
    }

    /// create a new window and show the new frames
    namedWindow("gui", WINDOW_NORMAL);
    moveWindow("gui", 200, 600);
    createTrackbar("next frame", "gui", &slider_value, num_images, changeFrame, this);
    setTrackbarMin("next frame", "gui", 1);
    createTrackbar("frames", "gui", &slider_value2, 30, setNumOfFrames, this);
    setTrackbarMin("frames", "gui", 2);
    createTrackbar("threshold", "gui", &slider_value3, 100, setThreshold, this);
    setTrackbarMin("threshold", "gui", 10);
    setTrackbarPos("threshold", "gui", 50);
    imshow("gui", DispImage);
    waitKey(1);
}

void surfaceReconstruction::changeFrame(int x, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    if (x >=1 && x < myClass->num_images)
        myClass->showFrames(x);
    //TODO delete myClass from everywhere
}

void surfaceReconstruction::alignTwoFrames(int x, void* object) {
    auto *myClass = (surfaceReconstruction *) object;

    cout << endl << "Show lines between two frames ..." << endl;

    /// clear variables
    myClass->all_points.clear();
    myClass->l_points.clear();
    myClass->r_points.clear();

    /// declare variables
    VecArray r_uncolored_points, l_uncolored_points;
    vector<pair<vec, Vec3b>> l_points, r_points;
    pair<Eigen::Matrix3f, Eigen::Vector3f> R_t, total_R_t;

    /// create point cloud for left frame
    myClass->getPointCLoud(l_points, myClass->l_frame_index);
    /// get only the coordinates of the left points
    l_uncolored_points = myClass->getFirstData(l_points);

    /// set the index of right frame
    myClass->r_frame_index = myClass->l_frame_index + 1;

    /// create point cloud for right frame
    myClass->getPointCLoud(r_points, myClass->r_frame_index);
    /// get only the coordinates of the right points
    r_uncolored_points = myClass->getFirstData(r_points);

    /// kdtree construction
    VecArray tree_data = r_uncolored_points;
    myClass->pcloud.m_dst_KDTree = new KDTree(tree_data);

    vector<float> dist; VecArray nearestPoints;

    /// run kNearest
    const float t = vvr::getSeconds();
    myClass->pcloud.kNearest(myClass->getFirstData(l_points), nearestPoints, dist, 1);
    const float Time_of_aligment = vvr::getSeconds() - t;

    for (auto &nearestPoint : nearestPoints)
        myClass->l_points.emplace_back(nearestPoint);

    for (auto &r_uncolored_point : r_uncolored_points)
        myClass->r_points.emplace_back(r_uncolored_point);

    /// get metrics
    double total_distance = myClass->pcloud.vectorSum(dist);
    cout << "Total distance = " << total_distance << endl;
    cout << "Mean distance = " << total_distance/double(dist.size()) << endl;
    echo(Time_of_aligment);

    cout << "End" << endl;

    myClass->draw_lines = true;
    myClass->draw_frame = false;
    myClass->draw_segmentation = false;
}

void surfaceReconstruction::setNumOfFrames(int x, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->num_frames = x;
}

void surfaceReconstruction::setThreshold(int x, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->threshold = x;
}

void surfaceReconstruction::drawLeftFrame(int x, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    drawFrame(myClass->l_frame_index, object);
}

void surfaceReconstruction::drawRightFrame(int x, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->r_frame_index = myClass->l_frame_index + 1;
    drawFrame(myClass->r_frame_index, object);
}

void surfaceReconstruction::drawFrame(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;

    cout << endl << "Draw frame ..." << endl;

    myClass->pcloud.clearPoints();
    myClass->getPointCLoud(myClass->all_points, index);

    cout << "End" << endl;

    myClass->draw_frame = true;
    myClass->draw_lines = false;
    myClass->draw_segmentation = false;
}

void surfaceReconstruction::alignFramesKnn(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;

    cout << endl << "Align frames using knn ..." << endl;

    /// clear variable
    myClass->all_points.clear();

    /// declare variables
    VecArray r_uncolored_points, l_uncolored_points;
    vector<pair<vec,Vec3b>> l_points, r_points;
    pair<Eigen::Matrix3f, Eigen::Vector3f> R_t, total_R_t;

    /// create point cloud for left frame
    myClass->getPointCLoud(l_points, myClass->l_frame_index);
    /// get only the coordinates of the left points
    l_uncolored_points = myClass->getFirstData(l_points);

    /// set the index of right frame
    myClass->r_frame_index = myClass->l_frame_index+1;

    int frames = myClass->num_frames-1;
    int counter{0};
    float total_icp{0.0f}, total_knn{0.0f}, each_icp, each_knn;

    /// run for all frames
    while(counter < frames) {
        r_points.clear();

        /// create point cloud for right frame
        myClass->getPointCLoud(r_points, myClass->r_frame_index);
        /// get only the coordinates of the right points
        r_uncolored_points = myClass->getFirstData(r_points);

        /// transform points with the rotation and translation matrices
        if(counter > 0)
            myClass->pcloud.transformPoints(total_R_t, r_uncolored_points);

        VecArray tree_data = l_uncolored_points;
        myClass->pcloud.m_dst_KDTree = new KDTree(tree_data);

        float mean_distance;
        vector<float> dist;
        int iterations{2};

        /// run icp
        each_icp = vvr::getSeconds();
        R_t = myClass->pcloud.icp(r_uncolored_points, dist, mean_distance, iterations);
        total_icp += vvr::getSeconds() - each_icp;

        // TODO create total_R_t *= R_t
        if(counter > 0) {
            total_R_t.first *= R_t.first;
            total_R_t.second += R_t.second;
        }
        else
            total_R_t = R_t;

        /// transform points with the rotation and translation matrices
        myClass->pcloud.transformPoints(total_R_t, r_uncolored_points);

        /// kdtree construction
        VecArray tree_data2 = r_uncolored_points;
        myClass->pcloud.m_dst_KDTree = new KDTree(tree_data2);

        VecArray nearestPoints; dist.clear();

        /// run kNearest
        each_knn = vvr::getSeconds();
        myClass->pcloud.kNearest(myClass->getFirstData(l_points), nearestPoints, dist, 1);
        total_knn += vvr::getSeconds() - each_knn;

        float threshold = 2*mean_distance;
        /// remove nearest points with distance less than 2*threshold
        for (int i=0; i<l_points.size(); i++) {
            if (dist[i] > threshold)
                myClass->all_points.emplace_back(l_points[i]);
        }

        if (counter == frames-1) {
            for (int j=0; j<r_uncolored_points.size(); j++) {
                myClass->all_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);
            }
        }
        else {
            myClass->r_frame_index++;
            l_points.clear();
            for (int j=0; j<r_uncolored_points.size(); j++)
                l_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);
        }

        /// set l_points with r_points
        l_uncolored_points = r_uncolored_points;
        counter++;
    }

    cout << "For " << counter+1 << " frames :" << endl;
    cout << "Time for icp = " << total_icp << endl;
    cout << "Time for kNearest = " << total_knn << endl;

    cout << "End" << endl;

    myClass->draw_frame = true;
    myClass->draw_lines = false;
    myClass->draw_segmentation = false;
}

void surfaceReconstruction::alignFramesKnnSobel(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;

    cout << endl << "Align frames using knn-sobel ..." << endl;

    /// clear variable
    myClass->all_points.clear();

    /// set the index of right frame
    myClass->r_frame_index = myClass->l_frame_index+1;

    /// declare variables
    VecArray l_edges, r_edges, r_uncolored_points;
    vector<pair<vec,Vec3b>> l_points, r_points;
    pair<Eigen::Matrix3f, Eigen::Vector3f> R_t, total_R_t;
    Mat sobel_img, thresholded_img, bilateral_img;

    /// create point cloud for left frame
    myClass->getPointCLoud(l_points, myClass->l_frame_index);
    /// apply sobel
    myClass->pcloud.sobel(myClass->image_mat, sobel_img);
    /// apply thresholding
    myClass->pcloud.thresholding(sobel_img, thresholded_img);
    /// get the edges of image
    myClass->pcloud.getEdges(thresholded_img, l_edges, 0);

    int frames = myClass->num_frames-1;
    int counter{0};
    float total_icp{0.0f}, total_knn{0.0f}, each_icp, each_knn;

    /// run for all frames
    while(counter < frames) {
        r_points.clear(); r_edges.clear();

        /// create point cloud for right frame
        myClass->getPointCLoud(r_points, myClass->r_frame_index);
        /// get only the coordinates of the right points
        r_uncolored_points = myClass->getFirstData(r_points);

        /// apply sobel
        myClass->pcloud.sobel(myClass->image_mat, sobel_img);
        /// apply thresholding
        myClass->pcloud.thresholding(sobel_img, thresholded_img);
        /// get the edges of image
        myClass->pcloud.getEdges(thresholded_img, r_edges, 0);

        /// transform points with the rotation and translation matrices
        if(counter > 0)
            myClass->pcloud.transformPoints(total_R_t, r_edges);

        VecArray tree_data = l_edges;
        myClass->pcloud.m_dst_KDTree = new KDTree(tree_data);

        float mean_distance;
        vector<float> dist;
        int iterations{8};

        /// run icp
        each_icp = vvr::getSeconds();
        R_t = myClass->pcloud.icp(r_edges, dist, mean_distance, iterations);
        total_icp += vvr::getSeconds() - each_icp;
//        cout << "mean_distance = " << mean_distance << endl;

        // TODO create total_R_t *= R_t
        if(counter > 0) {
            total_R_t.first *= R_t.first;
            total_R_t.second += R_t.second;
        }
        else
            total_R_t = R_t;

        /// transform points with the rotation and translation matrices
        myClass->pcloud.transformPoints(total_R_t, r_uncolored_points);

        /// kdtree construction
        VecArray tree_data2 = r_uncolored_points;
        myClass->pcloud.m_dst_KDTree = new KDTree(tree_data2);

        VecArray nearestPoints; dist.clear();

        /// run kNearest
        each_knn = vvr::getSeconds();
        myClass->pcloud.kNearest(myClass->getFirstData(l_points), nearestPoints, dist, 1);
        total_knn += vvr::getSeconds() - each_knn;

        float threshold = 2*mean_distance;

        /// remove nearest points with distance less than 2*threshold
        for (int i=0; i<l_points.size(); i++) {
            if (dist[i] > threshold)
                myClass->all_points.emplace_back(l_points[i]);
        }

        if (counter == frames-1) {
            for (int j=0; j<r_uncolored_points.size(); j++) {
                myClass->all_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);
            }
        }
        else {
            myClass->r_frame_index++;
            l_points.clear();
            for (int j=0; j<r_uncolored_points.size(); j++)
                l_points.emplace_back(r_uncolored_points.at(j), r_points.at(j).second);
        }

        l_edges = r_edges;
        counter++;
    }

    cout << "For " << counter+1 << " frames :" << endl;
    cout << "Time for icp = " << total_icp << endl;
    cout << "Time for kNearest = " << total_knn << endl;

    cout << "End" << endl;

    myClass->draw_frame = true;
    myClass->draw_lines = false;
    myClass->draw_segmentation = false;
}

void surfaceReconstruction::segmentation(int index, void* object) {
    auto *myClass = (surfaceReconstruction *) object;

    cout << endl << "Segmentation ..." << endl;

    /// choose left or right frame for segmentation
    int frame_index;
    if (myClass->lFrame == myClass->rFrame) {
        cerr << "You must choose only one frame!" << endl;
        return;
    }
    else if (myClass->lFrame)
        frame_index = myClass->l_frame_index;
    else {
        myClass->r_frame_index = myClass->l_frame_index + 1;
        frame_index = myClass->r_frame_index;
    }

    /// clear variables
    myClass->all_points.clear();
    myClass->segments.clear();
    myClass->m_curvature.clear();
    myClass->m_normals.clear();
    myClass->planes.clear();
    myClass->pointFeatures.clear();
    myClass->final_segments.clear();

    /// get point cloud
    vector<pair<vec, Vec3b>> points;
    myClass->getPointCLoud(points, frame_index);
    for (auto &point : points) {
        myClass->all_points.emplace_back(point);
    }

    /// extract only the points without their colors
    VecArray vertices = myClass->getFirstData(points);

    /// mesh triangulation
    vector<int> *tri_indices = myClass->pcloud.triangulateMesh(vertices, myClass->mesh, 1.5f);

    /// compute normals
    myClass->pcloud.getNormals(myClass->mesh, tri_indices, myClass->m_normals);

    /// compute curvature
    myClass->pcloud.getCurvature(myClass->mesh, tri_indices, myClass->m_curvature);

    /// add original coordinates with their normals
    for (int i = 0; i < myClass->all_points.size(); i++) {
        myClass->m_normals[i] += myClass->all_points[i].first;
    }

    /// find ringNeighbours
    vector<int> *tri_indices_tmp = tri_indices; int rings{1};
    vector<int> *ringNeighbours = myClass->pcloud.getNRingNeighborhood(myClass->mesh, tri_indices_tmp, rings);

    /// store the above results for all points to struct
    for (int i = 0; i<myClass->mesh->getVertices().size(); i++) {
        myClass->pointFeatures.emplace_back(
        PointFeatures(i, vertices[i], myClass->m_normals[i], myClass->m_curvature[i], ringNeighbours[i]));
    }

    vector<int> hash;
    /// create hash table in order to check if a point has already been processed
    for (int i = 0; i < myClass->pointFeatures.size(); i++)
        hash.emplace_back(0);

    /// counting time
    const float t = vvr::getSeconds();

    /// segmentation
    vector<int> region;
    /// store the indices of each segment in separate vector, these indices refer to the structure pointFeatures
    for (int i = 0; i < myClass->pointFeatures.size(); i++) {
        if ( (!myClass->pointFeatures[i].neighborVertices.empty()) &&
             !(myClass->pcloud.zeroPoint(myClass->pointFeatures[i].coordinates)) &&
             (hash[i] == 0) ) {

            region.clear();
            myClass->pcloud.segmentation(myClass->pointFeatures, region, hash, i);
            if (region.size() > 20) {
//            if (region.size() != 0) {
                myClass->segments.emplace_back(region);
            }
        }
    }

    vector<int> pointsLeftOfPlane, pointsRightOfPlane, percentages;
    int sum_left, sum_right, my_index;
    vec plane_normal, centroid;
    vector<float> residuals;
    float residual, perc;

    /// for each segment, create a 3d-plane using regression between their points
    /// for each plane, extract the residuals and the number of points that exist in the left or in the right side
    for (int i=0; i<myClass->segments.size(); i++) {
        /// create plane
        myClass->pcloud.planeFitting(myClass->pointFeatures, myClass->segments[i], plane_normal, centroid);
        myClass->m_plane = Plane(centroid, plane_normal);
        myClass->planes.emplace_back(myClass->m_plane);

        /// get residuals
//        residual = myClass->pcloud.getResiduals(myClass->pointFeatures, myClass->segments[i], myClass->m_plane);
//        residuals.emplace_back(residual);

        /// get the number of points that exist in the left or in the right side of plane
        sum_left = sum_right = 0; perc = 0.0f;
        myClass->pcloud.getPlaneMetrics(myClass->pointFeatures, myClass->segments[i], myClass->m_plane, sum_left, sum_right, perc);
        pointsLeftOfPlane.emplace_back(sum_left);
        pointsRightOfPlane.emplace_back(sum_right);
        percentages.emplace_back(perc);
    }

    /// normalize residuals
//    myClass->pcloud.normalize(residuals);

    vector<VecArray> new_segments; VecArray old_segments;

    /// run second stage of segmentation in some clusters using the plane
    for (int i=0; i<myClass->segments.size(); i++) {
        cout << "Percentage of cluster " << i << " = " << percentages[i] << endl;

        if ((pointsRightOfPlane[i] != 0) && (pointsLeftOfPlane[i] != 0)) {
            if ((percentages[i] >= myClass->threshold)) {
                new_segments.clear();

                myClass->pcloud.segmentation2(myClass->pointFeatures, myClass->segments[i], new_segments, myClass->planes[i], myClass->mesh);
                myClass->final_segments.emplace_back(new_segments);
            }
            else {
                new_segments.clear(); old_segments.clear();

                for (int j=0; j<myClass->segments[i].size(); j++) {
                    my_index = myClass->segments[i][j];
                    old_segments.emplace_back(myClass->pointFeatures[my_index].coordinates);
                }
                new_segments.emplace_back(old_segments);
                myClass->final_segments.emplace_back(new_segments);
            }
        }
    }

    cout << endl;
    const float Time_of_segmentation = vvr::getSeconds() - t;
    echo(Time_of_segmentation);

    cout << "End" << endl;

    myClass->draw_segmentation = true;
    myClass->draw_lines = false;
    myClass->draw_frame = false;
    delete[] tri_indices, ringNeighbours;
}

void surfaceReconstruction::setLeftFrame(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->lFrame = !myClass->lFrame;
}

void surfaceReconstruction::setRightFrame(int index, void* object) {
    auto * myClass = (surfaceReconstruction*) object;
    myClass->rFrame = !myClass->rFrame;
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

//!*********************************************************************************************************************
//! getter functions
//!*********************************************************************************************************************
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

//!*********************************************************************************************************************
//! draw functions
//!*********************************************************************************************************************
void surfaceReconstruction::draw() {
    if (draw_frame) {
        for (int i=0; i<all_points.size(); i++)
            Point3D(all_points[i].first.x, all_points[i].first.y, all_points[i].first.z,
                    Colour(all_points[i].second[2], all_points[i].second[1], all_points[i].second[0])).draw();
    }

    if (draw_segmentation) {
        vector<vvr::Colour> cols;
        cols.emplace_back(Colour::yellow); cols.emplace_back(Colour::black); cols.emplace_back(Colour::blue);
        cols.emplace_back(Colour::darkOrange); cols.emplace_back(Colour::darkRed); cols.emplace_back(Colour::grey);
        cols.emplace_back(Colour::darkGreen); cols.emplace_back(Colour::magenta); cols.emplace_back(Colour::red);
        cols.emplace_back(Colour::orange); cols.emplace_back(Colour::green); cols.emplace_back(Colour::yellowGreen);
        cols.emplace_back(Colour::white);

        int type{0};
        for (int i=0; i<final_segments.size(); i++) {
            for (int j=0; j<final_segments[i].size(); j++) {
                if(type > cols.size())
                    type = 0;

                for (int m=0; m<final_segments[i][j].size(); m++) {

                    Point3D(final_segments[i][j][m].x,
                            final_segments[i][j][m].y,
                            final_segments[i][j][m].z, cols[type]).draw();
                }
                type++;
            }
        }
    }

    if(draw_lines) {
        for (auto &l_point : l_points)
            Point3D(l_point.x, l_point.y, l_point.z, Colour::blue).draw();

        /// add an offset in order to separate the scenes
        for (auto &r_point : r_points)
            Point3D(r_point.x, r_point.y, r_point.z+20, Colour::red).draw();

        LineSeg3D line;
        for (int i=0; i<l_points.size(); i+=55) {
            if(!pcloud.zeroPoint(l_points[i]) && !pcloud.zeroPoint(r_points[i])) {
                line.x1 = l_points[i].x;
                line.y1 = l_points[i].y;
                line.z1 = l_points[i].z;

                line.x2 = r_points[i].x;
                line.y2 = r_points[i].y;
                line.z2 = r_points[i].z+20;

                line.setColour(Colour::yellow);
                line.draw();
            }
        }
    }

//    mesh->draw(Colour::green, SOLID);
//    mesh->draw(Colour::black, WIRE);
//    mesh->draw(Colour::black, NORMALS);

    ///curvature
//    double min_v, max_v;
//    min_v = min(m_curvature); max_v = max(m_curvature);
//    for (int i=0; i<m_curvature.size(); i++)
//        m_curvature[i] = static_cast<float>(255 * ((abs(m_curvature[i]) - min_v) / (max_v - min_v) ));
//
//    for (int i=0; i<all_points.size(); i++) {
//        if (m_curvature[i] > 0.1)
//            Point3D(all_points[i].first.x, all_points[i].first.y, all_points[i].first.z, Colour::white).draw();
//    }
    ///curvature

    /// draw normals
//    double min_x{1}, min_y{1}, min_z{1};
//    double max_x{-1}, max_y{-1}, max_z{-1};
//    for (int i=1; i<m_normals.size(); i++) {
//        if(m_normals[i].x < min_x)
//            min_x = m_normals[i].x;
//        if(m_normals[i].y < min_y)
//            min_y = m_normals[i].y;
//        if(m_normals[i].z < min_z)
//            min_z = m_normals[i].z;
//
//        if(m_normals[i].x > max_x)
//            max_x = m_normals[i].x;
//        if(m_normals[i].y > max_y)
//            max_y = m_normals[i].y;
//        if(m_normals[i].z > max_z)
//            max_z = m_normals[i].z;
//    }

//    LineSeg3D line, line2;
//    vec n;
//    double x,y,z;
//    for (int i=0; i<m_normals.size(); i+=5) {
//            n = m_normals[i];
//            line.x1 = all_points[i].first.x;
//            line.y1 = all_points[i].first.y;
//            line.z1 = all_points[i].first.z;
//
//            line.x2 = n.x;
//            line.y2 = n.y;
//            line.z2 = n.z;
//
//            x = 255*(abs(n.x))/(max_x);
//            y = 255*(abs(n.y))/(max_y);
//            y = 255*(abs(n.z))/(max_z);
//
//            line.setColour(Colour::yellow);
//            line.setColour(Colour(x,y,z));
//            line.draw();
//    }
    /// draw normals

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

/// not used functions
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
/*
float surfaceReconstruction::vectorSum(const vector<float> &v) {
    float initial_sum{0.0f};
    return accumulate(v.begin(), v.end(), initial_sum);
}
VecArray surfaceReconstruction::getData(VecArray points, int num) {
    VecArray result;
    for (int i=0; i<num; i++) {
        result.emplace_back(points.at(i));
    }
    return result;
}
*/