#include "surfaceReconstruction.h"
#include "generic.h"

using namespace std;
using namespace vvr;

surfaceReconstruction::surfaceReconstruction(int &index) {
    initialize(index);
    createGui();
}

void surfaceReconstruction::initialize(int &index) {
    image_prefix = convertToStr(generic::convertToDatasetType(index));
    num_images = generic::DatasetSize(generic::convertToDatasetSize(index));
    stereo_dir += image_prefix;
    l_frame_index = index;
    r_frame_index = index+1;
}

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

Mat surfaceReconstruction::getFrame(int index) {
    ostringstream frame_to_stream; // declaring output string stream
    frame_to_stream << index; // frame_to_stream a number as a stream into output
    string frame_to_string = frame_to_stream.str(); // the str() coverts number into string
    return imread(stereo_dir + "/" + image_prefix + "_" + frame_to_string + ".png");
}

void surfaceReconstruction::createMesh(const vector< pair <Point3d,Vec3b>> &image_points, Mesh &mesh) {
    vector<vec> &mesh_vertices = m_mesh.getVertices();
    for (auto &point : image_points) {
        mesh_vertices.emplace_back(point.first.x, point.first.y, point.first.z);
    }
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
    myClass->getDepthImage(index);
    myClass->getImage(index);

    myClass->pcloud.clearPoints();
    myClass->m_mesh.getVertices().clear();

    myClass->pcloud.create(myClass->image_mat, myClass->depth_mat);
    Vec3d degree = {180.0, 0.0, 0.0};
    Mat rotation = myClass->pcloud.rotationMatrix(degree);
    myClass->pcloud.rotate(rotation);

    myClass->createMesh(myClass->pcloud.m_points, myClass->m_mesh);
}

void surfaceReconstruction::getDepthImage(int frame_index) {
    ImageRGBD depth_image(stereo_dir + "/" + image_prefix + "_" + generic::convertToStr(frame_index) + "_depth.png");
    depth_image.convertToMat();
    depth_image.getMat(depth_mat);
}

void surfaceReconstruction::getImage(int frame_index) {
    ImageRGB image(stereo_dir + "/" + image_prefix + "_" + generic::convertToStr(frame_index) + ".png");
    image.convertToMat();
    image.getMat(image_mat);
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

void surfaceReconstruction::draw() {
//    int counter{0};
    unsigned long counter{0};
    for (auto &i : m_mesh.getVertices()) {
        Point3D(i.x, i.y, i.z, Colour(pcloud.m_points.at((counter)).second[2] , pcloud.m_points.at(counter).second[1], pcloud.m_points.at(counter).second[0])).draw();
        counter++;
    }
}