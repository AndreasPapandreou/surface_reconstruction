#include "PointCloud.h"
#include "dataTypes.h"

bool operator==(const vec &v1, const vec &v2) {
    return (v1.x == v2.x &&
            v1.y == v2.y &&
            v1.z == v2.z);
}

bool operator!=(const vec &v1, const vec &v2) {
    return (v1.x != v2.x &&
            v1.y != v2.y &&
            v1.z != v2.z);
}

/* ------------------------------------------------------------------------------
 * Inputs       : An image in Mat and its depth one version in Mat
 * Description  : It creates the point cloud of each image. and stores all points
 *                and their colours in vector< pair <Point3d,Vec3b>>.
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void PointCloud::create(const Mat &image, Mat &depth_image)
{
    CameraConstants camera;
    Mat filtered_depth;
    for ( int i = 1; i < 3; i = i + 2 )
    {
        medianBlur ( depth_image, filtered_depth, i );
//        if( display_dst( DELAY_BLUR ) != 0 ) { return 0; }
    }

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
            vec point;
            point.x = xgrid.at<short>(i,j)*(filtered_depth.at<unsigned short>(i,j)/camera.constant/camera.mm_per_m);
            point.y = ygrid.at<short>(i,j)*(filtered_depth.at<unsigned short>(i,j)/camera.constant/camera.mm_per_m);
            point.z = filtered_depth.at<unsigned short>(i,j)/camera.mm_per_m;

            ///test
            // try to discretize the depth
//            point.z = int(point.z);
            ///test

//            if (point.x != 0 && point.y != 0 && point.z != 0) {
//            if (point.z != 0) {
                m_points.emplace_back(point, image.at<Vec3b>(i,j));
//            }
        }
    }
}

void PointCloud::clearPoints()
{
    m_points.clear();
}

// not considering the colors
void PointCloud::kNearest(const VecArray &src, VecArray &nearestPoints, vector<float> &dist, int kn) {
//    float distance;
//    const float t = vvr::getSeconds();
//
//    for (auto src_pt : src) {
//        for (int j=0; j<kn; j++) {
//            const KDNode **nearests = new const KDNode*[kn];
//            memset(nearests, NULL, kn * sizeof(KDNode*));
//
//            cout << "in" << endl;
//
//            m_dst_KDTree->kNearest(j, src_pt, m_dst_KDTree->root(), nearests, &distance);
//
//            cout << "nearest = " << (*nearests)->split_point << endl;
//
    // TODO check if i have to update *(nearests+j) to nearests[j]
//            nearestPoints.emplace_back((*nearests)->split_point);
//            dist.emplace_back(distance);
//        }
//    }
//
//    const float KDTree_knn_time = vvr::getSeconds() - t;
////    echo(KDTree_knn_time);

    float distance;
    const float t = vvr::getSeconds();

    for (auto src_pt : src) {
        const KDNode **nearests = new const KDNode*[kn];
        memset(nearests, NULL, kn * sizeof(KDNode*));

        for (int j=0; j<kn; j++) {

            m_dst_KDTree->kNearest(j, src_pt, m_dst_KDTree->root(), nearests, &distance);

            // TODO check if i have to update *(nearests+j) to nearests[j]
            nearestPoints.emplace_back((*nearests+j)->split_point);
            dist.emplace_back(distance);
        }
    }

    const float KDTree_knn_time = vvr::getSeconds() - t;
//    echo(KDTree_knn_time);

}

pair<Eigen::Matrix3f, Eigen::Vector3f> PointCloud::computeRigidTransform(const VecArray &src, const VecArray &dst) {
    int size = dst.size();
    vec src_center = getCentroid(src);
    vec dst_center = getCentroid(dst);

    vec src_centered_point, dst_centered_point;
    Eigen::MatrixXf X(3,size), Y(3,size);
    for (int i=0; i<size; i++) {
        src_centered_point = src.at(i) - src_center;
        dst_centered_point = dst.at(i) - dst_center;

        X(0,i) = src_centered_point.x/(double)size;
        X(1,i) = src_centered_point.y/(double)size;
        X(2,i) = src_centered_point.z/(double)size;

        Y(0,i) = dst_centered_point.x/(double)size;
        Y(1,i) = dst_centered_point.y/(double)size;
        Y(2,i) = dst_centered_point.z/(double)size;
    }

    // compute the covariance matrix
    Eigen::Matrix3f S = Y*X.transpose();

    // compute the singular value decomposition
    Eigen::JacobiSVD<Eigen::MatrixXf> svd;
    svd.compute(S, Eigen::ComputeThinU | Eigen::ComputeThinV );
    if (!svd.computeU() || !svd.computeV()) {
        std::cerr << "decomposition error" << endl;
    }

    // extract right singular vectors
    Eigen::Matrix3f V = svd.matrixV();

    // extract left singular vectors
    Eigen::Matrix3f U = svd.matrixU();

    // create diagonal matrix
    Eigen::MatrixXf diag_mat(3, 3);
    diag_mat.setZero();
    diag_mat(0,0) = 1;
    diag_mat(1,1) = 1;
    diag_mat(2,2) = V.determinant()*U.transpose().determinant();

    // compute rotation matrix
    Eigen::Matrix3f R = V*diag_mat*U.transpose();

    // compute translation vector
    Eigen::Vector3f t = dataTypes::convertToEigenVector(src_center) - R*dataTypes::convertToEigenVector(dst_center);

    return pair<Eigen::Matrix3f, Eigen::Vector3f>(R, t);
}

void PointCloud::transformPoints(pair<Eigen::Matrix3f, Eigen::Vector3f> &R_t, VecArray &points) {
    int size = static_cast<int>(points.size());

    Eigen::MatrixXf mat(3,size);
    dataTypes::convertToEigenMat(points, mat);

    Eigen::MatrixXf new_points(3,size);
    new_points = R_t.first*mat;
    for (int i=0; i<size; i++) {
        new_points(0,i) += R_t.second(0,0);
        new_points(1,i) += R_t.second(1,0);
        new_points(2,i) += R_t.second(2,0);
    }
    dataTypes::convertToVector(new_points, points);
}

void PointCloud::sobel(const Mat &img, Mat &new_img) {
    Mat img_gray;
    std::string window_name = "Sobel";
    int scale = 1;
    int delta = 0;
    int ddepth = CV_8UC1;
    //TODO test it
    //    int ddepth = CV_16S;

    GaussianBlur( img, img, Size(3,3), 0, 0, BORDER_DEFAULT );

    /// Convert it to gray
    cvtColor( img, img_gray, CV_BGR2GRAY );

    /// Create window
//    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    Sobel( img_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    Sobel( img_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, new_img );

//    imshow( window_name, new_img );
//    waitKey(0);
}

// img must br gray
void PointCloud::thresholding(const Mat &img, Mat &new_img) {
//  variable that representing the value that is to be given if pixel value is more than the threshold value.
    double max_value = 255.0;
    std::string window_name = "THRESH_BINARY";
//    namedWindow( window_name, CV_WINDOW_AUTOSIZE );
//    adaptiveThreshold(img, new_img, max_value, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 12);
    adaptiveThreshold(img, new_img, max_value, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 6);
//    imshow( window_name, new_img );
}

void PointCloud::getEdges(const Mat &img, VecArray &edges, const int &value) {
    CameraConstants camera;
    uchar val;

    for (int i=0; i<camera.image_height; i++) {
        for (int j=0; j<camera.image_width; j++) {
            val = img.at<uchar>(i,j);
            if ((int)val == value) {
                edges.emplace_back(m_points.at(i*camera.image_width + j).first);
            }
        }
    }
}

pair<Eigen::Matrix3f, Eigen::Vector3f> PointCloud::icp(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations) {
    vector<pair<Eigen::Matrix3f, Eigen::Vector3f>> all_R_t;
    VecArray nearestPoints;

    int counter{0};
    vector<double> weights;
    while(counter++ < iterations) {
        nearestPoints.clear();
        dist.clear();

        kNearest(src_points, nearestPoints, dist, 1);

        all_R_t.emplace_back(computeRigidTransform(nearestPoints, src_points));

        transformPoints(all_R_t.at(all_R_t.size()-1), src_points);

        mean_distance = vectorSum(dist)/(float)dist.size();
        normalize(dist);
        error = vectorSum(dist)/(float)dist.size();

//        cout << "iter = " << counter << endl;
//        cout << "mean_dist = " << mean_distance << endl;
//        cout << "error = " << error << endl;
    }

    pair<Eigen::Matrix3f, Eigen::Vector3f> R_t;
    R_t = all_R_t.at(all_R_t.size()-1);
    for(int i=all_R_t.size()-2; i>=0; i--) {
        R_t.first *= all_R_t.at(i).first;
        R_t.second += all_R_t.at(i).second;
    }
    return R_t;
}

vector<int>* PointCloud::triangulateMesh(const std::vector<vec>& vertices, Mesh*& mesh, const float threshold) {
    mesh = new Mesh();
    std::vector<vec> &modelVerts = mesh->getVertices();
    std::vector<vvr::Triangle> &tris = mesh->getTriangles();

    for (auto &d : vertices) modelVerts.push_back(d);

    CameraConstants camera;
//    float t{1.0f}; //old
//    float t{1.5f}; //correct for segment 1
    // sampling triangles using the depth for smoother normals
//    float t{0.1f};///correct
    int size = (camera.image_width - 1)*(camera.image_height - 1);

    for (int i = 0; i < size; i++) {
        tris.emplace_back(&modelVerts, i, i + 1 + camera.image_width, i + camera.image_width);

        if(abs(modelVerts[i].z - modelVerts[i + 1 + camera.image_width].z) <= threshold &&
            abs(modelVerts[i].z - modelVerts[i + camera.image_width].z) <= threshold &&
            abs(modelVerts[i + camera.image_width].z - modelVerts[i + 1 + camera.image_width].z) <= threshold) {}
        else {tris.erase(tris.begin() + tris.size());}

        tris.emplace_back(&modelVerts, i, i + 1, i + 1 + camera.image_width);

        if(abs(modelVerts[i].z - modelVerts[i + 1].z) <= threshold &&
            abs(modelVerts[i].z - modelVerts[i + 1 + camera.image_width].z) <= threshold &&
            abs(modelVerts[i + 1].z - modelVerts[i + 1 + camera.image_width].z) <= threshold) {}
        else {tris.erase(tris.begin() + tris.size());}
    }

    size = mesh->getTriangles().size();

    // for each vertex, store the triangles in which belongs to
    // allocate array of size vectors

    //TODO use smart pointers
    vector<int> *tri_indices = new vector<int>[size];

    int index;
    for (int i=0; i<size; i++) {
        const vvr::Triangle tri = mesh->getTriangles()[i];

        index = tri.vi1;
        tri_indices[index].emplace_back(i);

        index = tri.vi2;
        tri_indices[index].emplace_back(i);

        index = tri.vi3;
        tri_indices[index].emplace_back(i);
    }

//    mesh->update();
    return tri_indices;
}

void PointCloud::getNormals(Mesh*& mesh, vector<int>* tri_indices, VecArray &normals) {
    int size = mesh->getVertices().size();
    int index;

    // get normals
    vec normal;
    int counter{0};
    for (int i=0; i<size; i++) {
        normal.x = normal.y = normal.z = 0.0f;

        if(tri_indices[i].empty()) {
            counter++;
            normals.emplace_back(normal);
        }
        else {
            for (int j=0; j<tri_indices[i].size(); j++) {
                index = tri_indices[i][j];
                normal += mesh->getTriangles()[index].getNormal();
            }
            normal /= float(tri_indices[i].size());
//            normal += mesh->getVertices()[i];
            normals.emplace_back(-normal);
        }
    }
    cout << "points without triangle are " << counter << endl;
}

void PointCloud::getCurvature(Mesh*& mesh, vector<int>* tri_indices, vector<float> &curvature) {
    double adjacent_side1_len, adjacent_side2_len, opposite_side_len, radian, area;
    int size = mesh->getVertices().size();
    math::Triangle tri;
    vec v1, v2, v3;
    int index;

    // iterate through all vertices
    for (int i=0; i<size; i++) {
        area = 0.0;
        radian = 0.0;

        // if there are not neighbors (points without depth)
        if(tri_indices[i].empty()) {
            curvature.emplace_back(0.0f);
        }
        else {
            // get the neighbors of each vertex
            for (int j=0; j<tri_indices[i].size(); j++) {
                index = tri_indices[i][j];

                tri.a = mesh->getTriangles()[index].v1();
                tri.b = mesh->getTriangles()[index].v2();
                tri.c = mesh->getTriangles()[index].v3();
                area += tri.Area();

                vvr::Triangle tri2 = mesh->getTriangles()[index];
                v1 = tri2.v1();
                v2 = tri2.v2();
                v3 = tri2.v3();

                // compute the length of each side of triangles
                if(i == tri2.vi1) {
                    adjacent_side1_len = sqrt(pow(v1.x - v2.x,2) + pow(v1.y - v2.y,2) + pow(v1.z - v2.z,2));
                    adjacent_side2_len = sqrt(pow(v1.x - v3.x,2) + pow(v1.y - v3.y,2) + pow(v1.z - v3.z,2));
                    opposite_side_len = sqrt(pow(v2.x - v3.x,2) + pow(v2.y - v3.y,2) + pow(v2.z - v3.z,2));
                }
                else if(i == tri2.vi2) {
                    adjacent_side1_len = sqrt(pow(v1.x - v2.x,2) + pow(v1.y - v2.y,2) + pow(v1.z - v2.z,2));
                    adjacent_side2_len = sqrt(pow(v2.x - v3.x,2) + pow(v2.y - v3.y,2) + pow(v2.z - v3.z,2));
                    opposite_side_len = sqrt(pow(v1.x - v3.x,2) + pow(v1.y - v3.y,2) + pow(v1.z - v3.z,2));
                }
                else {
                    adjacent_side1_len = sqrt(pow(v3.x - v2.x,2) + pow(v3.y - v2.y,2) + pow(v3.z - v2.z,2));
                    adjacent_side2_len = sqrt(pow(v1.x - v3.x,2) + pow(v1.y - v3.y,2) + pow(v1.z - v3.z,2));
                    opposite_side_len = sqrt(pow(v2.x - v1.x,2) + pow(v2.y - v1.y,2) + pow(v2.z - v1.z,2));
                }

                radian += acos((pow(adjacent_side1_len,2) + pow(adjacent_side2_len,2) - pow(opposite_side_len,2))/
                          (2*adjacent_side1_len*adjacent_side2_len));
            }

            if(isnan((2*M_PI-radian)/(area/3)))
                curvature.emplace_back(0.0f);
//                curvature.emplace_back(1.0f); ///test
            else
                curvature.emplace_back((2*M_PI-radian)/(area/3));
        }
    }
    normalize(curvature);
}

void PointCloud::segmentation(const vector<PointFeatures> pointFeatures, vector<int> &region, vector<int> &hash, int start) {
    int index;
    region.emplace_back(start);

    for (int i=0; i<region.size(); i++) {
        start = region[i];

        for (int j=0; j<pointFeatures[start].neighborVertices.size(); j++) {
            index = pointFeatures[start].neighborVertices[j];

            if (hash[index] == 0) {
                region.emplace_back(index);
                hash[index] = 1;
            }
        }
    }
}

void PointCloud::planeFitting(const vector<PointFeatures> pointFeatures, const vector<int> &segment, vec &plane_normal, vec &centroid) {
    int size = segment.size();
    VecArray src;
    vec src_centered;
    int index;

    for (int i=0; i<size; i++) {
        index = segment[i];
        src.emplace_back(pointFeatures[index].coordinates);
    }

    /// get centroid
    centroid = getCentroid(src);

    /// copy coordinates to  matrix in Eigen format
    Eigen::MatrixXf X(3,size);
    for (int i=0; i<size; i++) {
        src_centered = src.at(i) - centroid;

        X(0,i) = src_centered.x/(double)size;
        X(1,i) = src_centered.y/(double)size;
        X(2,i) = src_centered.z/(double)size;
    }

    /// compute the singular value decomposition
    Eigen::JacobiSVD<Eigen::MatrixXf> svd;
    svd.compute(X, Eigen::ComputeThinU | Eigen::ComputeThinV );
    if (!svd.computeU() || !svd.computeV()) {
        std::cerr << "decomposition error" << endl;
    }

    /// extract left singular vectors
    Eigen::Matrix3f U = svd.matrixU();

    plane_normal.x = U(0,2); plane_normal.y = U(1,2); plane_normal.z = U(2,2);

//    cout << "Its singular values are:" << endl << svd.singularValues() << endl;
}

void PointCloud::segmentation2(const vector<PointFeatures> &pointFeatures, const vector<int> &segment, vector<VecArray> &new_segments,  math::Plane &plane, Mesh*& mesh) {
    VecArray region;
    vector<int> indices, hash;
    int index, index2, index3;
    vec p1, p2;

    for (int i=0; i<pointFeatures.size(); i++)
        hash.emplace_back(0);

    for (int i=0; i<segment.size(); i++) {
        index = segment[i];

        if (hash[index] == 0) {
            p1 = pointFeatures[index].coordinates;

            region.clear();
            region.emplace_back(p1);

            indices.clear();
            indices.emplace_back(index);

            for (int j=0; j<indices.size(); j++) {
                index2 = indices[j];

                for (int m=0; m<pointFeatures[index2].neighborVertices.size(); m++) {
                    index3 = pointFeatures[index2].neighborVertices[m];
                    p2 = pointFeatures[index3].coordinates;

                    if (hash[index3] == 0) {
//                        if (!contains(region, p2) && (plane.AreOnSameSide(p1, p2))) {
                        if ((plane.AreOnSameSide(p1, p2))) {
                            indices.emplace_back(index3);
                            region.emplace_back(p2);
                            hash[index3] = 1;
                        }
                    }
                }
            }
//            if (region.size() != 0)
            if (region.size() > 20)
                new_segments.emplace_back(region);
        }
    }
}

float PointCloud::getResiduals(const vector<PointFeatures> pointFeatures, const vector<int> &segment, math::Plane &plane) {
    float residuals{0.0f};
    int index;
    vec point;

    for (int i=0; i<segment.size(); i++) {
        index = segment[i];
        point = pointFeatures[index].coordinates;
        residuals += plane.Distance(point);
    }
    return abs(residuals / float(segment.size()));
}

/// get the number of points that exist in the left or in the right side of plane
void PointCloud::getPlaneMetrics(vector<PointFeatures> &features, const vector<int> &segment, math::Plane &plane, int &leftPoints, int &rightPoints, float &perc) {
    int index; vec point;

    for (int i=0; i<segment.size(); i++) {
        index = segment[i];
        point = features[index].coordinates;

        if (plane.SignedDistance(point) > 0)
            leftPoints+=1;
        else
            rightPoints+=1;
    }

    if (leftPoints > rightPoints)
        perc = 100.0f*(float(leftPoints - rightPoints)/leftPoints);
    else
        perc = 100.0f*(float(rightPoints - leftPoints)/rightPoints);
}

/// \description
/// The structure vector<int>* tri_indices stores the indeces of triangles that each vertex of mesh belongs to, it has the below
/// form  for only two vertices :
///   [ 0  => [ 10 20 ]
///    1 ] => [ 40 50 ]
/// That means that the vertex with index 0 belongs to triangles with indeces 10 and 20 and
/// the vertex with index 1 belongs to triangles with indeces 40 and 50.
/// The getNRingNeighborhood function firstly stores the Nth ring neighbors of each vertex strating from 1-ring. Then
/// for the next-ring it updates the structure vector<int>* tri_indices in order to store for each vertex of mesh the
/// triangles where the next-ring neighbors belong to and so on. It returns vector<int>* ringNeighbours, which
/// for each vertex of mesh contains the neighborhoud vertices of mesh.
/// \param mesh
/// \param tri_indices
/// \param rings
/// \return vector<int>*
vector<int>* PointCloud::getNRingNeighborhood(Mesh*& mesh, vector<int>*& tri_indices, int &rings) {
    int size = mesh->getVertices().size();
    int index, index2, index3;

    //TODO use smart pointers
//    static vector<int> *ringNeighbours = new vector<int>[size];
    vector<int> *ringNeighbours = new vector<int>[size];

    // iterate through all vertices of mesh
    for (int i=0; i<size; i++) {
        if(tri_indices[i].empty()) {}
        else {
            // store Nth ring neighbors
            for (int j=0; j<tri_indices[i].size(); j++) {
                index = tri_indices[i][j];

                const vvr::Triangle tri = mesh->getTriangles()[index];

                if(i != tri.vi1)
                    if (!contains(ringNeighbours[i], tri.vi1))
                        ringNeighbours[i].emplace_back(tri.vi1);

                if(i != tri.vi2)
                    if (!contains(ringNeighbours[i], tri.vi2))
                        ringNeighbours[i].emplace_back(tri.vi2);

                if(i != tri.vi3)
                    if (!contains(ringNeighbours[i], tri.vi3))
                       ringNeighbours[i].emplace_back(tri.vi3);
            }
        }
    }

    rings--;
    vector<int> new_indices, indicesForSearching;

    // update the tri_indices in order to contain the triangles in which the next ring vertices belong to
    if(rings > 0) {
        // iterate through all vertices of mesh
        for (int i=0; i<size; i++) {
            new_indices.clear();

            if(tri_indices[i].empty()) {}
            else {
                // iterate through all triangles each vertex belongs to
                for (int j=0; j<tri_indices[i].size(); j++) {
                    indicesForSearching.clear();

                    // get the index of triangle
                    index = tri_indices[i][j];

                    const vvr::Triangle tri = mesh->getTriangles()[index];

                    // store the vertices of triangle with the above index
                    if(i != tri.vi1)
                        indicesForSearching.emplace_back(tri.vi1);
                    if(i != tri.vi2)
                        indicesForSearching.emplace_back(tri.vi2);
                    if(i != tri.vi3)
                        indicesForSearching.emplace_back(tri.vi3);

                    // search in which triangles do the above vertices belong to
                    for (int n=0; n<indicesForSearching.size(); n++) {
                        index2 = indicesForSearching[n];

                        // iterate through each vertex and store the triangles where each belongs
                        for (int m=0; m<tri_indices[index2].size(); m++) {
                            index3 = tri_indices[index2][m];

                            // store the indices of each triangle
                            if (!contains(new_indices, index3))
                                new_indices.emplace_back(index3);
                        }
                    }
                }
            }
            tri_indices[i].clear();
            tri_indices[i] = new_indices;
        }
        getNRingNeighborhood(mesh, tri_indices, rings);
    }
    return ringNeighbours;
}

Mat PointCloud::rotationMatrix(const Vec3d &degree)
{
    Vec3d radian;
    radian[0] = degree[0]*M_PI/180.; // x_axis
    radian[1] = degree[1]*M_PI/180.; // y_axis
    radian[2] = degree[2]*M_PI/180.; // z_axis

    // calculate rotation about x axis
    Mat rotation_x = (Mat_<float>(3,3) <<
                                       1, 0, 0,
            0, cos(radian[0]), -sin(radian[0]),
            0, sin(radian[0]), cos(radian[0]));

    // calculate rotation about y axis
    Mat rotation_y = (Mat_<float>(3,3) <<
                                       cos(radian[1]), 0, sin(radian[1]),
            0, 1, 0,
            -sin(radian[1]), 0, cos(radian[1]));

    // calculate rotation about z axis
    Mat rotation_z = (Mat_<float>(3,3) <<
                                       cos(radian[2]), -sin(radian[2]), 0,
            sin(radian[2]), cos(radian[2]), 0,
            0, 0, 1);

    // get the final rotation matrix
    Mat rotation = rotation_z * rotation_y * rotation_x;
    return rotation;
}

void PointCloud::rotate(const Mat &rotation_mat) {
    for (auto &point : m_points) {
        Mat current_point = (Mat_<float>(3,1) << point.first.x, point.first.y, point.first.z);
        Mat result = rotation_mat*current_point;
        point.first.x = result.at<float>(0);
        point.first.y = result.at<float>(1);
        point.first.z = result.at<float>(2);
    }
}

void PointCloud::getBoundingBox(VecArray &vertices, Box3D &box)
{
    double max_x = vertices[0].x, max_y = vertices[0].y, max_z = vertices[0].z;
    double min_x = vertices[0].x, min_y = vertices[0].y, min_z = vertices[0].z;
    int size = vertices.size();
    for (int i = 1; i < size; i++) {
        if (vertices[i].x > max_x)	max_x = vertices[i].x;
        if (vertices[i].y > max_y)	max_y = vertices[i].y;
        if (vertices[i].z > max_z)  max_z = vertices[i].z;
        if (vertices[i].x < min_x)  min_x = vertices[i].x;
        if (vertices[i].y < min_y)	min_y = vertices[i].y;
        if (vertices[i].z < min_z)	min_z = vertices[i].z;
    }

    box.x1 = max_x;
    box.y1 = max_y;
    box.z1 = max_z;
    box.x2 = min_x;
    box.y2 = min_y;
    box.z2 = min_z;
}

vec PointCloud::getCentroid(const VecArray &points) {
    vec center(0,0,0);
    for (const auto &i : points) {
        center += i;
    }
    return center/(float)points.size();
}

double PointCloud::dot_product(const vec &v1, const vec &v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

vec PointCloud::cross_product(const vec &v1, const vec &v2) {
    vec res;
    res.x = v1.y * v2.z - v1.z * v2.y;
    res.y = v1.z * v2.x - v1.x * v2.z;
    res.z = v1.x * v2.y - v1.y * v2.x;
    return res;
}

template <typename T>
void PointCloud::normalize(vector<T> &values) {
    T min_value = min(values);
    T max_value = max(values);
    T diff = max_value - min_value;
    for (T &value : values) {
        value = (value - min_value)/diff;
    }
}

float PointCloud::vectorSum(const vector<float> &v) {
    float initial_sum{0.0f};
    return accumulate(v.begin(), v.end(), initial_sum);
}

template <typename T>
T PointCloud::min(const vector<T> &values) {
    T min_value{values.at(0)};
    for(int i=1; i<values.size(); i++) {
        if (values.at(i) < min_value)
            min_value = values.at(i);
    }
    return min_value;
}

template <typename T>
T PointCloud::max(const vector<T> &values) {
    T max_value{values.at(0)};
    for(int i=1; i<values.size(); i++) {
        if (values.at(i) > max_value)
            max_value = values.at(i);
    }
    return max_value;
}

template<typename T>
bool PointCloud::contains(const vector<T> &values, const T value) {
    for (int i=0; i<values.size(); i++) {
        if (values[i] == value)
            return true;
    }
    return false;
}

bool PointCloud::contains(const vector<vec> &values, const vec value) {
    for (int i=0; i<values.size(); i++) {
        if ((values[i].x == value.x) && (values[i].y == value.y) && (values[i].z == value.z))
            return true;
    }
    return false;
}

bool PointCloud::zeroPoint(const vec p) {
    return ( (p.x == 0.0f) && (p.y == 0.0f) && (p.z == 0.0f) );
}


/// not used function
/*
vec PointCloud::convertTo3d(const Mat &image, const Mat &depth_image, Point2d &point)
{
    //TODO replace depth_image with class variable
    Mat filtered_depth;
    for ( int i = 1; i < 3; i = i + 2 )
    {
        medianBlur ( depth_image, filtered_depth, i );
//        if( display_dst( DELAY_BLUR ) != 0 ) { return 0; }
    }

    CameraConstants camera;
    short xgrid, ygrid;
    xgrid = static_cast<short>(point.x + 1 + (camera.topLeft[0] - 1) - camera.center[0]);
    ygrid = static_cast<short>(point.y + 1 + (camera.topLeft[1] - 1) - camera.center[1]);

    vec res;
    res.x = xgrid*filtered_depth.at<unsigned short>(point.y,point.x)/camera.constant/camera.mm_per_m;
    res.y = ygrid*filtered_depth.at<unsigned short>(point.y,point.x)/camera.constant/camera.mm_per_m;
    res.z = filtered_depth.at<unsigned short>(point.y,point.x)/camera.mm_per_m;
    return res;
}

void PointCloud::getPoints(vector< pair <vec,Vec3b>> &points)
{
    points = m_points;
}

int PointCloud::getRgbdId(const ImageRGBD &image)
{
    return image.m_id;
}

//void PointCloud::findCorrespondingPoints(const Mat &l_frame_rgb, const Mat &r_frame_rgb, const Mat &l_frame_rgbd, const Mat &r_frame_rgbd, vector< pair <Point3d,Vec3b>> &l_points, vector< pair <Point3d,Vec3b>> &r_points) {
//    CameraConstants camera;
//
//    // the parameter slide show how many pixels to slide the window in each direction
//    int window_size{70}, slide{180};
//    int top_row, top_col, width, height;
//
//    int num_points{10};
//    int searching_points[num_points];
//    std::random_device rd; // obtain a random number from hardware
//    std::mt19937 eng(rd()); // seed the generator
//    int left_range{20*camera.image_height/100};
//    int right_range{80*camera.image_height/100};
//    std::uniform_int_distribution<> distr(left_range, right_range); // define the range
//    for(int n=0; n<num_points; ++n)
//        searching_points[n] = distr(eng);
//
//    for (int row=0; row<num_points; row++) {
//        for (int col=0; col<num_points; col++) {
//             cv::Rect img_roi(searching_points[col], searching_points[row], window_size, window_size);
//             Mat img_template = l_frame_rgb(img_roi);
//
//             top_col = searching_points[col]-slide;
//             top_row = searching_points[row]-slide;
//             width = 2*slide;
//             height = 2*slide;
//
//             validate(top_col, top_row, width, height);
//
//             cv::Rect cube_roi(top_col, top_row, width, height);
//             Mat img_cube = r_frame_rgb(cube_roi);
//
//             Mat result;
//             matchTemplate(img_cube, img_template, result, 5);
//
//             cv::Point best_match;
//             minMaxLoc(result, nullptr, nullptr, nullptr, &best_match);
//
//             Point2d left_point(searching_points[col], searching_points[row]);
//             l_points.emplace_back(convertTo3d(l_frame_rgb, l_frame_rgbd, left_point));
//
//             Point2d right_point = Point2d(best_match.x+top_col, best_match.y+top_row);
//             r_points.emplace_back(convertTo3d(r_frame_rgb, r_frame_rgbd, right_point));
//
////             namedWindow("test_left", WINDOW_NORMAL);
////             namedWindow("test_right", WINDOW_NORMAL);
////             circle(l_frame_rgb, left_point, 1, Scalar(0, 0, 255), FILLED, LINE_8);
////             circle(r_frame_rgb, right_point, 1, Scalar(0, 0, 255), FILLED, LINE_8);
////             imshow("test_left", l_frame_rgb);
////             imshow("test_right", r_frame_rgb);
//        }
//    }
//}

//void PointCloud::findAdjacentPoints(const Mat &l_frame_rgb, const Mat &r_frame_rgb, const Mat &l_frame_rgbd, const Mat &r_frame_rgbd, vector< pair <Point3d,Vec3b>> &l_points, vector< pair <Point3d,Vec3b>> &r_points) {
//    // the parameter slide show how many pixels to slide the window in each direction
////    int window_size{50}, slide{150};
//    int window_size{30}, slide{100};
//    int top_row, top_col, width, height;
//    bool update{false};
//    int left{0}, right{0}, up{0}, down{0}; //data for the new point 0->left, 1->right, 2->up, 3->down
//
//    for (int row=0; row<r_frame_rgb.rows - window_size; row++) {
//        cout << "row = " << row << endl;
//        for (int col=0; col<r_frame_rgb.cols - window_size; col++) {
////    for (int row=0; row<200; row++) {
////        cout << "row = " << row << endl;
////        for (int col=0; col<10; col++) {
////            cout << "col = " << col << endl;
//             cv::Rect img_roi(col, row, window_size, window_size);
//             Mat img_template = l_frame_rgb(img_roi);
//
//             if (update) {
//                 if (left > right) {
//                    top_col = col - slide;
//                 }
//                 else {
//                    top_col = col;
//                 }
//                 if (up > down) {
//                     top_row = row - slide;
//                 }
//                 else {
//                     top_row = row;
//                 }
//                 width = slide;
//                 height = slide;
//
////                 cout << "btop_col =" << top_col << endl;
////                 cout << "btop_row =" << top_row << endl;
////                 cout << "bwidth =" << width << endl;
////                 cout << "bheight =" << height << endl << endl;
//             }
//             else {
//                 top_col = col - slide;
//                 top_row = row - slide;
//                 width = 2 * slide;
//                 height = 2 * slide;
//             }
//
//             validate(top_col, top_row, width, height, slide);
//
////            cout << "atop_col =" << top_col << endl;
////            cout << "atop_row =" << top_row << endl;
////            cout << "awidth =" << width << endl;
////            cout << "aheight =" << height << endl;
////            cout << "col =" << col << endl << endl;
//
//             cv::Rect cube_roi(top_col, top_row, width, height);
//             Mat img_cube = r_frame_rgb(cube_roi);
//
//             Mat result;
//             matchTemplate(img_cube, img_template, result, 5);
//
//             cv::Point best_match;
//             minMaxLoc(result, nullptr, nullptr, nullptr, &best_match);
//
//             Point2d left_point(col, row);
//             l_points.emplace_back(convertTo3d(l_frame_rgb, l_frame_rgbd, left_point));
//
//             Point2d right_point =  Point2d(best_match.x+top_col, best_match.y+top_row);
//             r_points.emplace_back(convertTo3d(r_frame_rgb, r_frame_rgbd, right_point));
//
//
////            if(row == 0 && col == 49) {
//            if(row == 1 && col == 0) {
//                update = true;
//            }
//            else {
//                if (right_point.x <= left_point.x)
//                    left++;
//                else
//                    right++;
//                if (right_point.y <= left_point.y)
//                    up++;
//                else
//                    down++;
//            }
//        }
//    }
//}

//void PointCloud::findAdjacentPoints(const Mat &l_frame, const Mat &r_frame) {
//    int flag{1};
//    // how many pixels to slide the window in each direction
//    int window_size{50}, slide{150};
//    int top_row, top_col, width, height;
//
////    cout << "last = " << (r_frame.cols - window_size)*(r_frame.rows - window_size) << endl; = 253700
//    for (int row=0; row<r_frame.rows - window_size; row++) {
//        for (int col=0; col<r_frame.cols - window_size; col++) {
//
//            if (flag++ == 53701) {
//                cv::Rect img_roi(col, row, window_size, window_size);
//                Mat img_template = l_frame(img_roi);
//
//                top_col = col-slide;
//                top_row = row-slide;
//                width = 2*slide;
//                height = 2*slide;
//
//                validate(top_col, top_row, width, height);
//
//                cv::Rect cube_roi(top_col, top_row, width, height);
//                Mat img_cube = r_frame(cube_roi);
//
//                Mat result;
//                matchTemplate(img_cube, img_template, result, 5);
//
//                cv::Point best_match;
//                minMaxLoc(result, nullptr, nullptr, nullptr, &best_match);
//
//                namedWindow("img_template", WINDOW_NORMAL);
//                imshow("img_template", img_template);
//                namedWindow("img_cube", WINDOW_NORMAL);
//                imshow("img_cube", img_cube);
//                cout << "template size = " << img_template.size << endl;
//                cout << "img_cube size = " << img_cube.size << endl;
//
//                Point pt_old =  Point(col, row);
//                Point pt_new =  Point(best_match.x+top_col, best_match.y+top_row);
////                Point pt_new =  Point(best_match.x, best_match.y);
//
//                Mat my_l_depth_mat, my_r_depth_mat;
//                ImageRGB depth_image("../data/meeting_small_1/meeting_small_1_28.png");
//                depth_image.convertToMat();
//                depth_image.getMat(my_l_depth_mat);
//                ImageRGB depth_image2("../data/meeting_small_1/meeting_small_1_29.png");
//                depth_image2.convertToMat();
//                depth_image2.getMat(my_r_depth_mat);
//
//                namedWindow("test_left", WINDOW_NORMAL);
//                namedWindow("test_right", WINDOW_NORMAL);
//
//                circle( my_l_depth_mat, pt_old, 1, Scalar( 0, 0, 255 ), FILLED, LINE_8 );
//                circle( my_r_depth_mat, pt_new, 1, Scalar( 0, 0, 255 ), FILLED, LINE_8 );
//
//                imshow("test_left", my_l_depth_mat);
//                imshow("test_right", my_r_depth_mat);
//
//                cout << "old x = " << col << endl;
//                cout << "old y = " << row << endl;
//                cout << "new x = " << best_match.x << endl;
//                cout << "new y = " << best_match.y << endl;
//            }
//        }
//    }
//}

//void PointCloud::validate(int &top_col, int &top_row, int &width, int &height, int &slide) {
//    if (top_col < 0) {
//        width += top_col;
//        top_col = 0;
//    }
//
//    if (top_row < 0) {
//        height += top_row;
//        top_row = 0;
//    }
//
//    if ((width + top_col) > 640)
//        width = 640 - top_col;
//
//    if ((height + top_row) > 480)
//        height = 480 - top_row;
//
//    // check it
////    if (width == 0)
////        width = slide;
////    if (height == 0)
////        height = slide;
//}

void PointCloud::kNearest2(const VecArray &src, VecArray &nearestPoints, vector<int> &indices, vector<float> &dist, int kn) {
//    float distance;
//    const float t = vvr::getSeconds();
//
//    for (auto src_pt : src) {
//        for (int j=0; j<kn; j++) {
//            const KDNode **nearests = new const KDNode*[kn];
//            memset(nearests, NULL, kn * sizeof(KDNode*));
//
//            cout << "in" << endl;
//
//            m_dst_KDTree->kNearest(j, src_pt, m_dst_KDTree->root(), nearests, &distance);
//
//            cout << "nearest = " << (*nearests)->split_point << endl;
//
    // TODO check if i have to update *(nearests+j) to nearests[j]
//            nearestPoints.emplace_back((*nearests)->split_point);
//            dist.emplace_back(distance);
//        }
//    }
//
//    const float KDTree_knn_time = vvr::getSeconds() - t;
////    echo(KDTree_knn_time);

    float distance;
    const float t = vvr::getSeconds();

    for (auto src_pt : src) {
        const KDNode **nearests = new const KDNode*[kn];
        memset(nearests, NULL, kn * sizeof(KDNode*));

        for (int j=0; j<kn; j++) {
//            cout << 10 << endl;
//            cout << "j = " << j << endl;
//            cout << "src_pt = " << src_pt << endl;
//            cout << "m_dst_KDTree->root() = " << m_dst_KDTree->root()->split_point << endl;
            m_dst_KDTree->kNearest(j, src_pt, m_dst_KDTree->root(), nearests, &distance);
//            cout << 11 << endl;

            nearestPoints.emplace_back(nearests[j]->split_point);
            indices.emplace_back(nearests[j]->index);

//            nearestPoints.emplace_back((*nearests)->split_point);
//            indices.emplace_back((*nearests)->index);
            dist.emplace_back(distance);
        }
    }

    const float KDTree_knn_time = vvr::getSeconds() - t;
//    echo(KDTree_knn_time);
}

//void PointCloud::validate(int &top_col, int &top_row, int &width, int &height) {
//    CameraConstants camera;
//
//    if (top_col < 0) {
//        width += top_col;
//        top_col = 0;
//    }
//
//    if (top_row < 0) {
//        height += top_row;
//        top_row = 0;
//    }
//
//    if ((width + top_col) > camera.image_width)
//        width = camera.image_width - top_col;
//
//    if ((height + top_row) > camera.image_height)
//        height = camera.image_height - top_row;
//}

//pair<Eigen::Matrix3f, Eigen::Vector3f> PointCloud::computeRigidTransform(const VecArray &src, const VecArray &dst, vector<double> &weights) {
//    int size = dst.size();
//    vec src_center = getCentroid(src);
//    vec dst_center = getCentroid(dst);
//
//    vec src_centered_point, dst_centered_point;
//    Eigen::MatrixXf X(3,size), Y(3,size);
//    for (int i=0; i<size; i++) {
//        src_centered_point = src.at(i) - src_center;
//        dst_centered_point = dst.at(i) - dst_center;
//
//        X(0,i) = src_centered_point.x/(double)size;
//        X(1,i) = src_centered_point.y/(double)size;
//        X(2,i) = src_centered_point.z/(double)size;
//
//        Y(0,i) = dst_centered_point.x/(double)size;
//        Y(1,i) = dst_centered_point.y/(double)size;
//        Y(2,i) = dst_centered_point.z/(double)size;
//    }
//
//    cout << "before diagonal" << endl;
//    // create diagonal matrix
//    Eigen::DiagonalMatrix<float, Eigen::Dynamic> w(size);
//    for (int i=0; i<size; i++) {
//        w.diagonal()[i] = float(weights[i]);
////        cout << w.diagonal()[i] << endl;
//    }
//
////    Eigen::MatrixXf W(size, size);
////    int index{0};
////    for (int i=0; i<size; i++) {
////        for (int j=0; j<size; j++) {
////            if (i == j) {
////                W(i,j) = float(weights[index]);
////                index++;
////            }
////            else {
////                W(i,j) = 0.0f;
////            }
////        }
////    }
//
//    // compute the covariance matrix
////    Eigen::Matrix3f S = Y*W*X.transpose();
//    Eigen::Matrix3f S = Y*w*X.transpose();
//
//    // compute the singular value decomposition
//    Eigen::JacobiSVD<Eigen::MatrixXf> svd;
//    svd.compute(S, Eigen::ComputeThinU | Eigen::ComputeThinV );
//    if (!svd.computeU() || !svd.computeV()) {
//        std::cerr << "decomposition error" << endl;
//    }
//
//    // extract right singular vectors
//    Eigen::Matrix3f V = svd.matrixV();
//
//    // extract left singular vectors
//    Eigen::Matrix3f U = svd.matrixU();
//
//    // create diagonal matrix
//    Eigen::MatrixXf diag_mat(3, 3);
//    diag_mat.setZero();
//    diag_mat(0,0) = 1;
//    diag_mat(1,1) = 1;
//    diag_mat(2,2) = V.determinant()*U.transpose().determinant();
//
//    // compute rotation matrix
//    Eigen::Matrix3f R = V*diag_mat*U.transpose();
//
//    // compute translation vector
//    Eigen::Vector3f t = dataTypes::convertToEigenVector(src_center) - R*dataTypes::convertToEigenVector(dst_center);
//
//    return pair<Eigen::Matrix3f, Eigen::Vector3f>(R, t);
//}

void PointCloud::laplacian(const Mat &img, Mat &new_img) {
    Mat img_gray;
    int kernel_size = 3;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_8UC1;
    std::string window_name = "Laplace";

    int c;

    /// Remove noise by blurring with a Gaussian filter
    GaussianBlur(img, img, Size(3, 3), 0, 0, BORDER_DEFAULT);

    /// Convert the image to grayscale
    cvtColor(img, img_gray, CV_BGR2GRAY);

    /// Create window
    namedWindow(window_name, CV_WINDOW_AUTOSIZE);

    /// Apply Laplace function
    Mat abs_dst;

    Laplacian(img_gray, new_img, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(new_img, abs_dst);

    /// Show what you got
    imshow(window_name, abs_dst);

//    waitKey(0);
}

void PointCloud::cannyThreshold(const Mat &img, Mat &new_img)
{
    Mat img_gray, detected_edges;
    int edgeThresh = 1;
    int lowThreshold = 50;
    int const max_lowThreshold = 100;
    int ratio = 3;
    int kernel_size = 3;
    std::string window_name = "Edge Map";

    new_img.create( img.size(), img.type() );

    /// Convert the image to grayscale
    cvtColor( img, img_gray, CV_BGR2GRAY );

    /// Reduce noise with a kernel 3x3
    blur(img_gray, detected_edges, Size(3,3) );

    /// Canny detector
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    new_img = Scalar::all(0);

    img.copyTo( new_img, detected_edges);
    imshow( window_name, new_img );

//    waitKey(0);
}

pair<Eigen::Matrix3f, Eigen::Vector3f> PointCloud::icpNormals(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations, Mat &l_img, Mat &l_depth, Mat &r_img, Mat &r_depth) {
    vector<pair<Eigen::Matrix3f, Eigen::Vector3f>> all_R_t;
    VecArray nearestPoints;

    int counter{0};
    vector<double> weights;
    double value;
    VecArray l_normals, r_normals;
    while(counter++ < iterations) {
        nearestPoints.clear();
        dist.clear();

        kNearest(src_points, nearestPoints, dist, 1);

        weights.clear();
        l_normals.clear();
        r_normals.clear();

        computeNormals(l_img, l_depth, l_normals);
        computeNormals(r_img, r_depth, r_normals);
        for (int i=0; i<nearestPoints.size(); i++) {
            value = dot_product(l_normals[i], r_normals[i]);
            if (isnan(value)) {
                weights.emplace_back(0.0);
            }
            else {
                weights.emplace_back(value);
            }
//            cout << weights[i] << endl;
        }

        normalize(weights);
        all_R_t.emplace_back(computeRigidTransform(nearestPoints, src_points, weights));

        transformPoints(all_R_t.at(all_R_t.size()-1), src_points);

        mean_distance = vectorSum(dist)/(float)dist.size();
        normalize(dist);
        error = vectorSum(dist)/(float)dist.size();

        cout << "iter = " << counter << endl;
        cout << "mean_dist = " << mean_distance << endl;
        cout << "error = " << error << endl;
    }

    pair<Eigen::Matrix3f, Eigen::Vector3f> R_t;
    R_t = all_R_t.at(all_R_t.size()-1);
    for(int i=all_R_t.size()-2; i>=0; i--) {
        R_t.first *= all_R_t.at(i).first;
        R_t.second += all_R_t.at(i).second;
    }
    return R_t;
}
pair<Eigen::Matrix3f, Eigen::Vector3f> PointCloud::icpWeights(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations) {
    vector<pair<Eigen::Matrix3f, Eigen::Vector3f>> all_R_t;
    VecArray nearestPoints;

    int counter{0};
    vector<double> weights;
    while(counter++ < iterations) {
        nearestPoints.clear();
        dist.clear();

        kNearest(src_points, nearestPoints, dist, 1);

        float max_dist = max(dist);
        weights.clear();
        for (int i=0; i<src_points.size(); i++)
            weights.emplace_back(1.0f - dist[i]/max_dist);

        all_R_t.emplace_back(computeRigidTransform(nearestPoints, src_points, weights));

        transformPoints(all_R_t.at(all_R_t.size()-1), src_points);

        mean_distance = vectorSum(dist)/(float)dist.size();
        normalize(dist);
        error = vectorSum(dist)/(float)dist.size();

        cout << "iter = " << counter << endl;
        cout << "mean_dist = " << mean_distance << endl;
        cout << "error = " << error << endl;
    }

    pair<Eigen::Matrix3f, Eigen::Vector3f> R_t;
    R_t = all_R_t.at(all_R_t.size()-1);
    for(int i=all_R_t.size()-2; i>=0; i--) {
        R_t.first *= all_R_t.at(i).first;
        R_t.second += all_R_t.at(i).second;
    }
    return R_t;
}
pair<Eigen::Matrix3f, Eigen::Vector3f> PointCloud::icpCurves(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations, vector<double> &weights) {
    vector<pair<Eigen::Matrix3f, Eigen::Vector3f>> all_R_t;
    VecArray nearestPoints;

    int counter{0};
    while(counter++ < iterations) {
        nearestPoints.clear();
        dist.clear();

        kNearest(src_points, nearestPoints, dist, 1);

        all_R_t.emplace_back(computeRigidTransform(nearestPoints, src_points, weights));

        transformPoints(all_R_t.at(all_R_t.size()-1), src_points);

        mean_distance = vectorSum(dist)/(float)dist.size();
        normalize(dist);
        error = vectorSum(dist)/(float)dist.size();

        cout << "iter = " << counter << endl;
        cout << "mean_dist = " << mean_distance << endl;
        cout << "error = " << error << endl;
    }

    pair<Eigen::Matrix3f, Eigen::Vector3f> R_t;
    R_t = all_R_t.at(all_R_t.size()-1);
    for(int i=all_R_t.size()-2; i>=0; i--) {
        R_t.first *= all_R_t.at(i).first;
        R_t.second += all_R_t.at(i).second;
    }
    return R_t;
}

//void PointCloud::computeNormals(const Mat &img, const Mat &depth_img, Mat &normals) {
////    std::string window_name = "original img";
////    namedWindow( window_name, CV_WINDOW_AUTOSIZE );
////    imshow(window_name, img);
//
//    // 1 : smoothing using bilateral filter
//    Mat filtered_img;
//    bilateral(img, filtered_img);
//
////    std::string window_name2 = "bilateral img";
////    namedWindow( window_name2, CV_WINDOW_AUTOSIZE );
////    imshow(window_name2, filtered_img);
//
//    // 2 : depth gradient computation
//    int scale = 1;
//    int delta = 0;
//    int ddepth = CV_16SC1;
//    Mat img_gray;
//
//    /// Convert it to gray
//    cvtColor(filtered_img, img_gray, CV_BGR2GRAY);
//
//    /// Generate grad_x and grad_y
//    Mat der_z_x, der_z_y, sobel_img; // der_z_x = derivative of z to x
//
//    /// Gradient X
//    Sobel(img_gray, der_z_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
//
//    /// Gradient Y
//    Sobel(img_gray, der_z_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
//    addWeighted( der_z_x, 0.5, der_z_y, 0.5, 0, sobel_img );
//
////    std::string window_name3 = "Sobel img";
////    namedWindow( window_name3, CV_WINDOW_AUTOSIZE );
////    imshow(window_name3, sobel_img);
////
////    std::string window_name4 = "der_z_x";
////    namedWindow( window_name4, CV_WINDOW_AUTOSIZE );
////    imshow(window_name4, der_z_x);
////
////    std::string window_name5 = "der_z_y";
////    namedWindow( window_name5, CV_WINDOW_AUTOSIZE );
////    imshow(window_name5, der_z_y);
//
//    CameraConstants camera;
//
//    // 3 : normal estimation from depth gradients
//    // parameterize a 3D point (X,Y,Z) as a function of a pixel (x,y)
//    Mat xgrid, ygrid;
//    xgrid.create(camera.image_height,camera.image_width, CV_16SC1);
//    ygrid.create(camera.image_height,camera.image_width, CV_16SC1);
//
//    for (int i=0; i<camera.image_height; i++) {
//        for (int j=0; j<camera.image_width; j++) {
//            xgrid.at<short>(i,j) = static_cast<short>(j + 1 + (camera.topLeft[0] - 1) - camera.center[0]);
//            ygrid.at<short>(i,j) = static_cast<short>(i + 1 + (camera.topLeft[1] - 1) - camera.center[1]);
//        }
//    }
//
//    Mat der_x_x, der_x_y, der_y_x, der_y_y, x, y, z;
//    der_x_x.create(camera.image_height,camera.image_width, CV_16SC1);
//    der_x_y.create(camera.image_height,camera.image_width, CV_16SC1);
//    der_y_x.create(camera.image_height,camera.image_width, CV_16SC1);
//    der_y_y.create(camera.image_height,camera.image_width, CV_16SC1);
//    x.create(camera.image_height,camera.image_width, CV_16SC1);
//    y.create(camera.image_height,camera.image_width, CV_16SC1);
//    z.create(camera.image_height,camera.image_width, CV_16SC1);
////    res.create(camera.image_height,camera.image_width, CV_16SC3);
//
//      ///    if x == i
//    short c = short(camera.constant)/short(camera.mm_per_m);
//    for (int i=0; i<camera.image_height; i++) {
//        for (int j=0; j<camera.image_width; j++) {
//            der_x_x.at<short>(i,j) = xgrid.at<short>(i, j)*der_z_x.at<short>(i,j)/c;
//            der_x_y.at<short>(i,j) = (depth_img.at<unsigned short>(i,j) + xgrid.at<short>(i,j)*der_z_y.at<short>(i,j))/c;
//
//            der_y_x.at<short>(i,j) = (depth_img.at<unsigned short>(i,j) + ygrid.at<short>(i,j)*der_z_x.at<short>(i,j))/c;
//            der_y_y.at<short>(i,j) = (ygrid.at<short>(i,j)*der_z_y.at<short>(i,j))/c;
//
//            // compute the cross product between tangent vectors
//            // u_x = vec(der_x_x, der_y_x, der_z_x)
//            // u_y = vec(der_x_y, der_y_y, der_z_y)
//            x.at<short>(i,j) = der_y_x.at<short>(i,j) * der_z_y.at<short>(i,j) - der_z_x.at<short>(i,j) * der_y_y.at<short>(i,j);
//            y.at<short>(i,j) = der_z_x.at<short>(i,j) * der_x_y.at<short>(i,j) - der_x_x.at<short>(i,j) * der_z_y.at<short>(i,j);
//            z.at<short>(i,j) = der_x_x.at<short>(i,j) * der_y_y.at<short>(i,j) - der_y_x.at<short>(i,j) * der_x_y.at<short>(i,j);
//
//            // cross product
//            //    A = a1 * i + a2 * j + a3 * k
//            //    B = b1 * i + b2 * j + b3 * k.
//            //    (a2*b3 - a3*b2) * i
//            //    (a3*b1 - a1*b3) * j
//            //    (a1*b2 - a2*b1) * k
//
//            normals.at<Vec3s>(i,j).val[0] = z.at<short>(i,j);
//            normals.at<Vec3s>(i,j).val[1] = y.at<short>(i,j);
//            normals.at<Vec3s>(i,j).val[2] = x.at<short >(i,j);
//        }
//    }
////    std::string window_name6 = "x";
////    namedWindow( window_name6, CV_WINDOW_AUTOSIZE );
////    imshow(window_name6, x);
////
////    std::string window_name7 = "y";
////    namedWindow( window_name7, CV_WINDOW_AUTOSIZE );
////    imshow(window_name7, y);
////
////    std::string window_name8 = "z";
////    namedWindow( window_name8, CV_WINDOW_AUTOSIZE );
////    imshow(window_name8, z);
//
//
//    std::string window_name9 = "result";
//    namedWindow( window_name9, CV_WINDOW_AUTOSIZE );
//    imshow(window_name9, normals);
//}

//void PointCloud::computeNormals(const Mat &img, const Mat &depth_img, VecArray &normals) {
//    double minVal;
//    double maxVal;
//    Point minLoc;
//    Point maxLoc;
//
//    /// 1 : smoothing using bilateral filter
//    Mat filtered_img;
////    bilateral(img, filtered_img);
//
//    Mat img_float;
//    img.convertTo(img_float, CV_32F, 1.0/255.0);
//
//    CameraConstants camera;
//
//    /// 2 : depth gradient computation
//    int scale = 1;
//    int delta = 0;
//    int ddepth = CV_32F;
////    int ddepth = CV_64F;
//    Mat img_gray;
//
////     Convert it to gray
////    cvtColor(filtered_img, img_gray, CV_BGR2GRAY);
//    cvtColor(img_float, img_gray, CV_BGR2GRAY);
//
//    // Generate grad_x and grad_y
//    Mat der_z_x, der_z_y;
//
//    // Gradient X
//    Sobel(img_gray, der_z_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
////    convertScaleAbs( der_z_x, der_z_x);
////    cout << der_z_x << endl;
//
//    // Gradient Y
//    Sobel(img_gray, der_z_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
////    convertScaleAbs( der_z_y, der_z_y );
////    cout << der_z_y << endl;
//
//    /// 3 : normal estimation from depth gradients
//    Mat xgrid, ygrid;
//    xgrid.create(camera.image_height,camera.image_width, CV_16SC1);
//    ygrid.create(camera.image_height,camera.image_width, CV_16SC1);
//
//    for (int i=0; i<camera.image_height; i++) {
//        for (int j=0; j<camera.image_width; j++) {
//            xgrid.at<short>(i,j) = static_cast<short>(j + 1 + (camera.topLeft[0] - 1) - camera.center[0]);
//            ygrid.at<short>(i,j) = static_cast<short>(i + 1 + (camera.topLeft[1] - 1) - camera.center[1]);
//        }
//    }
//
//    Mat xgrid_float, ygrid_float, depth_img_float;
//    minMaxLoc( xgrid, &minVal, &maxVal, &minLoc, &maxLoc );
//    xgrid.convertTo(xgrid_float, CV_32F, 1.0/maxVal);
//
//    minMaxLoc( ygrid, &minVal, &maxVal, &minLoc, &maxLoc );
//    ygrid.convertTo(ygrid_float, CV_32F, 1.0/maxVal);
//
//    minMaxLoc( depth_img, &minVal, &maxVal, &minLoc, &maxLoc );
//    depth_img.convertTo(depth_img_float, CV_32F, 1.0/maxVal);
//
////    Mat res_x, res_y, res_z;
////    res_x.create(camera.image_height,camera.image_width, CV_32F);
////    res_y.create(camera.image_height,camera.image_width, CV_32F);
////    res_z.create(camera.image_height,camera.image_width, CV_32F);
//
//    int counter{0};
//    float der_x_x, der_x_y, der_y_x, der_y_y;
//    float c = camera.constant/camera.mm_per_m;
//    vec my_vec, t_x, t_y;
//    for (int i=0; i<camera.image_height; i++) {
//        for (int j=0; j<camera.image_width; j++) {
//            // compute the partial derivatives
//            der_x_x = xgrid_float.at<float>(i, j)*der_z_x.at<float>(i,j)/c;
//            der_x_y = (depth_img_float.at<float>(i,j) + xgrid_float.at<float>(i,j)*der_z_y.at<float>(i,j))/c;
//            der_y_x = (depth_img_float.at<float>(i,j) + ygrid_float.at<float>(i,j)*der_z_x.at<float>(i,j))/c;
//            der_y_y = (ygrid_float.at<float>(i,j)*der_z_y.at<float>(i,j))/c;
//
////            der_x_x = xgrid_float.at<float>(i, j)*der_z_x.at<float>(i,j)/c;
////            der_x_y = (depth_img_float.at<float>(i,j) + xgrid_float.at<float>(i,j)*der_z_y.at<float>(i,j))/c;
////            der_y_x = (depth_img_float.at<float>(i,j) + ygrid_float.at<float>(i,j)*der_z_x.at<float>(i,j))/c;
////            der_y_y = (ygrid_float.at<float>(i,j)*der_z_y.at<float>(i,j))/c;
//
//
//            // define the tangent vectors of the surface
//            t_x.x = der_x_x; t_x.y = der_y_x; t_x.z = der_z_x.at<float>(i,j);
//            t_y.x = der_x_y; t_y.y = der_y_y; t_y.z = der_z_y.at<float>(i,j);
//
//
//
//            // a normal vector is obtained from the cross product of two tangent vectors
//            my_vec = cross_product(t_x, t_y);
//
////            if(my_vec.Length() != 0.0f) {
//                // normalize the normal
////                my_vec /= my_vec.Length();
////            }
//
//
//
//            if (counter > 101800 && counter < 101900) {
//                cout << my_vec << endl;
//                cout << my_vec.Length() << endl;
//                cout << "depth_img = " << depth_img_float.at<float>(i,j) << endl;
//                cout << "der_z_x = " << der_z_x.at<float>(i,j) << endl;
//                cout << "der_z_y = " << der_z_y.at<float>(i,j) << endl;
//                cout << "der_x_x = " << der_x_x << endl;
//                cout << "der_x_y = " << der_x_y << endl;
//                cout << "der_y_x = " << der_y_x << endl;
//                cout << "der_y_y = " << der_y_y << endl << endl;
//            }
//
//            // normalize the normal
//            my_vec /= my_vec.Length();
//
//            // add the normal
//            normals.emplace_back(my_vec);
//
////            res_x.at<float>(i,j) = my_vec.x;
////            res_y.at<float>(i,j) = my_vec.y;
////            res_z.at<float>(i,j) = my_vec.z;
//
////            res.at<Vec3f>(i,j).val[0] = my_vec.z;
////            res.at<Vec3f>(i,j).val[1] = my_vec.y;
////            res.at<Vec3f>(i,j).val[2] = my_vec.x;
//
//            counter++;
//        }
//    }
//
////    Mat res_x_final, res_y_final, res_z_final;
////    minMaxLoc( res_x, &minVal, &maxVal, &minLoc, &maxLoc );
////    res_x.convertTo(res_x_final, CV_16SC1);
////
////    minMaxLoc( res_y, &minVal, &maxVal, &minLoc, &maxLoc );
////    res_y.convertTo(res_y_final, CV_16SC1);
////
////    minMaxLoc( res_z, &minVal, &maxVal, &minLoc, &maxLoc );
////    res_z.convertTo(res_z_final, CV_16SC1);
////
////    for (int i=0; i<camera.image_height; i++) {
////        for (int j=0; j<camera.image_width; j++) {
////            normals.emplace_back(vec(res_x_final.at<short>(i,j), res_y_final.at<short>(i,j), res_z_final.at<short>(i,j)));
////        }
////    }
//
////    cout << res_x_final << endl;
//
////    std::string window_name9 = "result";
////    namedWindow( window_name9, CV_WINDOW_AUTOSIZE );
////    imshow(window_name9, res);
//}

//void PointCloud::computeNormals(const Mat &img, const Mat &depth_img, VecArray &normals) {
//    // find min and max of point.x, point.y and point.z
//    vector<float> points_x, points_y, points_z;
//    for (int i=0; i<m_points.size(); i++) {
//        points_x.emplace_back(m_points[i].first.x);
//        points_y.emplace_back(m_points[i].first.y);
//        points_z.emplace_back(m_points[i].first.z);
//    }
//    float min_points_x, min_points_y, min_points_z, max_points_x, max_points_y, max_points_z;
//    min_points_x = min(points_x);
//    min_points_y = min(points_y);
//    min_points_z = min(points_z);
//    max_points_x = max(points_x);
//    max_points_y = max(points_y);
//    max_points_z = max(points_z);
//
//    cout << "min_points_x = " << min_points_x << endl;
//    cout << "min_points_y = " << min_points_y << endl;
//    cout << "min_points_z = " << min_points_z << endl;
//    cout << "max_points_x = " << max_points_x << endl;
//    cout << "max_points_y = " << max_points_y << endl;
//    cout << "max_points_z = " << max_points_z << endl;
//
//    points_x.clear();
//    points_y.clear();
//    points_z.clear();
//
//    double minVal;
//    double maxVal;
//    Point minLoc;
//    Point maxLoc;
//
//    /// 1 : smoothing using bilateral filter
//    Mat filtered_img;
//    bilateral(img, filtered_img);
//
//    Mat img_float;
//    filtered_img.convertTo(img_float, CV_32F, 1.0/255.0);
////    img.convertTo(img_float, CV_32F, 1.0/255.0);
//
//    CameraConstants camera;
//
//    /// 2 : depth gradient computation
//    int scale = 1;
//    int delta = 0;
//    int ddepth = CV_32F;
//    Mat img_gray;
//
////     Convert it to gray
////    cvtColor(filtered_img, img_gray, CV_BGR2GRAY);
//    cvtColor(img_float, img_gray, CV_BGR2GRAY);
//
//    // Generate grad_x and grad_y
//    Mat der_z_x, der_z_y;
//
//    // Gradient X
//    Sobel(img_gray, der_z_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
////    convertScaleAbs( der_z_x, der_z_x);
//
//    // Gradient Y
//    Sobel(img_gray, der_z_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
////    convertScaleAbs( der_z_y, der_z_y );
//
//    /// 3 : normal estimation from depth gradients
//    Mat xgrid, ygrid;
//    xgrid.create(camera.image_height,camera.image_width, CV_16SC1);
//    ygrid.create(camera.image_height,camera.image_width, CV_16SC1);
//
//    for (int i=0; i<camera.image_height; i++) {
//        for (int j=0; j<camera.image_width; j++) {
//            xgrid.at<short>(i,j) = static_cast<short>(j + 1 + (camera.topLeft[0] - 1) - camera.center[0]);
//            ygrid.at<short>(i,j) = static_cast<short>(i + 1 + (camera.topLeft[1] - 1) - camera.center[1]);
//        }
//    }
//
//    Mat xgrid_float, ygrid_float, depth_img_float;
//    minMaxLoc( xgrid, &minVal, &maxVal, &minLoc, &maxLoc );
//    xgrid.convertTo(xgrid_float, CV_32F, 1.0/maxVal);
//
//    minMaxLoc( ygrid, &minVal, &maxVal, &minLoc, &maxLoc );
//    ygrid.convertTo(ygrid_float, CV_32F, 1.0/maxVal);
//
//    minMaxLoc( depth_img, &minVal, &maxVal, &minLoc, &maxLoc );
//    depth_img.convertTo(depth_img_float, CV_32F, 1.0/maxVal);
//
//    Mat res_x, res_y, res_z, res, res2;
//    res.create(camera.image_height,camera.image_width, CV_32FC3);
//    res2.create(camera.image_height,camera.image_width, CV_16SC3);
////    res_y.create(camera.image_height,camera.image_width, CV_32F);
////    res_z.create(camera.image_height,camera.image_width, CV_32F);
//
//    int counter{0};
//    float der_x_x, der_x_y, der_y_x, der_y_y;
//    float c = camera.constant/camera.mm_per_m;
//    vec my_vec, t_x, t_y;
//    for (int i=0; i<camera.image_height; i++) {
//        for (int j=0; j<camera.image_width; j++) {
//            // compute the partial derivatives
//            der_x_x = xgrid_float.at<float>(i, j)*der_z_x.at<float>(i,j)/c;
//            der_x_y = (depth_img_float.at<float>(i,j) + xgrid_float.at<float>(i,j)*der_z_y.at<float>(i,j))/c;
//            der_y_x = (depth_img_float.at<float>(i,j) + ygrid_float.at<float>(i,j)*der_z_x.at<float>(i,j))/c;
//            der_y_y = (ygrid_float.at<float>(i,j)*der_z_y.at<float>(i,j))/c;
//
//            // define the tangent vectors of the surface
//            t_x.x = der_x_x; t_x.y = der_y_x; t_x.z = der_z_x.at<float>(i,j);
//            t_y.x = der_x_y; t_y.y = der_y_y; t_y.z = der_z_y.at<float>(i,j);
//
//            // a normal vector is obtained from the cross product of two tangent vectors
//            my_vec = cross_product(t_x, t_y);
//
////            if(my_vec.Length() != 0.0f) {
//                // normalize the normal
////                my_vec /= my_vec.Length();
////            }
//
//            if (counter > 101800 && counter < 101900) {
//                cout << my_vec << endl;
//                cout << my_vec.Length() << endl;
//                cout << "depth_img = " << depth_img_float.at<float>(i,j) << endl;
//                cout << "der_z_x = " << der_z_x.at<float>(i,j) << endl;
//                cout << "der_z_y = " << der_z_y.at<float>(i,j) << endl;
//                cout << "der_x_x = " << der_x_x << endl;
//                cout << "der_x_y = " << der_x_y << endl;
//                cout << "der_y_x = " << der_y_x << endl;
//                cout << "der_y_y = " << der_y_y << endl << endl;
//            }
//
//            // normalize the normal
//            my_vec /= my_vec.Length();
////            my_vec /= 100.0f;
//
//            // add the normal
//            normals.emplace_back(my_vec);
//
//            points_x.emplace_back(my_vec.x);
//            points_y.emplace_back(my_vec.y);
//            points_z.emplace_back(my_vec.z);
//
////            res_x.at<float>(i,j) = my_vec.x;
////            res_y.at<float>(i,j) = my_vec.y;
////            res_z.at<float>(i,j) = my_vec.z;
//
////            res.at<Vec3f>(i,j).val[0] = my_vec.z;
////            res.at<Vec3f>(i,j).val[1] = my_vec.y;
////            res.at<Vec3f>(i,j).val[2] = my_vec.x;
//
//            counter++;
//        }
//    }
//
//    float new_min_points_x, new_min_points_y, new_min_points_z, new_max_points_x, new_max_points_y, new_max_points_z;
//    new_min_points_x = min(points_x);
//    new_min_points_y = min(points_y);
//    new_min_points_z = min(points_z);
//    new_max_points_x = max(points_x);
//    new_max_points_y = max(points_y);
//    new_max_points_z = max(points_z);
//
//    cout << "new_min_points_x = " << new_min_points_x << endl;
//    cout << "new_min_points_y = " << new_min_points_y << endl;
//    cout << "new_min_points_z = " << new_min_points_z << endl;
//    cout << "new_max_points_x = " << new_max_points_x << endl;
//    cout << "new_max_points_y = " << new_max_points_y << endl;
//    cout << "new_max_points_z = " << new_max_points_z << endl;
//
//    //    v’ = (v-min)/(max-min) * (newmax-newmin) + newmin
//    for (int i=0; i<normals.size(); i++) {
////        normals[i].x = (normals[i].x-new_min_points_x)/(new_max_points_x - new_min_points_x) * (max_points_x - min_points_x) + min_points_x;
////        normals[i].y = (normals[i].y-new_min_points_y)/(new_max_points_y - new_min_points_y) * (max_points_y - min_points_y) + min_points_y;
////        normals[i].z = (normals[i].z-new_min_points_z)/(new_max_points_z - new_min_points_z) * (max_points_z - min_points_z) + min_points_z;
////        cout << normals[i] << endl;
////        normals[i] /= normals[i].Length();
//    }
//
//
////    Mat res_x_final, res_y_final, res_z_final;
////    minMaxLoc( res_x, &minVal, &maxVal, &minLoc, &maxLoc );
////    res_x.convertTo(res_x_final, CV_16SC1);
////
////    minMaxLoc( res_y, &minVal, &maxVal, &minLoc, &maxLoc );
////    res_y.convertTo(res_y_final, CV_16SC1);
////
////    minMaxLoc( res_z, &minVal, &maxVal, &minLoc, &maxLoc );
////    res_z.convertTo(res_z_final, CV_16SC1);
////
////    for (int i=0; i<camera.image_height; i++) {
////        for (int j=0; j<camera.image_width; j++) {
////            normals.emplace_back(vec(res_x_final.at<short>(i,j), res_y_final.at<short>(i,j), res_z_final.at<short>(i,j)));
////        }
////    }
//
////    cout << res_x_final << endl;
//    Mat final;
//    final.create(camera.image_height,camera.image_width, CV_16SC3);
//
////    minMaxLoc( res, &minVal, &maxVal, &minLoc, &maxLoc );
//    res.convertTo(final, CV_16SC3);
//
////    cout << res << endl;
//
//    //    res.convertTo(final, CV_16SC3);
////    cout << final << endl;
//
//    std::string window_name9 = "result";
//    namedWindow( window_name9, CV_WINDOW_AUTOSIZE );
//    imshow(window_name9, final);
//}

//void PointCloud::edgeDetection(const Mat &img, const Mat &depth_img) {
//    double t_rgb1=40.0, t_rgb2=100.0, t_hc1=0.6, t_hc2=1.2, t_dd=0.04, t_search=100;
//    Mat img_gray;
//
//    /// convert it to gray
//    cvtColor(img, img_gray, CV_BGR2GRAY);
//    std::string window_name = "grey img";
//    namedWindow( window_name, CV_WINDOW_AUTOSIZE );
//    imshow( window_name, img_gray );
//    /// convert it to gray
//
//    /// canny
//    Mat Ergb, canny_img;
//    int kernel_size = 3;
//
//    canny_img.create( img.size(), img.type() );
//
//    // Convert the image to grayscale
//    cvtColor( img, img_gray, CV_BGR2GRAY );
//
//    // Reduce noise with a kernel 3x3
//    blur(img_gray, Ergb, Size(3,3) );
//
//    // Canny detector
//    Canny( Ergb, Ergb, t_rgb1, t_rgb2, kernel_size );
//
//    // Using Canny's output as a mask, we display our result
//    canny_img = Scalar::all(0);
//
//    img.copyTo( canny_img, Ergb);
//    std::string window_name2 = "canny of rgb";
//    namedWindow( window_name2, CV_WINDOW_AUTOSIZE );
//    imshow( window_name2, canny_img );
//    /// canny
//
//    /// normal estimation
//    CameraConstants camera;
////    Mat normals;
////    normals.create(camera.image_height,camera.image_width, CV_16SC3);
////    computeNormals(img, depth_img, normals);
//    /// normal estimation
//}

void PointCloud::kmeans(VecArray &points) {
    vector<array<float, 3>> data;
    data = dataTypes::convertTo3dArray(points);

//    std::vector<std::array<float, 3>> data{{4.f, 4.f, 4.f}, {2.f, 2.f, 2.f}, {1200.f, 1200.f, 1200.f}, {3.f, 3.f, 3.f}};
    auto cluster_data = dkm::kmeans_lloyd(data, 3);

//    std::tuple<std::array<T, N>

    std::cout << "Means:" << std::endl;
    std::cout << "num of means = " << std::get<0>(cluster_data).size() << std::endl;
    for (const auto& mean : std::get<0>(cluster_data)) {
        std::cout << "\t(" << mean[0] << "," << mean[1] << "," << mean[2] << ")" << std::endl;
    }

    std::cout << "\nCluster labels:" << std::endl;
    std::cout << "\tPoint:";
    for (const auto& point : data) {
        std::stringstream value;
        value << "(" << point[0] << "," << point[1] << "," << point[2] << ")";
        std::cout << std::setw(20) << value.str();
    }
    std::cout << std::endl;
    std::cout << "\tLabel:";
    for (const auto& label : std::get<1>(cluster_data)) {
        std::cout << std::setw(20) << label;
    }
    std::cout << std::endl;

}

void PointCloud::getCurvatureOld(Mesh*& mesh, vector<int>* tri_indices, vector<float> &curvature, VecArray &normals) {
    int size = mesh->getVertices().size();
    Eigen::MatrixXf Cov(3,3);
    int index;
    vec center, norm;
    VecArray neighbors;

    // iterate through all vertices
    for (int i=0; i<size; i++) {
        center.x = center.y = center.z = 0.0f;
        neighbors.clear();

        // if there are not neighbors (points without depth)
        if(tri_indices[i].empty()) {
            curvature.emplace_back(0.0f);
        }
        else {
            // get the neighbors of each vertex
            for (int j=0; j<tri_indices[i].size(); j++) {
                index = tri_indices[i][j];

                if (i != mesh->getTriangles()[index].vi1) {
                    neighbors.emplace_back(mesh->getTriangles()[index].v1());
                    center += mesh->getTriangles()[index].v1();
                }

                if (i != mesh->getTriangles()[index].vi2) {
                    neighbors.emplace_back(mesh->getTriangles()[index].v2());
                    center += mesh->getTriangles()[index].v2();
                }

                if (i != mesh->getTriangles()[index].vi3) {
                    neighbors.emplace_back(mesh->getTriangles()[index].v3());
                    center += mesh->getTriangles()[index].v3();
                }
            }

            // compute the center of the neighboring points of each vertex
            center /= float(neighbors.size());

            // compute the covariance matrix
            Eigen::MatrixXf P(neighbors.size(),3);
            for (int j=0; j<neighbors.size(); j++) {
                P(j,0) = neighbors[j].x - center.x;
                P(j,1) = neighbors[j].y - center.y;
                P(j,2) = neighbors[j].z - center.z;
            }

            Cov = P.transpose()*P;

            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(Cov);
//            if (eigensolver.info() != Success) abort();
//            if (i==0) {
//                cout << "The eigenvalues of Cov are:\n" << eigensolver.eigenvalues() << endl;
//                cout << "Here's a matrix whose columns are eigenvectors of Cov \n"
//                     << "corresponding to these eigenvalues:\n"
//                        << eigensolver.eigenvectors() << endl;
//            }

            // extract the normals and curvatures using the pair of (eigen value, eigen vector)
            curvature.emplace_back((eigensolver.eigenvalues()[0])/
                                   (eigensolver.eigenvalues()[0] + eigensolver.eigenvalues()[1] + eigensolver.eigenvalues()[2]));

            norm.x = eigensolver.eigenvectors()(0);
            norm.y = eigensolver.eigenvectors()(1);
            norm.z = eigensolver.eigenvectors()(2);
            normals.emplace_back(-norm);
        }
    }
}

// compute the angle between the data point normal direction and the 4 neighboring points normal direction
void PointCloud::getNormalAngle(const Mat &original_img, const Mat &thresholded_img, const Mat &depth_img, const int &value, vector<double> &curve_degrees) {
    CameraConstants camera;
    uchar val;
    vec edge_normal, neighbor_normal;

    /// 1 : smoothing using bilateral filter
    Mat filtered_img;
    bilateral(original_img, filtered_img);

    /// 2 : depth gradient computation
    Mat img_gray;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_32FC1;

    // Convert it to gray
    cvtColor(filtered_img, img_gray, CV_BGR2GRAY);

    // The angle parameter between data point and neighboring points is calculated by summing all the
    // normal angle among its neighboring points:
    double sum{0};
    for (int i=0; i<camera.image_height; i++) {
        for (int j=0; j<camera.image_width; j++) {
            val = thresholded_img.at<uchar>(i,j);
            if ((int)val == value) {
                //TODO validate that exist 4 neighbors

                // get the basic normal
                computeNormal(img_gray, depth_img, i, j, edge_normal);

                // get the first neighbor's normal
                computeNormal(img_gray, depth_img, i-1, j, neighbor_normal);
                sum += getAngle(edge_normal, neighbor_normal);

                // get the second neighbor's normal
                computeNormal(img_gray, depth_img, i, j-1, neighbor_normal);
                sum += getAngle(edge_normal, neighbor_normal);

                // get the third neighbor's normal
                computeNormal(img_gray, depth_img, i+1, j, neighbor_normal);
                sum += getAngle(edge_normal, neighbor_normal);

                // get the fourth neighbor's normal
                computeNormal(img_gray, depth_img, i, j+1, neighbor_normal);
                sum += getAngle(edge_normal, neighbor_normal);

//                cout << "degree = " << sum << endl;

                //TODO check it
                if (!isnan(sum))
                    curve_degrees.emplace_back(sum);
                sum = 0.0;
            }
        }
    }
}

//void PointCloud::triangulateMesh(const std::vector<vec>& vertices, Mesh*& mesh) {
//    mesh = new Mesh();
//    std::vector<vec> &modelVerts = mesh->getVertices();
//    std::vector<vvr::Triangle> &tris = mesh->getTriangles();
//
//    modelVerts = vertices;
//
//    CameraConstants camera;
//    int size = (camera.image_height - 1) * (camera.image_width - 1);
//    for (int i = 0; i < size; i++) {
//        tris.emplace_back(&modelVerts, i, i + 1 + camera.image_width, i + camera.image_width);
//        tris.emplace_back(&modelVerts, i, i + 1, i + 1 + camera.image_width);
//
////        tris.emplace_back(&modelVerts, i, i + 1 + camera.image_height, i + camera.image_height);
////        tris.emplace_back(&modelVerts, i, i + 1, i + 1 + camera.image_height);
//    }
//}

// Region growing segmentation algorithm
//void PointCloud::segmentation(Mesh*& mesh, vector<PointFeatures> pointFeatures, vector<VecArray> &segments) {
//    VecArray region, tree_data, nearest, src, deleted;
//    vector<float> dist;
//    vector<int> indices;
//    vector<PointFeatures> seeds;
//    vec neighbor;
//    int index;
//
//    m_neighbours = 70;
//    m_angle_th = 30;
//    m_curv_th = 0.8;
//
//    // sort data by their curvature values in ascending order
////    std::sort(pointFeatures.begin(), pointFeatures.end(), PointFeaturesComparator());
//
////    int n = int(pointFeatures.size())*85/100;
////    m_curv_th = pointFeatures[n].curvature;
//
//    /// test
////    vector<float> test_curv;
////    float sum{0.0f};
////    for (int p=0; p<pointFeatures.size(); p++) {
////        test_curv.emplace_back(pointFeatures[p].curvature);
////        sum+=pointFeatures[p].curvature;
////    }
////    sum /= float(pointFeatures.size());
//////    m_curv_th = sum;
////    cout << "mean of curv is " << sum << endl;
////    cout << "max of curv is " << max(test_curv) << endl;
////    cout << "min of curv is " << min(test_curv) << endl;
//    /// test
//
//    for (int k=0; k<pointFeatures.size(); k++)
//        tree_data.emplace_back(pointFeatures[k].coordinates);
//
//    m_dst_KDTree = new KDTree(tree_data);
//
//    int tmp{0};
//    while(pointFeatures.size() >= m_neighbours) {
//
////        if (pointFeatures[0].coordinates.Length() != 0) {
//
////            cout << " point = " << pointFeatures[0].coordinates << endl;
//
//            if (tmp == 1)
//                break;
//            tmp++;
//
//            cout << endl << "points are " << pointFeatures.size() << endl;
//
//            region.clear();
//            seeds.clear();
//
//            // get the point with minimum curvature
//            seeds.emplace_back(pointFeatures[0]);
//            region.emplace_back(pointFeatures[0].coordinates);
//
//            m_dst_KDTree->m_root = m_dst_KDTree->deleteNode(m_dst_KDTree->m_root, pointFeatures[0].coordinates);
//
//            if(pointFeatures.size() == 1)
//                break;
//
//            pointFeatures.erase(pointFeatures.begin());
//
//            for (int i=0; i<seeds.size(); i++) {
//
//                cout << "seeds size = " << seeds.size() << endl;
////                cout << "i = " << i << endl;
////                cout << " seed = " << seeds[i].coordinates << endl;
//
//            if (seeds.size() > 20000)
//                break;
//
//                src.clear();
//                dist.clear();
//                nearest.clear();
//                indices.clear();
//
//                // find nearest neighbours of current seed point
//                src.emplace_back(seeds[i].coordinates);
//                kNearest2(src, nearest, indices, dist, m_neighbours);
//
//                // iterate through all neighbors
////                cout << "num = " << nearest.size() << endl;
//
//                for(int j=0; j<nearest.size(); j++) {
////                    cout << "neighbor = " << nearest[j] << endl;
//
//                    double val =  abs(getAngle(seeds[i].normal, pointFeatures[indices[j]].normal));
//
//                    if (val < m_angle_th) {
////                        cout << "in" << endl;
//                        region.emplace_back(pointFeatures[indices[j]].coordinates);
//
//                        // delete pointFeatures.begin() + indices[j] from tree
//                        m_dst_KDTree->m_root = m_dst_KDTree->deleteNode(m_dst_KDTree->m_root, pointFeatures[indices[j]].coordinates);
//
//                        if(pointFeatures.size() == 1)
//                            break;
//
//                        if (val < 10) {
//                            seeds.emplace_back(pointFeatures[indices[j]]);
//                            pointFeatures.erase(pointFeatures.begin() + indices[j]);
//                        }
//
////                    for (int k=0; k<pointFeatures[indices[j]].neighborVertices.size(); k++) {
////                        index = pointFeatures[indices[j]].neighborVertices[k];
//////                        neighbor = mesh->getVertices()[index];
////                        seeds.emplace_back(pointFeatures[index]);
////                    }
//
////                    if(pointFeatures[indices[j]].curvature < m_curv_th) {
////                        seeds.emplace_back(pointFeatures[indices[j]]);
////                    }
//
//                    }
//                }
//            }
//            segments.emplace_back(region);
////        }
//    }
//
//    for (int i=0; i<segments.size(); i++) {
//        cout << "segment " << i << " with " << segments[i].size() << " points" << endl;
//    }
//    cout << "global segments are " << segments.size() << endl;
//}
//void PointCloud::segmentation2(Mesh*& mesh, vector<PointFeatures> pointFeatures, vector<VecArray> &segments) {
//    VecArray region, tree_data, nearest, src, deleted;
//    vector<float> dist;
//    vector<int> indices;
//    vector<PointFeatures> seeds;
//    vec neighbor;
//    int index;
//
//    m_neighbours = 30;
//    m_angle_th = 15;
//    m_curv_th = 0.8;
//
//    // sort data by their curvature values in ascending order
////    std::sort(pointFeatures.begin(), pointFeatures.end(), PointFeaturesComparator());
//
//    for (int k=0; k<pointFeatures.size(); k++)
//        tree_data.emplace_back(pointFeatures[k].coordinates);
//
//    m_dst_KDTree = new KDTree(tree_data);
//
//    int tmp{0};
//    while(pointFeatures.size() >= m_neighbours) {
//
//        if (tmp == 1)
//            break;
//        tmp++;
//
//        cout << "points are " << pointFeatures.size() << endl;
//
//        region.clear();
//        seeds.clear();
//
//        // get the point with minimum curvature
//        seeds.emplace_back(pointFeatures[0]);
//        region.emplace_back(pointFeatures[0].coordinates);
//
//        m_dst_KDTree->m_root = m_dst_KDTree->deleteNode(m_dst_KDTree->m_root, pointFeatures[0].coordinates);
//
//        if(pointFeatures.size() == 1)
//            break;
//
//        pointFeatures.erase(pointFeatures.begin()+1);
//
//        for (int i=0; i<seeds.size(); i++) {
//            cout << "seeds size = " << seeds.size() << endl;
//
//            src.clear();
//            dist.clear();
//            nearest.clear();
//            indices.clear();
//
//            // find nearest neighbours of current seed point
//            src.emplace_back(seeds[i].coordinates);
//            kNearest2(src, nearest, indices, dist, m_neighbours);
//
//            // iterate through all neighbors
//            for(int j=0; j<nearest.size(); j++) {
//
//                double val =  abs(getAngle(seeds[i].normal, pointFeatures[indices[j]].normal));
//
//                if (val < m_angle_th) {
//                    region.emplace_back(pointFeatures[indices[j]].coordinates);
//
//                    // delete pointFeatures.begin() + indices[j] from tree
//                    m_dst_KDTree->m_root = m_dst_KDTree->deleteNode(m_dst_KDTree->m_root, pointFeatures[indices[j]].coordinates);
//
//                    if(pointFeatures.size() == 1)
//                        break;
//
//                    pointFeatures.erase(pointFeatures.begin() + indices[j]);
//
//                }
//            }
//        }
//        segments.emplace_back(region);
//    }
//
//    cout << "global segments are " << segments.size() << endl;
//}
//void PointCloud::segmentation3(Mesh*& mesh, vector<PointFeatures> pointFeatures, vector<VecArray> &segments) {
//    VecArray region;
//
//    int index, index2, start;
//    for (int i=0; i<pointFeatures.size(); i++) {
//        if (pointFeatures[i].curvature > 0.3) {
//            region.clear();
//
//            start = i;
//
//            index = start;
//            vector<int> seen;
//            float curv;
//            for (int l=0; l<20000; l++) {
//
//                curv = -1.0f;
//                for (int j=0; j<pointFeatures[index].neighborVertices.size(); j++) {
//
//                    bool flag{false};
//                    for (int k=0; k<seen.size(); k++) {
//                        if (pointFeatures[index].neighborVertices[j] == seen[k])
//                            flag = true;
//                    }
//
//                    if (pointFeatures[pointFeatures[index].neighborVertices[j]].curvature > curv && flag == false) {
//                        curv = pointFeatures[pointFeatures[index].neighborVertices[j]].curvature;
//                        start = pointFeatures[index].neighborVertices[j];
//                        seen.emplace_back(start);
//                    }
//                }
//
//                index = start;
//                region.emplace_back(pointFeatures[start].coordinates);
//            }
//            segments.emplace_back(region);
//        }
//    }
//}
//void PointCloud::segmentation4(const vector<PointFeatures> pointFeatures, VecArray &region, double const &t, int &start, vector<int> &seen, int &iter) {
//    if (iter > 300)
//        return;
//    else
//        iter++;
//
//    int index2;
//    for (int j=0; j<pointFeatures[start].neighborVertices.size(); j++) {
//
//        index2 = pointFeatures[start].neighborVertices[j];
//
//        if (getAngle(pointFeatures[index2].normal, pointFeatures[start].normal) < t) {
//
//            region.emplace_back(pointFeatures[index2].coordinates);
//
//            bool flag{false};
//            for( int m=0; m<seen.size(); m++) {
//                if (seen[m] == index2)
//                    flag = true;
//            }
//            if (!flag) {
//                seen.emplace_back(index2);
//                segmentation4(pointFeatures, region, t, index2, seen, iter);
//            }
//        }
//    }
//}

//void PointCloud::segmentation(const vector<PointFeatures> pointFeatures, vector<PointFeatures> &region, vector<int> &hash, int start) {
//    int index;
//    vector<int> indices;
//    indices.emplace_back(start);
//
//    for (int i=0; i<indices.size(); i++) {
//        start = indices[i];
//
//        for (int j=0; j<pointFeatures[start].neighborVertices.size(); j++) {
//            index = pointFeatures[start].neighborVertices[j];
//
//            if (hash[index] == 0) {
//                region.emplace_back(pointFeatures[index]);
//                indices.emplace_back(index);
//                hash[index] = 1;
//            }
//        }
//    }
//
////    int index;
////    double angle, threshold{1.0};
////    vector<int> indices;
////    indices.emplace_back(start);
////
////    for (int i=0; i<indices.size(); i++) {
////        start = indices[i];
////
////        for (int j=0; j<pointFeatures[start].neighborVertices.size(); j++) {
////            index = pointFeatures[start].neighborVertices[j];
////            angle = abs(getAngle(pointFeatures[start].normal, pointFeatures[index].normal));
////
////            if ((hash[index] == 0) && (angle < threshold)) {
////                region.emplace_back(pointFeatures[index]);
////                indices.emplace_back(index);
////                hash[index] = 1;
////            }
////        }
////    }
//
//
////    int index;
////    double angle, angle_t{4.0}, curv_t{0.3};
////    float curv;
////    vector<int> indices;
////    indices.emplace_back(start);
////
////    for (int i=0; i<indices.size(); i++) {
////        start = indices[i];
////
////        for (int j=0; j<pointFeatures[start].neighborVertices.size(); j++) {
////            index = pointFeatures[start].neighborVertices[j];
////            angle = abs(getAngle(pointFeatures[start].normal, pointFeatures[index].normal));
////            curv = pointFeatures[index].curvature;
////
//////            cout << angle << endl;
////
////            if ((hash[index] == 0)) {
////                if (angle < angle_t) {
////                    region.emplace_back(pointFeatures[index]);
//////                if ( abs(pointFeatures[start].coordinates.z - pointFeatures[index].coordinates.z) < 0.1 ) {
////                    if (curv < curv_t) {
////                        hash[index] = 1;
////                        indices.emplace_back(index);
////                    }
////                }
////            }
////        }
////    }
//}

//void PointCloud::planeFitting2(const vector<PointFeatures> &segment, vec &plane_normal, vec &centroid) {
//    double sum_x{0.0}, sum_y{0.0}, sum_z{0.0}, sum_x_y{0.0}, sum_x_z{0.0}, sum_y_z{0.0}, sum_x_2{0.0}, sum_y_2{0.0};
//    int size = segment.size();
//
//    for (int i=0; i<size; i++) {
//            sum_x += segment[i].coordinates.x;
//            sum_y += segment[i].coordinates.y;
//            sum_z += segment[i].coordinates.z;
//
//            sum_x_2 += pow(segment[i].coordinates.x,2);
//            sum_y_2 += pow(segment[i].coordinates.y,2);
//
//            sum_x_y += segment[i].coordinates.x * segment[i].coordinates.y;
//            sum_x_z += segment[i].coordinates.x * segment[i].coordinates.z;
//            sum_y_z += segment[i].coordinates.y * segment[i].coordinates.z;
//    }
//
//    Eigen::MatrixXd X(3,3);
//    X(0,0) = sum_x_2; X(0,1) = sum_x_y; X(0,2) = sum_x;
//    X(1,0) = sum_x_y; X(1,1) = sum_y_2; X(1,2) = sum_y;
//    X(2,0) = sum_x;   X(2,1) = sum_y;   X(2,2) = size;
//
//    Eigen::VectorXd b(3);
//    b(0) = sum_x_z; b(1) = sum_y_z; b(2) = sum_z;
//
//    Eigen::VectorXd res(3);
//    res = X.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
//
////    cout << "The least-squares solution is:\n"
////         << X.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b) << endl;
//
//    plane_normal.x = res(0); plane_normal.y = res(1); plane_normal.z = res(2);
//
//    double normal_size = sqrt( pow(plane_normal.x,2) + pow(plane_normal.y,2) + pow(plane_normal.z,2) );
//    plane_normal /= normal_size;
//
//}

//void PointCloud::segmentation2(Mesh*& mesh, vector<int>*& tri_indices, const vector<PointFeatures> segment, vector<vector<PointFeatures>> &new_segments) {
//    VecArray vertices;
//    vector<vvr::Triangle> tris;
//    int index, index2;
//    float threshold{0.5f};
//    bool flag{true};
//    vec v1, v2, v3;
//
//    // check the triangles of segment
//    // if the depth of vertices of a triangle are less than t => store it with their vertices
//    // lastly add all vertices and triangles to new mesh
//    for (int i=0; i<segment.size(); i++) {
//
//        index = segment[i].vec_index;
//
//        // iterate through the neighbour triangles of vertex of mesh with the above index
//        for (int j=0; j<tri_indices[index].size(); j++) {
//            index2 = tri_indices[index][j];
//            vvr::Triangle tri = mesh->getTriangles()[index2];
//            v1 = tri.v1(); v2 = tri.v2(); v3 = tri.v3();
//
//            if (abs( v1.z - v2.z ) < threshold &&
//                abs( v1.z - v3.z ) < threshold &&
//                abs( v3.z - v2.z ) < threshold) {
//
//                // add vertex v1
//                if (!contains(vertices, v1))
//                    vertices.emplace_back(v1);
//                else
//                    flag = false;
//
//                // add vertex v2
//                if (!contains(vertices, v2))
//                    vertices.emplace_back(v2);
//                else
//                    flag = false;
//
//                // add vertex v3
//                if (!contains(vertices, v3))
//                    vertices.emplace_back(v3);
//                else
//                    flag = false;
//
//                if(flag)
//                    tris.emplace_back(tri);
//
//                flag = true;
//            }
//        }
//    }
//
//    cout << "there are " << vertices.size() << " vertices" << endl;
//    cout << "there are " << tris.size() << " triangles" << endl;
//
//    Mesh *new_mesh = new Mesh();
//    std::vector<vec> &modelVerts = new_mesh->getVertices();
//    std::vector<vvr::Triangle> &modelTris = new_mesh->getTriangles();
//
//    for (auto &d : vertices) modelVerts.push_back(d);
//    for (auto &d : tris) modelTris.push_back(d);
//
//    vector<int>* tri_indices_tmp = tri_indices;
//    int rings{1};
//    cout << "finding ringNeighbours ..." << endl;
//    vector<int> *ringNeighbours = getNRingNeighborhood(new_mesh, tri_indices_tmp, rings);
//
//    vector<PointFeatures> pointFeatures;
//
//    int size = new_mesh->getVertices().size();
//    for (int i=0; i<size; i++)
//        pointFeatures.push_back(PointFeatures(i, vertices[i], ringNeighbours[i]));
//
//    vector<PointFeatures> region;
//    vector<int> hash;
//    for (int i=0; i<pointFeatures.size(); i++) {
//        hash.emplace_back(0);
//    }
//    for (int i=0; i<pointFeatures.size(); i++) {
//        if ((pointFeatures[i].neighborVertices.size() != 0) && !(zeroPoint(pointFeatures[i].coordinates)) && (hash[i] == 0)) {
//            region.clear();
//            segmentation(pointFeatures, region, hash, i, true);
//            new_segments.emplace_back(region);
//        }
//    }
//}

// void PointCloud::segmentation2(Mesh*& mesh, vector<PointFeatures> pointFeatures, vector<VecArray> &segments) {
//    VecArray region, tree_data, src;
//    vector<float> dist;
//    vector<int> tmpInd, alreadySeen;
//    vector<PointFeatures> seeds;
//    vector<PointFeatures> nearest, oldPointFeatures;
//    vec neighbor;
//    int index, deleted{0};
//
//    oldPointFeatures = pointFeatures;
//
////    m_neighbours = 30;
//    m_angle_th = 20;
////    m_curv_th = 0.8;
//
//    // sort data by their curvature values in ascending order
////    std::sort(pointFeatures.begin(), pointFeatures.end(), PointFeaturesComparator());
//
//    // erase the 1th element
////    myvector.erase (myvector.begin()+0);
//
//    int tmp{0};
//    while(pointFeatures.size() >= m_neighbours) {
//        cout << "points are " << pointFeatures.size() << endl;
//
//        region.clear();
//        seeds.clear();
//        tmpInd.clear();
//
//        // get the point with minimum curvature
//        seeds.emplace_back(pointFeatures[0]);
//        region.emplace_back(pointFeatures[0].coordinates);
//
//        if(pointFeatures.size() == 1)
//            break;
//
//        cout << "the first seed point is = " << pointFeatures[0].coordinates << endl;
//        int index3;
//        for(int p=0; p<pointFeatures[0].neighborVertices.size(); p++) {
//            index3 = pointFeatures[0].neighborVertices[p];
//            cout << "neighbor = " << oldPointFeatures[index3].coordinates << " with index = " << index3 << endl;
//        }
//        cout << endl;
//
//
////        deleted.emplace_back(pointFeatures[0].coordinates);
//        pointFeatures.erase(pointFeatures.begin());
//        tmpInd.emplace_back(deleted); // store the index of deleted point
//        deleted++;
//
//        for (int i=0; i<seeds.size(); i++) {
//            cout << "seeds size = " << seeds.size() << endl;
//            cout << "seed = " << seeds[i].coordinates << endl;
//
//            if(seeds.size() > 10)
//                break;
//
//            nearest.clear();
////            tmpInd.clear();
//
//            cout << "neighbors are" << endl;
//
//            // find nearest neighbours of current seed point
//            for (int k=0; k<oldPointFeatures[i].neighborVertices.size(); k++) {
//                index = oldPointFeatures[i].neighborVertices[k];
//
//                std::vector<PointFeatures>::iterator it2;
//                it2 = nearest.begin();
//                it2 = nearest.insert ( it2 , oldPointFeatures[index] );
////                nearest.emplace_back(oldPointFeatures[index]);
//
//                std::vector<int>::iterator it;
//                // store the indices of nearest vertices of mesh
//                it = tmpInd.begin();
//                it = tmpInd.insert ( it , index );
//
////                tmpInd.emplace_back(index);
//
//                cout << oldPointFeatures[index].coordinates << " with index = " << index << endl;
//
////                deleted.emplace_back(pointFeatures[index].coordinates); /// check index
////                pointFeatures.erase(pointFeatures.begin() + index - 1);
//            }
//
//            // iterate through all neighbors
//            for(int j=0; j<nearest.size(); j++) {
//
//                double val =  abs(getAngle(seeds[i].normal, nearest[j].normal));
//
//                if (val < m_angle_th) {
//                    region.emplace_back(nearest[j].coordinates);
//
//                    cout << "point " << nearest[j].coordinates << " with index " << tmpInd[j] << endl;
//                    // find nearest neighbours of current seed point
//                    for (int k=0; k<nearest[j].neighborVertices.size(); k++) {
//                        index = nearest[j].neighborVertices[k];
//
//                        // check if any new neighbor has already seen iterating the vector tmpInd
//                        bool flag{true};
//                        for (int m=0; m<tmpInd.size(); m++) {
//                            if (tmpInd[m] == index) {
//                                flag = false;
//                                break;
//                            }
//                        }
//
//                        if (flag) {
//                            seeds.emplace_back(oldPointFeatures[index]);
//
//                            std::vector<int>::iterator it;
//                            it = tmpInd.begin();
//                            it = tmpInd.insert ( it , index );
//                            cout << "neighbor " << oldPointFeatures[index].coordinates << " with index = " << index << endl;
//
////                            deleted.emplace_back(pointFeatures[index].coordinates);
////                            pointFeatures.erase(pointFeatures.begin() + index - 1);
//                        }
//                    }
//                    if(pointFeatures.size() == 1)
//                        break;
//                }
//            }
//        }
//        segments.emplace_back(region);
//        if (tmp == 0)
//            break;
//    }
//
//    for (int i=0; i<segments.size(); i++) {
//        cout << "segment " << i << " with " << segments[i].size() << " points" << endl;
//    }
//    cout << "global segments are " << segments.size() << endl;
//}

//void PointCloud::segmentation2(const vector<PointFeatures> segment, vector<vector<PointFeatures>> &new_segments) {
void PointCloud::segmentation2(const vector<PointFeatures> pointFeatures, vector<PointFeatures> &region, vector<int> &hash, int start) {
    int index;
    double angle, threshold{100000.0};
    vector<int> indices;
    indices.emplace_back(start);

    for (int i=0; i<indices.size(); i++) {
        start = indices[i];

        for (int j=0; j<pointFeatures[start].neighborVertices.size(); j++) {
            index = pointFeatures[start].neighborVertices[j];
            angle = abs(getAngle(pointFeatures[start].normal, pointFeatures[index].normal));
//            cout << angle << endl;

            cout << "p1 = " << pointFeatures[start].coordinates << " and p2 = " << pointFeatures[index].coordinates << endl;


            if (isnan(angle))
                angle = 0.0;

            if (hash[index] == 0) {
                region.emplace_back(pointFeatures[index]);

                if (angle < threshold) {
                    indices.emplace_back(index);
                    hash[index] = 1;
                }
            }
        }
    }

//    vector<PointFeatures> region;
//    vector<int> indices, hash;
//    double threshold(1.0), angle;
//    int index, index2;
//
//    for (int i=0; i<segment.size(); i++) {
//        hash.emplace_back(0);
//    }
//
//    // iterate all points of segment
//    for (int i=0; i<segment.size(); i++) {
//
//        if (hash[i] == 0) {
//            region.clear();
//            indices.clear();
//            indices.emplace_back(i);
//
//            // iterate all candidate indices to belong to the same segment
//            for (int j=0; j<indices.size(); j++) {
//                index = indices[j];
//
//                // iterate neighbor vertices
//                for (int m=0; m<segment[index].neighborVertices.size(); m++) {
//                    index2 = segment[index].neighborVertices[m];
//
//                    angle = getAngle(segment[index].normal, segment[index2].normal);
//
//                    if (angle < threshold) {
//                        indices.emplace_back(index2);
//                        hash[index2] = 1;
//                        region.emplace_back(segment[index2]);
//                    }
//                }
//            }
//        }
//        new_segments.emplace_back(region);
//    }
}

void PointCloud::setCurvatureThreshold(float value) {
    m_curv_th = value;
}

void PointCloud::setSmoothnessThreshold(float value) {
    m_angle_th = value;
}

void PointCloud::setNumberOfNeighbours(int value) {
    m_neighbours = value;
}

//void PointCloud::normalsFiltering(Mesh*& mesh, vector<PointFeatures> &features, VecArray &normals) {
//    float pointDist, normalDist, luminanceDist, weight, totalWeight;
//    VecArray filtered_normals;
//    vec normal, vertex;
//    float a,b,c,d;
//    int index;
//
//    a=1.0f;
//    b=1.0f;
//    c=0.0f;
//    for (int i=0; i<features.size(); i++) {
//        totalWeight = 0.0f;
//        normal.x = normal.y = normal.z = 0.0f;
//
//        for (int j=0; j<features[i].neighborVertices.size(); j++) {
//            index = features[i].neighborVertices[j];
//            vertex = mesh->getVertices()[index];
//
//            pointDist = getNorm(features[i].coordinates,  vertex, 2);
//            normalDist = getNorm(features[i].normal, normals[index], 1);
//            luminanceDist = features[i].luminance - features[index].luminance;
//
////            weight = exp(a*pointDist) * exp(b*normalDist) * exp(c*luminanceDist/d)  ;
//            weight = exp(a*pointDist) * exp(b*normalDist) * exp(c*luminanceDist);
//
//            normal += weight*normals[index];
//
//            totalWeight += weight;
//        }
//        normal /= totalWeight;
//        filtered_normals.emplace_back(normal);
////        normals[i] = normal;
//    }
//    normals = filtered_normals;
//}

float PointCloud::getLuminance(Vec3b rgb) {
    return float(0.2126 * rgb[2] + 0.7152 * rgb[1] + 0.0722 * rgb[0]);
}

float PointCloud::getDistance(const vec p1, const vec p2) {
    return float(sqrt(pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2) + pow(p1.z - p2.z,2)));
}

float PointCloud::getNorm(const vec p1, const vec p2, int index) {
    // norm 1
    if (index == 1)
        return float(abs(p1.x - p2.x) + abs(p1.y - p2.y) + abs(p1.z - p2.z));
        // norm 2
    else
        return float(sqrt(pow(p1.x - p2.x,2) + pow(p1.y - p2.y,2) + pow(p1.z - p2.z,2)));
}

void PointCloud::computeNormal(const Mat &img, const Mat &depth_img, int x, int y, vec &normal) {
    /// 1 : smoothing using bilateral filter
    Mat filtered_img;
    bilateral(img, filtered_img);

    /// 2 : depth gradient computation
    Mat img_gray;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_32FC1;

    // Convert it to gray
    cvtColor(filtered_img, img_gray, CV_BGR2GRAY);

    // Generate grad_x and grad_y
    Mat der_z_x, der_z_y, sobel_img;

    // Gradient X
    Sobel(img_gray, der_z_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);

    // Gradient Y
    Sobel(img_gray, der_z_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
    addWeighted( der_z_x, 0.5, der_z_y, 0.5, 0, sobel_img );

    /// 3 : normal computation
    CameraConstants camera;

    auto xgrid = static_cast<short>(y + 1 + (camera.topLeft[0] - 1) - camera.center[0]);
    auto ygrid = static_cast<short>(x + 1 + (camera.topLeft[1] - 1) - camera.center[1]);

    float len{0.0};
    float c = camera.constant/camera.mm_per_m;
    float der_x_x, der_x_y, der_y_x, der_y_y;
    vec t_x, t_y;

    der_x_x = xgrid*der_z_x.at<float>(x,y)/c;
    der_x_y = (depth_img.at<unsigned short>(x,y) + xgrid*der_z_y.at<float>(x,y))/c;
    der_y_x = (depth_img.at<unsigned short>(x,y) + ygrid*der_z_x.at<float>(x,y))/c;
    der_y_y = (ygrid*der_z_y.at<float>(x,y))/c;

    // define the tangent vectors of the surface
    t_x.x = der_x_x; t_x.y = der_y_x; t_x.z = der_z_x.at<float>(x,y);
    t_y.x = der_x_y; t_y.y = der_y_y; t_y.z = der_z_y.at<float>(x,y);

    // a normal vector is obtained from the cross product of two tangent vectors
    normal = cross_product(t_x, t_y);
    normal /= normal.Length();
}

void PointCloud::computeNormals(const Mat &img, const Mat &depth_img, VecArray &normals) {//    // find min and max of point.x, point.y and point.z
    double minVal;
    double maxVal;
    Point minLoc;
    Point maxLoc;

    /// 1 : smoothing using bilateral filter
    Mat filtered_img;
//    bilateral(img, filtered_img);
//    bilateral(img, filtered_img);
//    bilateral(depth_img, filtered_img);

    Mat img_float;
//    filtered_img.convertTo(img_float, CV_32F, 1.0/255.0);
//    img.convertTo(img_float, CV_32F, 1.0/255.0);

    minMaxLoc( depth_img, &minVal, &maxVal, &minLoc, &maxLoc );
    depth_img.convertTo(img_float, CV_32F, 1.0/maxVal);

    CameraConstants camera;

    /// 2 : depth gradient computation
    int scale = 1;
    int delta = 0;
    int ddepth = CV_32F;
//    int ddepth = CV_16S;
//    int ddepth = CV_64F;
    Mat img_gray;

//     Convert it to gray
//    cvtColor(filtered_img, img_gray, CV_BGR2GRAY);
//    cvtColor(img_float, img_gray, CV_BGR2GRAY);

    // Generate grad_x and grad_y
    Mat der_z_x, der_z_y;

    // Gradient X
    Sobel(img_float, der_z_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
//    convertScaleAbs( der_z_x, der_z_x);

//    cout << der_z_x << endl;

    // Gradient Y
    Sobel(img_float, der_z_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
//    convertScaleAbs( der_z_y, der_z_y );

    /// 3 : normal estimation from depth gradients
    Mat xgrid, ygrid;
    xgrid.create(camera.image_height,camera.image_width, CV_16SC1);
    ygrid.create(camera.image_height,camera.image_width, CV_16SC1);

    for (int i=0; i<camera.image_height; i++) {
        for (int j=0; j<camera.image_width; j++) {
            xgrid.at<short>(i,j) = static_cast<short>(j + 1 + (camera.topLeft[0] - 1) - camera.center[0]);
            ygrid.at<short>(i,j) = static_cast<short>(i + 1 + (camera.topLeft[1] - 1) - camera.center[1]);
        }
    }

//    Mat xgrid_float, ygrid_float, depth_img_float;
//    minMaxLoc( xgrid, &minVal, &maxVal, &minLoc, &maxLoc );
//    xgrid.convertTo(xgrid_float, CV_32F, 1.0/maxVal);
//
//    minMaxLoc( ygrid, &minVal, &maxVal, &minLoc, &maxLoc );
//    ygrid.convertTo(ygrid_float, CV_32F, 1.0/maxVal);
//
//    minMaxLoc( depth_img, &minVal, &maxVal, &minLoc, &maxLoc );
//    depth_img.convertTo(depth_img_float, CV_32F, 1.0/maxVal);
//
//    Mat res_x, res_y, res_z, res, res2;
//    res.create(camera.image_height,camera.image_width, CV_32FC3);
//    res2.create(camera.image_height,camera.image_width, CV_16SC3);
//    res_y.create(camera.image_height,camera.image_width, CV_32F);
//    res_z.create(camera.image_height,camera.image_width, CV_32F);

    vector<float> points_x, points_y, points_z;

    int counter{0};
    float der_x_x, der_x_y, der_y_x, der_y_y;
    float c = camera.constant/camera.mm_per_m;
    vec my_vec, t_x, t_y;
    for (int i=0; i<camera.image_height; i++) {
        for (int j=0; j<camera.image_width; j++) {
            // compute the partial derivatives
            der_x_x = (xgrid.at<short>(i, j)*der_z_x.at<float>(i,j))/camera.constant/camera.mm_per_m;
            der_x_y = (depth_img.at<unsigned short>(i,j) + xgrid.at<short>(i,j)*der_z_y.at<float>(i,j))/camera.constant/camera.mm_per_m;

            der_y_x = (depth_img.at<unsigned short>(i,j) + ygrid.at<short>(i,j)*der_z_x.at<float>(i,j))/camera.constant/camera.mm_per_m;
            der_y_y = (ygrid.at<short>(i,j)*der_z_y.at<float>(i,j))/camera.constant/camera.mm_per_m;

            // define the tangent vectors of the surface
            t_x.x = der_x_x; t_x.y = der_y_x; t_x.z = der_z_x.at<float>(i,j);
            t_y.x = der_x_y; t_y.y = der_y_y; t_y.z = der_z_y.at<float>(i,j);

            // a normal vector is obtained from the cross product of two tangent vectors
            my_vec = cross_product(t_x, t_y);

//            if(my_vec.Length() != 0.0f) {
            // normalize the normal
//                my_vec /= my_vec.Length();
//            }

            // normalize the normal
            my_vec /= my_vec.Length();

            if (counter > 101800 && counter < 101900) {
                cout << "vec = " << my_vec << endl;
                cout << "len = " << my_vec.Length() << endl;
                cout << "der_z_x = " << der_z_x.at<float>(i,j) << endl;
                cout << "der_z_y = " << der_z_y.at<float>(i,j) << endl;
                cout << "der_x_x = " << der_x_x << endl;
                cout << "der_x_y = " << der_x_y << endl;
                cout << "der_y_x = " << der_y_x << endl;
                cout << "der_y_y = " << der_y_y << endl << endl;
            }

            // add the normal
            normals.emplace_back(my_vec);

//            res.at<Vec3f>(i,j).val[0] = my_vec.z;
//            res.at<Vec3f>(i,j).val[1] = my_vec.y;
//            res.at<Vec3f>(i,j).val[2] = my_vec.x;

            counter++;
        }
    }


//    Mat res_x_final, res_y_final, res_z_final;
//    minMaxLoc( res_x, &minVal, &maxVal, &minLoc, &maxLoc );
//    res_x.convertTo(res_x_final, CV_16SC1);
//
//    minMaxLoc( res_y, &minVal, &maxVal, &minLoc, &maxLoc );
//    res_y.convertTo(res_y_final, CV_16SC1);
//
//    minMaxLoc( res_z, &minVal, &maxVal, &minLoc, &maxLoc );
//    res_z.convertTo(res_z_final, CV_16SC1);
//
//    for (int i=0; i<camera.image_height; i++) {
//        for (int j=0; j<camera.image_width; j++) {
//            normals.emplace_back(vec(res_x_final.at<short>(i,j), res_y_final.at<short>(i,j), res_z_final.at<short>(i,j)));
//        }
//    }

//    cout << res_x_final << endl;
//    Mat final;
//    final.create(camera.image_height,camera.image_width, CV_16SC3);

//    minMaxLoc( res, &minVal, &maxVal, &minLoc, &maxLoc );
//    res.convertTo(final, CV_16SC3);

//    cout << res << endl;

    //    res.convertTo(final, CV_16SC3);
//    cout << final << endl;

//    std::string window_name9 = "result";
//    namedWindow( window_name9, CV_WINDOW_AUTOSIZE );
//    imshow(window_name9, final);
}

 //returns the inverse cosine of a number (argument) in degrees.
// The value which is returned by the acos() function always lies between –\pi to +\pi
double PointCloud::getAngle(const vec &p1, const vec &p2) {
    double numerator = dot_product(p1, p2);
    double denominator = p1.Length()*p2.Length();
    double result = acos(numerator/denominator);
    return result * 180.0 / M_PI;
}

 void PointCloud::bilateral(const Mat &img, Mat &new_img) {
//    std::string window_name = "Bilateral";
//    namedWindow( window_name, CV_WINDOW_AUTOSIZE );
    bilateralFilter(img, new_img, 15, 80, 80);
//    imshow( window_name, new_img );
//    waitKey(0);
}
*/