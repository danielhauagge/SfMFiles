#include "ply.hpp"

Ply::Ply()
{
}

size_t
Ply::addVertex(const Eigen::Vector3d& v, const Color& color)
{
    size_t idx = _vertices.size();
    _vertices.push_back(Vertex(v, color));
    return idx;
}

size_t
Ply::addVertex(const Eigen::Vector3d& v, const Eigen::Vector3d& n, const Color& color)
{
    size_t idx = _vertexWithNormal.size();
    _vertexWithNormal.push_back(Vertex(v, n, color));
    return idx;
}

void
Ply::addEdge(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Color& color)
{
    size_t idx1, idx2;
    idx1 = addVertex(v1, color);
    idx2 = addVertex(v2, color);

    _edges.push_back(Edge(idx1, idx2, color));
}

void
Ply::addCamera(const BDATA::Camera& cam, int imWidth, int imHeight, const Color& color)
{
    Eigen::Vector3d camCenter;
    cam.center(camCenter);
    size_t idxCenter = addVertex(camCenter);

    double imCorners[4][2] = {
        { 0,  0},
        { imWidth,  0},
        { imWidth, imHeight},
        { 0, imHeight}
    };

    size_t edgeIdxs[4];
    for (int i = 0; i < 4; i++) {
        Eigen::Vector2d im(imCorners[i][0], imCorners[i][1]);
        Eigen::Vector3d world;
        cam.im2world(im, world, imWidth, imHeight);
        edgeIdxs[i] = addVertex(world);
    }

    // Add edges between camera center and corners
    for (int i = 0; i < 4; i++) {
        _edges.push_back(Edge(idxCenter, edgeIdxs[i], color));
    }

    // Add edges for im frame
    for (int i = 0; i < 4; i++) {
        _edges.push_back(Edge(edgeIdxs[i], edgeIdxs[(i + 1) % 4], color));
    }

    // Mark up direction
    if(imWidth > 0 && imHeight > 0) {
        Eigen::Vector2d imTopIm(0, imHeight / 2);
        Eigen::Vector2d imCenterIm(0, 0);
        Eigen::Vector3d imTopW, imCenterW;
        cam.im2world(imTopIm, imTopW);
        cam.im2world(imCenterIm, imCenterW);

        addEdge(imTopW, imCenterW, color);
    }
}

void
Ply::addComment(const std::string& comment)
{
    _comments.push_back(comment);
}

void
Ply::writeToFile(const std::string& fname)
{
    std::ofstream plyF(fname.c_str());
    assert(plyF.good());

    plyF << "ply\n"
         << "format ascii 1.0\n";

    if(_comments.size()) {
        for(std::vector<std::string>::iterator cmt = _comments.begin(), cmtEnd = _comments.end(); cmt != cmtEnd; cmt++) {
            plyF << "comment ";
            char last;
            for(std::string::iterator c = cmt->begin(), cEnd = cmt->end(); c != cEnd; c++) {
                plyF << (*c);
                last = *c;
                if(*c == '\n' && (c + 1) != cEnd) plyF << "comment ";
                //if(*c == '\n') plyF << "comment ";
            }
            //plyF << "\n";
            if(last != '\n') plyF << "\n";
        }
    }

    if(_vertices.size()) {
        plyF << "element vertex " << _vertices.size() << "\n"
             << "property float x\n"
             << "property float y\n"
             << "property float z\n"
             << "property uchar red\n"
             << "property uchar green\n"
             << "property uchar blue\n";
    }

    if(_vertexWithNormal.size()) {
        plyF << "element vertex " << _vertexWithNormal.size() << "\n"
             << "property float x\n"
             << "property float y\n"
             << "property float z\n"
             << "property float nx\n"
             << "property float ny\n"
             << "property float nz\n"
             << "property uchar red\n"
             << "property uchar green\n"
             << "property uchar blue\n";
    }

    if(_edges.size() > 0) {
        plyF << "element edge " << _edges.size() << "\n"
             << "property int vertex1\n"
             << "property int vertex2\n"
             << "property uchar red\n"
             << "property uchar green\n"
             << "property uchar blue\n";
    }

    plyF << "end_header\n";

    if(_vertices.size()) {
        for (std::vector<Vertex>::iterator vertex = _vertices.begin(); vertex != _vertices.end(); vertex++) {
            plyF << vertex->pos[0] << " " << vertex->pos[1] << " " << vertex->pos[2] << " "
                 << (int)vertex->color.r << " " << (int)vertex->color.g << " " << (int)vertex->color.b << "\n";
        }
    }

    if(_vertexWithNormal.size()) {
        for (std::vector<Vertex>::iterator vertex = _vertexWithNormal.begin(); vertex != _vertexWithNormal.end(); vertex++) {
            plyF << vertex->pos[0] << " " << vertex->pos[1] << " " << vertex->pos[2] << " "
                 << vertex->normal[0] << " " << vertex->normal[1] << " " << vertex->normal[2] << " "
                 << (int)vertex->color.r << " " << (int)vertex->color.g << " " << (int)vertex->color.b << "\n";
        }
    }

    if(_edges.size() > 0) {
        for(std::vector<Edge>::iterator edge = _edges.begin(); edge != _edges.end(); edge++) {
            plyF << edge->idx1 << " " << edge->idx2 << " "
                 << (int)edge->color.r << " " << (int)edge->color.g << " " << (int)edge->color.b << "\n";
        }
    }
}
