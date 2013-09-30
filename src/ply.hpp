#ifndef __PLY_HPP__
#define __PLY_HPP__

#include <SfMFiles/sfmfiles>

SFMFILES_NAMESPACE_BEGIN

class Ply
{
public:
    class Color
    {
    public:
        uint8_t r, g, b;
        Color(uint8_t r, uint8_t g, uint8_t b): r(r), g(g), b(b) {}
    };

    class Edge
    {
    public:
        size_t idx1, idx2;  // Vertex indexes
        Color color;

        Edge(size_t idx1, size_t idx2, const Color &color): idx1(idx1), idx2(idx2), color(color) {}
    };

    class Vertex
    {
    public:
        Eigen::Vector3d pos, normal;
        Color color;

        Vertex(const Eigen::Vector3d &pos, const Color &color): pos(pos), color(color) {}
        Vertex(const Eigen::Vector3d &pos, const Eigen::Vector3d &normal, const Color &color):
            pos(pos), normal(normal), color(color) {}
    };

    class Face
    {
    public:
        std::vector<size_t> idxs; // Vertex indexes
        Face(const std::vector<size_t> &idxs): idxs(idxs) {}
    };

    Ply();

    /// Adds a vertex with an optional color
    size_t addVertex(const Eigen::Vector3d &v, const Color &color = Color(0, 0, 0));

    /// Adds vertex with normal and an optional color
    size_t addVertex(const Eigen::Vector3d &v, const Eigen::Vector3d &n, const Color &color = Color(0, 0, 0));

    /// Add an edge to the data (not supported by MeshLab)
    void addEdge(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Color &color = Color(0, 0, 0));
    void addFace(std::vector<Eigen::Vector3d> &vertices, const Color &color = Color(0, 0, 0));

    void addCamera(const Bundler::Camera &cam, int imWidth, int imHeight, const Color &color = Color(0, 0, 0));

    void writeToFile(const std::string &fname);

    void addComment(const std::string &comment);

private:
    std::vector<Vertex> _vertices;
    std::vector <Vertex> _vertexWithNormal;
    std::vector<Edge> _edges;
    std::vector<Face> _faces;
    std::vector<std::string> _comments;
};

SFMFILES_NAMESPACE_END

#endif // __PLY_HPP__