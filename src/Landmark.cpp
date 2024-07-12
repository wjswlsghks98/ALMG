#include <Landmark.h>

LandmarkObject::LandmarkObject(Eigen::Vector3d Pos, Eigen::Vector3d DeltaPos): pos(Pos), delta_pos(DeltaPos) { }

std::vector<Point3d> LandmarkObject::getLandmarkEdges(void)
{
    // In Lanelet2, z component should indicate the lower part of the element.
    std::vector<Point3d> edges;
    Point3d p1(utils::getId(), pos.x() + delta_pos.x(), pos.y() + delta_pos.y(), pos.z() - delta_pos.z()); 
    Point3d p2(utils::getId(), pos.x() - delta_pos.x(), pos.y() - delta_pos.y(), pos.z() - delta_pos.z());
    
    edges.push_back(p1);
    edges.push_back(p2);

    return edges; // This can be inserted directly for defining linestrings.
}

Eigen::Vector3d LandmarkObject::getLandmarkPos(void) { return pos; }

void LandmarkObject::setStateIdxs(std::vector<int> idxs) { state_idxs = idxs; }

std::vector<int> LandmarkObject::getStateIdxs(void) { return state_idxs; }

void LandmarkObject::update(Eigen::Vector3d NewPos, Eigen::Vector3d NewDeltaPos)
{
    pos = NewPos; 
    delta_pos = NewDeltaPos;
}