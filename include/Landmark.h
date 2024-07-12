#pragma once

#include <vector>
#include <Eigen/Dense>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>

using namespace lanelet;

class LandmarkObject
{
/*
LandmarkObject Class is for conveniently storing positional and indexing information for regulatory elements.
In this work, traffic light and stop line are considered. 

> property: state_idxs may need fixing in the future for generic usage with multiple trips.

Jinhwan Jeon, 2024
*/
private:
    Eigen::Vector3d pos; // 3d position of landmark object
    Eigen::Vector3d delta_pos; // 3d delta-position of landmark object: used to mark edges
    std::vector<int> state_idxs;
public:
    LandmarkObject(Eigen::Vector3d Pos, Eigen::Vector3d DeltaPos);
    std::vector<Point3d> getLandmarkEdges(void); // Return the edges of the object for linestring generation (Regulatory Elements)
    Eigen::Vector3d getLandmarkPos(void); // Return the centroid pos
    void setStateIdxs(std::vector<int> idxs); // Set vehicle state_idxs
    std::vector<int> getStateIdxs(void); // Return the vehicle state_idxs where this landmark object is observed. --> May need to fix in the future for Multiple Trip Integration.
    void update(Eigen::Vector3d NewPos, Eigen::Vector3d NewDeltaPos);
};