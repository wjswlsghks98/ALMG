#pragma once

#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <limits>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <lanelet2_projection/LocalCartesian.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_modified/ArcLineString.h>
#include <lanelet2_modified/MultiArcLineString.h>
#include <lanelet2_modified/LaneletModified.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_routing/RoutingGraph.h>

#include "Common.h"
#include "MatlabEngine.hpp"

using namespace math;
using namespace misc;
using namespace event;
using namespace single_trip;

namespace py = pybind11;

class SingleTripAnalyzer
{
/*
Module for automatic map generation of single trip.
This module will be called from MapGenerator.py with pybind11 bindings

[Input data]:
idxs, matched_idxs (python list)
vehicleParams (python dictionary containing numpy arrays)
TrajEventIntvs (numpy array)

Jinhwan Jeon, 2024
*/
public:
    SingleTripAnalyzer(std::vector<int> idxs, std::vector<int> matched_idxs, 
                       std::map<std::string, py::array_t<double>> vehicleParams, 
                       py::array_t<int> TrajEventIntvs, 
                       py::array_t<double> Tf_Pos, py::array_t<double> Tf_DeltaPos,
                       py::array_t<double> St_Pos, py::array_t<double> St_DeltaPos);

    // Initializing Tf_vector and St_vector
    void addRegulatoryElementStateIdxs(int i, std::vector<int> idxs, std::string reg_type);
    
    // Visualize regelems for debugging.
    void visualize(void);

    // Main script of Single Trip Map Generation
    std::string run(void);

    // Function for checking if input variables are well transferred from python to c++
    void showParams(int idx);

    // Destructor
    ~SingleTripAnalyzer();

private:
    std::vector<int> idxs_;
    std::vector<int> matched_idxs_;
    std::map<std::string, Eigen::MatrixXd> vehicleParams_;
    Eigen::MatrixXi TrajEventIntvs_;
    std::vector<LandmarkObject> Tf_vector; // Tf_List will be generated using this data
    std::vector<LandmarkObject> St_vector; // St_List will be generated using this data
    std::vector<LabeledLanelet> LaneletList; // Labeled Lanelets will be used for map generation
    std::vector<std::vector<int>> IdxList; // Cluster of state idxs for clustering lane point covariance
    std::vector<RegulatoryElementPtr> RegelemList; // Pointers of regulatory elements will be integrated into maps.
    LaneletMap Map;

    std::string save(const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr, std::string arc_approx_flag);

    // Non-overlapping Interval Handling Module
    void NormalHandler(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                       std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                       int lb, int ub, int i, int label, bool call_from_overlap, int overlap_idx, const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr);
    
    // Nested from Normal Handler --> Lane Keeping Maneuver
    void LKM(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
             std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
             int lb, int ub, int i, int label);
    
    // Nested from Normal Handler --> Right/Left Turn
    void RLT(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
             std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
             int lb, int ub, int i, int label);
    
    // Nested from Normal Handler --> Roundabout
    void RA(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
            std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
            int lb, int ub, int i, int label, const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr);

    // Get Overlap Intervals (Before Actually Updating)
    void getOverlapIntervals(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                             std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                             int lb, int ub, int i, int label, std::vector<std::vector<int>>& intv, std::string& type,
                             const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr);

    void getLeftOverlapIntervals(LaneletMap &SampleMap, std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                                 std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                 int lb, int ub, int label, std::vector<std::vector<int>>& intv);

    void getRightOverlapIntervals(LaneletMap& SampleMap, std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                                  std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                  int lb, int ub, int label, std::vector<std::vector<int>>& intv);

    void getSameOverlapIntervals(LaneletMap& SampleMap, std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                                 std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                 int lb, int ub, int label, std::vector<std::vector<int>>& intv);
    
    // Update Overlapping Intervals
    void OverlapHandler(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List,
                        std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                        std::vector<std::vector<int>>& SameLaneOverlapIntv,
                        std::vector<std::vector<int>>& LeftLaneOverlapIntv,
                        std::vector<std::vector<int>>& RightLaneOverlapIntv,
                        std::vector<std::vector<int>> intv, std::string type, int i, bool end_flag,
                        const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr);

    void flushSameOverlap(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List,
                          std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                          std::vector<std::vector<int>> flush_intv, int i,
                          const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr);
    
    void flushLeftOverlap(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List,
                          std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                          std::vector<std::vector<int>> flush_intv, int i,
                          const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr);

    void flushRightOverlap(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List,
                           std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                           std::vector<std::vector<int>> flush_intv, int i,
                           const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr);

    // Generate Lanelet and Regulatory element and save.
    void generateLanelet(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List,
                         std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                         int lb, int ub, std::map<std::string, std::string> tags, int label);

    void addPoints2LineString(int LaneletListIdx, std::vector<Point3d> points, int lb, int ub, 
                              std::string dir, std::string options);

    
};