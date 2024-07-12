#pragma once

#include <algorithm>
#include <deque>
#include <iterator>
#include <numeric>
#include <set>
#include <string>
#include <vector>
#include <optional>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h> // #include <lanelet2_core/primitives/LineString.h> is included.

#include <boost/math/distributions/chi_squared.hpp>

#include <Landmark.h>

using namespace lanelet;

class LabeledLanelet: public Lanelet
{
/*
Lanelets with additional labels indicating which driving event the current lanelet belongs to.

Jinhwan Jeon, 2024
*/
private:
    int label_;
    Lanelet llt;
public:
    LabeledLanelet() : Lanelet() { }
    LabeledLanelet(Id id, LineString3d leftBound, LineString3d rightBound, int label, const AttributeMap& attributes = AttributeMap(),
                   RegulatoryElementPtrs regulatoryElements = RegulatoryElementPtrs())
            : Lanelet(id, leftBound, rightBound, attributes, regulatoryElements), 
              llt(Lanelet(id, leftBound, rightBound, attributes, regulatoryElements)), label_(label) {}
    
    int getLabel(void) { return label_; }
    Lanelet getLanelet(void) { return llt;}
};

namespace event
{
    enum ManeuverType{ LKM, RT, LT, RLC, LLC, RA };
}

namespace math
{
    // Returns the S03 --> R3 logarithm map
    Eigen::Vector3d Log_map(Eigen::Matrix3d R)
    {   
        double tr_3 = R.trace();
        double mag;

        if(tr_3 < 1e-6)
        {
            double psi = acos(0.5 * (tr_3 - 1));
            mag = psi / (2 * sin(psi));
        }
        else
            mag = 0.5 - 1/12 * tr_3 + 1/60 * tr_3 * tr_3;
        

        Eigen::Vector3d vec(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
        return mag * vec; 
    }

    bool chi_squared_test(Eigen::Vector3d x_t, Eigen::Vector3d x, Eigen::Matrix3d cov, double degreesOfFreedom, double confidenceLevel)
    {
        Eigen::Vector3d residual_vec = x_t - x;
        
        // std::cout << "Covariance matrix: " << std::endl;
        // std::cout << cov << std::endl;
        // std::cout << std::endl;
        // std::cout << "Actual Residual: " << std::endl;
        // std::cout << residual_vec << std::endl;

        double chi_squared_dist = residual_vec.transpose() * cov.inverse() * residual_vec;
        double chi_squared_thres = boost::math::quantile(boost::math::chi_squared(degreesOfFreedom), confidenceLevel);
        // std::cout << "Chisq Dist: " << chi_squared_dist << ", Chisq Thres: " << chi_squared_thres << std::endl;

        if (chi_squared_thres < chi_squared_dist)
            return false;
        else
            return true;
    }
}

namespace misc
{   
    // Check if point is matched to linestring using reliability information
    bool checkPoint2LineStringMatch(LineString3d ls, BasicPoint3d p, Eigen::Matrix3d cov, double thres)
    {
        // Project point to the target linestring
        BasicPoint3d prj_p = geometry::project(ls, p);

        // 3 degrees of Freedom, 99% confidence level is used.
        bool chi_squared_test_bool = math::chi_squared_test(prj_p, p, cov, 3.0, 0.99);

        if (chi_squared_test_bool)
            return true; // If chi_squared_test is passed, return true right away.
        else 
        {
            // Event if chi_squared_test is failed, if euclidean norm of residual is smaller than threshold, return true.
            if ((prj_p - p).norm() < thres) 
                return true;
            else
                return false;
        }        
    }

    // Generate LineStrings from given vector of points and attributes
    LineString3d getLineString(std::vector<Point3d> points, std::string type_string, std::string subtype_string)
    {
        LineString3d ls(utils::getId(), points);
        ls.attributes()["type"] = type_string;
        ls.attributes()["subtype"] = subtype_string;

        return ls;
    }
    
    // Generate basic lanelet from given values.
    LabeledLanelet getLanelet(LineString3d LeftLineString,LineString3d RightLineString,
                              std::string subtype_string, std::string location_string,
                              std::string participants_string, std::string one_way_string,
                              std::string speed_limit_string, int label)
    {
        LabeledLanelet llt(utils::getId(), LeftLineString, RightLineString, label);
        llt.attributes()["subtype"] = subtype_string;
        llt.attributes()["location"] = location_string;
        llt.attributes()["participants"] = participants_string;
        llt.attributes()["one_way"] = one_way_string;
        llt.attributes()["speed_limit"] = speed_limit_string;
        return llt;
    }

    // Generate LaneletMap from given lanelets and regulatory elment pointers
    void getLaneletMap(LaneletMap& Map,std::vector<LabeledLanelet> LaneletList, std::vector<RegulatoryElementPtr> RegelemList)
    {
        for (LabeledLanelet llt : LaneletList)
        {
            Lanelet llt_ = llt.getLanelet();
            Map.add(llt_);
        }

        for (RegulatoryElementPtr reg_ptr : RegelemList)
            Map.add(reg_ptr);
    }

    // Get the label of the lanelet by seaarch through ids
    int getLaneletLabel(Lanelet llt, std::vector<LabeledLanelet> LaneletList)
    {
        for (LabeledLanelet labeled_llt : LaneletList)
        {
            if (labeled_llt.id() == llt.id())
                return labeled_llt.getLabel();
        }
        return -1;
    }

    // Get the relative index of lanelet inside the LaneletList, given Id
    int getLaneletIdx(std::vector<LabeledLanelet> LaneletList, Id id)
    {
        for (int i=0;i<LaneletList.size();i++)
        {
            if (LaneletList[i].id() == id)
                return i;
        }
        return -1;
    }

    // Get the LaneletList Idx from overlapping point id and left/right information
    std::vector<Id> getLaneletIdx2(std::vector<LabeledLanelet> LaneletList, Id id, const std::string& dir, const std::string& fb)
    {
        std::vector<Id> matched_ids;
        Id p_id1, p_id2;

        for (int i=0;i<LaneletList.size();i++)
        {
            if (fb == "front")
            {
                p_id1 = LaneletList[i].leftBound().front().id();
                p_id2 = LaneletList[i].rightBound().front().id();
            }
            else if (fb == "back")
            {
                p_id1 = LaneletList[i].leftBound().back().id();
                p_id2 = LaneletList[i].rightBound().back().id();
            }
                
            if (p_id1 == id)
                matched_ids.push_back(LaneletList[i].leftBound().id());
            else if (p_id2 == id)    
                matched_ids.push_back(LaneletList[i].rightBound().id());
        }
        if (matched_ids.empty())
            matched_ids.push_back(-1);

        // for (int i=0;i<LaneletList.size();i++)
        // {
        //     LineString3d ls; Id p_id;
        //     if (dir == "left")
        //         ls = LaneletList[i].leftBound();
        //     else
        //         ls = LaneletList[i].rightBound();
            
        //     if (fb == "front")
        //         p_id = ls.front().id();
        //     else if (fb == "back")
        //         p_id = ls.back().id();
            
        //     // std::cout << "Queried Id: " << id << ", Comparison Id: " << p_id << std::endl;

        //     if (p_id == id)
        //     {
        //         if (dir == "left")
        //             matched_ids.push_back(LaneletList[i].leftBound().id());
        //         else 
        //             matched_ids.push_back(LaneletList[i].rightBound().id());
        //     }
        // }
        // if (matched_ids.empty())
        //     matched_ids.push_back(-1);
        
        // std::cout << "Matched Ids: ";
        // for (Id id : matched_ids)
        //     std::cout << id << " ";
        // std::cout << std::endl;

        return matched_ids;
    }

    // Get LineString from lanelet using LineString Id
    int getLineStringfromId(const std::vector<LabeledLanelet>& LaneletList,const Id& ls_id)
    {
        // Need to fix this part. 
        // Need to keep all the Id same! 
        // std::vector<Point3d> pts;
        // bool found_flag = false;
        for (int i=0;i<LaneletList.size();i++)
        {
            if (LaneletList[i].leftBound().id() == ls_id)
            {
                // ConstLineString3d cls = LaneletList[i].leftBound();
                // for (int j=0;j<cls.size();j++)
                // {
                //     Point3d pt(utils::getId(),cls[j].x(),cls[j].y(),cls[j].z());
                //     pts.push_back(pt);
                // }
                return i;
                           
            }    
            else if (LaneletList[i].rightBound().id() == ls_id)
            {
                // ConstLineString3d cls = LaneletList[i].rightBound();
                // for (int j=0;j<cls.size();j++)
                // {
                //     Point3d pt(utils::getId(),cls[j].x(),cls[j].y(),cls[j].z());
                //     pts.push_back(pt);
                // }
                return i;
            }
        }
        // LineString3d ls = misc::getLineString(pts,"virtual","");
        // return ls;

        // if (!found_flag)
        //     std::cout << "Search Failed" << std::endl;
        std::cout << "Search Failed" << std::endl;
        return -1;
    }

    // Get the overlap type for each state index 'idx'
    int getOverlapType(std::vector<std::pair<double, Lanelet>> nearest, std::vector<Point3d> LeftLanePointsObj, Eigen::MatrixXd LeftLanePointsCov,
                       std::vector<Point3d> RightLanePointsObj, Eigen::MatrixXd RightLanePointsCov, int idx, double thres)
    {
        LineString3d ls_l = (nearest[0].second).leftBound(); Point3d p_l = LeftLanePointsObj[idx];
        LineString3d ls_r = (nearest[0].second).rightBound(); Point3d p_r = RightLanePointsObj[idx];
        
        double dLL = geometry::distance(ls_l,p_l); double dLR = geometry::distance(ls_l,p_r);
        double dRL = geometry::distance(ls_r,p_l); double dRR = geometry::distance(ls_r,p_r);
        Eigen::Matrix3d covL = Eigen::Map<Eigen::Matrix<double, 3, 3>>((LeftLanePointsCov.col(idx)).data());
        Eigen::Matrix3d covR = Eigen::Map<Eigen::Matrix<double, 3, 3>>((RightLanePointsCov.col(idx)).data());

        try
        {
            if (dLL < dLR && dRR < dRL)
            {
                // Case 0: Same Lane
                // Double check if the target point is actually correctly matched with the nearest linestring.
                bool bool1 = misc::checkPoint2LineStringMatch(ls_l, p_l.basicPoint(), covL, thres);
                bool bool2 = misc::checkPoint2LineStringMatch(ls_r, p_r.basicPoint(), covR, thres);

                if (bool1 && bool2)
                    return 0;
                else
                    throw std::runtime_error("Distance threshold is satisfied for same lane match, but probability based matching has invalid results");
                    
            }
            else if (dLL > dLR && dRR < dRL)
            {
                // Case 1 / 3 : State is located left of pre-computed lanelet(nearest[0].second)
                // Check if the target point is actually correctly matched with the nearest linestring.
                if (misc::checkPoint2LineStringMatch(ls_l, p_r.basicPoint(), covR, thres))
                    return 1;
                else if (misc::checkPoint2LineStringMatch(nearest[1].second.rightBound(), p_l.basicPoint(), covL, thres))
                {
                    // Case 3: State is located in the middle of lanelets nearest[0].second and nearest[1].second.
                    // But in this case, vehicle position is located closer to nearest[0].second.
                    return 3;
                }
                else
                    return -1;
            }
            else if (dLL < dLR && dRR > dRL)
            {
                // Case 2 / 4 : State is located right of pre-computed lanelet(nearest[0].second)
                // Check if the target point is actually correctly matched with the nearest linestring.
                if (misc::checkPoint2LineStringMatch(ls_r, p_l.basicPoint(), covL, thres))
                    return 2;
                else if (misc::checkPoint2LineStringMatch(nearest[1].second.leftBound(), p_r.basicPoint(), covR, thres))
                {
                    // Case 4: State is located in the middle of lanelets nearest[0].second and nearest[1].second.
                    // But in this case, vehicle position is located closer to nearest[0].second.
                    return 4;
                }
                else
                    return -2;
            }
            else
            {
                std::cout << "====================================================" << std::endl;
                std::cout << "Distance from" << std::endl;
                std::cout << "Left LineString to Left Lane Point: " << dLL << std::endl;
                std::cout << "Left LineString to Right Lane Point: " << dLR << std::endl;
                std::cout << "Right LineString to Left Lane Point: " << dRL << std::endl;
                std::cout << "Right LineString to Right Lane Point: " << dRR << std::endl;
                throw std::runtime_error("Above type of distance configuration is impossible. Please check for other implementation errors.");
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        return -10;
    } 

    // Get overlapping lanelet match type intervals given Idxs (2d vector)
    std::vector<std::vector<int>> getMatchIntv(std::vector<std::vector<int>> Idxs, int label)
    {
        std::vector<std::vector<int>> intv;
        intv.push_back({Idxs[0][0],0,Idxs[0][1],label});

        for (int i=0;i<Idxs.size()-1;i++)
        {
            if(Idxs[i][1] != Idxs[i+1][1])
            {
                intv[intv.size()-1][1] = Idxs[i][0];
                intv.push_back({Idxs[i+1][0],0,Idxs[i+1][1],label});
            }
        }
        intv[intv.size()-1][1] = Idxs[Idxs.size()-1][0];
        return intv;
    }

    // Determine whether to stack or flush overlap intervals
    std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> intervalOverlapHandler(std::vector<std::vector<int>> OverlapIntv, std::vector<std::vector<int>> intv, bool end_flag)
    {
        std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> output;
        bool flush_flag = false;

        if (end_flag)
            flush_flag = true;

        if (OverlapIntv.empty())
        {
            for (std::vector<int> intv_row : intv)
                OverlapIntv.push_back(intv_row);
            
            intv = std::vector<std::vector<int>>();
        }
        else
        {
            if(intv[0][0] == OverlapIntv[OverlapIntv.size()-1][1] + 1)
            {
                for (std::vector<int> intv_row : intv)
                    OverlapIntv.push_back(intv_row);

                intv = std::vector<std::vector<int>>();
            }
            else
                flush_flag = true;
        }

        if (flush_flag)
        {
            output.first = OverlapIntv;
            output.second = intv;
        }
        else
        {   
            output.first = std::vector<std::vector<int>>();
            output.second = OverlapIntv;
        }
        return output;
    }

    // Initialize vectors of LineString3d for Regulatory Elements
    void getRegelemListInit(std::vector<LineString3d>& obj_ls_list, std::vector<LandmarkObject> obj_list, std::string reg_type)
    {
        std::string type_string;
        std::string subtype_string;

        if (reg_type == "TrafficLights")
        {
            type_string = "traffic_light";
            subtype_string = "red_yellow_green";
        }
        else if (reg_type == "StopLines")
        {
            type_string = "stop_line";
            subtype_string = "";
        }

        for (int i=0;i<obj_list.size();i++)
        {
            LandmarkObject obj = obj_list[i];
            std::vector<Point3d> edges = obj.getLandmarkEdges();
            obj_ls_list.push_back(misc::getLineString(edges,type_string,subtype_string));
        }
    }

    // Append the split indices that are close to the object.
    void addSplitIdxs(std::set<int>& split_idxs, Eigen::MatrixXd samplePos, LandmarkObject Obj, int lb)
    {
        // Find the closest state index (relative). 
        Eigen::Vector3d ObjPos = Obj.getLandmarkPos();

        Eigen::VectorXd norms = (samplePos - ObjPos.replicate(1,samplePos.cols())).colwise().norm();
        std::vector<double> norms_db(norms.data(), norms.data() + norms.size());

        int candIdx = std::distance(norms_db.begin(), std::min_element(norms_db.begin(), norms_db.end()));

        // Add the candidate index to split idx only if it is in the valid range. 
        // 
        if (candIdx > 1 && candIdx < norms_db.size()-2)
            split_idxs.insert(lb + candIdx);
    }

    // Process overlap flush intervals (currently for same lane only...)
    std::vector<std::vector<int>> processFlushIntv(std::vector<LabeledLanelet> LaneletList, std::vector<std::vector<int>> flush_intv)
    {
        /*
        Return format: [[lb, ub, id], ...] where
        id can be -1 or actual lanelet ID
        
        [Basic Rules]
        
        If current row's id is negative (not -1), search for row that has -1 after the current row.
        1) If there exists -1, cluster all the rows from current to the row that has -1 as the id, and set id as -1 : No match ** Label should be set as the one equal to the label for original id -1
        2) If there exists any match after the current row, all the rows in between can be considered as valid match (* match should appear before -1)
        3) Other than 1) and 2), show warning and change current id to positive.
        
        [Examples(Simplified version, only containing ids)]
        1. 
        [-1001, 1002, -1003] --> [1001, 1002, 1003] with warning at 1003. 
        * -1001 is changed without warning because there is valid match at 1002. Since -1003 is a isolated negative number, show warning.
        
        2. 
        [-1, 1001, -1002, -1] --> [-1, 1001, -1] without any warning. 
        * -1002 is clustered with -1.         
        */
        
        // For easier handling, copy flush interval to queue
        std::deque<std::vector<int>> flush_q;
        for (const auto& v: flush_intv)
            flush_q.push_back(v);
        
        std::deque<std::vector<int>> intv;
        std::vector<std::vector<int>> compressed_intv;
        int cnt = 0;

        try
        {
            while (true)
            {
                std::vector<int> popped_intv;
                if (flush_q[cnt][2] > 0) // matched id is positive (valid match!)
                {
                    for (int i=0;i<cnt;i++) // pop all the intervals that have negative id before the current interval with positive matched id
                    {
                        popped_intv = flush_q.front();
                        flush_q.pop_front();

                        if(popped_intv[2] == -1 || popped_intv[2] > 0)
                            throw std::runtime_error("Overlap matches with positive or -1 id should not be above!");
                        else
                        {
                            popped_intv[2] *= -1;
                            popped_intv[3] = LaneletList[misc::getLaneletIdx(LaneletList,popped_intv[2])].getLabel(); // Set label equal to the matched lanelet.
                            intv.push_back(popped_intv);
                        }
                    }
                    // Pop the interval with positive matched id
                    popped_intv = flush_q.front();
                    flush_q.pop_front();
                    intv.push_back(popped_intv);

                    if (flush_q.empty())
                        break;
                }

                else if (flush_q[cnt][2] < -1)
                {
                    if (flush_q.size() == 1) // Isolated negative ID
                    {
                        popped_intv = flush_q.front();
                        flush_q.pop_front();
                        popped_intv[2] *= -1;
                        popped_intv[3] = LaneletList[misc::getLaneletIdx(LaneletList,popped_intv[2])].getLabel();
                        std::cerr << "Warning: Isolated overlap occurred near Lanelet ID " << popped_intv[2] << ". Matching overlap to corresponding lanelet." << std::endl;
                        intv.push_back(popped_intv);
                        break;
                    }
                    else
                        cnt = cnt + 1; // Move downwards
                }

                else if (flush_q[cnt][2] == -1)
                {
                    int minus_one_label = flush_q[cnt][3];

                    for (int i=0;i<cnt;i++) // // pop all the intervals that have negative id before the current interval with id -1
                    {
                        popped_intv = flush_q.front();
                        flush_q.pop_front();
                        if(popped_intv[2] == -1 || popped_intv[2] > 0)
                            throw std::runtime_error("Overlap matches with positive or -1 id should not be above!");
                        else
                        {
                            popped_intv[2] = -1; // Set match to be invalid
                            popped_intv[3] = minus_one_label; // Set label equal to the matched lanelet.
                            intv.push_back(popped_intv);
                        }
                    }
                    // Pop the interval with -1 id
                    popped_intv = flush_q.front();
                    flush_q.pop_front();
                    intv.push_back(popped_intv);

                    if (flush_q.empty())
                        break;
                }
            }

            std::vector<int> popped_intv;
            int n = intv.size();
            // If there are multiple rows of same id, concatenate them.

            for (int i=0;i<n;i++)
            {
                popped_intv = intv.front();
                intv.pop_front();

                if (compressed_intv.empty())
                    compressed_intv.push_back(popped_intv);
                else
                {
                    if (compressed_intv[compressed_intv.size()-1][2] == popped_intv[2]) // Equal ID (Need to concatenate)
                    {
                        if (compressed_intv[compressed_intv.size()-1][1] + 1 != popped_intv[0])
                            throw std::runtime_error("Overlap matches have same id but are not continuous.");
                        else 
                            compressed_intv[compressed_intv.size()-1][1] = popped_intv[1]; // Update the upper bound of current interval using the popped_intv's upper bound value.
                    }
                    else // Different ID (Need to stack)
                        compressed_intv.push_back(popped_intv);
                }
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        return compressed_intv;
    }

    // Cluster LineString Ids for arc spline approximation
    void clusterLineStrings(std::vector<std::vector<Id>>& LineStringCluster, ConstLanelets reachableSetNext, ConstLanelets reachableSetPrev, LabeledLanelet labeled_llt)
    {
        bool flag = true;

        for (int i=0;i<LineStringCluster.size();i++)
        {
            std::vector<Id> cluster = LineStringCluster[i];
            auto it = std::find(cluster.begin(), cluster.end(), labeled_llt.leftBound().id());
            if (it != cluster.end())
            {
                flag = false; break;
            }
            
        }

        if (flag)
        {
            std::vector<Id> leftIds, rightIds;
            for (int i=0;i<reachableSetPrev.size();i++)
            {
                if (reachableSetPrev[i].id() != labeled_llt.id())
                {
                    leftIds.push_back(reachableSetPrev[i].leftBound().id());
                    rightIds.push_back(reachableSetPrev[i].rightBound().id());
                }
            }

            for (int i=0;i<reachableSetNext.size();i++)
            {
                leftIds.push_back(reachableSetNext[i].leftBound().id());
                rightIds.push_back(reachableSetNext[i].rightBound().id());
            }

            LineStringCluster.push_back(leftIds);
            LineStringCluster.push_back(rightIds);
            // Clustered linestring data will be in
            // left-right-left-right format.
        }
    } 

}

/*
misc methods that are currently valid for single trip 

> LandmarkObject.state_idxs will not be used for multiple trip integration.
*/ 

namespace single_trip
{
    // Check if two objects are matched, based on state_idxs
    bool isMatchedObjects(LandmarkObject Obj1, LandmarkObject Obj2)
    {
        std::vector<int> idxs1 = Obj1.getStateIdxs();
        std::vector<int> idxs2 = Obj2.getStateIdxs();

        std::set<int> set1(idxs1.begin(),idxs1.end());
        std::set<int> set2(idxs2.begin(),idxs2.end());
        std::set<int> intersection;
        std::set_intersection(set1.begin(), set1.end(), set2.begin(), set2.end(),
                              std::inserter(intersection, intersection.begin()));

        // Return true if there is any intersection in the state_idxs
        if (intersection.empty())
            return false;
        else
            return true;
    }

    // Check if object state idx intersects with given sample_idxs_set
    bool isIntersect(LandmarkObject Obj, std::set<int> sample_idxs_set)
    {
        std::vector<int> state_idxs = Obj.getStateIdxs();
        std::set<int> state_idxs_set(state_idxs.begin(),state_idxs.end());
        std::set<int> intersection;
        std::set_intersection(sample_idxs_set.begin(), sample_idxs_set.end(), state_idxs_set.begin(), state_idxs_set.end(),
                                  std::inserter(intersection, intersection.begin()));
        if (!intersection.empty())
            return true;
        else
            return false;
    }

    // Return the set intersection of state idxs [lb,ub] and object state_idxs 
    std::vector<int> getIntersectingObjectIdxs(std::vector<LandmarkObject> Objs, std::set<int> sample_idxs_set)
    {
        std::vector<int> ObjIdxs;

        for (int i=0;i<Objs.size();i++)
        {
            if (single_trip::isIntersect(Objs[i],sample_idxs_set))
                ObjIdxs.push_back(i);
        }

        return ObjIdxs;
    }

    // Searches for appropriate index for point insertion in given linestring.
    int findPointInsertIdx(LineString3d ls, Point3d p, std::map<std::string,Eigen::MatrixXd> vehicleParams_, int state_idx)
    {
        Eigen::Matrix3d Ri = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>((vehicleParams_["Orientation"].col(state_idx)).data());
        std::vector<double> relative_xs;
        std::vector<int> idxs;

        for (int i=0;i<ls.size();i++)
        {
            Eigen::Vector3d diff = ls[i].basicPoint() - p.basicPoint();
            if (diff.norm() < 5)
            {
                Eigen::Vector3d x_conv = {1, 0, 0};
                double rel_x = x_conv.transpose() * Ri.transpose() * diff;
                
                if(relative_xs.size() > 0)
                {
                    if (relative_xs[relative_xs.size()-1] < 0 && rel_x > 0)
                        return idxs[idxs.size()-1];
                    else
                    {
                        relative_xs.push_back(rel_x);
                        idxs.push_back(i);
                    } 
                }
                else
                {
                    relative_xs.push_back(rel_x);
                    idxs.push_back(i);
                }
            }
        }

        // If there is no appropriate match, return the index with minimum absolute value.
        double minAbsValue = abs(relative_xs[0]);
        int minAbsIdx = 0;
        for (int i=1;i<relative_xs.size();i++)
        {
            if (abs(relative_xs[i]) < minAbsValue)
            {
                minAbsValue = relative_xs[i];
                minAbsIdx = i;
            }
        }

        if (idxs[minAbsIdx] == 0)
            return 0;
        else if (idxs[minAbsIdx] == ls.size()-1)
            return ls.size()-1;
        
        std::cout << "Appropriate insert location search failed..." << std::endl;
        return idxs[minAbsIdx];
    }
    
}