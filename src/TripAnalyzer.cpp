#include <TripAnalyzer.h>

/*
Public Methods for SingleTrip Analyzer

*/

SingleTripAnalyzer::SingleTripAnalyzer(std::vector<int> idxs, std::vector<int> matched_idxs, 
                                       std::map<std::string, py::array_t<double>> vehicleParams, 
                                       py::array_t<int> TrajEventIntvs,
                                       py::array_t<double> Tf_Pos, py::array_t<double> Tf_DeltaPos,
                                       py::array_t<double> St_Pos, py::array_t<double> St_DeltaPos): idxs_(std::move(idxs)), matched_idxs_(std::move(matched_idxs))
{
    // initialize vehicleParams_
    for (auto& pair: vehicleParams)
    {
        std::string key = pair.first;
        py::array_t<double> arr = pair.second;

        py::buffer_info buf1 = arr.request();
        double *ptr1 = (double *) buf1.ptr;
        if (key == "LeftCov" || key == "RightCov")
        {
            Eigen::Map<Eigen::MatrixXd> matrix1(ptr1, buf1.shape[0], buf1.shape[1]);
            vehicleParams_[key] = matrix1;
        }
        else
        {
            Eigen::Map<Eigen::MatrixXd> matrix1(ptr1, buf1.shape[1], buf1.shape[0]);
            vehicleParams_[key] = matrix1.transpose();
        }
    }

    // initialize TrajEventIntvs_

    py::buffer_info buf2 = TrajEventIntvs.request();
    int *ptr2 = (int *) buf2.ptr;
    Eigen::Map<Eigen::MatrixXi> matrix2(ptr2, buf2.shape[0], buf2.shape[1]);
    TrajEventIntvs_ = matrix2;

    // initialize Tf vectors
    py::buffer_info buf3 = Tf_Pos.request();
    double *ptr3 = (double *) buf3.ptr;
    // Eigen::Map<Eigen::MatrixXd> matrix3(ptr3, buf3.shape[1], buf3.shape[0]);
    // Eigen::MatrixXd TfPos = matrix3.transpose();
    Eigen::Map<Eigen::MatrixXd> matrix3(ptr3, buf3.shape[0], buf3.shape[1]);
    Eigen::MatrixXd TfPos = matrix3;

    py::buffer_info buf4 = Tf_DeltaPos.request();
    double *ptr4 = (double *) buf4.ptr;
    // Eigen::Map<Eigen::MatrixXd> matrix4(ptr4, buf4.shape[1], buf4.shape[0]);
    // Eigen::MatrixXd TfDeltaPos = matrix4.transpose();
    Eigen::Map<Eigen::MatrixXd> matrix4(ptr4, buf4.shape[0], buf4.shape[1]);
    Eigen::MatrixXd TfDeltaPos = matrix4;

    for (int i=0;i<TfPos.cols();i++)
        Tf_vector.push_back(LandmarkObject(TfPos.col(i),TfDeltaPos.col(i)));
    
    // initialize St vectors

    py::buffer_info buf5 = St_Pos.request();
    double *ptr5 = (double *) buf5.ptr;
    // Eigen::Map<Eigen::MatrixXd> matrix5(ptr5, buf5.shape[1], buf5.shape[0]);
    // Eigen::MatrixXd StPos = matrix5.transpose();
    Eigen::Map<Eigen::MatrixXd> matrix5(ptr5, buf5.shape[0], buf5.shape[1]);
    Eigen::MatrixXd StPos = matrix5;

    py::buffer_info buf6 = St_DeltaPos.request();
    double *ptr6 = (double *) buf6.ptr;
    // Eigen::Map<Eigen::MatrixXd> matrix6(ptr6, buf6.shape[1], buf6.shape[0]);
    // Eigen::MatrixXd StDeltaPos = matrix6.transpose();
    Eigen::Map<Eigen::MatrixXd> matrix6(ptr6, buf6.shape[0], buf6.shape[1]);
    Eigen::MatrixXd StDeltaPos = matrix6;

    for (int i=0;i<StPos.cols();i++)
        St_vector.push_back(LandmarkObject(StPos.col(i),StDeltaPos.col(i)));
}

void SingleTripAnalyzer::addRegulatoryElementStateIdxs(int i, std::vector<int> idxs, std::string reg_type)
{
    // Add state_idxs to traffic lights 
    if (reg_type == "TrafficLights")
        Tf_vector[i].setStateIdxs(idxs);
    
    // Add state_idxs to stop lines
    else if(reg_type == "StopLines")
        St_vector[i].setStateIdxs(idxs);    
}

std::string SingleTripAnalyzer::run(void)
{
    // std::cout << "Single Trip Map Generation in [Lanelet2] format" << std::endl;
    std::cout << "====================================================================================================================" << std::endl;

    // Generate Point objects and regulatory elements before generating lanelets
    Eigen::MatrixXd LeftLanePoints = vehicleParams_["Left"];
    Eigen::MatrixXd RightLanePoints = vehicleParams_["Right"];

    try // Check matrix size
    {
        if (LeftLanePoints.rows() != RightLanePoints.rows() || LeftLanePoints.cols() != RightLanePoints.cols())
            throw std::runtime_error("Sizes of Left and Right Lane Point matrices do not match. Please check.");
    }
    catch(const std::exception& e)
    {
        std::cerr << "Program ended with exception: " << e.what() << std::endl;
    }

    // Generating Point Objects
    std::vector<Point3d> LeftLanePointsObj;
    std::vector<Point3d> RightLanePointsObj;

    for (int i=0;i<LeftLanePoints.cols();i++)
    {
        Point3d pL(utils::getId(),LeftLanePoints(0,i),LeftLanePoints(1,i),LeftLanePoints(2,i));
        Point3d pR(utils::getId(),RightLanePoints(0,i),RightLanePoints(1,i),RightLanePoints(2,i));
        LeftLanePointsObj.push_back(pL);
        RightLanePointsObj.push_back(pR);
    }

    // Generating LineString3d objects for Regulatory Elements
    std::vector<LineString3d> Tf_List;
    std::vector<LineString3d> St_List;

    misc::getRegelemListInit(Tf_List,Tf_vector,"TrafficLights");
    misc::getRegelemListInit(St_List,St_vector,"StopLines");

    // std::cout << "A Total of " << Tf_List.size() << " Traffic Lights were registered in this trip." << std::endl;
    // std::cout << "A Total of " << St_List.size() << " Stoplines were registered in this trip." << std::endl;

    std::vector<std::vector<int>> SameLaneOverlapIntv;
    std::vector<std::vector<int>> LeftLaneOverlapIntv;
    std::vector<std::vector<int>> RightLaneOverlapIntv;

    // Start MATLAB 
    std::unique_ptr<matlab::engine::MATLABEngine> matlabPtr = matlab::engine::startMATLAB();

    // Iterate over each trajectory intervals
    for (int i=0;i<TrajEventIntvs_.rows();i++)
    {
        int lb = TrajEventIntvs_(i,0) - 1;
        int ub = TrajEventIntvs_(i,1) - 1;
        int label = TrajEventIntvs_(i,2);
        bool end_flag;

        if (i == TrajEventIntvs_.rows()-1)
            end_flag = true;
        else
            end_flag = false;

        switch(label)
        {
            case event::ManeuverType::RLC: // Case 1: Right Lane Change --> Data Discarded
                std::cout << "[Event " << i << "]: Right Lane Change for State Idxs " << lb << " to " << ub << ". Discarding data..." << std::endl;
                break;

            case event::ManeuverType::LLC: // Case 2: Left Lane Change --> Data Discarded
                std::cout << "[Event " << i << "]: Left Lane Change for State Idxs " << lb << " to " << ub << ". Discarding data..." << std::endl;
                break;

            default: // Remaining Cases: Lane-Keeping, Right Turn, Left Turn, Roundabout
                // std::cout << "Event " << i << ": State Idxs " << lb << " to " << ub << std::endl;
                
                std::vector<int> sample_idxs(ub-lb+1);
                std::iota(sample_idxs.begin(), sample_idxs.end(), lb);

                bool possible_overlap_flag = false;

                if (~idxs_.empty()) // If overlap is detected
                {
                    for (int j=0;j<idxs_.size();j++)
                    {
                        // Search if overlap candidate state index idxs_[j] lies within the indices of sample_idxs.
                        // Since the vector sample_idxs is sorted, we can use binary_search.
                        if (std::binary_search(sample_idxs.begin(), sample_idxs.end(), idxs_[j]))
                        {
                            // std::cout << "State Idx " << idxs_[j] << " matched with State Idx " << matched_idxs_[j] << std::endl;
                            possible_overlap_flag = true;
                            break;
                        }
                    }
                }

                if (possible_overlap_flag)
                {
                    // Run overlapped case methods
                    std::cout << "[Event " << i << "]: Possible Loop Closure from State Idxs " << lb << " to " << ub << "." << std::endl;

                    // Overlap Functions..
                    std::vector<std::vector<int>> intv; std::string type;
                    getOverlapIntervals(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,lb,ub,i,label,intv,type,matlabPtr);
                    OverlapHandler(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,
                                   SameLaneOverlapIntv,LeftLaneOverlapIntv,RightLaneOverlapIntv,
                                   intv,type,i,end_flag,matlabPtr);
                }
                else
                    NormalHandler(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,lb,ub,i,label,false,-1,matlabPtr);
        }
    }

    std::cout << "====================================================================================================================" << std::endl;
    std::cout << "Single Trip Map Generation Finished." << std::endl;

    std::cout << "Do you want to save map? [Y/N]: ";
    std::string input;
    std::cin >> input;

    std::string curr_save_time = "-1";

    if (input == "Y" || input == "y")
        return save(matlabPtr);   
    else
        return curr_save_time;
}

void SingleTripAnalyzer::showParams(int idx)
{
    /*
    Check if all the inputs are well transferred from python numpy arrays to c++ Eigen matrices
    - vehicleParams_
      * Position: passed
      * Orientation: passed
      * Left, Right: passed
      * LeftCov, RightCov: passed
    - TrajEventIntvs_: passed
    - Tf_vector: passed
    - St_vector: passed

    */

    // vehicleParams_

    // // Pos
    // Eigen::Vector3d pos = vehicleParams_["Position"].col(idx);
    // std::cout << "Position from c++: , Idx " << idx << std::endl;
    // std::cout << pos << std::endl;

    // // Orientation
    // Eigen::Matrix3d ori = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>((vehicleParams_["Orientation"].col(idx)).data());
    // std::cout << "Orientation from c++: , Idx " << idx << std::endl;
    // std::cout << ori << std::endl;

    // // Left, Right Lane 
    // Eigen::Vector3d left = vehicleParams_["Left"].col(idx);
    // std::cout << "Left Lane from c++: , Idx " << idx << std::endl;
    // std::cout << left << std::endl;

    // // Left, Right Cov    
    // Eigen::Matrix3d covL = Eigen::Map<Eigen::Matrix<double, 3, 3>>((vehicleParams_["LeftCov"].col(idx)).data());
    // std::cout << "Covariance from c++: , Idx " << idx << std::endl;
    // std::cout << covL << std::endl;

    // // TrajEventIntvs_
    // std::cout << "TrajEventIntvs from c++ : " << std::endl;
    // std::cout << TrajEventIntvs_ << std::endl;

    // std::cout << "===================<Computed in C++>===================" << std::endl;
    // // Tf_vector
    // for (int i=0;i<Tf_vector.size();i++)
    // {
    //     Eigen::Vector3d Tf_pos = Tf_vector[i].getLandmarkPos();
    //     std::cout << "Position of Traffic Light " << i << ": " << std::endl;
    //     std::cout << Tf_pos << std::endl;
    // }

    // // St_vector
    // for (int i=0;i<St_vector.size();i++)
    // {
    //     Eigen::Vector3d St_pos = St_vector[i].getLandmarkPos();
    //     std::cout << "Position of Stopline " << i << ": " << std::endl;
    //     std::cout << St_pos << std::endl;
    // }
}

/*
Private Methods for SingleTrip Analyzer


*/

std::string SingleTripAnalyzer::save(const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr)
{
    LaneletMap Map;
    misc::getLaneletMap(Map, LaneletList, RegelemList);

    std::cout << "Saving map..." << std::endl;
    
    auto now = std::chrono::system_clock::now();
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
    std::tm* localTime = std::localtime(&currentTime);
    std::ostringstream oss;
    oss << std::put_time(localTime, "%Y-%m-%d-%H-%M-%S");
    std::string curr_time = oss.str();
    std::string fileName = "OSM/" + curr_time + ".osm";
    
    projection::LocalCartesianProjector projector(Origin({36.5222, 127.3032}));
    write(fileName, Map, projector);
    
    return curr_time;
}

// Lanelet generation for normal classes
void SingleTripAnalyzer::NormalHandler(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                                       std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                       int lb, int ub, int i, int label, bool call_from_overlap, int overlap_idx,
                                       const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr)
{
    if (call_from_overlap)
    {
        std::cout << "[Event " << i << " - " << overlap_idx << "]: (NormalHandler) "; 
    }
    else
        std::cout << "[Event " << i << "]: ";

    switch(label)
    {
        case event::ManeuverType::LKM:
            std::cout << "Lane Keeping Maneuver from State Idxs " << lb << " to " << ub << "." << std::endl;
            LKM(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,lb,ub,i,label);
            break;
        
        case event::ManeuverType::RT:
            std::cout << "Right Turn from State Idxs " << lb << " to " << ub << "." << std::endl;
            RLT(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,lb,ub,i,label);
            break;

        case event::ManeuverType::LT:
            std::cout << "Left Turn Maneuver from State Idxs " << lb << " to " << ub << "." << std::endl;
            RLT(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,lb,ub,i,label);
            break;
        
        case event::ManeuverType::RA:
            std::cout << "Roundabout from State Idxs " << lb << " to " << ub << "." << std::endl;
            RA(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,lb,ub,i,label,matlabPtr);
            break;
    }

}

void SingleTripAnalyzer::LKM(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                             std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                             int lb, int ub, int i, int label)
{
    std::vector<int> sample_idxs(ub-lb+1);
    std::iota(sample_idxs.begin(), sample_idxs.end(), lb);
    std::set<int> sample_idxs_set(sample_idxs.begin(), sample_idxs.end());

    std::vector<int> Tf_ObjIdxs = single_trip::getIntersectingObjectIdxs(Tf_vector,sample_idxs_set);
    std::vector<int> St_ObjIdxs = single_trip::getIntersectingObjectIdxs(St_vector,sample_idxs_set);

    // std::cout << Tf_ObjIdxs.size() << " Traffic Lights found." << std::endl;
    // std::cout << St_ObjIdxs.size() << " Stoplines found." << std::endl;

    std::set<int> Tf_ObjPairedIdxs_set;

    // Find 'alone' traffic lights
    for (int i=0;i<St_ObjIdxs.size();i++)
    {
        Eigen::Vector3d St_Pos = St_vector[St_ObjIdxs[i]].getLandmarkPos();
        std::vector<int> matched_St_Tf;

        // std::cout << "Stopline " << St_ObjIdxs[i] << " is matched with Traffic Light(s) ";

        for (int j=0;j<Tf_ObjIdxs.size();j++)
        {
            if (single_trip::isMatchedObjects(St_vector[St_ObjIdxs[i]],Tf_vector[Tf_ObjIdxs[j]]))
            {
                matched_St_Tf.push_back(Tf_ObjIdxs[j]); // To avoid complexity, global index of traffic light is appended, instead of simply appending 'j'.
                // std::cout << Tf_ObjIdxs[j] << " ";
            }
                
        }
        // std::cout << std::endl;
        
        if (!matched_St_Tf.empty())
        {
            /*
            There exists traffic light - stop line pair (May have many traffic lights per one stop line)
            May need to fix this part for future generalization
             
            [Cases]
            matched_St_Tf.size() == 1: Single traffic light - stop line match --> remove traffic light idx
            matched_St_Tf.size() == 2: Two traffic light - stop line match --> remove two traffic light idx (coupled)
            * This may not be true for rare situations where two traffic lights are not coupled.
            matched_St_Tf.size() == 3: Three traffic light - stop line match --> remove two traffic light idx (coupled)
             
            Others: Not handled (leave as error for now)
            */
            try
            {
                if (matched_St_Tf.size() > 3)
                    throw std::runtime_error("Currently not able to handle more than 3 traffic lights per stop lines.");
            }
            catch(const std::exception& e)
            {
                std::cerr << "Program ended with exception: " << e.what() << std::endl;
            }
            
            if(matched_St_Tf.size() == 3)
            {
                std::vector<double> dist;
                for (int j=0; j<matched_St_Tf.size();j++)
                {
                    Eigen::Vector3d Tf_Pos = Tf_vector[matched_St_Tf[j]].getLandmarkPos();
                    dist.push_back((St_Pos - Tf_Pos).norm());
                }
                // Get local index of the traffic light with the largest distance from the matched stop line.
                int aloneIdx = std::distance(dist.begin(), std::max_element(dist.begin(), dist.end()));

                // std::cout << "Distances to stop line: ";
                // for (double elem : dist) {
                //     std::cout << elem << " ";
                // }
                // std::cout << std::endl;
                // std::cout << "Stopline " << St_ObjIdxs[i] << " is matched with 3 Traffic Lights, but Traffic Light " << matched_St_Tf[aloneIdx] << " is alone." << std::endl;

                matched_St_Tf.erase(matched_St_Tf.begin() + aloneIdx);
            }

            Tf_ObjPairedIdxs_set.insert(matched_St_Tf.begin(), matched_St_Tf.end());
        }
    }
    std::set<int> Tf_ObjAloneIdxs_set;
    std::set<int> Tf_ObjIdxs_set(Tf_ObjIdxs.begin(),Tf_ObjIdxs.end());
    std::set_difference(Tf_ObjIdxs_set.begin(), Tf_ObjIdxs_set.end(), 
                        Tf_ObjPairedIdxs_set.begin(), Tf_ObjPairedIdxs_set.end(), 
                        std::inserter(Tf_ObjAloneIdxs_set, Tf_ObjAloneIdxs_set.begin()));
    
    // Split state intervals using Stoplines and Traffic Lights(alone)
    Eigen::MatrixXd samplePos = vehicleParams_["Position"].middleCols(lb,ub-lb+1);

    std::set<int> split_idxs;

    for (int idx : St_ObjIdxs)
        misc::addSplitIdxs(split_idxs, samplePos, St_vector[idx], lb); // Need to implement misc::addSplitIdx properly.
    
    for (int idx : Tf_ObjAloneIdxs_set)
        misc::addSplitIdxs(split_idxs, samplePos, Tf_vector[idx], lb);

    // Generate split intervals
    std::vector<std::vector<int>> split_intvs(split_idxs.size() + 1, std::vector<int>(2));
    split_intvs.front().front() = lb;
    split_intvs.back().back() = ub;
    int cnt = 0;
    for (auto it = split_idxs.begin(); it != split_idxs.end(); it++)
    {
        split_intvs[cnt][1] = *it;
        split_intvs[cnt+1][0] = *it;
        cnt++;
    }

    std::map<std::string, std::string> tags;
    tags["left_type"] = "line_thin";
    tags["left_subtype"] = "dashed";
    tags["right_type"] = "line_thin";
    tags["right_subtype"] = "dashed";

    // Generate labeled lanelet
    for (std::vector<int> intv_row : split_intvs)
    {
        int split_lb = intv_row.front();
        int split_ub = intv_row.back();
        generateLanelet(Tf_List,St_List,
                        LeftLanePointsObj,RightLanePointsObj,
                        split_lb,split_ub,tags,event::ManeuverType::LKM);
    }
}

void SingleTripAnalyzer::RLT(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                             std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                             int lb, int ub, int i, int label)
{
    std::map<std::string, std::string> tags;
    tags["left_type"] = "virtual";
    tags["left_subtype"] = "";
    tags["right_type"] = "virtual";
    tags["right_subtype"] = "";

    generateLanelet(Tf_List,St_List,
                    LeftLanePointsObj,RightLanePointsObj,
                    lb,ub,tags,label);
}

void SingleTripAnalyzer::RA(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                            std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                            int lb, int ub, int i, int label, const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr)
{
    Eigen::MatrixXd pos = vehicleParams_["Position"].middleCols(lb,ub-lb+1);

    using namespace matlab::engine;

    // Create MATLAB data array factory
    matlab::data::ArrayFactory factory;

    std::vector<matlab::data::Array> args({
        factory.createArray<double>({static_cast<size_t>(pos.rows()), static_cast<size_t>(pos.cols())}, pos.data(), pos.data() + pos.size()),
        factory.createArray<double>({2, 2},{5e-4, 0,0, 5e-4})
    });
    matlab::data::TypedArray<double> const result = matlabPtr->feval(u"FitRoundabout",args);
    std::vector<size_t> dimensions = result.getDimensions();
    std::vector<std::vector<int>> intvs(static_cast<int>(dimensions[0]),std::vector<int>(2));
    
    for (int i=0; i<static_cast<int>(dimensions[0]);i++)
    {
        for (int j=0; j<static_cast<int>(dimensions[1]);j++)
            intvs[i][j] = static_cast<int>(result[i][j]) - 1; // MATLAB indexing starts from 1
    }
    
    std::map<std::string, std::string> tags;
    tags["left_type"] = "line_thin";
    tags["left_subtype"] = "solid";
    tags["right_type"] = "line_thin";
    tags["right_subtype"] = "solid";

    for (std::vector<int> intv : intvs)
    {
        int lb_ = lb + intv.front();
        int ub_ = lb + intv.back();
        generateLanelet(Tf_List,St_List,
                        LeftLanePointsObj,RightLanePointsObj,
                        lb_,ub_,tags,event::ManeuverType::RA);
    }
}

// Get Overlap Intervals (Before Actually Updating)
void SingleTripAnalyzer::getOverlapIntervals(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                                             std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                             int lb, int ub, int i, int label, std::vector<std::vector<int>>& intv, std::string& type,
                                             const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr)
{
    bool non_overlap_flag = false;

    std::vector<int> sample_idxs(ub-lb+1);
    std::iota(sample_idxs.begin(), sample_idxs.end(), lb);

    // Compute yaw angle difference for overlap matched indices
    std::vector<double> rot_diff;

    for (int i=0;i<idxs_.size();i++)
    {
        if (std::binary_search(sample_idxs.begin(), sample_idxs.end(), idxs_[i]))
        {
            // std::cout << "State Idx " << matched_idxs_[i] << " is matched with State Idx " << idxs_[i] << std::endl;
            Eigen::Matrix3d Ri = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>((vehicleParams_["Orientation"].col(idxs_[i])).data());
            Eigen::Matrix3d Rj = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>((vehicleParams_["Orientation"].col(matched_idxs_[i])).data());
            Eigen::Vector3d vec = math::Log_map(Ri.transpose() * Rj);
            rot_diff.push_back(abs(vec[2]));
        }
    }

    LaneletMap SampleMap;
    misc::getLaneletMap(SampleMap, LaneletList, RegelemList);

    // 1. Check Travel Direction
    // In the future, be aware that there may be opposite trajectory that shares common Linestring.
    Eigen::MatrixXd LeftLanePointsCov = vehicleParams_["LeftCov"];
    Eigen::MatrixXd RightLanePointsCov = vehicleParams_["RightCov"];

    std::transform(rot_diff.begin(), rot_diff.end(), rot_diff.begin(), [](double val) { return val > M_PI/2 ? val : 0.0; });
    int numZeros = std::count_if(rot_diff.begin(), rot_diff.end(), [](double val) { return val == 0.0; });

    if (numZeros == rot_diff.size())
    {
        std::vector<int> overlap_type;
        /* Overlap Types
        -2: Located on the Right of the matched lanelet, but not directly (No linestring sharing)
        -1: Located on the Left of the matched lanelet, but not directly (No linestring sharing)
        0: Located on the Same lane as the matched lanelet
        1: Located on the Left of the matched lanelet
        2: Located on the Right of the matched lanelet
        3: Located in the middle of two lanelets, and also located left to nearest[0] --> should be handled with multiple trips (deprecated)
        4: Located in the middle of two lanelets, and also located right to nearest[0] --> should be handled with mulitple trips (deprecated)
        */ 
        
       // In case the covariance matrix is showing too high reliability, use the euclidean distance threshold also.
       double euclidean_threshold = 0.5;

        for (int i=0;i<idxs_.size();i++)
        {
            int idx = idxs_[i];
            if (std::binary_search(sample_idxs.begin(), sample_idxs.end(), idx))
            {
                Eigen::Vector3d Pi = vehicleParams_["Position"].col(idx);
                std::vector<std::pair<double, Lanelet>> nearest = geometry::findNearest(SampleMap.laneletLayer,BasicPoint2d(Pi[0],Pi[1]),2);
                overlap_type.push_back(misc::getOverlapType(nearest,
                                                            LeftLanePointsObj,LeftLanePointsCov,
                                                            RightLanePointsObj,RightLanePointsCov,
                                                            idx,euclidean_threshold));
            }
        }

        // Normal overlaps
        int num_zeros = std::count_if(overlap_type.begin(), overlap_type.end(), [](int val) { return val == 0; });
        int num_ones = std::count_if(overlap_type.begin(), overlap_type.end(), [](int val) { return val == 1; });
        int num_twos = std::count_if(overlap_type.begin(), overlap_type.end(), [](int val) { return val == 2; });

        // Special overlaps
        int num_minus_ones = std::count_if(overlap_type.begin(), overlap_type.end(), [](int val) { return val == -1; });
        int num_minus_twos = std::count_if(overlap_type.begin(), overlap_type.end(), [](int val) { return val == -2; });
        int num_threes = std::count_if(overlap_type.begin(), overlap_type.end(), [](int val) { return val == 3; });
        int num_fours = std::count_if(overlap_type.begin(), overlap_type.end(), [](int val) { return val == 4; });

        std::cout << " ** Overlap Type: ";
        if (num_minus_ones == 0 && num_minus_twos == 0)
        {            
            try
            {
                if (num_zeros == 0 && num_twos == 0)
                {
                    std::cout << "Left Overlap" << std::endl;
                    getLeftOverlapIntervals(SampleMap,Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,lb,ub,label,intv);
                    type = "left";
                }
                else if (num_zeros == 0 && num_ones == 0)
                {
                    std::cout << "Right Overlap" << std::endl;
                    getRightOverlapIntervals(SampleMap,Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,lb,ub,label,intv);
                    type = "right";
                }
                else if (num_ones == 0 && num_twos == 0)
                {
                    std::cout << "Same Lane Overlap" << std::endl;
                    getSameOverlapIntervals(SampleMap,Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,lb,ub,label,intv);
                    type = "same";
                }
                else 
                {
                    if (num_threes > 0 || num_fours > 0)
                        throw std::runtime_error("This trip should be divided into two or more subtrips for stable integration. Complex type of overlap is detected: Overlap Type 3 or 4");
                    else
                        throw std::runtime_error("Matching between input trajectory and existing map is inconsistent. Please check if trajectory event is well classified.(1)");
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        else
        {
            try
            {
                if (num_zeros == 0 && num_ones == 0 && num_twos == 0)
                {
                    if (num_minus_ones == overlap_type.size() || num_minus_twos == overlap_type.size())
                        non_overlap_flag = true;
                    else
                        throw std::runtime_error("No direct match, but not perfectly 'not overlapped'. Please check if the trajectory event is well classified.");
                }
                else
                {
                    // Extreme cases 
                    // Example: Overlap at right turn region, but moves into different lane when making right turns.
                    // * Lanelet xxxx: Lane #3 --(Right Turn)--> Lane #2
                    // * Current matched trip: Lane #3 --(Right Turn)--> Lane #1
                    // In this case, the linestrings should affect each other.

                    non_overlap_flag = true;
                    // throw std::runtime_error("Matching between input trajectory and existing map is inconsistent. Please check if trajectory event is well classified.(2)");
                }
                    
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }
    else
    {
        std::cout << "Opposite Direction. Need to check if linestring is overlapped? To be done in the future" << std::endl;
    }

    if (non_overlap_flag)
    {
        std::cout << "No map overlapping. Performing normal integration." << std::endl;
        NormalHandler(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,lb,ub,i,label,false,-1,matlabPtr);
    }
}

void SingleTripAnalyzer::getLeftOverlapIntervals(LaneletMap& SampleMap, std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                                                 std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                                 int lb, int ub, int label, std::vector<std::vector<int>>& intv)
{
    
}

void SingleTripAnalyzer::getRightOverlapIntervals(LaneletMap& SampleMap, std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                                                  std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                                  int lb, int ub, int label, std::vector<std::vector<int>>& intv)
{

}

void SingleTripAnalyzer::getSameOverlapIntervals(LaneletMap& SampleMap, std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List, 
                                                 std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                                 int lb, int ub, int label, std::vector<std::vector<int>>& intv)
{
    std::vector<std::vector<int>> Idxs(ub-lb+1,std::vector<int>(2));
    for (int i=lb;i<=ub;i++)
    {
        Idxs[i-lb][0] = i;
        // Lanelet matched_lanelet = Lanelet(); bool match_flag = false;

        Eigen::Vector3d Pi = vehicleParams_["Position"].col(i);
        std::vector<std::pair<double,Lanelet>> nearest = geometry::findNearest(SampleMap.laneletLayer,BasicPoint2d(Pi[0],Pi[1]),2);

        if (geometry::inside(nearest[0].second,BasicPoint2d(Pi[0],Pi[1])))
        {
            Lanelet matched_lanelet = nearest[0].second;
            int matched_lanelet_label = misc::getLaneletLabel(matched_lanelet,LaneletList);

            if (matched_lanelet_label == label)
                Idxs[i-lb][1] = matched_lanelet.id();
            else if (matched_lanelet_label == -1)
                Idxs[i-lb][1] = -1;
            else
                Idxs[i-lb][1] = -matched_lanelet.id(); // Vehicle driving maneuver mismatch.
        }    
        else
            Idxs[i-lb][1] = -1;
    }
    intv = misc::getMatchIntv(Idxs,label);
}

// Update Overlapping Intervals

void SingleTripAnalyzer::OverlapHandler(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List,
                                        std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                        std::vector<std::vector<int>>& SameLaneOverlapIntv,
                                        std::vector<std::vector<int>>& LeftLaneOverlapIntv,
                                        std::vector<std::vector<int>>& RightLaneOverlapIntv,
                                        std::vector<std::vector<int>> intv, std::string type, int i, bool end_flag,
                                        const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr)
{
    if (!intv.empty())
    {
        std::pair<std::vector<std::vector<int>>, std::vector<std::vector<int>>> res;

        if (type == "same")
        {
            res = misc::intervalOverlapHandler(SameLaneOverlapIntv,intv,end_flag);
            for (std::vector<int> rows : res.second)
                SameLaneOverlapIntv.push_back(rows); // stack intervals
            flushSameOverlap(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,res.first,i,matlabPtr); // integrate flush intervals into map 
        }
        else if (type == "left")
        {
            res = misc::intervalOverlapHandler(LeftLaneOverlapIntv,intv,end_flag);
            for (std::vector<int> rows : res.second)
                LeftLaneOverlapIntv.push_back(rows); // stack intervals
            flushLeftOverlap(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,res.first,i,matlabPtr); // integrate flush intervals into map 
        }
        else if (type == "right")
        {
            res = misc::intervalOverlapHandler(RightLaneOverlapIntv,intv,end_flag);
            for (std::vector<int> rows : res.second)
                RightLaneOverlapIntv.push_back(rows); // stack intervals
            flushRightOverlap(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,res.first,i,matlabPtr); // integrate flush intervals into map 
        }
    }
}

void SingleTripAnalyzer::flushSameOverlap(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List,
                                          std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                          std::vector<std::vector<int>> flush_intv, int i,
                                          const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr)
{    
    std::vector<std::vector<int>> processed_intv = misc::processFlushIntv(LaneletList,flush_intv);
    // Augment lanelet linestrings with overlapped data

    for (int j=0;j<processed_intv.size();j++)
    {
        std::vector<int> curr_intv_row = processed_intv[j];
        int lb = curr_intv_row[0];
        int ub = curr_intv_row[1];
        Id id = curr_intv_row[2];
        int label = curr_intv_row[3];

        if (id == -1) // Not matched to any of the lanelet. Add new lanelet to LaneletMap
        {
            int org_size = LaneletList.size();
            NormalHandler(Tf_List,St_List,LeftLanePointsObj,RightLanePointsObj,lb,ub,i,label,true,j+1,matlabPtr);

            // After adding lanelet, need to link with nearby lanelets appropriately.
            // For process interval that lies in between, both of the 'if' statements are executed.            
            if (j < processed_intv.size()-1)
            {
                std::vector<int> next_intv_row = processed_intv[j+1];
                int lanelet_idx = misc::getLaneletIdx(LaneletList,next_intv_row[2]);
                Point3d pL = LaneletList[lanelet_idx].leftBound().front();
                Point3d pR = LaneletList[lanelet_idx].rightBound().front();
                int idx = IdxList[lanelet_idx].front();
                std::vector<Point3d> pLs; pLs.push_back(pL);
                std::vector<Point3d> pRs; pRs.push_back(pR);
                addPoints2LineString(LaneletList.size()-1,pLs,idx,idx,"left","back");
                addPoints2LineString(LaneletList.size()-1,pRs,idx,idx,"right","back");
            }

            if (j > 0)
            {
                std::vector<int> prev_intv_row = processed_intv[j-1];
                int lanelet_idx = misc::getLaneletIdx(LaneletList,prev_intv_row[2]);
                Point3d pL = LaneletList[lanelet_idx].leftBound().back();
                Point3d pR = LaneletList[lanelet_idx].rightBound().back();
                int idx = IdxList[lanelet_idx].back();
                std::vector<Point3d> pLs; pLs.push_back(pL);
                std::vector<Point3d> pRs; pRs.push_back(pR);
                addPoints2LineString(org_size,pLs,idx,idx,"left","front");
                addPoints2LineString(org_size,pRs,idx,idx,"right","front");
            }
        }
        else // Matched to some lanelet. Add lane points to corresponding linestrings.
        {   
            std::cout << "[Event " << i << " - " << j+1 << "]: (Overlap) Integrating overlapped data to Lanelet ID: " << id << " from State Idxs " << lb << " to " << ub << "." << std::endl;
            int lanelet_idx = misc::getLaneletIdx(LaneletList,id);
            addPoints2LineString(lanelet_idx,LeftLanePointsObj,lb,ub,"left","normal");
            addPoints2LineString(lanelet_idx,RightLanePointsObj,lb,ub,"right","normal");
        }
    }
}

void SingleTripAnalyzer::flushLeftOverlap(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List,
                                          std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                          std::vector<std::vector<int>> flush_intv, int i,
                                          const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr)
{

}

void SingleTripAnalyzer::flushRightOverlap(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List,
                                           std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                           std::vector<std::vector<int>> flush_intv, int i,
                                           const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr)
{

}

// Generate Lanelet with all regulatory elements attached.
void SingleTripAnalyzer::generateLanelet(std::vector<LineString3d> Tf_List, std::vector<LineString3d> St_List,
                                         std::vector<Point3d> LeftLanePointsObj, std::vector<Point3d> RightLanePointsObj,
                                         int lb, int ub, std::map<std::string, std::string> tags, int label)
{
    std::vector<Point3d> sampledLeftLanePointsObj;
    std::vector<Point3d> sampledRightLanePointsObj;
    std::vector<int> idxs;

    for (int i=lb;i<ub+1;i++)
    {
        sampledLeftLanePointsObj.push_back(LeftLanePointsObj[i]);
        sampledRightLanePointsObj.push_back(RightLanePointsObj[i]);
        idxs.push_back(i);
    }

    LineString3d LeftLane = misc::getLineString(sampledLeftLanePointsObj,tags["left_type"],tags["left_subtype"]);
    LineString3d RightLane = misc::getLineString(sampledRightLanePointsObj,tags["right_type"],tags["right_subtype"]);

    LabeledLanelet llt = misc::getLanelet(LeftLane,RightLane,"road","urban","vehicle","yes","50 kmh",label);
    
    // How to return values if there are no stop lines?
    
    std::vector<int> sample_idxs(ub-lb+1);
    std::iota(sample_idxs.begin(), sample_idxs.end(), lb);
    std::set<int> sample_idxs_set(sample_idxs.begin(),sample_idxs.end());

    for (int i=0;i<Tf_vector.size();i++)
    {
        if (single_trip::isIntersect(Tf_vector[i],sample_idxs_set))
        {
            LineString3d Tf_ls = Tf_List[i];
            Tf_ls.attributes()[AttributeName::Type] = AttributeValueString::TrafficLight;
            Tf_ls.attributes()[AttributeName::Subtype] = AttributeValueString::RedYellowGreen;
            RegulatoryElementPtr tf = lanelet::TrafficLight::make(utils::getId(),{},{Tf_ls});
            RegelemList.push_back(tf);
            llt.addRegulatoryElement(tf);
        }
    }
    LaneletList.push_back(llt);
    IdxList.push_back(idxs);

    bool stop_line_flag = false;
    LaneletWithStopLine llt_ = {Lanelet(), LineString3d()};

    for (int i=0;i<St_vector.size();i++)
    {
        if (single_trip::isIntersect(St_vector[i],sample_idxs_set))
        {
            LineString3d St_ls = St_List[i];
            St_ls.attributes()[AttributeName::Type] = AttributeValueString::StopLine;
            LaneletWithStopLine llt_ = {llt.getLanelet(), St_ls};
            LaneletsWithStopLines llts_;
            llts_.push_back(llt_);

            RegulatoryElementPtr awsPtr = AllWayStop::make(utils::getId(),{},llts_);
            RegelemList.push_back(awsPtr);
            break;
        }
    }
}

// Add points to linestring at the appropriate positions
void SingleTripAnalyzer::addPoints2LineString(int LaneletListIdx, std::vector<Point3d> points, int lb, int ub, 
                                              std::string dir, std::string options)
{
    int idx;
    if (options == "normal")
    {
        for (int i=lb;i<=ub;i++)
        {
            // Left Lane
            if (dir == "left")
            {
                int idxL = single_trip::findPointInsertIdx(LaneletList[LaneletListIdx].leftBound(),points[i],vehicleParams_,i);
                LaneletList[LaneletListIdx].leftBound().insert(LaneletList[LaneletListIdx].leftBound().begin() + idxL + 1, points[i]); // check if insert position is appropriate.
                idx = idxL;
            }
            // Right Lane
            else if (dir == "right")
            {
                int idxR = single_trip::findPointInsertIdx(LaneletList[LaneletListIdx].rightBound(),points[i],vehicleParams_,i);
                LaneletList[LaneletListIdx].rightBound().insert(LaneletList[LaneletListIdx].rightBound().begin() + idxR + 1, points[i]); // check if insert position is appropriate.
                idx = idxR;
            }
            std::vector<int> vec = IdxList[LaneletListIdx];
            auto it = std::find(vec.begin(), vec.end(), i);
            if (it == vec.end()) // Insert index only if it does not exist.
                IdxList[LaneletListIdx].insert(IdxList[LaneletListIdx].begin() + idx + 1, i);
        }
    }
    else if (options == "front")
    {
        // Size of points should be 1 in this case.
        if (dir == "left")
            LaneletList[LaneletListIdx].leftBound().insert(LaneletList[LaneletListIdx].leftBound().begin(),points.front()); 
        else if (dir == "right")
            LaneletList[LaneletListIdx].rightBound().insert(LaneletList[LaneletListIdx].rightBound().begin(),points.front());

        std::vector<int> vec = IdxList[LaneletListIdx];
        auto it = std::find(vec.begin(), vec.end(), lb); // in this case, lb == ub
        if (it == vec.end()) // Insert index only if it does not exist.
            IdxList[LaneletListIdx].insert(IdxList[LaneletListIdx].begin(), lb);
    }
    else if (options == "back")
    {
        // Size of points should be 1 in this case.
        if (dir == "left")
            LaneletList[LaneletListIdx].leftBound().push_back(points.back()); 
        else if (dir == "right")
            LaneletList[LaneletListIdx].rightBound().push_back(points.back());

        std::vector<int> vec = IdxList[LaneletListIdx];
        auto it = std::find(vec.begin(), vec.end(), lb); // in this case, lb == ub
        if (it == vec.end()) // Insert index only if it does not exist.
            IdxList[LaneletListIdx].push_back(lb);
    }
}

SingleTripAnalyzer::~SingleTripAnalyzer()
{
    matlab::engine::terminateEngineClient();
}

// Python Bindings
PYBIND11_MODULE(ALMG, m){
    py::class_<SingleTripAnalyzer>(m,"SingleTripAnalyzer")
        .def(py::init<std::vector<int> ,std::vector<int>, std::map<std::string, py::array_t<double>>, py::array_t<int>, py::array_t<double>, py::array_t<double>, py::array_t<double>, py::array_t<double>>())
        .def("addRegulatoryElementStateIdxs", &SingleTripAnalyzer::addRegulatoryElementStateIdxs)
        .def("run", &SingleTripAnalyzer::run)
        .def("showParams", &SingleTripAnalyzer::showParams);
}