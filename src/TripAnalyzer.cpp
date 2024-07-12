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

void SingleTripAnalyzer::visualize(void)
{   
    using namespace matplot;
    std::vector<double> x_tf, y_tf;
    std::vector<double> x_st, y_st;
    std::vector<double> vpos_x, vpos_y;

    Eigen::MatrixXd vpos = vehicleParams_["Position"];
    
    for (int i=0;i<vpos.cols();i++)
    {
        Eigen::VectorXd column = vpos.col(i);
        vpos_x.push_back(column[0]);
        vpos_y.push_back(column[1]);
    }

    for (int i=0;i<Tf_vector.size();i++)
    {
        Eigen::Vector3d Tf_pos = Tf_vector[i].getLandmarkPos();
        x_tf.push_back(Tf_pos.x());
        y_tf.push_back(Tf_pos.y());
    }

    for (int i=0;i<St_vector.size();i++)
    {
        Eigen::Vector3d St_pos = St_vector[i].getLandmarkPos();
        x_st.push_back(St_pos.x());
        y_st.push_back(St_pos.y());
    }

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
    std::string arc_approx_flag;
    std::cout << "Perform arc spline approximation of linestrings? [Y/N]: ";
    std::cin >> arc_approx_flag;
    if (arc_approx_flag == "Y" || arc_approx_flag == "y")
        arcSplineApprox(matlabPtr);
    
    std::cout << "====================================================================================================================" << std::endl;
    std::cout << "Single Trip Map Generation Finished." << std::endl;

    std::cout << "Do you want to save map? [Y/N]: ";
    std::string input;
    std::cin >> input;

    std::string curr_save_time = "-1";

    if (input == "Y" || input == "y")
        return save(matlabPtr, arc_approx_flag);   
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

std::string SingleTripAnalyzer::save(const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr, std::string arc_approx_flag)
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
    std::string fileName = "/mnt/hdd1/SJ_Dataset/MatlabFiles/github/Mapping/MapData/OSM/" + curr_time + ".osm";
    
    projection::LocalCartesianProjector projector(Origin({36.5222, 127.3032}));
    write(fileName, Map, projector);

    // Call MATLAB function for visualization of osm map data (with arguments if arc spline approximation is performed)
    // From MATLAB, save figure automatically. 
    matlab::data::ArrayFactory factory;
    matlab::data::CharArray charArray1 = factory.createCharArray(fileName);
    matlab::data::CharArray charArray2 = factory.createCharArray(arc_approx_flag);
    matlab::data::CellArray cellArray = factory.createCellArray({1,2});
    cellArray[0] = charArray1; cellArray[1] = charArray2;
    matlabPtr->feval(u"VisualizeMap",0,std::vector<matlab::data::Array>({cellArray}));
    
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
    matlab::data::TypedArray<double> const result = matlabPtr->feval(u"RoundaboutFit",args);
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

// Perform multiple-arc spline approximation across multiple linestrings
void SingleTripAnalyzer::arcSplineApprox(const std::unique_ptr<matlab::engine::MATLABEngine>& matlabPtr)
{
    // Perform arc spline approximation of LineStrings
    LaneletMap Map;
    misc::getLaneletMap(Map, LaneletList, RegelemList);
    traffic_rules::TrafficRulesPtr trafficRules{traffic_rules::TrafficRulesFactory::instance().create(Locations::Germany, Participants::Vehicle)};
    routing::RoutingGraphUPtr routingGraph = routing::RoutingGraph::build(Map, *trafficRules);
    double inf = std::numeric_limits<double>::infinity();
    std::vector<std::vector<Id>> LineStringCluster;

    // Clustering of reachable lanelets (linestrings)
    for (int i=0;i<LaneletList.size();i++)
    {
        // Reachable lanelets from the current lanelet 
        ConstLanelets reachableSetNext = routingGraph->reachableSet(Map.laneletLayer.get(LaneletList[i].id()),inf,0,false); 
        // lanelets that can be reached to the current lanelet
        ConstLanelets reachableSetPrev = routingGraph->reachableSetTowards(Map.laneletLayer.get(LaneletList[i].id()),inf,0,false);

        // Cluster LineStrings for optimization
        misc::clusterLineStrings(LineStringCluster,reachableSetNext,reachableSetPrev,LaneletList[i]);
    }

    std::vector<std::vector<Eigen::Vector3d>> full_endNodes(LineStringCluster.size());
    std::vector<std::vector<Eigen::Vector3d>> full_normNodes(LineStringCluster.size());
    std::vector<std::vector<Eigen::Vector3d>> full_midNodes(LineStringCluster.size());
    std::vector<std::vector<std::pair<int,int>>> full_endNodeTrackers(LineStringCluster.size());
    std::vector<std::vector<std::pair<int,int>>> full_normNodeTrackers(LineStringCluster.size());
    std::vector<std::vector<std::pair<int,int>>> full_midNodeTrackers(LineStringCluster.size());

    for (int i=0;i<LineStringCluster.size();i++)
    {
        using namespace matlab::engine;
        matlab::data::ArrayFactory factory;
        std::vector<Id> cluster = LineStringCluster[i];
        Eigen::VectorXi front_ids(cluster.size());
        Eigen::VectorXi back_ids(cluster.size());
        matlab::data::CellArray pts_m = factory.createCellArray({1,cluster.size()});

        for (int j=0;j<cluster.size();j++)
        {
            for (int k=0;k<LaneletList.size();k++)
            {
                if (LaneletList[k].leftBound().id() == cluster[j])
                {
                    Eigen::MatrixXd pts(3,LaneletList[k].leftBound().size());

                    for (int l=0;l<LaneletList[k].leftBound().size();l++)
                        pts.col(l) << LaneletList[k].leftBound()[l].x(), LaneletList[k].leftBound()[l].y(), LaneletList[k].leftBound()[l].z();
                    
                    pts_m[j] = factory.createArray<double>({3,LaneletList[k].leftBound().size()},pts.data(),pts.data()+pts.size());
                    front_ids[j] = LaneletList[k].leftBound().front().id();
                    back_ids[j] = LaneletList[k].leftBound().back().id();
                    break;
                }
                else if (LaneletList[k].rightBound().id() == cluster[j])
                {
                    Eigen::MatrixXd pts(3,LaneletList[k].rightBound().size());

                    for (int l=0;l<LaneletList[k].rightBound().size();l++)
                        pts.col(l) << LaneletList[k].rightBound()[l].x(), LaneletList[k].rightBound()[l].y(), LaneletList[k].rightBound()[l].z();
                    
                    pts_m[j] = factory.createArray<double>({3,LaneletList[k].rightBound().size()},pts.data(),pts.data()+pts.size());
                    front_ids[j] = LaneletList[k].rightBound().front().id();
                    back_ids[j] = LaneletList[k].rightBound().back().id();
                    break;
                } 
            }
            
        }
        // std::cout << "Cluster #" << i << " of size " << cluster.size() << " Completed" << std::endl;
        // std::cout << "Front Ids" << std::endl;
        // std::cout << front_ids << std::endl;
        // std::cout << "Back Ids" << std::endl;
        // std::cout << back_ids << std::endl;
        matlab::data::TypedArray<int> front_ids_m = factory.createArray<int>({1,static_cast<size_t>(front_ids.size())},front_ids.data(),front_ids.data()+front_ids.size());
        matlab::data::TypedArray<int> back_ids_m = factory.createArray<int>({1,static_cast<size_t>(back_ids.size())},back_ids.data(),back_ids.data()+back_ids.size());  

        matlabPtr->setVariable(u"pts_",pts_m);
        matlabPtr->setVariable(u"front_ids_",front_ids_m);
        matlabPtr->setVariable(u"back_ids_",back_ids_m);
        matlabPtr->eval(u"obj = MultiLineStringFit(pts_,front_ids_,back_ids_);");
        matlabPtr->eval(u"obj.fit();");

        // Need to fix this part! Then build again!
        matlabPtr->eval(u"[endNodes_,normNodes_,midNodes_,endNodeTrackers_,normNodeTrackers_,midNodeTrackers_] = obj.getParams();");
        matlab::data::TypedArray<double> endNodes_ = matlabPtr->getVariable(u"endNodes_");
        matlab::data::TypedArray<double> normNodes_ = matlabPtr->getVariable(u"normNodes_");
        matlab::data::TypedArray<double> midNodes_ = matlabPtr->getVariable(u"midNodes_");
        matlab::data::TypedArray<double> endNode_trackers_ = matlabPtr->getVariable(u"endNodeTrackers_");
        matlab::data::TypedArray<double> normNode_trackers_ = matlabPtr->getVariable(u"normNodeTrackers_");
        matlab::data::TypedArray<double> midNode_trackers_ = matlabPtr->getVariable(u"midNodeTrackers_");

        // Convert MATLAB format array data into usable formats
        std::vector<size_t> endNodeDim = endNodes_.getDimensions();
        std::vector<Eigen::Vector3d> endNodes(endNodeDim[1]);
        for (int j=0; j<static_cast<int>(endNodeDim[1]);j++)
        {
            Eigen::Vector3d endNode(endNodes_[0][j],endNodes_[1][j],endNodes_[2][j]); 
            endNodes[j] = endNode;
        }

        std::vector<size_t> normNodeDim = normNodes_.getDimensions();
        std::vector<Eigen::Vector3d> normNodes(normNodeDim[1]);
        for (int j=0; j<static_cast<int>(normNodeDim[1]);j++)
        {
            Eigen::Vector3d normNode(normNodes_[0][j],normNodes_[1][j],normNodes_[2][j]); 
            normNodes[j] = normNode;
        }

        std::vector<size_t> midNodeDim = midNodes_.getDimensions();
        std::vector<Eigen::Vector3d> midNodes(midNodeDim[1]);
        for (int j=0; j<static_cast<int>(midNodeDim[1]);j++)
        {
            Eigen::Vector3d midNode(midNodes_[0][j],midNodes_[1][j],midNodes_[2][j]); 
            midNodes[j] = midNode;
        }

        std::vector<size_t> endNodeTrackerDim = endNode_trackers_.getDimensions();
        std::vector<std::pair<int,int>> endNodeTrackers(endNodeTrackerDim[1]);
        for (int j=0; j<static_cast<int>(endNodeTrackerDim[1]);j++)
        {
            std::pair<int,int> endNodeIntv(static_cast<int>(endNode_trackers_[0][j]),static_cast<int>(endNode_trackers_[1][j]));
            endNodeTrackers[j] = endNodeIntv;
        }

        std::vector<size_t> normNodeTrackerDim = normNode_trackers_.getDimensions();
        std::vector<std::pair<int,int>> normNodeTrackers(normNodeTrackerDim[1]);
        for (int j=0; j<static_cast<int>(normNodeTrackerDim[1]);j++)
        {
            std::pair<int,int> normNodeIntv(static_cast<int>(normNode_trackers_[0][j]),static_cast<int>(normNode_trackers_[1][j]));
            normNodeTrackers[j] = normNodeIntv;
        }

        std::vector<size_t> midNodeTrackerDim = midNode_trackers_.getDimensions();
        std::vector<std::pair<int,int>> midNodeTrackers(midNodeTrackerDim[1]);
        for (int j=0; j<static_cast<int>(midNodeTrackerDim[1]);j++)
        {
            std::pair<int,int> midNodeIntv(static_cast<int>(midNode_trackers_[0][j]),static_cast<int>(midNode_trackers_[1][j]));
            midNodeTrackers[j] = midNodeIntv;
        }

        full_endNodes[i] = endNodes;

        if (normNodes.empty())
        {
            // In order to prevent memory leak if normNode is empty, we add dummy vector
            Eigen::Vector3d vec(-1,-1,-1);
            std::vector<Eigen::Vector3d> vecs(1); vecs.push_back(vec);
            full_normNodes[i] = vecs;
        }
        else
            full_normNodes[i] = normNodes;

        full_midNodes[i] = midNodes;
        full_endNodeTrackers[i] = endNodeTrackers;
        full_normNodeTrackers[i] = normNodeTrackers;
        full_midNodeTrackers[i] = midNodeTrackers;
    }
    // Visualization of parameterization results
    // visualize(full_endNodes,full_normNodes,full_midNodes,full_endNodeTrackers,full_normNodeTrackers,full_midNodeTrackers);
    
    // Update LineStrings data that is filled with raw lane points with arc nodes.
    updateLineStrings(LineStringCluster,
                      full_endNodes,full_normNodes,full_midNodes,
                      full_endNodeTrackers,full_normNodeTrackers,full_midNodeTrackers);
}

void SingleTripAnalyzer::visualize(const std::vector<std::vector<Eigen::Vector3d>>& full_endNodes,
                                   const std::vector<std::vector<Eigen::Vector3d>>& full_normNodes,
                                   const std::vector<std::vector<Eigen::Vector3d>>& full_midNodes,
                                   const std::vector<std::vector<std::pair<int,int>>>& full_endNodeTrackers,
                                   const std::vector<std::vector<std::pair<int,int>>>& full_normNodeTrackers,
                                   const std::vector<std::vector<std::pair<int,int>>>& full_midNodeTrackers)
{
    using namespace matplot;
    // projector for local (x, y) --> (lat, lon)
    projection::LocalCartesianProjector projector(Origin({36.5222, 127.3032, 181.1957})); 

    std::vector<std::vector<std::vector<double>>> approx_x;
    std::vector<std::vector<std::vector<double>>> approx_y;
    std::vector<double> endNodes_x;
    std::vector<double> endNodes_y;
    std::vector<double> normNodes_x;
    std::vector<double> normNodes_y;
    std::vector<double> u = linspace(0,1);

    for (int i=0;i<full_endNodes.size();i++)
    {
        // For each cluster of linestrings.
        std::vector<Eigen::Vector3d> endNodes = full_endNodes[i];
        std::vector<Eigen::Vector3d> normNodes = full_normNodes[i];
        std::vector<Eigen::Vector3d> midNodes = full_midNodes[i];
        std::vector<std::pair<int,int>> endNodeTrackers = full_endNodeTrackers[i];
        std::vector<std::pair<int,int>> normNodeTrackers = full_normNodeTrackers[i];
        std::vector<std::pair<int,int>> midNodeTrackers = full_midNodeTrackers[i];

        std::vector<std::vector<double>> approx_ls_x;
        std::vector<std::vector<double>> approx_ls_y;

        for (int j=0;j<endNodeTrackers.size();j++)
        {
            // For each linestring, accumulate multiple arc approximation results
            int endlbIdx = endNodeTrackers[j].first;
            int endubIdx = endNodeTrackers[j].second;
            int normlbIdx = normNodeTrackers[j].first;
            int normubIdx = normNodeTrackers[j].second;
            int midlbIdx = midNodeTrackers[j].first;
            int midubIdx = midNodeTrackers[j].second;

            GPSPoint endNodeLbGPS = projector.reverse(endNodes[endlbIdx]);
            GPSPoint endNodeUbGPS = projector.reverse(endNodes[endubIdx]);
            
            endNodes_x.push_back(endNodeLbGPS.lat);
            endNodes_y.push_back(endNodeLbGPS.lon);
            endNodes_x.push_back(endNodeUbGPS.lat);            
            endNodes_y.push_back(endNodeUbGPS.lon);

            std::vector<Eigen::Vector3d> curr_arcNodes;

            std::vector<double> xs, ys;
            if (normlbIdx ==  -1 && normubIdx == -1)
            {
                curr_arcNodes.push_back(endNodes[endlbIdx]);
                curr_arcNodes.push_back(endNodes[endubIdx]);
            }
            else
            {
                curr_arcNodes.push_back(endNodes[endlbIdx]);
                for (int k=normlbIdx;k<=normubIdx;k++)
                {
                    curr_arcNodes.push_back(normNodes[k]);

                    GPSPoint normNodeGPS = projector.reverse(normNodes[k]);
                    normNodes_x.push_back(normNodeGPS.lat);
                    normNodes_y.push_back(normNodeGPS.lon);
                }
                    
                curr_arcNodes.push_back(endNodes[endubIdx]);
            }

            for (int k=0;k<midubIdx-midlbIdx+1;k++)
            {
                Eigen::Vector2d A1(curr_arcNodes[k].x(), curr_arcNodes[k].y());
                Eigen::Vector2d A2(curr_arcNodes[k+1].x(), curr_arcNodes[k+1].y());
                Eigen::Vector2d N1(midNodes[midlbIdx+k].x(), midNodes[midlbIdx+k].y());

                Eigen::Vector2d v1 = N1 - A1; 
                Eigen::Vector2d v2 = A2 - A1;
                double v11 = v1.transpose() * v1; 
                double v22 = v2.transpose() * v2; 
                double v12 = v1.transpose() * v2;
                double b = 0.5 / (v11*v22 - v12*v12);
                double k1 = b * v22 * (v11 - v12);
                double k2 = b * v11 * (v22 - v12);
                Eigen::Vector2d Xc = A1 + k1*v1 + k2*v2;
                Eigen::Vector2d m = 0.5 * (A1 + A2);
                double hsq = 0.5 * (A1 - A2).norm();
                double dist = (m - Xc).norm();
                double k_norm = hsq*hsq/dist;
                Eigen::Vector2d C = m + (m-Xc)/dist * k_norm;
                double w = hsq/sqrt(hsq*hsq + k_norm*k_norm);

                for (int l=0;l<u.size();l++)
                {
                    Eigen::Vector2d approx = ((1-u[l])*(1-u[l]) * A1 + 2 * u[l] * (1-u[l]) * w * C + u[l]*u[l] * A2)/((1-u[l])*(1-u[l]) + 2 * u[l] * (1-u[l]) * w + u[l] * u[l]);
                    Eigen::Vector3d approx3d(approx.x(),approx.y(),0.5*(curr_arcNodes[k].z() + curr_arcNodes[k+1].z()));
                    GPSPoint approx_revproj = projector.reverse(approx3d);
                    xs.push_back(approx_revproj.lat);
                    ys.push_back(approx_revproj.lon);
                }
            }
            approx_ls_x.push_back(xs);
            approx_ls_y.push_back(ys);
        }
        approx_x.push_back(approx_ls_x);
        approx_y.push_back(approx_ls_y);
    }

    auto s1 = scatter(endNodes_x,endNodes_y); 
    s1->marker_color("b");
    // s1->display_name("End Nodes");
    hold(on); axis(equal); grid(on);

    auto s2 = scatter(normNodes_x,normNodes_y);
    s2->marker_color("g");
    // s2->display_name("Normal Nodes");
    
    for (int i=0;i<approx_x.size();i++)
    {
        for (int j=0;j<approx_x[i].size();j++)
        {
            // std::cout << "Cluster #" << i << ", LineString #" << j << " size: " << approx_x[i][j].size() << std::endl;
            plot(approx_x[i][j], approx_y[i][j],"--k"); 
        }
    } 
    
    legend({"End Nodes", "Normal Nodes", "Arc Spline Approximation"});
    
    xlabel("Global X");
    ylabel("Global Y");
    show();

}

void SingleTripAnalyzer::updateLineStrings(const std::vector<std::vector<Id>>& LineStringCluster,
                                           const std::vector<std::vector<Eigen::Vector3d>>& full_endNodes,
                                           const std::vector<std::vector<Eigen::Vector3d>>& full_normNodes,
                                           const std::vector<std::vector<Eigen::Vector3d>>& full_midNodes,
                                           const std::vector<std::vector<std::pair<int,int>>>& full_endNodeTrackers,
                                           const std::vector<std::vector<std::pair<int,int>>>& full_normNodeTrackers,
                                           const std::vector<std::vector<std::pair<int,int>>>& full_midNodeTrackers)
{
    // Generate new LineStrings using node information --> Create Lanelets
    // Pass boolean to MATLAB whether arc spline approximation was performed or not.
    // Using this information, visualization of the generated osm file will be done.

    for (int i=0;i<LineStringCluster.size();i++)
    {
        std::vector<Eigen::Vector3d> endNodes = full_endNodes[i];
        std::vector<Eigen::Vector3d> normNodes = full_normNodes[i];
        std::vector<Eigen::Vector3d> midNodes = full_midNodes[i];
        std::vector<std::pair<int,int>> endNodeTrackers = full_endNodeTrackers[i];
        std::vector<std::pair<int,int>> normNodeTrackers = full_normNodeTrackers[i];
        std::vector<std::pair<int,int>> midNodeTrackers = full_midNodeTrackers[i];

        // For each LineString Id in the cluster, we have to find the index of the related LaneletList.
        std::vector<Id> LineStringIds = LineStringCluster[i];

        std::vector<Point3d> endNodes_p(endNodes.size());
        std::vector<Point3d> normNodes_p(normNodes.size());
        std::vector<Point3d> midNodes_p(midNodes.size());

        for (int j=0;j<endNodes.size();j++)
        {
            Point3d pt(utils::getId(),endNodes[j].x(),endNodes[j].y(),endNodes[j].z());
            endNodes_p[j] = pt;
        }

        for (int j=0;j<normNodes.size();j++)
        {
            Point3d pt(utils::getId(),normNodes[j].x(),normNodes[j].y(),normNodes[j].z());
            normNodes_p[j] = pt;
        }

        for (int j=0;j<midNodes.size();j++)
        {
            Point3d pt(utils::getId(),midNodes[j].x(),midNodes[j].y(),midNodes[j].z());
            midNodes_p[j] = pt;
        }

        for (int j=0;j<LineStringIds.size();j++)
        {
            int endNodeIdxLb = endNodeTrackers[j].first; int endNodeIdxUb = endNodeTrackers[j].second;
            int normNodeIdxLb = normNodeTrackers[j].first; int normNodeIdxUb = normNodeTrackers[j].second;
            int midNodeIdxLb = midNodeTrackers[j].first; int midNodeIdxUb = midNodeTrackers[j].second; 

            // Obtain the arc nodes for current LineString
            std::vector<Point3d> arcNodes_;
            std::vector<Point3d> midNodes_;
            arcNodes_.push_back(endNodes_p[endNodeIdxLb]);
            if (normNodeIdxLb != -1 && normNodeIdxUb != -1)
            {
                for (int k=normNodeIdxLb;k<=normNodeIdxUb;k++)
                    arcNodes_.push_back(normNodes_p[k]);
            }
            arcNodes_.push_back(endNodes_p[endNodeIdxUb]);

            for (int k=midNodeIdxLb;k<=midNodeIdxUb;k++)
                midNodes_.push_back(midNodes_p[k]);

            std::vector<Point3d> lsNodes_;
            lsNodes_.push_back(arcNodes_.front());
            for (int k=0;k<midNodes_.size();k++)
            {
                lsNodes_.push_back(midNodes_[k]);
                lsNodes_.push_back(arcNodes_[k+1]);
            }

            for (int k=0;k<LaneletList.size();k++)
            {
                if (LineStringIds[j] == LaneletList[k].leftBound().id())
                {
                    // Current LineString is matched with the left LineString of the k-th Lanelet
                    // Convert the data with lsNodes_
                    LaneletList[k].leftBound().clear();
                    for (Point3d node : lsNodes_)
                        LaneletList[k].leftBound().push_back(node);
                }
                else if(LineStringIds[j] == LaneletList[k].rightBound().id())
                {
                    // Currnet LineString is matched with the right LineString of the k-th Lanelet
                    // Convert the data with lsNodes_
                    LaneletList[k].rightBound().clear();
                    for (Point3d node : lsNodes_)
                        LaneletList[k].rightBound().push_back(node);
                }

            }
        }
    }
}

SingleTripAnalyzer::~SingleTripAnalyzer()
{
    matlab::engine::terminateEngineClient();
}

// Python Bindings
PYBIND11_MODULE(research, m){
    py::class_<SingleTripAnalyzer>(m,"SingleTripAnalyzer")
        .def(py::init<std::vector<int> ,std::vector<int>, std::map<std::string, py::array_t<double>>, py::array_t<int>, py::array_t<double>, py::array_t<double>, py::array_t<double>, py::array_t<double>>())
        .def("addRegulatoryElementStateIdxs", &SingleTripAnalyzer::addRegulatoryElementStateIdxs)
        .def("run", &SingleTripAnalyzer::run)
        .def("showParams", &SingleTripAnalyzer::showParams);
}