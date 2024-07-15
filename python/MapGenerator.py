import os
import sys
import platform
import datetime
import pickle as pkl
import scipy.io as sio
import numpy as np
from natsort import natsorted
import matlab.engine

# import research as m

class MapGenerator:
    """
    Given the date and trip number, MapGenerator module performs automatic data analysis and generate/update lane-level maps.
    
    Goal of this project is to create a mapping framework that supports automatic generation and efficient update of lane-level maps. 
    Possible applications for this project is infrastructure-based ADAS, autonomous driving and etc.

    Jinhwan Jeon, 2024
    """
    def __init__(self,dates,trips,main_file_path):
        self.dates = dates
        self.trips = trips
        self.main_file_path = main_file_path

    def run(self):
        # Need to check if user wants to load map or not.
        if len(self.trips) == 1:
            print("[Single Trip Mode]")
            # Run Single Trip Analysis
            self.SingleTrip(self.dates,self.trips[0])
        else:
            print("[Multiple Trip Mode]")
            for date, idx in enumerate(self.dates):
                for trip in self.trips[idx]:
                    # Add additional functions for multiple trip augmentation (TBD)
                    self.SingleTrip(date,trip)

    def SingleTrip(self,date,tripNo):
        """
        Generating Lanelet2 Map for Single trip. 
        
        """
        # Driving maneuver event detection and classification
        
        TrajEventIntvs = np.array(self.TrajectoryEventDetection(date,tripNo))

        # Regulatory element detection and localization (precomputed data)
        Tf = self.VisionEventDetection(date,tripNo,"TrafficLights")
        St = self.VisionEventDetection(date,tripNo,"StopLines")
        
        # Load loop closure detection (LCD) results and optimized vehicle states
        idxs, matched_idxs, vehicleParams = self.loadData(date,tripNo)

        Tf_Objs = Tf['Objects'] # Tf_Obj.state_idxs
        St_Objs = St['Objects']
        Tf_Pos = Tf['Pos'] # --> Centroid Position
        St_Pos = St['Pos'] # --> Centroid Position
        Tf_DeltaPos = Tf['DeltaPos']
        St_DeltaPos = St['DeltaPos']              

        cpp_module = m.SingleTripAnalyzer(idxs, matched_idxs, vehicleParams, TrajEventIntvs, Tf_Pos, Tf_DeltaPos, St_Pos, St_DeltaPos)

        for i, Tf_Obj in enumerate(Tf_Objs):
            cpp_module.addRegulatoryElementStateIdxs(i,Tf_Obj.state_idxs,"TrafficLights")
        
        for i, St_Obj in enumerate(St_Objs):
            cpp_module.addRegulatoryElementStateIdxs(i,St_Obj.state_idxs,"StopLines")
        
        curr_time = cpp_module.run()
            
        print("current time is : ", curr_time)            

    def TrajectoryEventDetection(self,date,tripNo):
        """
        Driving maneuver event detection and classification

        Run matlab DMDC.m file for event detection and classification

        """
        # Driving Maneuver Detection and Classification
        eng = matlab.engine.start_matlab()
        eng.cd(r'python')
        res = eng.DMDC(date,tripNo)
        eng.quit()
        return res

    def VisionEventDetection(self,date,tripNo,featureType):
        """
        Regulatory element detection and localization is not the main focus of the current repository.
        So we present code for loading pre-computed(via SfM) locations of regulatory elements

        Obtaining the position of regulatory elements will be presented in future works        
        """
        with open(os.path.join(self.main_file_path,date,str(tripNo),featureType+".pkl"),'rb') as f:
            data = pkl.load(f)
        
        return data

    def loadData(self,date,tripNo):
        # Load Vehicle Position
        T = np.array((sio.loadmat(os.path.join(self.main_file_path,"data",date,str(tripNo),"timestamps")))["timestamps"][0])
        R = np.zeros((9,len(T))) 
        P = np.zeros((3,len(T)))
        statesRaw = (sio.loadmat(os.path.join(self.main_file_path,"data",date,str(tripNo),"states")))["states"]
        left = np.zeros((3,len(T)))
        right = np.zeros((3,len(T)))

        for i in range(len(T)):
            R[:,i] = statesRaw[0][i][0][0]['R'].reshape((9,1)).T
            P[:,i] = statesRaw[0][i][0][0]['P'].T
            left_sample = statesRaw[0][i][0][0]['left'][:,0].T
            right_sample = statesRaw[0][i][0][0]['right'][:,0].T
            left[:,i] = statesRaw[0][i][0][0]['P'].T + statesRaw[0][i][0][0]['R'] @ left_sample
            right[:,i] = statesRaw[0][i][0][0]['P'].T + statesRaw[0][i][0][0]['R'] @ right_sample

        vehicleParams = dict(timestamps=T, Orientation=R, Position=P, Left=left, Right=right)
        with open(os.path.join(self.main_file_path,"data",date,str(tripNo),"match.pkl"),'rb') as f:
            match = pkl.load(f)
        idxs = np.where(match > 0)[0]
        match_idxs = []

        for idx in idxs:
            match_idxs.append(int(match[idx]))

        # Load lane point covariance information
        opt = sio.loadmat(os.path.join(self.main_file_path,"data",date,str(tripNo),"opts"))["opts"][0][0]

        leftCov = opt[-1][0][0][0]
        rightCov = opt[-1][0][0][1]
        # rightEdgeCov = opt[-1][0][0][2] 
        vehicleParams["LeftCov"] = leftCov
        vehicleParams["RightCov"] = rightCov
        
        return idxs, match_idxs, vehicleParams
