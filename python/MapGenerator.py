import os
import sys
import platform
import warnings
import datetime
import pickle as pkl
import scipy.io as sio
from scipy.stats.distributions import chi2
import numpy as np
from natsort import natsorted
import matlab.engine

import research as m
from Mapping.EventDetection.VisionBasedDetection.FeatureExtractor import FeatureAnalyzer

root_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(root_dir,".."))

class MapGenerator:
    """
    Given the date and trip number, MapGenerator module performs automatic data analysis and generate/update lane-level maps.
    Further extending the Lanelet2 format, the lane-level map is saved by perform arc-spline approximation.
    Also, MapGenerator can handle multiple trips, updating the landmarks and lane-level map information based on reliability information.

    Goal of this project is to create a mapping framework that supports automatic generation and efficient update of lane-level maps. 
    Possible applications for this project is infrastructure-based ADAS, autonomous driving and etc.

    Jinhwan Jeon, 2024
    """
    def __init__(self,dates,trips,modes):
        self.dates = dates
        self.trips = trips
        self.modes = modes

        # D Drive directory is different for Windows and Linux
        if platform.system() == 'Windows':
            self.header_path = "D:/SJ_Dataset"
        elif platform.system() == 'Linux':
            self.header_path = "/mnt/hdd1/SJ_Dataset"

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
        # ** Important
        # Check if the requested single trip is already performed.
        eng = matlab.engine.start_matlab()

        if not self.loadMapInfo(date,tripNo):
            # Run Sensor Fusion and Trajectory Based Event Detection
            TrajEventIntvs = np.array(self.TrajectoryEventDetection(date,tripNo,eng))
            # print(TrajEventIntvs)
            
            # Run Vision Based Event Detection 
            Tf = self.VisionEventDetection(date,tripNo,"TrafficLights")
            St = self.VisionEventDetection(date,tripNo,"StopLines")
            
            # Load LCD Information
            idxs, matched_idxs, vehicleParams = self.loadData(date,tripNo,eng)

            eng.quit()

            Tf_Objs = Tf['Objects'] # Tf_Obj.state_idxs
            St_Objs = St['Objects']
            Tf_Pos = Tf['Pos'] # --> Centroid Position
            St_Pos = St['Pos'] # --> Centroid Position
            Tf_DeltaPos = Tf['DeltaPos']
            St_DeltaPos = St['DeltaPos']              

            cpp_module = m.SingleTripAnalyzer(idxs, matched_idxs, vehicleParams, TrajEventIntvs, Tf_Pos, Tf_DeltaPos, St_Pos, St_DeltaPos)

            # cpp_module.showParams(0)

            for i, Tf_Obj in enumerate(Tf_Objs):
                cpp_module.addRegulatoryElementStateIdxs(i,Tf_Obj.state_idxs,"TrafficLights")
            
            for i, St_Obj in enumerate(St_Objs):
                cpp_module.addRegulatoryElementStateIdxs(i,St_Obj.state_idxs,"StopLines")
            
            curr_time = cpp_module.run()
            
            print("current time is : ", curr_time)
            # cpp_module.visualize()
            # key_in = input("Single Drive Analysis Finished. Do you want to save map data? [Y/N] : ")
            # if key_in == "Y" or key_in == "y":
            #     Map = generateMap(LaneletList,RegelemList)
            #     self.save(Map,LaneletList,date,tripNo)
        
    def loadMapInfo(self,date,tripNo):
        pkl_folder_path = os.path.join(self.header_path,"MatlabFiles/github/Mapping/MapData/Additional")
        pkl_files = os.listdir(pkl_folder_path)
        flag = False

        for pkl_file in pkl_files:
            if pkl_file[-3:] == "pkl":
                with open(os.path.join(pkl_folder_path,pkl_file), 'rb') as f:
                    pkl_data = pkl.load(f)

                if pkl_data["date"] == date + "-" + str(tripNo):
                    print("Single Trip is already computed for queried trip.")
                    print("Previously computed trip information: ")
                    for key, value in pkl_data["labels"].items():
                        print("Lanelet ID {} : Driving maneuver label {}".format(key,value))

                    key_in = input("Perform Single Trip Analysis over again? [Y/N] : ")
                    if key_in == "N" or key_in == "n":
                        flag =  True
        return flag

    def TrajectoryEventDetection(self,date,tripNo,eng):
        eng.addpath('.')
        res = eng.TED(date,tripNo)
        return res

    def VisionEventDetection(self,date,tripNo,featureType):
        # Check if SfM results can be loaded
        skip_flag = False

        files = os.listdir(os.path.join(self.header_path,"2023",date,"rlog","TripData",str(tripNo)))
        if featureType + ".pkl" in files:
            # Pre-computed file exists. Load file
            skip_flag = True
            key_in = input("Vision based Event Detection Finished for Object: {}. Perform Vision based Event Detection Again? [Y/N]: ".format(featureType))
            if key_in == "Y" or key_in == "y":
                skip_flag = False

        if not skip_flag:
            FA = FeatureAnalyzer(date,tripNo,featureType)
            FA.labelHandler()
            FA.trackHandler()            
            FA.sfmHandler() # LCD is performed inside Sfm
            # SfM result is saved automatically.

        with open(os.path.join(self.header_path,"2023",date,"rlog","TripData",str(tripNo),featureType+".pkl"),'rb') as f:
            data = pkl.load(f)
        
        return data

    def loadData(self,date,tripNo,eng):
        # Load Vehicle Position
        T = np.array((sio.loadmat(os.path.join(self.header_path,"2023",date,"rlog/TripData",str(tripNo),"timestamps")))["timestamps"][0])
        R = np.zeros((9,len(T))) 
        P = np.zeros((3,len(T)))
        statesRaw = (sio.loadmat(os.path.join(self.header_path,"2023",date,"rlog/TripData",str(tripNo),"states")))["states"]
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

        if platform.system() == 'Windows':
            eng.cd(r'..\Mapping\EventDetection\VisionBasedDetection\VBOW')
        elif platform.system() == 'Linux':
            eng.cd(r'Mapping/EventDetection/VisionBasedDetection/VBOW')   

        dataPath = os.path.join(self.header_path,"2023",date,"rlog/TripData",str(tripNo),"images")
        dataFrameRate = 4
        temporalConstant = 40
        images = natsorted(os.listdir(dataPath))
        load_flag = True # Change to False if you don't want to use the pre-computed data
        match = np.array(eng.detectLoop(dataPath,images,dataFrameRate,temporalConstant,vehicleParams["Position"],load_flag,0)) 
        idxs = np.where(match > 0)[0]
        match_idxs = []

        for idx in idxs:
            match_idxs.append(int(match[idx]))

        # Load lane point covariance information
        opt = sio.loadmat(os.path.join(self.header_path,"2023",date,"rlog/TripData",str(tripNo),"opts"))["opts"][0][0]

        leftCov = opt[-1][0][0][0]
        rightCov = opt[-1][0][0][1]
        # rightEdgeCov = opt[-1][0][0][2] 
        vehicleParams["LeftCov"] = leftCov
        vehicleParams["RightCov"] = rightCov
        
        return idxs, match_idxs, vehicleParams
    
    def loadTrack(self,date,tripNo,featureType):
        dataPath = os.path.join(self.header_path,"2023",date,"rlog/TripData",str(tripNo),"labels",featureType)
        
        with open(os.path.join(dataPath,"tracks.pickle"), 'rb') as f:
            track_data = pkl.load(f)
        
        return track_data

    """
    Overlap Detection and Interval Segmentation Handling Part
    
    * Left and Right Overlap Interval extraction is not done yet. They will be similar to "Same Lane" Overlap
    """

    def save(self,Map,LaneletList,date,tripNo):
        print("Saving Map")
        
        # Several attributes ~~
        # .osm is saved as a separate file
        #
        #
        curr_time = datetime.datetime.now()
        text = [str(curr_time.year),str(curr_time.month),str(curr_time.day),str(curr_time.hour),str(curr_time.minute),str(curr_time.second)]
        others_save_path = os.path.join(self.header_path,"MatlabFiles/github/Mapping/MapData/Additional","-".join(text)+".pkl")

        # Save .pkl file

        others = dict()
        # Trip date information
        others["date"] = [date + "-" + str(tripNo)]
        # Lanelet Id tagged with maneuver type
        others["labels"] = dict()
        for labeledLanelet in LaneletList:
            others["labels"][labeledLanelet.id] = labeledLanelet.label

        with open(others_save_path,'wb') as f:
            pkl.dump(others,f,protocol=pkl.HIGHEST_PROTOCOL)
    

if __name__ == '__main__':
    date_of_interest = "2023-02-24"
    trip_of_interest = [24]
    Map = MapGenerator(date_of_interest,trip_of_interest)
    Map.run()
