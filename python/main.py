import sys
import os
root_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(root_dir,".."))

# sys.path.append("../")
# sys.path.append("../Mapping")

from Mapping.MapGeneration.MapGenerator import MapGenerator
from Mapping.MapGeneration.MapReader import MapReader

date_of_interest = "2023-02-24"
trip_of_interest = [24]
# date_of_interest = "2023-02-05"
# trip_of_interest = [8]
Map = MapGenerator(date_of_interest,trip_of_interest,'test')
Map.run()
# # reader = MapReader()

