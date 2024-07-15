import os
root_dir = os.path.dirname(os.path.abspath(__file__))

from MapGenerator import MapGenerator

# date_of_interest = "2023-02-24"
# trip_of_interest = [24]
date_of_interest = "2023-02-05"
trip_of_interest = [8]

Map = MapGenerator(date_of_interest,trip_of_interest,root_dir)
Map.run()

