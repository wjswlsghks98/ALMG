# File Structure
Below is the internal file structure and its details.

```
|
|- +model_CNN/+ops
|- Mapping/EventDetection/VisionBasedDetection   : Dummy python script for reading .pkl files of pre-computed regulatory elements
|
|- data
      |- 2023-02-05/8
               |- RTC.mat                        : Pre-computed arc spline approximation result of vehicle trajectory
               |- StopLines.pkl                  : Pre-computed Structure from Motion(SfM) results for StopLines
               |- TrafficLights.pkl              : Pre-computed Structure from Motion(SfM) results for TrafficLights
               |- match.pkl                      : Loop Closure Detection(LCD) results
               |- opts.mat                       : Pre-computed vehicle state sensor fusion results
               |- states.mat                     : Some states extracted from opts.mat
               |- timestamps.mat                 : Unix timestamp of trip
      |
      |- 2023-02-24/24                           : Data inside this folder is the same as folder 2023-02-05/8
|
|- ALMG.cpython-38-x86_64-linux-gnu.so           : C++ library generated after building this repository (Do not delete)
|
|- DMDC.m                                        : Load RTC.mat and perform classification of driving scenarios by calling RoadTypeClassifier2.m
|
|- MapGenerator.py                               : Backbone module for calling functions for map generation
|
|- RoadTypeClassifier2.m                         : Module for driving maneuver event classification
|
|- main.py                                       : Main python script for running our automatic lane map generation framework

```
