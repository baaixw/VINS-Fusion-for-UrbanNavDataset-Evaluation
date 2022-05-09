_____________________
# VINS-Fusion for UrbanNavDataset Evaluation

## 1. Prerequisites
please refer to VINS-Fusion Github

## 2. Build 
###
    mkdir catkin/src
    cd catkin/src
    mkdir result
    cd catkin/src
    git clone https://github.com/baaixw/catkin_V1
    cd ..
    catkin_make

## 3. UrbanNavDataset
https://github.com/IPNL-POLYU/UrbanNavDataset
## 4. How to run?
###
    cd catkin
    ./all.sh
  important note: remember to change your own path for result and data
## 5. Evaluation results
### Day 20210508 by xiwei: the frames of VIO and Groundtruth are successfully aligned 
### 5.1  Evaluation of Whampoa data:UrbanNav-HK-Deep-Urban-1
- When Left camera is used, set estimate_extrinsic: 0 (Have an accurate extrinsic parameters) and the evaluation result as follows:
###
      max	  15.405519
      mean 	0.681131
      median	0.384471
      min	 0.000118
      rmse	1.142427
      sse	 2004.693594
      std 	0.917169

- Trajectory
<p align="center">
  <img width="712pix" src="result/trajectoryLeft.png">
</p>

- When Stereo camera is used, set estimate_extrinsic: 1,and set td: 0.05 
- Interestingly, we play the bag file starting from second 1 (rosbag  play -s 1 Whampoasensors.bag), then the data can initialize successfully, otherwise fail. This is interesting point, there must be a reason behind that. The evaluation result as follows:
  ###
      max	  2.372637
      mean	0.327725
      median	0.197800
       min	0.000059
      rmse	0.505415
       sse	392.107044
       std	0.384760
  ###

- Trajectory
<p align="center">
  <img width="712pix" src="result/trajectoryStereo.png">
</p>

### 5.2 Evaluation of TST data:UrbanNav-HK-Medium-Urban-1
- When Left camera is used, set estimate_extrinsic: 0 (Have an acurate extrinsic parameters) The evaluation result as follows:

  ### 
      max	4.726060
      mean	0.734495
      median	0.401529
       min	0.000484
      rmse	1.145298
       sse	1029.691026
       std	0.878763
- Trajectory
<p align="center">
  <img width="712pix" src="result/tstMono.png">
</p>

- Stereo camera is used, set estimate_extrinsic: 1,and set td: 0. The evaluation result as follows:
###
       max	2.909680
      mean	0.480456
      median 0.314631
       min	0.000184
      rmse	0.726379
       sse	414.187215
       std	0.544783
- Trajectory
<p align="center">
  <img width="712pix" src="result/trajectoryTSTstereo.png">
</p>

### 5.3 Evaluation of Mongkok data:UrbanNav-HK-Harsh-Urban-1
Fail,  T-T

## 6. Evaluation Tools and command line
- Python package for the evaluation of odometry and SLAM:
github.com/MichaelGrupp/evo
- evo_rpe tum groundTruth.csv vio.csv --plot --plot_mode xyz --save_plot ./VINSplot --save_results ./VINS.zip
## 7. Acknowledgements
#### The VINS-Fusion (https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) framework is used for performance evaluation of dataset collected in Hong Kong urban canyons. When performing coordinate transformation，some functions are derived from GraphGNSSLib (https://github.com/weisongwen/GraphGNSSLib). We appreciate and respect the authors' efforts for their contribution to the research community. If there is any thing inappropriate, please contact me through 19078299r@connect.polyu.hk (BAI).