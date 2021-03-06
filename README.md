<h1 align='center'> Veronica </h1>
<p align="center">
    <a href="https://github.com/waynerobotics/veronica">
      <img alt="version" src="https://img.shields.io/badge/version-beta_v1.1-blue.svg?style=plastic" />
    </a>
    <a href="https://github.com/waynerobotics/veronica/wiki">
      <img src="https://img.shields.io/badge/veronica-wiki-00f6900.svg?style=plastic" />
    </a>
    <a href="https://github.com/waynerobotics/veronica/issues">
      <img alt="Issues" src="https://img.shields.io/github/issues/waynerobotics/veronica?style=plastic" />
    </a>
 </p>

ROS workspace for the Wayne Robotics team's IGVC '21 robot: Veronica. The repo has has been re-organized into a workspace to contain all the core packages in accordance with the current software architecture [here](https://github.com/waynerobotics/vision/wiki/Software). The previous vision package has also been merged into this repo. 

Sub-modules | About
--- | ---
[Alexa](https://github.com/waynerobotics/alexa) | The robot's Alexa layer that runs roslaunch files from voice commands using Alexa skills and Flask-Ask.
[GUI](https://github.com/waynerobotics/Ego_GUI) | Simple GUI to launch different modes. Uses Tkinter/Qt in Python.
[Motor Driver](https://github.com/waynerobotics/Motor-Drivers) | The node on the Arduino that listens to command velocities and translates to differential-drive PWM output.
[Status Indicator LED Strip](https://github.com/waynerobotics/status_strip) | The RGB LED strip that shows the robot's status / diagnostics codes.
[IGVC Solids](https://github.com/waynerobotics/igvc_solids) | Contains the CAD files of the 3D-Printed cases, machined uprights and others.
[Vision Master](https://github.com/ringo47/vision_master) and [Vision](https://github.com/waynerobotics/vision) | Initial vision workspaces that have been merged into this base repo.


<!--[![version](https://img.shields.io/badge/version-beta-blue.svg?style=plastic)](https://shields.io/)  [![wiki](https://img.shields.io/badge/wiki-passing-blue.svg?style=plastic)](https://github.com/waynerobotics/vision/wiki)  [![GitHub issues](https://img.shields.io/github/issues/waynerobotics/veronica?style=plastic)](https://github.com/waynerobotics/veronica/issues)-->

Software | version
--- | ---
Ubuntu | 18.04
ROS | Melodic
OpenCV | 3.4.1
Python | 3.6

### Before you commit.
- [x] Github recently renamed the 'master' branch to 'main' branch so make sure to commit to the correct branch.
- [x] Add a .gitignore file and add everything except /src and /media.
- [x] The weight files for the neural networks will be large (~500MB for YOLOv3) and exceed file size for regular code commits. Install _Large file Storage_, Git LFS from [here](https://git-lfs.github.com/). 

### Installation
Simply clone and build as follows:
```
git clone https://github.com/waynerobotics/veronica.git && cd veronica/
catkin_make
```

### Software architecture
For the full high level oveview of each mode, go to the [wiki](https://github.com/waynerobotics/veronica/wiki).

The main competition software flow should align with the below diagram.
In competition mode, the robot will be initialized for a full course run. The robot will have access to all its major functions ie(lane keeping, waypoint nav, obstacle avoidance).

[![](https://github.com/waynerobotics/vision/blob/master/media/class_diag.jpg)](https://mermaid-js.github.io/mermaid-live-editor/#/edit/eyJjb2RlIjoiY2xhc3NEaWFncmFtXG4gICAgaW5pdGlhbGl6ZSA8fC0tIFBhcmFtZXRlclNlcnZlclxuICAgIGluaXRpYWxpemUgLS18PiBzdGF0dXNcbiAgICBpbml0aWFsaXplIC0tfD4gbGFuZTJsYXNlclxuICAgIGluaXRpYWxpemUgLS18PiBMaWRhclxuICAgIGxhbmUybGFzZXIgLS18PiBub19tYW5zX2xhbmRcbiAgICBsYW5lMmxhc2VyIC0tfD4gaXJhX21lcmdlclxuICAgIExpZGFyIC0tfD4gaXJhX21lcmdlclxuICAgIGlyYV9tZXJnZXIgLS18PiBHTWFwcGluZ1xuICAgIHdoZWVsX2VuY29kZXIgLS18PiBMb2NhbGl6YXRpb25cbiAgICBMb2NhbGl6YXRpb24gLS18PiBHTWFwcGluZ1xuICAgIG5vX21hbnNfbGFuZCAtLXw-IHdheXBvaW50X2dlbmVyYXRpb25cbiAgICB3YXlwb2ludF9nZW5lcmF0aW9uIC0tfD4gZ2xvYmFsX3BsYW5uZXJcbiAgICBHTWFwcGluZyAtLXw-IGdsb2JhbF9wbGFubmVyXG4gICAgZ2xvYmFsX3BsYW5uZXIgLS18PiBsb2NhbF9wbGFubmVyXG4gICAgbG9jYWxfcGxhbm5lciAtLXw-IGRpZmZfZHJpdmVcbiAgICBcbiAgICByZWNvdmVyX2JlaGF2aW9ycyAtLXw-IGdsb2JhbF9wbGFubmVyXG4gICAgZ2xvYmFsX3BsYW5uZXIgLS18PiByZWNvdmVyX2JlaGF2aW9yc1xuICAgICUld2hlZWxfZW5jb2RlciAtLXw-IHN0YXR1c1xuICAgICUlSU1VIC0tfD4gc3RhdHVzXG4gICAgJSVHUFMgLS18PiBzdGF0dXNcbiAgICB0ZWxlb3AgLS18PiBkaWZmX2RyaXZlXG4gICAgSU1VIC0tfD4gTG9jYWxpemF0aW9uXG4gICAgR1BTIC0tfD4gTG9jYWxpemF0aW9uXG4gICAgXG5cbiAgICBjbGFzcyBzdGF0aWNfdHJhbnNmb3JtX3B1Ymxpc2hlcntcbiAgICAgICAgK3RmIGJhc2VfbGlua190b19JTVVcbiAgICAgICAgK3RmIGJhc2VfbGlua190b19HUFNcbiAgICAgICAgK3RmIGJhc2VfbGlua190b19jYW1lcmFcbiAgICAgICAgK3RmIGNhbWVyYV9saWRhclxuICAgIH1cbiAgICBjbGFzcyBpbml0aWFsaXple1xuICAgICAgICArY2FtZXJhXG4gICAgICAgICtsaWRhclxuICAgIH1cbiAgICBjbGFzcyB3aGVlbF9lbmNvZGVye1xuICAgICAgICArbmF2X21zZ3MvT2RvbWV0cnkgL29kb20xXG4gICAgfVxuICAgIGNsYXNzIFBhcmFtZXRlclNlcnZlcntcbiAgICAgICtnYWluc1xuICAgICAgK2RldmljZUlEc1xuICAgIH1cbiAgICBjbGFzcyBsYW5lMmxhc2Vye1xuICAgICAgICArTGFzZXJTY2FuIC9sYW5lX2xhc2VyXG4gICAgICAgICtsYW5lX2xhc2VyKGltZylcbiAgICB9XG4gICAgY2xhc3MgTGlkYXJ7XG4gICAgICAgICtMYXNlclNjYW4gL2xhc2VyXG4gICAgfVxuICAgIGNsYXNzIGlyYV9tZXJnZXJ7XG4gICAgICAgICtMYXNlclNjYW4gL3NjYW5cbiAgICB9XG4gICAgY2xhc3MgSU1Ve1xuICAgICAgICArc2Vuc29yX21zZ3MvSU1VIC9pbXUxXG4gICAgfVxuICAgIGNsYXNzIEdQU3tcbiAgICAgICAgK3NlbnNvcl9tc2dzL05hdlNhdEZpeCBcbiAgICB9XG4gICAgY2xhc3MgTG9jYWxpemF0aW9ue1xuICAgICAgICArc3RhdGljX3RyYW5zZm9ybSBtYXBfb2RvbV9iYXNlX2xpbmtcbiAgICAgICAgK2V4dGVuZGVkX2thbG1hbl9maWx0ZXIoKVxuICAgIH1cbiAgICBjbGFzcyBHTWFwcGluZ3tcbiAgICAgICAgK25hdl9tc2dzL09jY3VwYW5jeUdyaWQgL21hcFxuICAgIH1cbiAgICBjbGFzcyBzdGF0dXN7XG4gICAgICAgICtib29sIElNVVxuICAgICAgICArYm9vbCBHUFNcbiAgICAgICAgK2Jvb2wgRHJpdmVcbiAgICAgICAgK2Jvb2wgQ2FtZXJhXG4gICAgICAgICtib29sIExpREFSXG4gICAgfVxuICAgIGNsYXNzIGRpZmZfZHJpdmV7XG4gICAgICAgICtQV01cbiAgICB9XG4gICAgY2xhc3MgZ2xvYmFsX3BsYW5uZXJ7XG4gICAgICAgICtuYXZfbXNnc1BhdGggUGF0aFxuICAgIH1cbiAgICBjbGFzcyBsb2NhbF9wbGFubmVye1xuICAgICAgICArVHdpc3QgL2NtZF92ZWxcbiAgICB9XG4gICAgY2xhc3Mgbm9fbWFuc19sYW5ke1xuICAgICAgICArYm9vbCAvc3RhdHVzXG4gICAgfVxuICAgIGNsYXNzIHdheXBvaW50X2dlbmVyYXRpb257XG4gICAgICAgICtwYXRoc1xuICAgIH1cbiAgICBjbGFzcyByZWNvdmVyX2JlaGF2aW9yc3tcbiAgICB9XG4gICAgY2xhc3MgQWxleGF7XG4gICAgfVxuICAgIGNsYXNzIHRlbGVvcHtcbiAgICAgICAgK1R3aXN0IC9jbWRfdmVsXG4gICAgfVxuICAgICAgICAgICAgIiwibWVybWFpZCI6eyJ0aGVtZSI6ImRlZmF1bHQiLCJ0aGVtZVZhcmlhYmxlcyI6eyJiYWNrZ3JvdW5kIjoid2hpdGUiLCJwcmltYXJ5Q29sb3IiOiIjRUNFQ0ZGIiwic2Vjb25kYXJ5Q29sb3IiOiIjZmZmZmRlIiwidGVydGlhcnlDb2xvciI6ImhzbCg4MCwgMTAwJSwgOTYuMjc0NTA5ODAzOSUpIiwicHJpbWFyeUJvcmRlckNvbG9yIjoiaHNsKDI0MCwgNjAlLCA4Ni4yNzQ1MDk4MDM5JSkiLCJzZWNvbmRhcnlCb3JkZXJDb2xvciI6ImhzbCg2MCwgNjAlLCA4My41Mjk0MTE3NjQ3JSkiLCJ0ZXJ0aWFyeUJvcmRlckNvbG9yIjoiaHNsKDgwLCA2MCUsIDg2LjI3NDUwOTgwMzklKSIsInByaW1hcnlUZXh0Q29sb3IiOiIjMTMxMzAwIiwic2Vjb25kYXJ5VGV4dENvbG9yIjoiIzAwMDAyMSIsInRlcnRpYXJ5VGV4dENvbG9yIjoicmdiKDkuNTAwMDAwMDAwMSwgOS41MDAwMDAwMDAxLCA5LjUwMDAwMDAwMDEpIiwibGluZUNvbG9yIjoiIzMzMzMzMyIsInRleHRDb2xvciI6IiMzMzMiLCJtYWluQmtnIjoiI0VDRUNGRiIsInNlY29uZEJrZyI6IiNmZmZmZGUiLCJib3JkZXIxIjoiIzkzNzBEQiIsImJvcmRlcjIiOiIjYWFhYTMzIiwiYXJyb3doZWFkQ29sb3IiOiIjMzMzMzMzIiwiZm9udEZhbWlseSI6IlwidHJlYnVjaGV0IG1zXCIsIHZlcmRhbmEsIGFyaWFsIiwiZm9udFNpemUiOiIxNnB4IiwibGFiZWxCYWNrZ3JvdW5kIjoiI2U4ZThlOCIsIm5vZGVCa2ciOiIjRUNFQ0ZGIiwibm9kZUJvcmRlciI6IiM5MzcwREIiLCJjbHVzdGVyQmtnIjoiI2ZmZmZkZSIsImNsdXN0ZXJCb3JkZXIiOiIjYWFhYTMzIiwiZGVmYXVsdExpbmtDb2xvciI6IiMzMzMzMzMiLCJ0aXRsZUNvbG9yIjoiIzMzMyIsImVkZ2VMYWJlbEJhY2tncm91bmQiOiIjZThlOGU4IiwiYWN0b3JCb3JkZXIiOiJoc2woMjU5LjYyNjE2ODIyNDMsIDU5Ljc3NjUzNjMxMjglLCA4Ny45MDE5NjA3ODQzJSkiLCJhY3RvckJrZyI6IiNFQ0VDRkYiLCJhY3RvclRleHRDb2xvciI6ImJsYWNrIiwiYWN0b3JMaW5lQ29sb3IiOiJncmV5Iiwic2lnbmFsQ29sb3IiOiIjMzMzIiwic2lnbmFsVGV4dENvbG9yIjoiIzMzMyIsImxhYmVsQm94QmtnQ29sb3IiOiIjRUNFQ0ZGIiwibGFiZWxCb3hCb3JkZXJDb2xvciI6ImhzbCgyNTkuNjI2MTY4MjI0MywgNTkuNzc2NTM2MzEyOCUsIDg3LjkwMTk2MDc4NDMlKSIsImxhYmVsVGV4dENvbG9yIjoiYmxhY2siLCJsb29wVGV4dENvbG9yIjoiYmxhY2siLCJub3RlQm9yZGVyQ29sb3IiOiIjYWFhYTMzIiwibm90ZUJrZ0NvbG9yIjoiI2ZmZjVhZCIsIm5vdGVUZXh0Q29sb3IiOiJibGFjayIsImFjdGl2YXRpb25Cb3JkZXJDb2xvciI6IiM2NjYiLCJhY3RpdmF0aW9uQmtnQ29sb3IiOiIjZjRmNGY0Iiwic2VxdWVuY2VOdW1iZXJDb2xvciI6IndoaXRlIiwic2VjdGlvbkJrZ0NvbG9yIjoicmdiYSgxMDIsIDEwMiwgMjU1LCAwLjQ5KSIsImFsdFNlY3Rpb25Ca2dDb2xvciI6IndoaXRlIiwic2VjdGlvbkJrZ0NvbG9yMiI6IiNmZmY0MDAiLCJ0YXNrQm9yZGVyQ29sb3IiOiIjNTM0ZmJjIiwidGFza0JrZ0NvbG9yIjoiIzhhOTBkZCIsInRhc2tUZXh0TGlnaHRDb2xvciI6IndoaXRlIiwidGFza1RleHRDb2xvciI6IndoaXRlIiwidGFza1RleHREYXJrQ29sb3IiOiJibGFjayIsInRhc2tUZXh0T3V0c2lkZUNvbG9yIjoiYmxhY2siLCJ0YXNrVGV4dENsaWNrYWJsZUNvbG9yIjoiIzAwMzE2MyIsImFjdGl2ZVRhc2tCb3JkZXJDb2xvciI6IiM1MzRmYmMiLCJhY3RpdmVUYXNrQmtnQ29sb3IiOiIjYmZjN2ZmIiwiZ3JpZENvbG9yIjoibGlnaHRncmV5IiwiZG9uZVRhc2tCa2dDb2xvciI6ImxpZ2h0Z3JleSIsImRvbmVUYXNrQm9yZGVyQ29sb3IiOiJncmV5IiwiY3JpdEJvcmRlckNvbG9yIjoiI2ZmODg4OCIsImNyaXRCa2dDb2xvciI6InJlZCIsInRvZGF5TGluZUNvbG9yIjoicmVkIiwibGFiZWxDb2xvciI6ImJsYWNrIiwiZXJyb3JCa2dDb2xvciI6IiM1NTIyMjIiLCJlcnJvclRleHRDb2xvciI6IiM1NTIyMjIiLCJjbGFzc1RleHQiOiIjMTMxMzAwIiwiZmlsbFR5cGUwIjoiI0VDRUNGRiIsImZpbGxUeXBlMSI6IiNmZmZmZGUiLCJmaWxsVHlwZTIiOiJoc2woMzA0LCAxMDAlLCA5Ni4yNzQ1MDk4MDM5JSkiLCJmaWxsVHlwZTMiOiJoc2woMTI0LCAxMDAlLCA5My41Mjk0MTE3NjQ3JSkiLCJmaWxsVHlwZTQiOiJoc2woMTc2LCAxMDAlLCA5Ni4yNzQ1MDk4MDM5JSkiLCJmaWxsVHlwZTUiOiJoc2woLTQsIDEwMCUsIDkzLjUyOTQxMTc2NDclKSIsImZpbGxUeXBlNiI6ImhzbCg4LCAxMDAlLCA5Ni4yNzQ1MDk4MDM5JSkiLCJmaWxsVHlwZTciOiJoc2woMTg4LCAxMDAlLCA5My41Mjk0MTE3NjQ3JSkifX0sInVwZGF0ZUVkaXRvciI6ZmFsc2V9)

## Lane detection overview
The current version uses OpenCV and the histogram to calculate lane lines. Below shows the visualization of all the previous steps. The equations of left and right lanes are calculated and the vehicle's relative position is calculated along with the radius of curvatures.
![](https://github.com/waynerobotics/veronica/blob/main/media/pipeline1.png)

After lanes are detected, the lanes are converted to a _sensor_msgs/LaserScan_ using [lane_laser_scan.py](https://github.com/waynerobotics/veronica/blob/main/src/lane_laser_scan/Scripts/lane_laser_scan.py) and merged using [ira_laser_tools](https://imgur.com/a/IiixGOe). The output is a merged scan that includes both lanes and other obstacles in one LaserScan message as show in the image below.
![Merged laser scan in rviz](https://i.imgur.com/eJWnvas.png)


