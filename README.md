# To Run the Code 
## open a terminal
```
  cd manifold_planning
  mkdir build 
  cd build
  cmake ..
  make
```
## in another terminal 
```
cd manifold_planning
cd pybullet
python3 sdf_server.py

```
## in the first terminal
```
./planner

```
## in another terminal 
```
cd manifold_planning
python3 visualization/plot_cspace.py
python3 visualization/play_trajectory.py

```
