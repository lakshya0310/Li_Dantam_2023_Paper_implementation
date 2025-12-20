import pybullet as p, time, pandas as pd
from simulation2 import frankaId

traj=pd.read_csv("../data/path.csv").values
for q in traj:
    p.resetJointState(frankaId,0,q[0])
    p.resetJointState(frankaId,1,q[1])
    p.stepSimulation()
    time.sleep(0.03)
