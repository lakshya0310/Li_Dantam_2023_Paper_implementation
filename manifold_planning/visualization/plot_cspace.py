import pandas as pd, matplotlib.pyplot as plt

path=pd.read_csv("../data/path.csv")
plt.plot(path.q1,path.q2,'r-',lw=3)
plt.axis('equal')
plt.title("C-space Path")
plt.show()
