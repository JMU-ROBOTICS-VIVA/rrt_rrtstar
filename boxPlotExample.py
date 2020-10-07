import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt

runTime = np.array([10.2, 11, 6.3])

sns.boxplot(x=runTime)
plt.rcParams["axes.labelsize"] = 16
plt.savefig(fname='boxPlot.pdf', dpi=300,bbox_inches='tight',pad_inches=0.05)
