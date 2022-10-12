import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams.update({'font.size': 10})

run_time = np.array([10.2, 11, 6.3])

sns.boxplot(x=run_time)
plt.title("Running Time")
plt.xlabel("seconds")

fig = plt.gcf().set_size_inches(4, 2)

plt.savefig(fname='box_plot.pdf', dpi=300, bbox_inches='tight', pad_inches=0.05)
