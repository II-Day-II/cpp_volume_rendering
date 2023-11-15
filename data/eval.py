from matplotlib import pyplot as plt
import pandas as pd
import numpy as np

d = pd.read_csv('eval.csv', quotechar='"')

plt.plot(d['StepSize'], d['FramesPerSecond'], linestyle='dotted', marker='o')
plt.title('Performance')
plt.ylabel('Frames per Second')
plt.xlabel('Step Size')

plt.show()
