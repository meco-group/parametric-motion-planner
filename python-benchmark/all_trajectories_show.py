import json
import matplotlib.pyplot as plt
import sys
sys.path.append('build/')
sys.path.append('python-benchmark/')

# Extract the data
# file_name_appendix = ""
# file_name_appendix = "_cell"
file_name_appendix = "_double"
data = json.load(open('python-benchmark/files/all_trajectories' + file_name_appendix + '.json'))

def show_trajectory(px, py, color='royalblue', alpha=1):
    plt.plot(px, py, color=color, alpha=alpha, linewidth=2)
    plt.plot([px[0], px[-1]], [py[0], py[-1]], 'ko', alpha=alpha)

plt.figure()
for px, py in zip(data['px'], data['py']):
    show_trajectory(px, py, alpha=0.7)

plt.gca().set_aspect('equal', adjustable='box')
plt.show()
