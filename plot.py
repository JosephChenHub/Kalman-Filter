import matplotlib.pyplot as plt


pred = []
with open("predict.txt", "r") as fin:
    line = fin.readlines()[0]
    line = line.rstrip()[:-1].split(",")
    pred = [float(e) for e in line]

step, r_dis, m_dis = [], [], []
with open("data.txt", "r") as fin:
    line = fin.readlines()
    tmp = line[0].rstrip().split(",")
    step = [float(e) for e in tmp]
    tmp = line[2].rstrip().split(",")
    r_dis = [float(e) for e in tmp]
    tmp = line[3].rstrip().split(",")
    m_dis = [float(e) for e in tmp]


plt.plot(step, r_dis, 'b', label="real")
plt.plot(step, m_dis, 'g', label="measure")
plt.plot(step, pred, 'r', label="optimal")

plt.legend()
plt.show()
