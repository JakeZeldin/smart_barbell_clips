import csv
import os
import glob
import numpy as np
import scipy
import matplotlib.pyplot as plt


def main():
    data_dir = os.path.join(".data", "static")
    means = []
    stddevs = []

    count = 0
    for filename in glob.glob(os.path.join(data_dir,
                                 "static_test_C4_senfusion_*.csv")):
        lin_acc = []
        with open(os.path.join(filename), 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                lin_acc.append([float(x) for x in row])
        lin_acc = np.array(lin_acc)
        means.append([np.mean(lin_acc[:,i]) for i in range(3)])
        stddevs.append([np.std(lin_acc[:,i]) for i in range(3)])
        count += 1

    for i in stddevs:
        print(i)

    for i in means:
        print(i)

    print(count)
    stddevs = np.array(stddevs)
    means = np.array(means)

    for i in range(3):
        plt.plot(means[:,i])

    plt.show()

    for i in range(3):
        plt.plot(lin_acc[:,i])

    plt.show()


if __name__ == "__main__":
    main()
