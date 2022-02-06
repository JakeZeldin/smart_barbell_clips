import csv
import os
import numpy as np
import scipy

def main():
    data_dir = os.path.join(".data", "static")
    means = []
    stddevs = []

    for filename in os.listdir(data_dir):
        lin_acc = []
        with open(os.path.join(data_dir, filename), 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                lin_acc.append([float(x) for x in row])
        lin_acc = np.array(lin_acc)
        means.append([np.mean(lin_acc[:,i]) for i in range(3)])
        stddevs.append([np.std(lin_acc[:,i]) for i in range(3)])

    for i in stddevs:
        print(i)

    for i in means:
        print(i)

if __name__ == "__main__":
    main()
