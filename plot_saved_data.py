import numpy as np
import matplotlib.pyplot as plt

data_path = "StoreData/"
data_folder = "4/" # set here the folder for the results you want to run


ticks = []
true = []
myOwn = []
Merged = []

def load_data():
    global ticks,true,myOwn,Merged

    ticks = np.load(data_path+data_folder+"ticks.npy",allow_pickle=False)
    true = np.load(data_path+data_folder+"true.npy",allow_pickle=True)
    myOwn = np.load(data_path+data_folder+"myOwn.npy",allow_pickle=True)
    Merged = np.load(data_path+data_folder+"Merged.npy",allow_pickle=True)



def plotData(ticks,true,myOwn,Merged):
    owncounter = 0
    for item in myOwn:
        if item != []:
            owncounter += 1
    print(owncounter," own datapoints ") 

    mergedcounter = 0
    for item in Merged:
        if item != []:
            mergedcounter += 1
    print(mergedcounter," merged datapoints ")
    # using only own points where also kmerged data is available

    delta_array_own = []
    delta_array_merged = []

    for i in range(0,len(ticks)):
        if Merged[i] == []:
            continue
        else:

            for j in range(len(true[i])):
                if len(Merged[i]) != len(true[i]) :
                    continue
                delta_array_own.append(1 -np.abs(true[i][j][1] - myOwn[i][j][1]))
                delta_array_merged.append(1 -np.abs(true[i][j][1] - Merged[i][j][1]))


    array_np_mean_own = np.mean(delta_array_own)
    array_np_mean_merged = np.mean(delta_array_merged)

    array_np_std_own = np.std(delta_array_own)
    array_np_std_merged = np.std(delta_array_merged)

    xpos = ["Own","Merged"]
    means = [array_np_mean_own,array_np_mean_merged]
    stds = [array_np_std_own,array_np_std_merged]
    print("Mean own: ",array_np_mean_own)
    print("Mean merged: ",array_np_mean_merged)

    print("Std own: ",array_np_std_own)
    print("Std merged: ",array_np_std_merged)

    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.bar(xpos, means,yerr=stds,align="center",ecolor='black',capsize=10)
    #ax1.invert_yaxis()
    ax1.set_ylabel("Accuracy of VRU detected")
    ax1.set_xlabel("Type of recognition")
    ax1.set_title("Comparison of Ergo Vehicle vs Merged VRU prediction for " + str(mergedcounter) + " datapoints")

    plt.show()

def main():
    load_data()
    plotData(ticks,true,myOwn,Merged)

main()