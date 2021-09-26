import matplotlib.pyplot as plt
from martinCarlaLibrary import EgoVehicle
import numpy as np

E = EgoVehicle(None,None,detectionDistance=30,reachDistance=20)

dist = range(0,30)

def calculateMean(distance,vehicle,alpha=0.5):
    if vehicle.detectionDistance != 0:
        facor = distance / vehicle.detectionDistance
    else:
        facor = 1
    facor = 1- facor*alpha
    # 0.85 when really far
    # about 1 when really close
    precision_detection = np.abs(np.random.normal(loc=facor, scale=0, size=None))
    if precision_detection>1.0:
        precision_detection = 1-np.abs((1-precision_detection))
        print("ERRRRRRRRRRRRRRROR")
    return precision_detection


def main():
    y = []
    e = []
    certainty = 0.02
    for d in dist:
        y.append(E.get_detection_accuaracy(d,certainty=certainty))
        e.append(calculateMean(d,E))
    print(dist)
    print(y)

    plt.scatter(dist,y)
    #plt.errorbar(dist, y, certainty, linestyle='None', marker='^')
    plt.plot(dist,e,c="grey")
    plt.xlabel("Distance d")
    plt.ylabel("Prediction level a")
    plt.legend(['Mean','VRU Prediction'],loc='upper right')
    plt.title("Example of VRU Prediction Simulation")
    plt.show()

if __name__ == '__main__':
    main()
