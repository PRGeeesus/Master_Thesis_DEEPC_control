import random
import numpy as np
import time
import matplotlib.pyplot as plt
# Area of circle = pi*r
# Area of square = 2*2
approximations = []
runtime = 10000
def main():
    iterations = 0
    RUnning = True
    nr_inside_circle = 0
    nr_outside_circle = 0
    while(RUnning):
        iterations += 1
        #time.sleep(0.1)
        pt_x = random.random()
        pt_y = random.random()
        hyp = np.sqrt((pt_x*pt_x) + (pt_y*pt_y))

        if hyp <= 1:
            nr_inside_circle += 1
        if hyp > 1:
            nr_outside_circle += 1
        
        ratio = (nr_inside_circle/(nr_inside_circle+nr_outside_circle))
        pi = ratio * 4
        print("Pi apprioximation: ",pi)
        approximations.append(pi -np.pi)
        if iterations > runtime:
            RUnning = False

main()
plt.plot(approximations)
plt.show()