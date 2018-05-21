import os 
import numpy as np 

path = os.path.dirname(os.path.abspath(__file__))

# logfile = path + '\\log\\gps_logs.txt'
logfile = path + '\\log\\acc_logs.txt'

x_list = []
with open(logfile, 'rt') as f:
   next(f)
   for line in f:    
        _, x = line.strip().split(',')
        # print(x)
        x_list.append(float(x))
# print(len(x_list))
# print(x_list)
X= np.array(x_list, dtype=float)

print("Xmean= {}".format(X.mean())) 
print("Xstd= {}".format(X.std())) 

