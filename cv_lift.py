import pandas as pd
import os

#Change file name here
file_name = '220131-213734_backsquat.csv'

df = pd.read_csv(f'CV_Lifts/{file_name}', usecols=['Time', 'X_m', 'Y_m'])

#Set delta to be change threshold in Y dir used to start a lift
maxVal = 0
maxIndex = 0
delta = 0.1
stage = 0
lift_start = 0
lift_end = 0
prev_val = 0

for index, val in enumerate(df['Y_m']):
    #Before lift starts
    if stage == 0:
        if val > maxVal:
            maxVal = val
            maxIndex = index
        else:
            if (maxVal - val) > delta:
                stage = 1
                lift_start = index
    #Lift has started and is moving down
    elif stage == 1:
        if val == 0:
            stage = 2
    #Lift is moving up
    elif stage == 2:
        if prev_val > val:
            #Lift is done
            lift_end = index
            break
        prev_val = val 

#Trim and save csv
lift = df.loc[lift_start:lift_end] 
lift.to_csv(f'CV_lifts_trimed/{file_name}')
