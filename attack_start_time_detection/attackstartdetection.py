import pandas as pd

def get_threshold_per_val(seg, val):
    output = []
    for j in range(len(seg)-1):
        if j == 0:
            time_old = seg.iloc[j][1]
            val_old = seg.iloc[j][val]
        else:
            # Get variance in time per call
            time_new = seg.iloc[j][1]
            time_dif = time_new/time_old
            time_old = time_new
            # Get variance in time per call
            val_new =  seg.iloc[j][val]
            val_dif = val_new - val_old
            val_old = val_new
            output.append(val_dif)
    variance = sum(output)
    res = abs(variance/(len(output)-1))
    return res

def get_threshold_per_seg(file, segmentsize):
    var_roll = []
    var_pitch = []
    var_yaw = []
    filelen = len(file)
    for i in range(int((filelen-1)/segmentsize)):
        a = i*segmentsize
        b = a + segmentsize
        segment = file[a:b]
        var_roll.append(get_threshold_per_val(segment, "Roll"))
        var_pitch.append(get_threshold_per_val(segment, "Pitch"))
        var_yaw.append(get_threshold_per_val(segment, "Yaw"))
    tresh_roll = max(var_roll)
    tresh_pitch = max(var_pitch)
    tresh_yaw = max(var_yaw)
    res = [tresh_roll,tresh_pitch,tresh_yaw]
    return res

def check_attack_time(file, segmentsize, tresholds):
    filelen = len(file)
    for i in range(int((filelen-1)/segmentsize)):
        a = i*segmentsize
        b = a + segmentsize
        segment = file[a:b]
        var_roll = get_threshold_per_val(segment, "Roll")
        if var_roll > tresholds[0]:
            print("Attack Detected at " + str(segment.iloc[0][0]))
            exit()
        var_pitch = get_threshold_per_val(segment, "Pitch")
        if var_pitch > tresholds[1]:
            print("Attack Detected at " + str(segment.iloc[0][0]))
            exit()
        var_yaw = get_threshold_per_val(segment, "Yaw")
        if var_pitch > tresholds[2]:
            print("Attack Detected at " + str(segment.iloc[0][0]))
            exit()
    print("No Attack detected")

#os.chdir(r"C:\Users\lisa_\Desktop\attackstart\test")
#[i for i in glob.glob('*.{}'.format("csv"))]
df = pd.read_csv('test\mission7new.csv', delimiter = ',')
len_file = len(df)
tresh = get_threshold_per_seg(df, 20)
df2 = pd.read_csv(r'test\allattacks7.csv', delimiter = ',')
check_attack_time(df2, 20, tresh)