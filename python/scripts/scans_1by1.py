from deltabot import DeltaBot
from deltabot import scan
from deltabot import camera
import csv
# import numpy as np
import pandas as pd
# import tensorflow as tf

from preprocessing import process_the_data

def series_to_supervised(data, n_in=1, n_out=1, dropnan=True):
	"""
	Frame a time series as a supervised learning dataset.
	Arguments:
		data: Sequence of observations as a list or NumPy array.
		n_in: Number of lag observations as input (X).
		n_out: Number of observations as output (y).
		dropnan: Boolean whether or not to drop rows with NaN values.
	Returns:
		Pandas DataFrame of series framed for supervised learning.
	"""
	n_vars = 1 if type(data) is list else data.shape[1]
	df = pd.DataFrame(data)
	cols, names = list(), list()
	# input sequence (t-n, ... t-1)
	for i in range(n_in, 0, -1):
		cols.append(df.shift(i))
		names += [('var%d(t-%d)' % (j+1, i)) for j in range(n_vars)]
	# forecast sequence (t, t+1, ... t+n)
	for i in range(0, n_out):
		cols.append(df.shift(-i))
		if i == 0:
			names += [('var%d(t)' % (j+1)) for j in range(n_vars)]
		else:
			names += [('var%d(t+%d)' % (j+1, i)) for j in range(n_vars)]
	# put it all together
	agg = pd.concat(cols, axis=1)
	agg.columns = names
	# drop rows with NaN values
	if dropnan:
		agg.dropna(inplace=True)
	return agg


with DeltaBot() as bot:  # port="sim" to try it without hardware
    cam = camera.Camera()
    # sc = scan.Scanner(bot, cam, data_dir="scans/data")
    hough = cam.remote.enable_hough(min_radius=4, max_radius=7, param2=20, min_dist=50)

    window_size = 10

    # first 10 steps to get history
    fh = open("data.csv", "w", newline="")
    writer = None
    with fh:
        for y in [0]:
            for x in [x / 10.0 for x in range(window_size+ 1)]:
                pose = bot.move_xy(x, y, scan.Circles())
                # print(bot.positions())

                # print(scan.Circles().read(frame, None))
                frame = cam.acquire(hough=hough)

                row = {"x": pose.x, "y": pose.y}  # need to add cx and cy in here somehow
                row["frame_id"] = frame.frame_id
                steps = bot.positions()
                row.update({"s0": steps[0], "s1": steps[1], "s2": steps[2]})

                row.update(dict(scan.Circles().read(frame, None)))
                if writer is None:
                    writer = csv.DictWriter(fh, fieldnames=list(row), restval="")
                    writer.writeheader()
                    initial_pos = row
                writer.writerow(row)
                fh.flush()


        while(True):
            #convert measured cx and cy values into mm
            data = process_the_data("", "", 0.1 / 11, 0.1 / 11, initial_pos["cx"], initial_pos["cy"])
            # print("data ", data)
            data_processed = data[["cx_mm", "cy_mm", "s0", "s1", "s2"]]
            #convert measured values into a window of history - so neural network knows where it has been
            
            series = series_to_supervised(data_processed, window_size-1, 1)

            print("series \n", series)
            
            inputs = series #.drop(["var1(t)", "var2(t)", "var3(t)", "var4(t)", "var5(t)"], axis=1)

            #gather from user where it should go next
            print("current position: \n", data_processed.iloc[-1])
            next_x = input("x:") 
            next_y = input("y:")

            inputs["var1(t+1)"] = float(next_x)
            inputs["var2(t+1)"] = float(next_y)
            inputs["cx_diff"] = inputs["var1(t+1)"] - inputs["var1(t)"]
            inputs["cy_diff"] = inputs["var2(t+1)"] - inputs["var2(t)"]
            # print(inputs)
            inputs = inputs[1:]
            
            print("inputs: \n", inputs)

            inputs.to_csv(f"window_{window_size}_test.csv")

            # np.savetxt("test.csv", inputs.astype("float16"), delimiter=',')

            #need to scale data
            
            #use values to predict steps
            #model = tf.keras.models.load_model('model.keras') #need to work out how to get tf installed
            
            #need to inverse transform

            pred_s0 =int(input("predicted s0: "))  #[1000,0,0] #model.predict(inputs) #need to work out how to get tf installed - test data for now
            pred_s1 =int(input("predicted s1: ")) 
            pred_s2 =int(input("predicted s2: ")) 
            
            #move bot
            bot.move_steps(pred_s0, pred_s1, pred_s2) #probably will fail formatting 
            
            #measure cx, cy
            frame_new = cam.acquire(hough=hough)
            row["frame_id"] = frame_new.frame_id
            steps = bot.positions()
            row.update({"s0": steps[0], "s1": steps[1], "s2": steps[2]})
            row.update(dict(scan.Circles().read(frame_new, None)))


            #process cx, cy
            writer.writerow(row)
            fh.flush()
            new_data = process_the_data("", "", 0.1 / 11, 0.1 / 11, initial_pos["cx"], initial_pos["cy"])

            #compare!
            print("new position: \n", new_data[["cx_mm", "cy_mm", "s0", "s1", "s2"]]) #.iloc[-1]

    # sc.run([(0, 0)] * 2, scan.Circles(), scan.TrackedCircle(roi_width=280), settle=1.0, name="0,0")


    bot.home
    print(bot.pose(), bot.z(), bot.strain())

