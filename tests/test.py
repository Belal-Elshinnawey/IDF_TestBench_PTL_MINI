from re import L
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

#Channel0:0,Channel3:0,Channel4:0,Channel5:3927,Channel6:0,Channel7:0\n

#initialize serial port
ser = serial.Serial()
ser.port = 'COM6' #Arduino serial port
ser.baudrate = 115200
ser.setDTR(False)
ser.setRTS(False)
ser.open()
if ser.is_open==True:
	print("\nAll right, serial port now open. Configuration:\n")
	print(ser, "\n") #print serial parameters

# Create figure for plotting
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(1, 1, 1)
xs = [] #store trials here (n)
y1 = [] #store values
y2 = [] #store values
y3 = [] #store values
y4 = [] #store values
y5 = [] #store values
y6 = [] #store values

counter = 0
# This function is called periodically from FuncAnimation
def animate(i):

    #Aquire and parse data from serial port
    line=ser.read_until(expected = b'\r\n')     #ascii
    line = line[0:line.index(b'\r\n')]
    #print(line)
    line_as_list = line.split(b',')
    points = []
    #print(line_as_list)
    if len(line_as_list) == 6:

        for ent in line_as_list:
            points.append(int(ent.split(b':')[1]))

 
        xs.append(i)
        y1.append(points[0])
        y2.append(points[1])
        y3.append(points[2])
        y4.append(points[3])
        y5.append(points[4])
        y6.append(points[5])



        ax.clear()
        ax.plot(xs,y1, label="Sensor1")
        ax.plot(xs,y2, label="Sensor2")
        ax.plot(xs,y3, label="Sensor3")
        ax.plot(xs,y4, label="Sensor4")
        ax.plot(xs,y5, label="Sensor5")
        ax.plot(xs,y6, label="Sensor6")
        # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)
        plt.title('Sensor Readings')
        plt.ylabel('Value')
        plt.legend()
        plt.axis([1, None, 0, 1.5e6]) #Use for arbitrary number of trials
        #plt.axis([i-10, i, 0, 1.5e6]) #Use for 100 trial demo

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate,cache_frame_data = False, interval = 0.1, repeat = False)
plt.show()