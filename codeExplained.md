# Code step by step
It is advised to have the full code laying next to you to quickly figure out where the code is situated within the whole code file. 
## Initializations
```python
hedge = MarvelmindHedge(tty = "/dev/ttyACM0",baud=500000, adr=None, debug=False)  # create MarvelmindHedge thread
hedge.start()
```
This will start the marvelmind beacon data acquisition on port ACM0 with a baudrate of 500 000. (highest available). 

```python
brake = 0
CW = 1
CCW = 2

#IOPi
en_motor_1 = 1
motor_A1_pin = 2
motor_B1_pin = 3
en_motor_2 = 4
motor_A2_pin = 5
motor_B2_pin = 6
en_motor_3 = 7
motor_A3_pin = 8
motor_B3_pin = 9
#GPIO
pwm_motor_1 = 12
pwm_motor_2 = 13
pwm_motor_3 = 19
#start variabelen
motor_1 = 0
motor_2 = 1
motor_3 = 2

usMotor_Status = brake

# setting up the pins
GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)

GPIO.setup(pwm_motor_1, GPIO.OUT)
GPIO.setup(pwm_motor_2, GPIO.OUT)
GPIO.setup(pwm_motor_3, GPIO.OUT)

bus1 = IOPi(0x21)
bus1.set_port_direction(0, 0x00)
bus1.set_port_direction(1, 0x00)

pi_pwm1 = GPIO.PWM(pwm_motor_1, 8000)
pi_pwm2 = GPIO.PWM(pwm_motor_2, 8000)
pi_pwm3 = GPIO.PWM(pwm_motor_3, 8000)
pi_pwm1.start(0)
pi_pwm2.start(0)
pi_pwm3.start(0)

bus1.write_pin(en_motor_1, 1)
bus1.write_pin(en_motor_2, 1)
bus1.write_pin(en_motor_3, 1)

# was 245
usSpeed = float(80)
```

Brake, Clock wise (CW) and counter clock wise (CCW) each have a number. They are used for the wheels. Then the pins are configured on the IOPI and each motor gets a number as well. After that, the PWM pins are being set up as well as the pins for the control of the motor driver. usSpeed is the speed of the motor (in a range between 0 and 8000) and usMotor_status is in which status each motor is (Brake, CW, CCW).

## Directions and movements
In the following block of code, each direction in which the rover can move has a definition as well as a definition (motorGo) to make the motors rotate in the right direction as well as with the right speed. 
```python
def stop():
    print('We stoppen')
    global usMotor_Status

    bus1.write_pin(en_motor_1, 1)
    bus1.write_pin(en_motor_2, 1)
    bus1.write_pin(en_motor_3, 1)

    usMotor_Status = brake
    
    
    motorGo(1, usMotor_Status, 0)
    motorGo(0, usMotor_Status, 0)
    motorGo(2, usMotor_Status, 0)
    

def achterwaards():
    print('achterwaarts')
    global usSpeed
    global usMotor_Status

    bus1.write_pin(en_motor_1, 1)
    bus1.write_pin(en_motor_2, 1)
    bus1.write_pin(en_motor_3, 1)

    usMotor_Status = CW
    motorGo(2, usMotor_Status, (usSpeed/8000)*100)
    usMotor_Status = CCW
    motorGo(1, usMotor_Status, (usSpeed/8000)*100)
    
    #disable 3rd motor
    usMotor_Status = brake
    motorGo(0, usMotor_Status, 0)
    bus1.write_pin(en_motor_1, 0)
    
def voorwaarts():
    print('voorwaarts')
    global usSpeed
    global usMotor_Status
    global k

    bus1.write_pin(en_motor_1, 1)
    bus1.write_pin(en_motor_2, 1)
    bus1.write_pin(en_motor_3, 1)

    if(k%2 == 0):
        usMotor_Status = CW
        motorGo(1, usMotor_Status, (1.75*usSpeed/8000)*100)
        usMotor_Status = CCW
        motorGo(2, usMotor_Status, (1.75*usSpeed/8000)*100)
    elif(k%2 ==1):
        usMotor_Status = CCW
        motorGo(2, usMotor_Status, (1.75*usSpeed/8000)*100)
        usMotor_Status = CW
        motorGo(1, usMotor_Status, (1.75*usSpeed/8000)*100)
    k += 1
    #disable 3rd motor
    usMotor_Status = brake
    motorGo(0, usMotor_Status, 0)

    bus1.write_pin(en_motor_1, 0)
    
def links():
    print('links')
    global usSpeed
    global usMotor_Status

    bus1.write_pin(en_motor_1, 1)
    bus1.write_pin(en_motor_2, 1)
    bus1.write_pin(en_motor_3, 1)
    #(usSpeed*1.86478437)
    
    usMotor_Status = CCW
    motorGo(0, usMotor_Status, (usSpeed*70/16000)*100)
    usMotor_Status = CW
    motorGo(2, usMotor_Status, (usSpeed/16000)*100)
    
    usMotor_Status = CW
    motorGo(1, usMotor_Status, (usSpeed/16000)*100)

def rechts():   
    print('rechts')
    global usSpeed
    global usMotor_Status

    bus1.write_pin(en_motor_1, 1)
    bus1.write_pin(en_motor_2, 1)
    bus1.write_pin(en_motor_3, 1)

    usMotor_Status = CW
    motorGo(0, usMotor_Status, (usSpeed*70/16000)*100)
    usMotor_Status = CCW
    motorGo(2, usMotor_Status, (usSpeed/16000)*100)
    
    usMotor_Status = CCW
    motorGo(1, usMotor_Status, (usSpeed/16000)*100)


def versnel():
    print('versnel: ')
    global usSpeed
    global usMotor_Status
    usSpeed = usSpeed + 80
    if (usSpeed > 8000):
        usSpeed = 8000

    print('versnel: ')
    motorGo(motor_1, usMotor_Status, (usSpeed/8000)*100)
    motorGo(motor_2, usMotor_Status, (usSpeed/8000)*100)
    motorGo(motor_3, usMotor_Status, (usSpeed/8000)*100)

def vertraag():
    print('versnel: ')
    global usSpeed
    global usMotor_Status
    usSpeed = usSpeed - 80
    if (usSpeed < 0):
        usSpeed = 0

    print('versnel: ')
    motorGo(motor_1, usMotor_Status, (usSpeed/8000)*100)
    motorGo(motor_2, usMotor_Status, (usSpeed/8000)*100)
    motorGo(motor_3, usMotor_Status, (usSpeed/8000)*100)

def draai_links():
    print('links draaien')
    global usSpeed
    global usMotor_Status
    usMotor_Status = CW

    bus1.write_pin(en_motor_1, 1)
    bus1.write_pin(en_motor_2, 1)
    bus1.write_pin(en_motor_3, 1)

    motorGo(motor_1, usMotor_Status, (usSpeed/8000)*100)
    motorGo(motor_2, usMotor_Status, (usSpeed/8000)*100)
    motorGo(motor_3, usMotor_Status, (usSpeed/8000)*100)

def draai_rechts():
    print('rechts draaien')
    global usSpeed
    global usMotor_Status

    bus1.write_pin(en_motor_1, 1)
    bus1.write_pin(en_motor_2, 1)
    bus1.write_pin(en_motor_3, 1)

    usMotor_Status = CCW
    motorGo(motor_1, usMotor_Status, (usSpeed/8000)*100)
    motorGo(motor_2, usMotor_Status, (usSpeed/8000)*100)
    motorGo(motor_3, usMotor_Status, (usSpeed/8000)*100)

def motorGo(motor, direction, pwm):
    global CW
    global CCW
    if (motor == 0):
        if(direction == CW):
            rospy.loginfo('CW')
            bus1.write_pin(motor_A1_pin, 0)
            bus1.write_pin(motor_B1_pin, 1)
            #GPIO.output(motor_A1_pin, GPIO.LOW)
            #GPIO.output(motor_B1_pin, GPIO.HIGH)
        elif(direction == CCW):
            rospy.loginfo('CCW')
            bus1.write_pin(motor_A1_pin, 1)
            bus1.write_pin(motor_B1_pin, 0)
            #GPIO.output(motor_A1_pin, GPIO.HIGH)
            #GPIO.output(motor_B1_pin, GPIO.LOW)
        else:
            bus1.write_pin(motor_A1_pin, 0)
            bus1.write_pin(motor_B1_pin, 1)
            #GPIO.output(motor_A1_pin, GPIO.LOW)
            #GPIO.output(motor_B1_pin, GPIO.LOW)
        rospy.loginfo(pwm)
        pi_pwm1.ChangeDutyCycle(pwm)
    if (motor == 1):
        if(direction == CW):
            rospy.loginfo('CW')
            bus1.write_pin(motor_A2_pin, 0)
            bus1.write_pin(motor_B2_pin, 1)
            #GPIO.output(motor_A1_pin, GPIO.LOW)
            #GPIO.output(motor_B1_pin, GPIO.HIGH)
        elif(direction == CCW):
            rospy.loginfo('CCW')
            bus1.write_pin(motor_A2_pin, 1)
            bus1.write_pin(motor_B2_pin, 0)
            #GPIO.output(motor_A1_pin, GPIO.HIGH)
            #GPIO.output(motor_B1_pin, GPIO.LOW)
        else:
            bus1.write_pin(motor_A2_pin, 0)
            bus1.write_pin(motor_B2_pin, 1)
            #GPIO.output(motor_A1_pin, GPIO.LOW)
            #GPIO.output(motor_B1_pin, GPIO.LOW)
        rospy.loginfo(pwm)
        pi_pwm2.ChangeDutyCycle(pwm)
    if (motor == 2):
        if(direction == CW):
            rospy.loginfo('CW')
            bus1.write_pin(motor_A3_pin, 0)
            bus1.write_pin(motor_B3_pin, 1)
            #GPIO.output(motor_A1_pin, GPIO.LOW)
            #GPIO.output(motor_B1_pin, GPIO.HIGH)
        elif(direction == CCW):
            rospy.loginfo('CCW')
            bus1.write_pin(motor_A3_pin, 1)
            bus1.write_pin(motor_B3_pin, 0)
            #GPIO.output(motor_A1_pin, GPIO.HIGH)
            #GPIO.output(motor_B1_pin, GPIO.LOW)
        else:
            bus1.write_pin(motor_A3_pin, 0)
            bus1.write_pin(motor_B3_pin, 1)
            #GPIO.output(motor_A1_pin, GPIO.LOW)
            #GPIO.output(motor_B1_pin, GPIO.LOW)
        rospy.loginfo(pwm)
        pi_pwm3.ChangeDutyCycle(pwm)
```
## Setup encoder reading
```python
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP)
global revcount
revcount = 0

def increaserev(channel):
    global revcount
    revcount += 1

GPIO.add_event_detect(10, GPIO.RISING, callback=increaserev)
```
## Setup distance detection sensors
```python
#achteruit
TRIG_a = 23
ECHO_a = 24

#vooruit
TRIG_v = 16
ECHO_v = 26

#links
TRIG_l = 21
ECHO_l = 20

#rechts
TRIG_r = 22
ECHO_r = 27

GPIO.setup(TRIG_a, GPIO.OUT)
GPIO.setup(ECHO_a, GPIO.IN)

GPIO.output(TRIG_a, False)

GPIO.setup(TRIG_v, GPIO.OUT)
GPIO.setup(ECHO_v, GPIO.IN)

GPIO.output(TRIG_v, False)

GPIO.setup(TRIG_l, GPIO.OUT)
GPIO.setup(ECHO_l, GPIO.IN)

GPIO.output(TRIG_l, False)

GPIO.setup(TRIG_r, GPIO.OUT)
GPIO.setup(ECHO_r, GPIO.IN)

GPIO.output(TRIG_r, False)
```
## Marvelmind data
Definition to get the marvelmind position from the beacons.
```python
def get_mm_position(positionRover):
    global hedge
    positie = hedge.position()
    positionRover = [positie[1]+0.137,positie[2]+0.118]
    return positionRover
```
## Kalman filter setup
Makes a kalman filter with the right dimensions.
```python
def get_tracker(dt):
    tracker = KalmanFilter(dim_x = 4, dim_z = 2)
    dt = dt
    tracker.F = np.array([[1, dt, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, dt],
                            [0, 0, 0, 1]])

    q = Q_discrete_white_noise(dim=2, dt=dt, var=0.003)
    tracker.Q = block_diag(q, q)
    tracker.B
    tracker.H = np.array([[1, 0, 0, 0],
                            [0, 0, 1, 0]])

    tracker.R = np.array([[0.04, 0],
                            [0, 0.04]])
    
    return tracker
```
## A* algorithm
This block of code contains the A* algorithm to find the path from start until the end. 
```python
map = {}

class Node:
    # Initialize the class
    def __init__(self, position=(), parent=()):
        self.position = position
        self.parent = parent
        self.g = 0 # Distance to start node
        self.h = 0 # Distance to goal node
        self.f = 0 # Total cost
    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position
    # Sort nodes
    def __lt__(self, other):
         return self.f < other.f
    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))
# A* search
def astar_search(map, start, end):

    # Create lists for open nodes and closed nodes
    open = []
    closed = []
    # Create a start node and an goal node
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Add the start node
    open.append(start_node)

    # Loop until the open list is empty
    while len(open) > 0:
        # Sort the open list to get the node with the lowest cost first
        open.sort()
        # Get the node with the lowest cost
        current_node = open.pop(0)
        # Add the current node to the closed list
        closed.append(current_node)

        # Check if we have reached the goal, return the path
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent
            #path.append(start) 
            # Return reversed path
            return path[::-1]
        # Unzip the current node position
        (x, y) = current_node.position
        # Get neighbors
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        # Loop neighbors
        for next in neighbors:
            # Get value from map
            map_value = map.get(next)
            # Check if the node is a wall
            if(map_value == '#'):
                continue
            # Create a neighbor node
            neighbor = Node(next, current_node)
            # Check if the neighbor is in the closed list
            if(neighbor in closed):
                continue
            # Generate heuristics (Manhattan distance)
            neighbor.g = abs(neighbor.position[0] - start_node.position[0]) + abs(neighbor.position[1] - start_node.position[1])
            neighbor.h = abs(neighbor.position[0] - goal_node.position[0]) + abs(neighbor.position[1] - goal_node.position[1])
            neighbor.f = neighbor.g + neighbor.h
            # Check if neighbor is in open list and if it has a lower f value
            if(add_to_open(open, neighbor) == True):
                # Everything is green, add neighbor to open list
                open.append(neighbor)
    # Return None, no path is found
    return None
# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f >= node.f):
            return False
    return True
```
## Point to point (P2P) driving with A* algorithm
To drive from P2P, the path in between the nodes must be known (listofNextNodes). Then the distance to drive to the first node in the path will be calculated.
```python
global map
global TRIG_a
global TRIG_l
global TRIG_r
global TRIG_v
global ECHO_a
global ECHO_l
global ECHO_r
global ECHO_v
#volg verkort pad
for afstand_i, afstand in enumerate(listofNextNodes):
    print(listofNextNodes)
    afstand_tot_volgende_punt = 0
    positionRover = get_mm_position(positionRover)
    print(positionRover)

    #initiating filter!
    tracker1 = get_tracker(0.2)
    tracker1.x = np.array([[positionRover[0], 0, positionRover[1], 0]]).T
    tracker1.P = np.eye(4)*500

    #bereken afstand tot volgende punt
    if(afstand[2] == "links"):
        afstand_tot_volgende_punt = positionRover[0]-afstand[0]
    elif(afstand[2] == "rechts"):
        afstand_tot_volgende_punt = afstand[0]-positionRover[0]
    elif(afstand[2] == "vooruit"):
        afstand_tot_volgende_punt = afstand[1]-positionRover[1]
    elif(afstand[2] == "achteruit"):
        afstand_tot_volgende_punt = positionRover[1]-afstand[1]
```
Depending on which direction the rover is going to, that direction's distance sensor will be monitored at all times.
```python
 einde_punt_bereikt = False
        #Check of we kunnen rijden
        TRIG = 0
        ECHO = 0

        if(afstand[2] == "links"):
            TRIG = TRIG_l
            ECHO = ECHO_l
        elif(afstand[2] == "rechts"):
            TRIG = TRIG_r
            ECHO = ECHO_r
        elif(afstand[2] == "vooruit"):
            TRIG = TRIG_v
            ECHO = ECHO_v
        elif(afstand[2] == "achteruit"):
            TRIG = TRIG_a
            ECHO = ECHO_a
```
After activating the sensor, a first scan will be executed to calculate how far an object is distanced from the rover. If it's far enough, the rover can start to drive in the desired direction.
```python
GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17900
        distance = round(distance, 1)

        if(distance > 15):
            if(afstand[2] == "links"):
                links()
            elif(afstand[2] == "rechts"):
                rechts()
            elif(afstand[2] == "vooruit"):
                voorwaarts()
            elif(afstand[2] == "achteruit"):
                achterwaards()
```
Whilst driving, we won't stop unless we reach the end or we encounter an obstacle. Data acquisitions are done every 0.2 seconds.
```python
t0 = time.time()
        aan_het_rijden = True        

        #begin met rijden
        i = 0
        closest_distance = 500
        while(not einde_punt_bereikt):
            #check om de 0.5s
            if (time.time() - t0 > 0.2):
```
Whilst driving, the rover checks the distance in the direction it's driving towards. If it finds an object within 50cm of the rover, it'll stop and generate an obstacle on the map. If it isn't driving but the distance is more than 50cm, it can continue to drive in that direction.
```python
t5 = time.time()

                GPIO.output(TRIG, True)

                time.sleep(0.00001)
                GPIO.output(TRIG, False)
                while GPIO.input(ECHO) == 0:
                    pulse_start = time.time()

                while GPIO.input(ECHO) == 1:
                    pulse_end = time.time()
                pulse_duration = pulse_end - pulse_start
                distance = pulse_duration * 17900
                distance = round(distance, 1)

                print("sensor dist", distance)

                if (distance <= 50 and aan_het_rijden == True):
                    stop()
                    aan_het_rijden = False
                    print(distance)
                    positionRover = get_mm_position(positionRover)
                    if(afstand[2] == "vooruit"):
                        #op de map wordt in de x richting alles gemarkeerd op x pos -40 cm tot x pos + 50cm
                        for x in range(int(round((positionRover[0]+0.035)*10, 0)-4), int(round((positionRover[0]+0.035)*10, 0)+5)):
                            #op de map wordt in de y richting alles gemarkeerd van y pos +10 tot y positie + 60 cm
                            for y in range(int(math.floor(positionRover[1]*10)+1), int(math.floor(positionRover[1]*10)+9)):
                                map[(x, y)] = "#"
                        return 2, map
                    elif(afstand[2] == "achteruit"):
                        for x in range(int(round((positionRover[0]+0.055)*10, 0)-4), int(round((positionRover[0]+.055)*10, 0)+5)):
                            for y in range(int(math.ceil(positionRover[1]*10)-9), int(math.ceil(positionRover[1]*10)-1)):
                                map[(x, y)] = "#"
                        return 2, map
                    elif(afstand[2] == "links"):
                        for x in range(int(math.ceil(positionRover[0]*10)-9), int(math.ceil(positionRover[0]*10)-1)):
                            for y in range(int(round((positionRover[1]-0.07)*10, 0)-4), int(round((positionRover[1]-0.07)*10, 0)+5)):
                                map[(x, y)] = "#"
                        return 2, map
                    elif(afstand[2] == "rechts"):
                        for x in range(int(math.floor(positionRover[0]*10)+1), int(math.floor(positionRover[0]*10)+9)):
                            for y in range(int(round((positionRover[1]-0.06)*10, 0)-4), int(round((positionRover[1]-0.06)*10, 0)+5)):
                                map[(x, y)] = "#"
                        return 2, map


                elif (aan_het_rijden == False and distance > 50):
                    if (afstand[2] == "vooruit"):
                        voorwaarts()
                        aan_het_rijden= True
                    elif (afstand[2] == "achteruit"):
                        achterwaards()
                        aan_het_rijden = True
                    elif (afstand[2] == "links"):
                        links()
                        aan_het_rijden = True
                    elif (afstand[2] == "rechts"):
                        rechts()
                        aan_het_rijden = True
```
If the rover's driving, then the kalman filter will be used to examin the incoming data with which the distance to the node will be calculated.
```python
if (aan_het_rijden == True):
    positionRover = get_mm_position(positionRover)
    print(positionRover)
    sleep(0.2)

    #predict, sla laatste prediction op, measurement en update
    tracker1.predict()
    prior = tracker1.x
    z = np.array([positionRover[0],positionRover[1]])
    z.shape = (2,1)
    tracker1.update(z)

    #Calculate error
    y = np.abs(z - np.dot(tracker1.H,prior))
    dist = np.linalg.norm(y)
    print("ERROR DISTANCE: {}".format(dist))
    #print("kf.x:\n {}".format(kf.x))

    #if error too big, reset to latest 
    if dist > 0.6 and i > 4:
        tracker1.x = prior
        print("Error distance: {}".format(dist))
    #if error is small enough, update the distance to the start position
    else:
        afstand_tot_einde = math.sqrt((mapPositions[i_de_positie_in_map][0]-positionRover[0])**2 + (mapPositions[i_de_positie_in_map][1]-positionRover[1])**2)
        if(afstand[2] == "links"):
            afstand_tot_volgende_punt = positionRover[0]-afstand[0]
        elif(afstand[2] == "rechts"):
            afstand_tot_volgende_punt = afstand[0]-positionRover[0]
        elif(afstand[2] == "vooruit"):
            afstand_tot_volgende_punt = afstand[1]-positionRover[1]
        elif(afstand[2] == "achteruit"):
            afstand_tot_volgende_punt = positionRover[1]-afstand[1]
```
Whilst driving in one direction, the rover monitors the distance sensor of the next direction the rover has to take. It can either put obstacles on the map or remove them to make better representations of the objects on the map.
```python
 if(afstand_i != len(listofNextNodes)-1):
    y = 0
    x = 0
    y_array = [-4,-3,-2,-1,0,1,2,3,4]
    x_array = [-4,-3,-2,-1,0,1,2,3,4]
    if(afstand[2] == "vooruit"):
        y = 4
    elif(afstand[2] == "achteruit"):
        y = -4
    elif(afstand[2] == "links"):
        x = -4
    elif(afstand[2] == "rechts"):
        x = 4
    if(listofNextNodes[afstand_i+1][2] == "links"):

        GPIO.output(21, True)
        time.sleep(0.00001)
        GPIO.output(21, False)

        while GPIO.input(20) == 0:
            pulse_start = time.time()

        while GPIO.input(20) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17900
        distance = round(distance, 1)

        if(distance < closest_distance):
            for y_element in y_array:
                map[(int(round((positionRover[0]+0.28-(distance/100))*10, 0)), int(round((positionRover[1]-0.07)*10, 0)+y_element))] = "#"
            closest_distance = distance
        elif(distance > closest_distance +10):
            for x in range(int(math.ceil((positionRover[0]-0.28-(distance/100))*10)), int(math.ceil((positionRover[0])*10))):
                map[(x, int(round((positionRover[1]-0.07)*10, 0)+y))] = "."


    elif(listofNextNodes[afstand_i+1][2] == "rechts"):

        GPIO.output(22, True)
        time.sleep(0.00001)
        GPIO.output(22, False)

        while GPIO.input(27) == 0:
            pulse_start = time.time()

        while GPIO.input(27) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17900
        distance = round(distance, 1)

        if(distance < closest_distance):
            for y_element in y_array:
                map[(int(round((positionRover[0]-0.28+(distance/100))*10, 0)), int(round((positionRover[1]-0.06)*10,0)+y_element))] = "#"
            closest_distance = distance

        elif(distance > closest_distance +10):
            for x in range(int(math.floor((positionRover[0])*10)), int(math.floor((positionRover[0]+0.28+(distance/100))*10))):
                map[(x, int(round((positionRover[1]-0.06)*10, 0)+y))] = "."

    elif(listofNextNodes[afstand_i+1][2] == "vooruit"):

        GPIO.output(16, True)
        time.sleep(0.00001)
        GPIO.output(16, False)

        while GPIO.input(26) == 0:
            pulse_start = time.time()

        while GPIO.input(26) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17900
        distance = round(distance, 1)

        if(distance < closest_distance):
            for x_element in x_array:
                map[(int(round((positionRover[0]+0.035)*10, 0)+x_element), int(round((positionRover[1]-0.295+(distance/100))*10,0)))] = "#"
            closest_distance = distance
        elif(distance > closest_distance +10):
            for y in range(int(math.floor((positionRover[1])*10)), int(math.floor((positionRover[1]+0.295+(distance/100))*10))):
                map[(int(round((positionRover[0]+0.035)*10, 0)+x), y)] = "."

    elif(listofNextNodes[afstand_i+1][2] == "achteruit"):

        GPIO.output(23, True)
        time.sleep(0.00001)
        GPIO.output(23, False)

        while GPIO.input(24) == 0:
            pulse_start = time.time()

        while GPIO.input(24) == 1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17900
        distance = round(distance, 1)

        if(distance < closest_distance):
            for x_element in x_array:
                map[(int(round((positionRover[0]+0.055)*10, 0)+x_element), int(round((positionRover[1]+0.295-(distance/100))*10,0)))] = "#"
            closest_distance = distance
        elif(distance > closest_distance +10):
            for y in range(int(math.ceil((positionRover[1]-0.295-(distance/100))*10)), int(math.ceil((positionRover[1])*10))):
                map[(int(round((positionRover[0]+0.035)*10, 0)+x), y)] = "."

```
## Simplifying the path
The A* algorith determines all the nodes through which the rover has to go through. However, the rover only needs the last nodes in one direction, when the direction changes. Therefore this code is used to only keep certain nodes from the path and add a direction to them.
```python
def bereken_path(path):
    #verkort pad naar enkel nodige punten
    listofNextNodes = [path[0]]
    richting = "nergens"
    direction = np.subtract(path[1], path[0])
    if(direction[0] == -1):
        richting = "links"
    elif(direction[0] == 1):
        richting = "rechts"
    elif(direction[1] == -1):
        richting = "achteruit"
    elif(direction[1] == 1):
        richting = "vooruit"
    print(richting)

    i = 0
    for element in path[1:]:
        richting_controle = "nergens"
        direction = np.subtract(element, listofNextNodes[i])
        if(direction[0] == -1):
            richting_controle = "links"
        elif(direction[0] == 1):
            richting_controle = "rechts"
        elif(direction[1] == -1):
            richting_controle = "achteruit"
        elif(direction[1] == 1):
            richting_controle = "vooruit"
        if(richting_controle == richting):
            listofNextNodes[i] = element
        else:
            listofNextNodes.append(element)
            listofNextNodes[i] = (listofNextNodes[i][0]/10, listofNextNodes[i][1]/10, richting)
            richting = richting_controle
            i += 1


        if(element[0] == path[-1][0] and element[1] == path[-1][1]):
            listofNextNodes[i] = (listofNextNodes[i][0]/10, listofNextNodes[i][1]/10, richting)

    print(listofNextNodes)

    return listofNextNodes
```
