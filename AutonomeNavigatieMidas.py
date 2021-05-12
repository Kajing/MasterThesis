#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import numpy as np
import math
from common_tools_pkg.marvelmind import MarvelmindHedge
import RPi.GPIO as GPIO
from time import sleep
import time
from common_tools_pkg.IOPi import IOPi
from common_tools_pkg.mpu6050 import MPU6050
from scipy.integrate import cumtrapz
from filterpy.kalman import KalmanFilter
from filterpy.common import kinematic_kf
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag
import click
import sys

#######################################################
###  motor variabelen en definities komen hieronder ###
#######################################################


hedge = MarvelmindHedge(tty = "/dev/ttyACM0",baud=500000, adr=None, debug=False)  # create MarvelmindHedge thread
hedge.start()


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
usSpeed = float(160)

def stop():
    print('We stoppen')
    global usMotor_Status

    bus1.write_pin(en_motor_1, 1)
    bus1.write_pin(en_motor_2, 1)
    bus1.write_pin(en_motor_3, 1)

    usMotor_Status = brake
    
    motorGo(1, usMotor_Status, 0)
    motorGo(2, usMotor_Status, 0)

    motorGo(0, usMotor_Status, 0)
    sleep(0.1)

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

    bus1.write_pin(en_motor_1, 1)
    bus1.write_pin(en_motor_2, 1)
    bus1.write_pin(en_motor_3, 1)

    usMotor_Status = CCW
    motorGo(2, usMotor_Status, (usSpeed/8000)*100)
    usMotor_Status = CW
    motorGo(1, usMotor_Status, (usSpeed/8000)*100)
    
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
    motorGo(0, usMotor_Status, (usSpeed*19/8000)*100)
    usMotor_Status = CW
    motorGo(2, usMotor_Status, (usSpeed/8000)*100)
    
    usMotor_Status = CW
    motorGo(1, usMotor_Status, (usSpeed/8000)*100)

def rechts():   
    print('rechts')
    global usSpeed
    global usMotor_Status

    bus1.write_pin(en_motor_1, 1)
    bus1.write_pin(en_motor_2, 1)
    bus1.write_pin(en_motor_3, 1)

    usMotor_Status = CW
    motorGo(0, usMotor_Status, (usSpeed*19/8000)*100)
    usMotor_Status = CCW
    motorGo(2, usMotor_Status, (usSpeed/8000)*100)
    
    usMotor_Status = CCW
    motorGo(1, usMotor_Status, (usSpeed/8000)*100)


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

#######################################################
###### Rotation angle var en def komen hieronder ######
#######################################################

GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP)
global revcount
revcount = 0

def increaserev(channel):
    global revcount
    revcount += 1

GPIO.add_event_detect(10, GPIO.RISING, callback=increaserev)

#######################################################
######    Setup van obstakeldetectie sensoren    ######
#######################################################

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

#######################################################
######            Get MarvelMind pos             ######
#######################################################

def get_mm_position(positionRover):
    global hedge
    positie = hedge.position()
    positionRover = [positie[1]+0.137,positie[2]+0.118]
    return positionRover

#######################################################
######               Kalman Filter               ######
#######################################################

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

#######################################################
######               A* algoritme                ######
#######################################################

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
# Draw a grid
'''
def draw_grid(map, width, height, spacing=2, **kwargs):
    for y in range(height):
        for x in range(width):
            print('%%-%ds' % spacing % draw_tile(map, (x, y), kwargs), end='')
        print()
'''
# Draw a tile
def draw_tile(map, position, kwargs):

    # Get the map value
    value = map.get(position)
    # Check if we should print the path
    if 'path' in kwargs and position in kwargs['path']: value = '+'
    # Check if we should print start point
    if 'start' in kwargs and position == kwargs['start']: value = '@'
    # Check if we should print the goal point
    if 'goal' in kwargs and position == kwargs['goal']: value = '$'
    # Return a tile value
    return value 
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

#######################################################
######            P2P driving met A*             ######
#######################################################

def P2P_aster(listofNextNodes, positionRover, afstand_tot_einde, mapPositions, i_de_positie_in_map):
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
        elif(afstnad[2] == "achteruit"):
            afstand_tot_volgende_punt = positionRover[1]-afstand[1]
        
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

        if(distance > 30):
            if(afstand[2] == "links"):
                links()
            elif(afstand[2] == "rechts"):
                rechts()
            elif(afstand[2] == "vooruit"):
                voorwaarts()
            elif(afstand[2] == "achteruit"):
                achterwaards()

        t0 = time.time()
        aan_het_rijden = True        

        #begin met rijden
        i = 0
        while(not einde_punt_bereikt):
            #check om de 0.5s
            if (time.time() - t0 > 0.2):
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

                if (distance <= 20 and aan_het_rijden == True):
                    stop()
                    aan_het_rijden = False
                    print(distance)
                    positionRover = get_mm_position(positionRover)
                    if(afstand[2] == "vooruit"):
                        #op de map wordt in de x richting alles gemarkeerd op x pos -40 cm tot x pos + 50cm
                        for x in range(int(round((positionRover[0]+0.035)*10, 0)-4), int(round((positionRover[0]+0.035)*10, 0)+5)):
                            #op de map wordt in de y richting alles gemarkeerd van y pos +10 tot y positie + 60 cm
                            for y in range(int(math.floor(positionRover[1]*10)+1), int(math.floor(positionRover[1]*10)+6)):
                                map[(x, y)] = "#"
                        return 2, map
                    elif(afstand[2] == "achteruit"):
                        for x in range(int(round((positionRover[0]+0.055)*10, 0)-4), int(round((positionRover[0]+.055)*10, 0)+5)):
                            for y in range(int(math.ceil(positionRover[1]*10)-6), int(math.ceil(positionRover[1]*10)-1)):
                                map[(x, y)] = "#"
                        return 2, map
                    elif(afstand[2] == "links"):
                        for x in range(int(math.ceil(positionRover[0]*10)-6), int(math.ceil(positionRover[0]*10)-1)):
                            for y in range(int(round((positionRover[1]-0.07)*10, 0)-4), int(round((positionRover[1]-0.07)*10, 0)+5)):
                                map[(x, y)] = "#"
                        return 2, map
                    elif(afstand[2] == "rechts"):
                        for x in range(int(math.floor(positionRover[0]*10)+1), int(math.floor(positionRover[0]*10)+6)):
                            for y in range(int(round((positionRover[1]-0.06)*10, 0)-4), int(round((positionRover[1]-0.06)*10, 0)+5)):
                                map[(x, y)] = "#"
                        return 2, map


                elif (aan_het_rijden == False and distance > 20):
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
                        elif(afstnad[2] == "achteruit"):
                            afstand_tot_volgende_punt = positionRover[1]-afstand[1]
                    if(afstand_i != len(listofNextNodes)-1):
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
                            
                            map[(round((positionRover[0]+0.28-(distance/100))*10, 0), round((positionRover[1]-0.07)*10, 0))] = "#"

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
                            
                            map[(round((positionRover[0]-0.28+(distance/100))*10, 0), round((positionRover[1]-0.06)*10,0))] = "#"

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
                            
                            map[(round((positionRover[0]+0.035)*10, 0), round((positionRover[1]-0.295+(distance/100))*10,0))] = "#"
                        
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
                            
                            map[(round((positionRover[0]+0.055)*10, 0), round((positionRover[1]+0.295-(distance/100))*10,0))] = "#"

                print(time.time()-t5) 
                i += 1
                t0 = time.time()    
            
            if(afstand_tot_volgende_punt <= 0.2):
                stop()
                einde_punt_bereikt = True
                if(afstand_tot_einde <= 0.16):
                    return 1, map
        sleep(2)
        
    return 1, map

#######################################################
######               Bereken path                ######
#######################################################

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
            print("test")
            listofNextNodes[i] = (listofNextNodes[i][0]/10, listofNextNodes[i][1]/10, richting)
    
    print(listofNextNodes)

    return listofNextNodes

#######################################################
######            Recht achteruit Kal            ######
#######################################################

def rijVooruitMetKalman(i_de_positie_in_map, mapPositions):
    # Sla positie rover op

    positionRover = np.array([])
     
    positionRover = get_mm_position(positionRover)

    startposY = positionRover[1]
    startposX = positionRover[0]
    
    tracker = get_tracker(0.2)

    tracker.x = np.array([[positionRover[0], 0, positionRover[1], 0]]).T
    tracker.P = np.eye(4)*500
    
    # Start met timen
    t0 = time.time()
    # nodig voor integratie
    t_data = np.array([0, 0.2])
    
    # start met rijden
    #achterwaards(motor_nummer_initieel_voorwaarts, motor_nummer_initieel_rechtsachter, motor_nummer_initieel_linksachter)

    # we zijn nog niet aan de eindbestemming
    not_at_start_position = True
    distance_to_start_position = 0.5
    i = 0
    # start met rijden tot we aan de bestemming aankomen. 
    while(not_at_start_position):
        
        # check distance van eindpunt om de 0.2 sec
        if (time.time()-t0 >= 0.2):
            
            # get marvelmind position data
            
            positionRover = get_mm_position(positionRover)
            sleep(0.2)
            if i == 12:
                voorwaarts()
            print(positionRover)
            
            #predict, sla laatste prediction op, measurement en update
            tracker.predict()
            prior = tracker.x
            z = np.array([positionRover[0],positionRover[1]])
            z.shape = (2,1)
            tracker.update(z)
 
            #Calculate error
            y = np.abs(z - np.dot(tracker.H,prior))
            dist = np.linalg.norm(y)
            print("ERROR DISTANCE: {}".format(dist))
            #print("kf.x:\n {}".format(kf.x))
            
            #if error too big, reset to latest 
            if dist > 1 and i > 1:
                tracker.x = prior
                print("Error distance: {}".format(dist))
            #if error is small enough, update the distance to the start position
            else:
                distance_to_start_position = 0.5 - np.abs(positionRover[1]-startposY)
                print(distance_to_start_position)
                #print("\nDist to start pos: {}".format(distance_to_start_position))
                #print("\npositionRove[1]: {}".format(positionRover[1]))
                #print("\npostitionY: {}".format(startposY))
            
            #reset t0
            t0 = time.time()
            i += 1
            print(i)
        
        global usSpeed
        if(distance_to_start_position <= 0.2):
            stop()
            not_at_start_position = False
        
        time.sleep(0.1)
    
    return 1

#######################################################
######       Naar volgende punt met Kalman       ######
#######################################################

def rij_naar_volgende_punt(i_de_positie_in_map, mapPositions):
    global revcount
    global map
    global TRIG_a
    global TRIG_l
    global TRIG_r
    global TRIG_v
    global ECHO_a
    global ECHO_l
    global ECHO_r
    global ECHO_v
    
    sleep(5)
    # Sla positie rover op
    positionRover = np.array([])
    positionRover = get_mm_position(positionRover)
    print(positionRover)
    sleep(0.5)

    
    #Instellen van de map
    start = (round(positionRover[0]*10, 0), round(positionRover[1]*10, 0))
    end = (round(mapPositions[i_de_positie_in_map][0]*10, 0), round(mapPositions[i_de_positie_in_map][1]*10, 0))
    path = astar_search(map, start, end)

    print(path)

    #berekent verkort pad
    listofNextNodes = bereken_path(path)

    # Begin met rijden
    afstand_tot_einde = math.sqrt((mapPositions[i_de_positie_in_map][0]-positionRover[0])**2 + (mapPositions[i_de_positie_in_map][1]-positionRover[1])**2)

    sleep(2)
    #rijd tot op 16cm afstand
    while(afstand_tot_einde > 0.16):
        
        res, map = P2P_aster(listofNextNodes, positionRover, afstand_tot_einde, mapPositions, i_de_positie_in_map)
        
       
        if(res == 2):
            positionRover = get_mm_position(positionRover)
            start = (round(positionRover[0]*10, 0), round(positionRover[1]*10, 0))
            path = astar_search(map, start, end)
            listofNextNodes = bereken_path(path)
        elif(res == 1):
            positionRover = get_mm_position(positionRover)
            afstand_tot_einde = math.sqrt((mapPositions[i_de_positie_in_map][0]-positionRover[0])**2 + (mapPositions[i_de_positie_in_map][1]-positionRover[1])**2)
            stop()
            print("hello")
            break
        



    print('op rust komen')
    sleep(5)

    positionRover = get_mm_position(positionRover)
    
    afstand_tot_volgende_pos_x = mapPositions[i_de_positie_in_map][0] - positionRover[0]
    afstand_tot_volgende_pos_y = mapPositions[i_de_positie_in_map][1] - positionRover[1]
    
    print("Afstand tot volgende x positie: ",afstand_tot_volgende_pos_x)
    print("Afstand tot volgende y positie: ",afstand_tot_volgende_pos_y)

    afstands_array = [afstand_tot_volgende_pos_x, afstand_tot_volgende_pos_y]

    afstand_tot_oude_positie = np.zeros(2)

    while(abs(afstand_tot_volgende_pos_x) > 0.05 or abs(afstand_tot_volgende_pos_y) > 0.05):
        
        # check in welke richting we moeten rijden.
        richting_x_pos = False
        richting_y_pos = False

        if (afstand_tot_volgende_pos_x >= 0):
            richting_x_pos = True
        if (afstand_tot_volgende_pos_y >= 0):
            richting_y_pos = True

        pulse_dividers = [0.00434, 0.00265]

        for afstand_i, afstand in enumerate(afstands_array):
            
            afstand_tot_volgende_pos = abs(afstand)
            revcount = 0
            if(afstand_i == 0):
                if(richting_x_pos == True):
                    rechts()
                else:
                    links()
            else:
                if(richting_y_pos == True):
                    voorwaarts()
                else:
                    achterwaards()
            print(math.ceil(afstand_tot_volgende_pos/pulse_dividers[afstand_i]))
            while(revcount < math.ceil(afstand_tot_volgende_pos/pulse_dividers[afstand_i])):
                pass
            stop()
            sleep(2)

        
        positionRover = get_mm_position(positionRover)
        print(positionRover)


        afstand_tot_volgende_pos_x = mapPositions[i_de_positie_in_map][0] - positionRover[0]
        afstand_tot_volgende_pos_y = mapPositions[i_de_positie_in_map][1] - positionRover[1]
        
        print("Afstand tot volgende x positie: ",afstand_tot_volgende_pos_x)
        print("Afstand tot volgende y positie: ",afstand_tot_volgende_pos_y)

        afstands_array = [afstand_tot_volgende_pos_x, afstand_tot_volgende_pos_y]

        sleep(2)
    i_de_positie_in_map += 1

    return 1, map

@click.command()

@click.option("--map", "-m", "map_file", required=True,
    help="Path to txt file of the map to be processed.",
    type=click.Path(exists=True, dir_okay=False, readable=True),
)

@click.option("--coord", "-c", "coord_file", required=True,
    help="Path to txt file of the coordinates to be processed.",
    type=click.Path(exists=True, dir_okay=False, readable=True),
)

@click.option("--output", "-o", "output_file", required=True,
    help="Path to txt file of the coordinates to be processed.",
    type=click.Path(exists=True, dir_okay=False),
)

def main(coord_file, map_file, output_file):
    global map
    """ 
    Reads the coordinates and a map in. Starts to drive to those coordinates with the help of the map. When arriving at the coordinates, it does all sorts of tests.
    """
    try:
        # Initate rospy node
        #pub = rospy.Publisher('chatter', String, queue_size=10)
        #rospy.init_node('talker', anonymous=True)

        # initiate map positions
        #mapPositions = np.array([[1.5, 4.]]) #rechthoekige map met 4 punten
        fileObj = open(coord_file, "r") #opens the file in read mode
        words = fileObj.read().splitlines() #puts the file into an array
        fileObj.close()
        mapPositions = []
        for line in words:
            coordinaat = line.split(",")
            mapPositions.append([float(coordinaat[0]), float(coordinaat[1])])

        print(mapPositions[0])
        # Staten
        INIT_STATE = 1
        MEASURING_STATE = 0
        DRIVING_STATE = 0

        # i'de positie in mapPositions
        i_de_positie_in_map = 0

        # Zijn we op het einde?
        atTheEnd = False
        stop()
        print("sleeping!")
        time.sleep(5)
                
        chars = ['c']
        start = None
        end = None
        width = 0
        height = 0
        # Open a file
        fp = open(map_file, 'r')

        # Loop until there is no more lines
        while len(chars) > 0:
            # Get chars in a line
            chars = [str(i) for i in fp.readline().strip()]
            #print(chars)
            # Calculate the width
            width = len(chars) if width == 0 else width
            # Add chars to map
            for x in range(len(chars)):
                map[(x, height)] = chars[x]
                if(chars[x] == '@'):
                    start = (x, height)
                elif(chars[x] == '$'):
                    end = (x, height)

            # Increase the height of the map
            if(len(chars) > 0):
                height += 1
        # Close the file pointer
        fp.close()
        while(not atTheEnd):
            
            # INIT STATE
            if (INIT_STATE == 1):
                nextState = rijVooruitMetKalman(i_de_positie_in_map, mapPositions)
                INIT_STATE = 0
                if (nextState == 1 and INIT_STATE == 0):
                    stop()
                    DRIVING_STATE = 1
                    
                else:
                    print("Something went wrong during the initialization!")
                    atTheEnd = True
                    stop()
                    
            
            # DRIVING STATE
            
            elif (DRIVING_STATE == 1):
                res, map = rij_naar_volgende_punt(i_de_positie_in_map, mapPositions)
                DRIVING_STATE = 0
                MEASURING_STATE = 1
                atTheEnd = True
                print("DRIVING STATE STOP")
                stop()
                hedge.stop()
            '''
            # MEASURING STATE
            elif (MEASURING_STATE == 1):
                MEASURING_STATE = 0
                DRIVING_STATE = 1
                if(i_de_positie_in_map > 3):
                    atTheEnd = True
            # ABORT, SOMETHING WRONG!!
            elif (INIT_STATE == 0 and MEASURING_STATE == 0 and DRIVING_STATE == 0):
                rospy.loginfo("Something went wrong!")
                atTheEnd = True
                stop()
                break
            '''
        f = open(output_file, "a")
        string = ""
        for y in range(0,84):
            for x in range(0,40):
                string += map[(x, y)]
                if(x == 39):
                    string += "\n"
                    f.write(string)
            string = ""
        f.close()        
        sys.exit()
        hedge.stop()
    except KeyboardInterrupt:
        
        stop()
        hedge.stop()
        print("You used ctrl+c")
        
#######################################################
######                    MAIN                   ######
#######################################################

if __name__ == '__main__':
    main()
    