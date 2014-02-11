#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
## Navigation à partir de la carte en 2d

- Lancer le noeud gmapping:
    * `roslaunch turtlebot_bringup minimal.launch`
    * `roslaunch turtlebot_navigation gmapping_demo.launch`

- Récupérer les images générées en `.pgm`
    
    map stockée dans le topic `/map`

    valeurs possibles pour chaque point:

     - 0 => Espace vide
     - 100 => Inconnu
     - 205 => obstacle

- Récupérer la position du robot dans la carte
- Définir un comportement d'exploration pour cartographier tout l'environnement (tout en évitant les obstacles)

## Kidnapping:

- Le SLAM se fait en fonction de l'odométrie. Soulever le robot pour le déplacer de quelques centimètres suffit à invalider sa carte.
- Après une kidnapping, il faut redémarrer gmapping.


'''


import rospy
from geometry_msgs.msg import Twist
import tf
from tf import TransformListener
import subprocess
import time
from threading import Thread


def readSLAM():
    command = 'rosrun map_server map_saver -f ./current_map'
    subprocess.check_output(command, shell=True)
    f = open('./current_map.pgm', 'r')
    f.readline()
    f.readline()
    line = f.readline()
    values = line.split(' ')
    data = {}
    data['width'] = int(values[0])
    data['height'] = int(values[1])
    f.readline()
    row_data = list(f.readline())
    data['data'] = row_data
    map_data = [ord(d) for d in data['data']]
    data['data'] = map_data
    f.close()
    f = open('./current_map.yaml', 'r')
    f.readline()
    values = f.readline().split(' ')
    data['rez'] = float(values[1])
    line = f.readline()
    values = line.split('[')
    values = values[1].split(']')
    values = values[0].split(', ')
    data['origin'] = {
        'x': float(values[0]), 'y': float(values[1]), 'orientation': float(values[2])}
    return data

my_tf = None

def getSLAMandPosition():
    data = readSLAM()
    t = my_tf.getLatestCommonTime('/base_footprint', '/map')
    position, quaternion = my_tf.lookupTransform('/base_footprint', '/map', t)
    quaternion = (
        quaternion[0],
        quaternion[1],
        quaternion[2],
        quaternion[3])
    euler = tf.transformations.euler_from_quaternion(quaternion)
    output = {
        'x': position[0], 'y': position[1],
        'orientation': euler[2]
    }

    data['position'] = output
    data['position']['x'] = (data['position']['x'] - data['origin']['x']) / data['rez']
    data['position']['y'] = data['height'] - (data['position']['y'] - data['origin']['y']) / data['rez']
    data['position']['orientation'] += data['origin']['orientation']
    return data

def action():
    while True:
        data = getSLAMandPosition()
        data['data'] = None
        print data

        idx = breadth_first_search(data, (data['position']['x'] * data['width']) + data['position']['y'])
        print idx % data['position']['width'], idx // data['position']['width']
        time.sleep(2)


def mark_px(pos, pix):
    f = open('./current_map.pgm', 'r')
    f2 = open('./current_map2.pgm', 'w')
    f2.write(f.readline())
    f2.write(f.readline())
    line = f.readline()
    f2.write(line)

    values = line.split(' ')
    f2.write(f.readline())
    row_data = list(f.readline())
    row_data[pos] = chr(125)
    row_data[pix] = chr(225)
    f2.write(''.join(row_data))
    f.close()
    f2.close()


def find_neighbours(idx, data):
    neighbours = [idx+1, idx-1, idx-data['width'], idx+data['width']]
    # We don't care about checking if the index is on the same row or col, since free space won't be on the edge
    # of the map.
    return [n for n in neighbours if n > 0 and n < len(data['data'])]

def breadth_first_search(data, start_pos):
    point_found = None

    stack = [start_pos]
    explored = []


    while not point_found:
        current_point = stack.pop(0)
        next_pts = find_neighbours(current_point, data)
        
        for tmppt in next_pts:
            if tmppt in explored:
                continue

            if data['data'][tmppt] == 254:
                stack.append(tmppt)
                explored.append(tmppt)
            elif data['data'][tmppt] == 205:
                return tmppt


if __name__ == '__main__':
    rospy.init_node('SLAM', anonymous=True)
    my_tf = TransformListener()
    # t2 = Thread(target=action)
    # # print getSLAMandPosition()
    # t2.setDaemon(True)
    # t2.start()

    data = getSLAMandPosition()
    p = int((data['position']['y'] * data['width']) + data['position']['x'])
    idx = breadth_first_search(data, p)
    mark_px(p, idx)
    data['data'] = None
    print data
    print idx % data['width'], idx // data['width']


    rospy.spin()

