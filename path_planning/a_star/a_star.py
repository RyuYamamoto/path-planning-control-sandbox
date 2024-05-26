# coding: utf-8

import math
from re import M
from typing import Tuple
import matplotlib.pyplot as plt

class MapGenerator:
  def __init__(self, width=60, height=60) -> None:
    self.width = width
    self.height = height

  # TODO
  def generete_map(self):
    ox, oy = 0, 0

    # initialize map
    map = [ [0 for _ in range(self.width+1)] for _ in range(self.height+1)]

    # create obstacle
    # outer wall
    for ix in range(self.width):
      map[ix][0] = -1
    for iy in range(self.height):
      map[self.width][iy] = -1
    for ix in range(ox, self.width):
      map[self.width - ix][self.height] = -1
    for iy in range(oy, self.height):
      map[ox][self.height - iy] = -1
    # inner wall
    for iy in range(20):
      map[10][iy] = -1
    for iy in range(20):
      map[20][self.height-iy] = -1

    return map

class Node:
  def __init__(self, position, parent, g) -> None:
    self.position = position
    self.parent = parent
    self.g = g
    self.h = 0
    self.f = 0

  # TODO
    self.h = 0
    self.f = 0

  def __eq__(self, other):
    return self.position == other.position

  def calc_cost(self, node_1, node_2):
    self.h = self.calc_heuristic(node_1, node_2)
    self.f= self.h + self.g

  def calc_heuristic(self, node_1: Tuple, node_2: Tuple):
    return math.sqrt((node_1[0] - node_2[0])**2 + (node_1[1] - node_2[1])**2)

class AStar:
  def __init__(self) -> None:
    self.motion = self.get_motion()
    print(self.motion)

  def get_motion(self):
    motion = [
        [1, 0, 1],
        [0, 1, 1],
        [-1, 0, 1],
        [0, -1, 1],
        [1, 1, math.sqrt(2)],
        [1, -1, math.sqrt(2)],
        [-1, 1, math.sqrt(2)],
        [-1, -1, math.sqrt(2)],
    ]
    return motion

  def plan(self, start_x , start_y, goal_x, goal_y):
    start = Node((start_x, start_y), None, 0)
    goal = Node((goal_x, goal_y), None, 0)

    open_list = [start]
    close_list = []

    while True:
      current_index = 0 
      f_min = 100000
      for idx, listed in enumerate(open_list):
        if listed.f < f_min:
          current_node = listed
          f_min = listed.f
          current_index = idx
      close_list.append(current_node)
      plt.plot(current_node.position[0], current_node.position[1], marker='x', color='blue', alpha=0.5)
      plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
      if(len(close_list) % 10 == 0):
       plt.pause(0.001)

      if goal.position == current_node.position:
        print('Goal Reached')
        path = []
        current = current_node
        while current is not None:
          path.append(current.position)
          current = current.parent
        return path[::-1], close_list

      open_list.pop(current_index)
      trace_list = []
      for move in self.motion:
        node = Node(
            (current_node.position[0] + move[0], current_node.position[1] + move[1]), current_node,
            current_node.g + move[2]
        )
        if self.map[node.position[0]][node.position[1]] == -1:
          continue

        trace_list.append(node)

      for listed in trace_list:

        if len([close_node for close_node in close_list if close_node == listed]) > 0:
          continue

        listed.calc_cost(goal.position, listed.position)
  
        if len([open_node for open_node in open_list if listed == open_node and open_node.g < listed.g]) > 0:
          continue

        open_list.append(listed)
      open_list = [i for j, i in enumerate(open_list) if i not in open_list[:j]]

  def set_map(self, map):
    self.map = map

if __name__ == '__main__':
  a_star = AStar()
  map_generator = MapGenerator(width=30, height=30)

  map = map_generator.generete_map()

  fig = plt.figure()
  ax = fig.add_subplot(111)
  ax.set_aspect("equal")
 
  ax.plot(5, 5, marker='*', color='magenta')
  ax.plot(25,25, marker='*', color='green')

  print(len(map))
  print(len(map[0]))

  for y in range(len(map[1])):
    for x in range(len(map[0])):
      if map[x][y] == -1:
        plt.plot(x, y, marker="s", color="black")

  a_star.set_map(map)
  results, close_list = a_star.plan(start_x=5, start_y=5, goal_x=25, goal_y=25)

  results = [i for j, i in enumerate(results) if i not in results[:j]]

  for result in results:
    ix = ax.plot(result[0], result[1], 'xr')
  plt.pause(0.001)
  plt.show()
