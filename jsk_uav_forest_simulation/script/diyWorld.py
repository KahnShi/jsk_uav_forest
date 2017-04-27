#!/usr/bin/env python

import time
import sys
import rospy
import math
import tf
import numpy as np

class diyWorld:
    def init(self):
        self.n_tree_pos_ = []
        self.file_world_template_ = open('../gazebo_model/world/world_template.world', 'r')
        self.file_tree_template_ = open('../gazebo_model/world/tree_template.world', 'r')
        self.file_diy_world_ = open('../gazebo_model/world/forest_diy.world', 'w')
        self.tree_template_lines_ = self.file_tree_template_.readlines()

    def writeTree(self, tree_pos):
        tree_lines = []
        for id in range (0, len(self.tree_template_lines_)):
            if id == 1:
                str_pos = ''
                for i in range (0, 6):
                    str_pos += str(tree_pos[i]) + ' '
                str_pose = "      <pose>" + str_pos + "</pose>\n"
                tree_lines.append(str_pose)
            else:
                tree_lines.append(self.tree_template_lines_[id])
        return tree_lines
    
    def writeDiyWorld(self):
        world_template_lines = self.file_world_template_.readlines()
        for line_id_world_template in range(0, len(world_template_lines)):
            self.file_diy_world_.write(world_template_lines[line_id_world_template])
            if line_id_world_template == 72:
                for tree_id in range(0, len(self.n_tree_pos_)):
                    tree_lines = self.writeTree(self.n_tree_pos_[tree_id])
                    for tree_line_id in range(0, len(tree_lines)):
                        self.file_diy_world_.write(tree_lines[tree_line_id])
                        
        print "\n\nfinished\n\n"
