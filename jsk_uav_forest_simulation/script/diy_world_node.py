#!/usr/bin/env python

import sys
import diyWorld

if __name__ == '__main__':
    my_world = diyWorld.diyWorld()
    my_world.init()
    # example
    my_world.n_tree_pos_.append([0, 0, 0, 0, 0, 0])
    my_world.n_tree_pos_.append([3, 0, 0, 0, 0, 0])
    my_world.n_tree_pos_.append([6, 0, 0, 0, 0, 0])
    my_world.writeDiyWorld()
    print "quit"
