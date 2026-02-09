#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 09:12:03 2021

@author: kendrick shepherd
"""

import sys
from Classes import Node
from Classes import Bar

def LoadData(input_geometry):
    nodedata = []
    bardata = []
    node_idx = 0
    bar_idx = 0
    nodeflag = False
    barflag = False
    # ensure that the input file is a CSV file
    if(input_geometry.split('.')[-1] !='csv'):
        print("Input must be a csv file. Please input a csv file with file extension .csv included in the end of the input file name.")
        sys.exit()
    # open the file
    with open(input_geometry, 'r') as f:
        # Iterate through all lines
        for line in f:
            # otherwise, split the line
            splitline = line.split();
            commaline = line.split(',')
            if(commaline[0].lower().strip()=='nodes'):
                barflag = False
                nodeflag = True
                continue
            elif(commaline[0].lower().strip()=='beams' or commaline[0].lower().strip()=='bars'):
                barflag = True
                nodeflag = False
                continue
            # Skip headers
            elif(commaline[0].lower().strip()=='index'):
                continue
            
            # node is flagged
            if(nodeflag):
                tempnode = Node(node_idx)
                
                tempnode.AddListIdx(int(commaline[0]))
                tempnode.AddLocation([float(commaline[1]), float(commaline[2])])
                tempnode.AddConstraint(commaline[3])
                tempnode.AddExternalXForce(float(commaline[4]))
                tempnode.AddExternalYForce(float(commaline[5]))
                
                node_idx += 1
                nodedata.append(tempnode)
            elif(barflag):
                tempbeam = Bar(bar_idx)
                
                tempbeam.AddNodeListIdxs([int(commaline[1]),int(commaline[2])])
                
                bar_idx += 1
                bardata.append(tempbeam)
                    
    # create dictionary between list indices and their nodes
    list_idx_to_node = {}
    
    # print data            
    for node in nodedata:
        list_idx_to_node.update({node.list_idx: node})
    for bar in bardata:        
        bar.AddInitNode(list_idx_to_node[bar.init_node_list_idx])
        bar.AddEndNode(list_idx_to_node[bar.end_node_list_idx])
        
        list_idx_to_node[bar.init_node_list_idx].AppendToBars(bar)
        list_idx_to_node[bar.end_node_list_idx].AppendToBars(bar)
    
    # do not add moment dofs when all bars incident with a node 
    # are truss elements
    for node in nodedata:
        node.SetNoMoment()
    
    return [nodedata, bardata]