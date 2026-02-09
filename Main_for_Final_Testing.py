#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 08:51:30 2021

@author: kendrick shepherd
"""

import sys

from ImportCSVData import LoadData

from Method_of_Joints import IterateUsingMethodOfJoints
from Structure_Operations import StaticallyDeterminate
from Structure_Operations import ComputeReactions


# from Master.Method_of_Joints import IterateUsingMethodOfJoints
# from Master.Structure_Operations import StaticallyDeterminate
# from Master.Structure_Operations import ComputeReactions

# perform the method of joints on a statically
# determinate truss
def MethodOfJoints( input_geometry):
    
    # load the input data
    [nodes, bars] = LoadCSV(input_geometry)
    
    # determine if the truss is statically determinate barring parallel or
    # concurrent reactions
    if not StaticallyDeterminate(nodes,bars):
        sys.exit("Cannot operate on a truss that is not statically determinate")
    
    # Compute reaction forces at the supports from external loads
    ComputeReactions(nodes)
    
    # Iterate through all bars using the method of joints
    # to compute internal member loads
    IterateUsingMethodOfJoints(nodes,bars)
    
    # return the answer
    return [nodes, bars]

def LoadCSV(input_geometry): 
    # load the input data only
    [nodes, bars] = LoadData(input_geometry)
    return nodes,bars

def LoadAndComputeReactions(input_geometry):
    # load the input data
    [nodes, bars] = LoadCSV(input_geometry)
    
    # determine if the truss is statically determinate barring parallel or
    # concurrent reactions
    if not StaticallyDeterminate(nodes,bars):
        sys.exit("Cannot operate on a truss that is not statically determinate")
    
    # Compute reaction forces at the supports from external loads
    ComputeReactions(nodes)

    return nodes,bars
