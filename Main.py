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

import Plotting_Method_of_Joints

# perform the method of joints on a statically
# determinate truss
def MethodOfJoints( input_geometry):
    
    # load the input data
    [nodes, bars] = LoadData(input_geometry)
    
    
    Plotting_Method_of_Joints.PlotStructureData(nodes,bars,"index")
    
    # determine if the truss is statically determinate barring parallel or
    # concurrent reactions
    if not StaticallyDeterminate(nodes,bars):
        sys.exit("Cannot operate on a truss that is not statically determinate")
    
    # Compute reaction forces at the supports from external loads
    ComputeReactions(nodes)
    
    # Check that correct reaction forces computed
    for node in nodes:
        # supports load in the x direction
        if 0 in node.ConstraintType():
            print("Reaction in x at node %d is %f" % (node.idx, node.xforce_reaction))
        # supports load in the x direction
        if 1 in node.ConstraintType():
            print("Reaction in y at node %d is %f." % (node.idx, node.yforce_reaction))
    
    # Iterate through all bars using the method of joints
    # to compute internal member loads
    IterateUsingMethodOfJoints(nodes,bars)
    
    # output computed forces in bars
    for bar in bars:
        val = round(bar.axial_load,2)
        print(bar.idx,val)

    # plotting functionality to visualize indices
    # see the problem statement for how to plot alternative information
    Plotting_Method_of_Joints.PlotStructureData(nodes,bars,"axial")
  
#'Gabled_Howe_Eight_Panel.csv'

#'Modified_Fan_Six_Panel.csv'

#'Scissor_Truss_Eight_Panel.csv'
# Run the plane truss function 
MethodOfJoints('Modified_Fan_Six_Panel.csv')