#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 12:37:32 2021

@author: kendrick shepherd
"""

import sys

import Geometry_Operations as geom

# Determine the unknown bars next to this node
def UnknownBars(node):
    list_of_unknown_bars = []
    for my_bar in node.bars:
        if my_bar.is_computed == False:
            list_of_unknown_bars.append(my_bar)
    return list_of_unknown_bars

# Determine if a node if "viable" or not
def NodeIsViable(node):
    # A node is viable for the Method of Joints if it has 1 or 2 unknown members
    unknowns = UnknownBars(node)
    if len(unknowns) >= 1 and len(unknowns) <= 2:
        return True
    return False
    
# Compute unknown force in bar due to sum of the
# forces in the x direction (Local X is aligned with local_x_bar)
def SumOfForcesInLocalX(node, local_x_bar):
    # This function solves for the force in 'local_x_bar'
    # Equation: Force_bar + Sum(Other_Forces * cos(theta)) = 0
    
    # 1. Get the vector for the local x bar (reference axis)
    local_x_vec = geom.BarNodeToVector(node, local_x_bar)
    
    sum_forces = 0.0
    
    # 2. Add contributions from External/Reaction Forces
    # We project the global X force and global Y force onto the local X vector
    f_net_x = node.GetNetXForce()
    f_net_y = node.GetNetYForce()
    
    # Project Global X (1,0) onto Local X
    sum_forces += f_net_x * geom.CosineVectors(local_x_vec, [1, 0])
    # Project Global Y (0,1) onto Local X
    sum_forces += f_net_y * geom.CosineVectors(local_x_vec, [0, 1])
    
    # 3. Add contributions from all OTHER bars connected to this node
    for bar in node.bars:
        if bar != local_x_bar:
            # All other bars effectively must be known (or calculated just prior)
            # Check if computed or if it's the 2nd unknown we just solved
            if bar.is_computed: 
                # Get vector of the other bar
                other_vec = geom.BarNodeToVector(node, bar)
                # Add component: Force * cos(theta)
                sum_forces += bar.axial_load * geom.CosineVectors(local_x_vec, other_vec)
                
    local_x_bar.axial_load = -sum_forces
    local_x_bar.is_computed = True
    # 4. Solve for the local_x_bar force
    # F_local + sum_forces = 0  =>  F_local = -sum_forces
    return -sum_forces

# Compute unknown force in bar due to sum of the 
# forces in the y direction (Local Y is perpendicular to unknown_bars[0])
def SumOfForcesInLocalY(node, unknown_bars):
    # This function solves for the force in unknown_bars[1]
    # We set our Local X axis to unknown_bars[0]. 
    # By summing forces in Y (perpendicular to Bar 0), Bar 0's force is eliminated.
    
    bar_0 = unknown_bars[0]     # Our Reference Axis (Local X)
    bar_target = unknown_bars[1] # The bar we are solving for
    
    local_x_vec = geom.BarNodeToVector(node, bar_0)
    
    sum_known_forces = 0.0
    
    # 1. Add External/Reaction Forces (Projected onto Local Y using Sine)
    f_net_x = node.GetNetXForce()
    f_net_y = node.GetNetYForce()
    
    # Project Global X (1,0) onto Local Y using SineVectors
    sum_known_forces += f_net_x * geom.SineVectors(local_x_vec, [1, 0])
    sum_known_forces += f_net_y * geom.SineVectors(local_x_vec, [0, 1])
    
    # 2. Add contributions from KNOWN bars
    for bar in node.bars:
        if bar.is_computed:
            other_vec = geom.BarNodeToVector(node, bar)
            sum_known_forces += bar.axial_load * geom.SineVectors(local_x_vec, other_vec)
            
            
    # 3. Solve for the target bar
    # F_target * sin(theta_target) + sum_knowns = 0
    # F_target = -sum_knowns / sin(theta_target)
    
    target_vec = geom.BarNodeToVector(node, bar_target)
    sin_theta = geom.SineVectors(local_x_vec, target_vec)
    
    if abs(sin_theta) < 1e-9:
        sys.exit(f"Error: Bars at node {node.idx} are collinear or invalid geometry.")

    bar_target.axial_load = -sum_known_forces / sin_theta
    bar_target.is_computed = True
        
    return bar_target.axial_load

# check if any member has an unknown force. if so, return true
# nodes is a list of all the nodes in my truss
def DoIhaveAnUnknownMember(nodes):
    for node in nodes:
        unknown_bars = UnknownBars(node)
        if len(unknown_bars) > 0:
            return True
    return False

# Perform the method of joints on the structure
def IterateUsingMethodOfJoints(nodes, bars):
    counter = 0
    # While there are still bars with unknown forces...
    while DoIhaveAnUnknownMember(nodes) == True:
        
        progress_made = False
        
        # Iterate through all nodes to find a "viable" one
        for node in nodes:
            if NodeIsViable(node):
                unknowns = UnknownBars(node)
                
                # CASE 1: Two Unknown Bars
                if len(unknowns) == 2:
                    # Solve for the 2nd bar first (using Sum Forces Y relative to 1st bar)
                    force_2 = SumOfForcesInLocalY(node, unknowns)
                    unknowns[1].SetAxialLoad(force_2)
                    unknowns[1].is_computed = True
                    
                    # Now that bar 2 is known, solve for bar 1 (using Sum Forces X along bar 1)
                    force_1 = SumOfForcesInLocalX(node, unknowns[0])
                    unknowns[0].SetAxialLoad(force_1)
                    unknowns[0].is_computed = True
                    
                    progress_made = True
                
                # CASE 2: One Unknown Bar
                elif len(unknowns) == 1:
                    # Solve directly by summing forces along the bar's axis
                    force = SumOfForcesInLocalX(node, unknowns[0])
                    unknowns[0].SetAxialLoad(force)
                    unknowns[0].is_computed = True
                    
                    progress_made = True
        
        # Check for infinite loops (truss stability issues or isolated islands)
        if not progress_made:
            sys.exit("Stuck! No viable nodes found. The truss may be unstable or statically indeterminate.")

        counter += 1
        # Safety break if iterations exceed realistic bounds (number of bars)
        if counter > len(bars) + 5:
            print("Too many iterations")
            return
            
    return