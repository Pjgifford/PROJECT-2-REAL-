#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 14 11:10:25 2021

@author: kendrick shepherd
"""
import math
import sys
import numpy as np

# Node member information
class Node:
    
    def __init__(self, idx):
        self.idx = idx
        self.location = []
        self.constraint = 'none'
        self.xforce_external = 0
        self.yforce_external = 0
        self.xforce_reaction = float("NAN")
        self.yforce_reaction = float("NAN")
        self.bars = []
        self.accepts_moment = False
    
    def AddListIdx(self, list_idx):
        self.list_idx = list_idx
                   
    def AddLocation(self, location):
        self.location = location
        
    def AddConstraint(self, constraint):
        self.constraint = constraint
    
    def AddExternalXForce(self, xforce):
        self.xforce_external = xforce
        
    def AddExternalYForce(self, yforce):
        self.yforce_external = yforce
                    
    def AddReactionXForce(self, xforce):
        if(0 in self.ConstraintType()):
            self.xforce_reaction = xforce
        else:
            sys.exit("Cannot append reaction force in x when constraint %s cannot support it" % self.constraint)
        
    def AddReactionYForce(self, yforce):
        if(1 in self.ConstraintType()):
            self.yforce_reaction = yforce
        else:
            sys.exit("Cannot append reaction force in y when constraint %s cannot support it" % self.constraint)

    def AppendToBars(self, beam):
        self.bars.append(beam)
                
    def SetNoMoment(self):
        self.accepts_moment = False
        
    def ConstraintType(self):
        # -1 if incompatible, 0 if x, 1 if y, 2 if moment
        if(self.constraint.lower() == 'none' or self.constraint.lower() == ''):
            return []
        elif(self.constraint.lower() == 'roller_no_xdisp'):
            return [0]
        elif(self.constraint.lower() == 'roller_no_ydisp'):
            return [1]
        elif(self.constraint.lower() == 'adisp'):
            return [-1] # cannot operate in generic frame of reference at the moment
        elif(self.constraint.lower() == 'moment'):
            return [2]
        elif(self.constraint.lower() == 'pin'):
            return [0, 1]
        elif(self.constraint.lower() == 'xdispmoment'):
            return [0, 2]
        elif(self.constraint.lower() == 'ydispmoment'):
             return [1, 2]
        elif(self.constraint.lower() == 'adispmoment'):
            return [-1, 2]
        elif(self.constraint.lower() == 'fixed'):
            return [0, 1, 2]
        else: # the current constraint type is not defined
            return [-1]
        
    def GetNetXForce(self):
        if(0 in self.ConstraintType() and np.isnan(self.xforce_reaction)):
            sys.exit("Cannot compute net x force without resolved x reaction force")
        else:
            if(0 in self.ConstraintType()):
                return self.xforce_external + self.xforce_reaction
            else:
                return self.xforce_external
    
    def GetNetYForce(self):
        if(1 in self.ConstraintType() and np.isnan(self.yforce_reaction)):
            sys.exit("Cannot compute net y force without resolved y reaction force")
        else:
            if(1 in self.ConstraintType()):
                return self.yforce_external + self.yforce_reaction
            else:
                return self.yforce_external

    def Print(self):
        print('NodeIdx = ', self.idx)
        print('Location = ', self.location)
        print('Constraint = ', self.constraint)
        print('X Force = ', self.xforce)
        print('Y Force = ', self.yforce)
        
        if(0 in self.ConstraintType()):
            print('Reaction X = ', self.xforce_reaction)
        if(1 in self.ConstraintType()):
            print('Reaction Y = ', self.yforce_reaction)
        print('')

# Beam member information
class Bar:
    
    def __init__(self, idx):
        self.idx = idx
        self.init_node_list_idx = -1
        self.end_node_list_idx = -1
        self.init_node = Node(-1)
        self.end_node = Node(-1)
        self.axial_load = float("NAN")
        self.is_computed = False
        
    def AddNodeListIdxs(self, list_idxs):
        self.init_node_list_idx = list_idxs[0]
        self.end_node_list_idx = list_idxs[1]
                
    def AddInitNode(self, init_node):
        self.init_node = init_node
        
    def AddEndNode(self, end_node):
        self.end_node = end_node
                
    def SetAxialLoad(self, force):
        self.axial_load = force
        
    def Print(self):
        print('BarIdx = ', self.idx)
        print('ListIdx Nodes = ', self.init_node_list_idx, ', ', self.end_node_list_idx)
        print('Axial load is ', self.axial_load)
        print('')