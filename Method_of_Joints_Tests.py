#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb  6 12:21:14 2025

@author: kendrickshepherd
"""

import Main_for_Final_Testing as Main
# import Master.Method_of_Joints as moj
import Method_of_Joints as moj

import unittest

class TestStructureOperations(unittest.TestCase):

    def test_Example_3_2_Reactions(self):
        nodes,bars = Main.MethodOfJoints("Example_3_2.csv")
        decimal_place = 3
        
        self.assertAlmostEqual(0, nodes[0].xforce_reaction, decimal_place)
        self.assertAlmostEqual(4, nodes[3].yforce_reaction, decimal_place)
        self.assertAlmostEqual(4, nodes[3].yforce_reaction, decimal_place)

    def test_Example_3_3_Reactions(self):
        nodes,bars = Main.MethodOfJoints("Example_3_3.csv")
        decimal_place = 3
        
        self.assertAlmostEqual(-141.42136, nodes[0].xforce_reaction, decimal_place)
        self.assertAlmostEqual(125.39385, nodes[0].yforce_reaction, decimal_place)
        self.assertAlmostEqual(191.0275, nodes[4].yforce_reaction, decimal_place)

    def test_Unknown_Bars_Example_3_3(self):
        nodes, bars = Main.LoadAndComputeReactions("Example_3_3.csv")
        
        num_unknowns = [2,3,3,3,2,5]
        for i in range(0,len(nodes)):
            node = nodes[i]
            unknowns = moj.UnknownBars(node)
            self.assertEqual(num_unknowns[i], len(unknowns))
        
        # artificially set some of these bars to be known
        bars[0].is_computed = True
        bars[1].is_computed = True
        bars[5].is_computed = True
        
        num_unknowns_now = [0,2,2,3,2,3]
        for  i in range(0,len(nodes)):
            node = nodes[i]
            unknowns = moj.UnknownBars(node)
            self.assertEqual(num_unknowns_now[i], len(unknowns))

        
    def test_NodeIsViable_Bars_Example_3_3(self):
        nodes, bars = Main.LoadAndComputeReactions("Example_3_3.csv")
        
        viable_node = [True,False,False,False,True,False]
        for i in range(0,len(nodes)):
            node = nodes[i]
            viable = moj.NodeIsViable(node)
            self.assertEqual(viable_node[i], viable)
        
        # artificially set some of these bars to be known
        bars[0].is_computed = True
        bars[1].is_computed = True
        bars[5].is_computed = True
        
        viable_node_now = [False,True,True,False,True,False]
        for  i in range(0,len(nodes)):
            node = nodes[i]
            viable = moj.NodeIsViable(node)
            self.assertEqual(viable_node_now[i], viable)

    def test_SumofForcesY_Example_3_3_Init(self):
        decimal_place = 2
        nodes, bars = Main.LoadAndComputeReactions("Example_3_3.csv")
        
        self.assertEqual(False,bars[1].is_computed)
        
        moj.SumOfForcesInLocalY(nodes[0], nodes[0].bars)
        
        self.assertEqual(True,bars[1].is_computed)
        self.assertAlmostEqual(728.95, bars[1].axial_load, decimal_place)

    def test_SumofForcesX_Example_3_3_Init(self):
        decimal_place = 2
        nodes, bars = Main.LoadAndComputeReactions("Example_3_3.csv")
        
        self.assertEqual(False,bars[1].is_computed)
        
        bars[1].axial_load = 728.952
        bars[1].is_computed = True
        moj.SumOfForcesInLocalX(nodes[0], bars[0])
        
        self.assertEqual(True,bars[0].is_computed)
        self.assertAlmostEqual(-692.781, bars[0].axial_load, decimal_place)

    def test_SumofForcesY_Example_3_3_Next(self):
        decimal_place = 2
        nodes, bars = Main.LoadAndComputeReactions("Example_3_3.csv")
        
        self.assertEqual(False,bars[1].is_computed)
        self.assertEqual(False,bars[0].is_computed)
        bars[1].axial_load = 728.952
        bars[1].is_computed = True
        bars[0].axial_load = -692.781
        bars[0].is_computed = True
        
        moj.SumOfForcesInLocalY(nodes[1], [bars[2],bars[3]])
        
        self.assertEqual(True,bars[3].is_computed)
        self.assertAlmostEqual(-639.19, bars[3].axial_load, decimal_place)

    def test_SumofForcesX_Example_3_3_Next(self):
        decimal_place = 2
        nodes, bars = Main.LoadAndComputeReactions("Example_3_3.csv")
        
        self.assertEqual(False,bars[1].is_computed)
        self.assertEqual(False,bars[0].is_computed)
        self.assertEqual(False,bars[3].is_computed)
        bars[1].axial_load = 728.952
        bars[1].is_computed = True
        bars[0].axial_load = -692.781
        bars[0].is_computed = True
        bars[3].axial_load = -639.190
        bars[3].is_computed = True
        
        moj.SumOfForcesInLocalX(nodes[1], bars[2])
        
        self.assertEqual(True,bars[2].is_computed)
        self.assertAlmostEqual(-207.055, bars[2].axial_load, decimal_place)

    def test_MethodOfJoints_Example_3_3(self):
        decimal_place = 2
        nodes, bars = Main.MethodOfJoints("Example_3_3.csv")
        
        bar_forces = [-692.781,
                      728.952,
                      -207.055,
                      -639.190,
                      -639.190,
                      728.951,
                      0.00,
                      -639.190,
                      521.896]
        
        for i in range(0,len(bars)):
            bar = bars[i]
            self.assertAlmostEqual(bar_forces[i], bar.axial_load, decimal_place)
        
    def test_MethodOfJoints_Example_3_2(self):
        decimal_place = 2
        nodes, bars = Main.MethodOfJoints("Example_3_2.csv")
        
        bar_forces = [-8.00,
                      6.928,
                      -3.00,
                      -5.00,
                      1.732,
                      3.464,
                      1.732,
                      -5.00,
                      -3.00,
                      6.928,
                      -8.00]
        
        for i in range(0,len(bars)):
            bar = bars[i]
            self.assertAlmostEqual(bar_forces[i], bar.axial_load, decimal_place)


if __name__ == '__main__':
    unittest.main()