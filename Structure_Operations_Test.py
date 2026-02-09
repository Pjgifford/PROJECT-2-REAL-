#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 20 08:20:47 2025

@author: kendrickshepherd
"""

import Main_for_Final_Testing as Main_for_Testing

import unittest

class TestStructureOperations(unittest.TestCase):

    def test_Example_3_2(self):
        nodes,bars = Main_for_Testing.MethodOfJoints("Example_3_2.csv")
        # print(nodes[0].constraint)
        # print(nodes[3].constraint)
        # print(nodes[0].xforce_reaction)
        # print(nodes[0].yforce_reaction)
        # print(nodes[3].yforce_reaction)
        decimal_place = 3
        
        self.assertAlmostEqual(0, nodes[0].xforce_reaction, decimal_place)
        self.assertAlmostEqual(4, nodes[3].yforce_reaction, decimal_place)
        self.assertAlmostEqual(4, nodes[3].yforce_reaction, decimal_place)

    def test_Example_3_3(self):
        nodes,bars = Main_for_Testing.MethodOfJoints("Example_3_3.csv")
        # print(nodes[0].constraint)
        # print(nodes[4].constraint)
        # print(nodes[0].xforce_reaction)
        # print(nodes[0].yforce_reaction)
        # print(nodes[4].yforce_reaction)
        decimal_place = 3
        
        self.assertAlmostEqual(-141.42136, nodes[0].xforce_reaction, decimal_place)
        self.assertAlmostEqual(125.39385, nodes[0].yforce_reaction, decimal_place)
        self.assertAlmostEqual(191.0275, nodes[4].yforce_reaction, decimal_place)

if __name__ == '__main__':
    unittest.main()