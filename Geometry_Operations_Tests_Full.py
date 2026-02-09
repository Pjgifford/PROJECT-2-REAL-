#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 20 08:20:47 2025

@author: kendrickshepherd
"""

import math
import Main_for_Final_Testing as Main_for_Testing
import Geometry_Operations as geom

import unittest

class TestGeometryOperationsPart1(unittest.TestCase):

    def test_BarNodeToVector(self):
        nodes,bars = Main_for_Testing.MethodOfJoints("Example_3_2.csv")
        
        vec1 = geom.BarNodeToVector(nodes[0], bars[0])
        vec2 = geom.BarNodeToVector(nodes[6], bars[0])

        # check x coordinates        
        self.assertAlmostEqual(2, vec1[0], 4)
        self.assertAlmostEqual(-2, vec2[0], 4)
        
        # check y coordinates        
        self.assertAlmostEqual(1.1547, vec1[1], 4)
        self.assertAlmostEqual(-1.1547, vec2[1], 4)

        
    def test_FindOtherNode(self):
        nodes,bars = Main_for_Testing.MethodOfJoints("Example_3_2.csv")
        this_node = nodes[0]
        other_node = nodes[6]

        self.assertEqual(other_node, geom.FindOtherNode(nodes[0], bars[0]))
        self.assertEqual(this_node, geom.FindOtherNode(nodes[6], bars[0]))

    def test_VectorTwoNorm(self):
        myvec = [1,4,7,2]
        vec_len = math.sqrt(1**2+4**2+7**2+2**2)
        correct_decimals = 6
        
        self.assertAlmostEqual(vec_len, geom.VectorTwoNorm(myvec),correct_decimals)
        
    def test_Length(self):
        nodes,bars = Main_for_Testing.MethodOfJoints("Example_3_2.csv")
        bar_len = geom.Length(bars[0])
        
        vec_len = math.sqrt(2**2+1.1547**2)
        correct_decimals = 6

        self.assertAlmostEqual(vec_len, bar_len,correct_decimals)
    
    def test_TwoDCrossProduct(self):
        x_dir = [1,0]
        y_dir = [0,1]
        
        first_quad = [2,3]
        correct_decimals = 6
        
        cross_xy = geom.TwoDCrossProduct(x_dir, y_dir)
        cross_yx = geom.TwoDCrossProduct(y_dir, x_dir)
        cross_xx = geom.TwoDCrossProduct(x_dir, x_dir)
        cross_yy = geom.TwoDCrossProduct(y_dir, y_dir)
        cross_xdirquad = geom.TwoDCrossProduct(x_dir, first_quad)
        cross_ydirquad = geom.TwoDCrossProduct(y_dir, first_quad)
        
        self.assertAlmostEqual(1, cross_xy, correct_decimals)
        self.assertAlmostEqual(-1, cross_yx, correct_decimals)
        self.assertAlmostEqual(0, cross_xx, correct_decimals)
        self.assertAlmostEqual(0, cross_yy, correct_decimals)
        self.assertAlmostEqual(3, cross_xdirquad, correct_decimals)
        self.assertAlmostEqual(-2, cross_ydirquad, correct_decimals)

    
    def test_DotProduct(self):
        vec1 = [1,4,6,7]
        vec2 = [2,6,6,9]
        vec3 = [1,-1,3,-2]
        
        dot_12 = 2+24+36+63
        dot_13 = 1-4+18-14
        dot_23 = 2-6+18-18
        correct_decimals = 6
        
        self.assertAlmostEqual(dot_12, geom.DotProduct(vec1, vec2), correct_decimals)
        self.assertAlmostEqual(dot_13, geom.DotProduct(vec1, vec3), correct_decimals)
        self.assertAlmostEqual(dot_23, geom.DotProduct(vec2, vec3), correct_decimals)
        
    
    def test_SineVectors(self):
        x_dir = [1,0]
        y_dir = [0,1]
        
        first_quad = [2,3]
        correct_decimals = 6
        
        cross_xy = geom.SineVectors(x_dir, y_dir)
        cross_yx = geom.SineVectors(y_dir, x_dir)
        cross_xx = geom.SineVectors(x_dir, x_dir)
        cross_yy = geom.SineVectors(y_dir, y_dir)
        cross_xdirquad = geom.SineVectors(x_dir, first_quad)
        cross_ydirquad = geom.SineVectors(y_dir, first_quad)

        self.assertAlmostEqual(1, cross_xy, correct_decimals)
        self.assertAlmostEqual(-1, cross_yx, correct_decimals)
        self.assertAlmostEqual(0, cross_xx, correct_decimals)
        self.assertAlmostEqual(0, cross_yy, correct_decimals)
        self.assertAlmostEqual(3/math.sqrt(13), cross_xdirquad, correct_decimals)
        self.assertAlmostEqual(-2/math.sqrt(13), cross_ydirquad, correct_decimals)
    

    def test_CosineVectors(self):
        x_dir = [1,0]
        y_dir = [0,1]
        
        first_quad = [2,3]
        correct_decimals = 6
        
        dot_xy = geom.CosineVectors(x_dir, y_dir)
        dot_yx = geom.CosineVectors(y_dir, x_dir)
        dot_xx = geom.CosineVectors(x_dir, x_dir)
        dot_yy = geom.CosineVectors(y_dir, y_dir)
        dot_xdirquad = geom.CosineVectors(x_dir, first_quad)
        dot_ydirquad = geom.CosineVectors(y_dir, first_quad)

        self.assertAlmostEqual(0, dot_xy, correct_decimals)
        self.assertAlmostEqual(0, dot_yx, correct_decimals)
        self.assertAlmostEqual(1, dot_xx, correct_decimals)
        self.assertAlmostEqual(1, dot_yy, correct_decimals)
        self.assertAlmostEqual(2/math.sqrt(13), dot_xdirquad, correct_decimals)
        self.assertAlmostEqual(3/math.sqrt(13), dot_ydirquad, correct_decimals)

class TestGeometryOperationsPart2(unittest.TestCase):

    def test_FindSharedNode(self):
        nodes,bars = Main_for_Testing.MethodOfJoints("Example_3_3.csv")
        
        self.assertEqual(nodes[0], geom.FindSharedNode(bars[0], bars[1]))
        self.assertEqual(nodes[5], geom.FindSharedNode(bars[1], bars[2]))
        self.assertEqual(nodes[1], geom.FindSharedNode(bars[0], bars[2]))
        self.assertEqual(nodes[1], geom.FindSharedNode(bars[2], bars[0]))
            
    def test_BarsToVectors(self):
        nodes,bars = Main_for_Testing.MethodOfJoints("Example_3_3.csv")

        bvec1, bvec2 = geom.BarsToVectors(bars[5], bars[6])
        correct_decimals = 6
        
        self.assertAlmostEqual(0, bvec1[0], correct_decimals)
        self.assertAlmostEqual(4.2265, bvec1[1], correct_decimals)
        self.assertAlmostEqual(2.6795, bvec2[0], correct_decimals)
        self.assertAlmostEqual(1.547, bvec2[1], correct_decimals)

    def test_CosineBars(self):
        nodes,bars = Main_for_Testing.MethodOfJoints("Example_3_3.csv")

        # print(geom.CosineBars(bars[5], bars[6]))

        cosbars = geom.CosineBars(bars[5], bars[6])
        cosrevbars = geom.CosineBars(bars[6], bars[5])
        correct_decimals = 6
    
        self.assertAlmostEqual(0.49999756474155765, cosbars, correct_decimals)
        self.assertAlmostEqual(0.49999756474155765, cosrevbars, correct_decimals)

    def test_SineBars(self):
        nodes,bars = Main_for_Testing.MethodOfJoints("Example_3_3.csv")

        # print(geom.SineBars(bars[5], bars[6]))

        sinbars = geom.SineBars(bars[5], bars[6])
        sinrevbars = geom.SineBars(bars[6], bars[5])
        correct_decimals = 6
        
        self.assertAlmostEqual(-0.8660268097769906, sinbars, correct_decimals)
        self.assertAlmostEqual(0.8660268097769906, sinrevbars, correct_decimals)

if __name__ == '__main__':
    unittest.main()