#!/bin/bash
OURFILE=~/test_save/test_c001
rosrun BagFromImages BagFromImages $OURFILE .bmp 10 ~/test000.bag .png 9 $OURFILE/test.bag 
