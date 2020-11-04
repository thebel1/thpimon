#!/usr/bin/python

import sys
from pimonLib import *

def main(argv):
    pimon = PiMon()
    print('Board Revision:\t{}'.format(hex(pimon.getBoardRev())))
    print('Temp:\t\t{} (deg. C)'.format(pimon.getTemp()))
    #print('ARM Memory:\t{} MB'.format(pimon.getArmMem()))

if __name__ == '__main__':
    main(sys.argv)