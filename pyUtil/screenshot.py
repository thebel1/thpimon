#!/usr/bin/python

import sys
import argparse
from pimonLib import *

def main(argv):
    pimon = PiMon()
    pimon.saveScreenshot('/tmp/screenshot.bmp')

if __name__ == '__main__':
    main(sys.argv)