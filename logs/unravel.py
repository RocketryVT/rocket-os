#! /usr/bin/env python

import os, re, shutil
from datetime import datetime, timedelta

files = [f for f in os.listdir(".") if os.path.isdir(f)]
files.sort()

for f in files:
    if f.startswith("LOG_"):
        for b in os.listdir(f):
            os.rename(os.path.abspath(f + "/" + b), os.path.abspath(".") + "/" + b)
            print("{} ==> {}".format(f + "/" + b, b))
        if not os.listdir(f):
            os.rmdir(f)
        else:
            print("Failed to remove folder " + f + " -- not empty")
