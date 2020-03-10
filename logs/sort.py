#! /usr/bin/env python

import os, re, shutil
from datetime import datetime, timedelta

wd = os.getcwd()
print(wd)

index_pat = re.compile("(?<=_)\d+(?=\.bag)")
title_pat = re.compile("LOG.+(?=_\d+\.bag)")
period = timedelta(minutes=10)

files = [f for f in os.listdir(".") if os.path.isfile(f)]
files.sort()

for f in files:
    if f.endswith(".bag") or f.endswith(".bag.active"):
        index = int(re.search(index_pat, f).group())
        title = re.search(title_pat, f).group()
        time = datetime.strptime(title, "LOG_%Y-%m-%d-%H-%M-%S")
        root_time = time - period*index;
        folder_name = root_time.strftime("LOG_%Y-%m-%d-%H-%M")
        if not os.path.exists(folder_name):
            os.mkdir(folder_name)
        os.rename(os.path.abspath(f), os.path.abspath(folder_name) + "/" + f)
        print("{} ==> {}".format(f, folder_name))
