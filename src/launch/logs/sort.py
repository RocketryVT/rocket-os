#! /usr/bin/env python

import os, re, shutil

wd = os.getcwd()
print(wd)

index_pat = re.compile("(?<=_)\d+(?=\.bag)")
title_pat = re.compile("LOG.+(?=_\d+\.bag)")

files = [f for f in os.listdir(".") if os.path.isfile(f)]
files.sort()

folder_name = None
for f in files:
    if f.endswith(".bag"):
        index = int(re.search(index_pat, f).group())
        if index == 0:
            folder_name = re.search(title_pat, f).group()
            if not os.path.exists(folder_name):
                os.mkdir(folder_name)
        if not folder_name:
            folder_name = re.search(title_pat, f).group()
            if not os.path.exists(folder_name):
                os.mkdir(folder_name)
        os.rename(os.path.abspath(f), os.path.abspath(folder_name) + "/" + f)
        print("{} ==> {}".format(f, folder_name))
