# -*- coding: utf-8 -*-
# python setup.py py2exe
from distutils.core import setup
import py2exe

#option = {
#    'bundle_files':3,
#    'compressed': 1,
#    'optimize': 2,    
#}
option = {
    'bundle_files':2,
    'compressed': 1,
    'optimize': 2,    
}
setup(
  options = {"py2exe": option},
  windows = [
    {"script" : "<APP_NAME>.py"}],
  zipfile = '<APP_NAME>.zip')
