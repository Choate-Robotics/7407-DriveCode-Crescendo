from pyfrc.tests import *
import runpy

def test_path():
    assert runpy.run_path('../robot.py') is not None