import os
from subprocess import Popen

my_env = os.environ.copy()
my_env["PATH"] = "../bin:" + my_env["PATH"]
path_to_dvr_file = "driver_input_files/ad_driver_example4.inp"

handle = Popen(["AeroDyn_Driver_Win32_Double.exe", path_to_dvr_file])
print(handle.stdout.read())
handle.flush()

handle = Popen(["../bin/AD_Interface_Test.exe", "50.0", "0.0125", "1.5", "3", "0.0", "0.0", "10.0", "10.0"], env=my_env)
print(handle.stdout.read())
handle.flush()

