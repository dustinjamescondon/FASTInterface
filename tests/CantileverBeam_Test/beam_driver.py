# Author: Dustin Condon
# Date  : August 2020

# Description:
#-------------
# This runs two simulations in different processes: one with added mass enabled, and one without.
# The std output from both are displayed in one console, so the output lines become interlaced. The output
# files for both simulations are different though. Once both processes have completed their simulation
# the outputs files are read in and plotted against each other.

import subprocess
import plot
import os
from subprocess import Popen

my_env = os.environ.copy()
my_env["PATH"] = "../../bin:" + my_env["PATH"]

#-------------------------------------------------
# Change these parameters to affect simulation
#.................................................
beam_length = 30
E = 1.0e+5

enable_added_mass = 1 # 0 == false; 1 == true
disable_added_mass = 0
simulation_time = 5.0
timestep        = 0.00125
mass            = 1500000.0
I               = mass * (beam_length **2)
initial_disp    = 0.0
spring_coeff    = 3 * E * I / (beam_length **3)
damping_coeff   = 0.0#2 * (spring_coeff * mass)**0.5
rpm             = 0.5
inflow_speed    = 1.0
added_mass_output_fname = "with_added_mass"
nonadded_mass_output_fname = "without_added_mass"
#-------------------------------------------------

#-------------------------------------------------
# Check that the executable exist first
#.................................................
if not os.path.isfile("../../bin/MassSpringDamper_Test_Win32_Release.exe"):
    print("Couldn't find MassSpringDamper_Test_Win32_Release.exe -- Have you built the Release x86 version of MassSpringDamper_Test?")
    exit(1);
#-------------------------------------------------

#-------------------------------------------------
# Run the non-added mass simulation with the given parameters
#.................................................
print('Simulating with the following parameters...')
print('---------------------------------------------')
proc1 = Popen(["../../bin/MassSpringDamper_Test_Win32_Release.exe" , str(disable_added_mass), str(simulation_time), str(timestep),
                str(mass), str(initial_disp), str(spring_coeff), str(damping_coeff), str(rpm), str(inflow_speed), nonadded_mass_output_fname], env = my_env)


#-------------------------------------------------
# Run the added mass simulation with the given parameters
#.................................................
print('Simulating with the following parameters...')
print('---------------------------------------------')
proc2 = Popen(["../../bin/MassSpringDamper_Test_Win32_Release.exe", str(enable_added_mass), str(simulation_time), str(timestep),
                 str(mass), str(initial_disp), str(spring_coeff), str(damping_coeff), str(rpm), str(inflow_speed), added_mass_output_fname], env = my_env)
#------------------------------------------------

proc1.wait()
proc2.wait()

print("\nSimulations completed!")

#-------------------------------------------------
# Plot the results from both simulation runs (with and without added mass)
#.................................................
plot.plot_output_files(nonadded_mass_output_fname + ".out", added_mass_output_fname + ".out")
#-------------------------------------------------
