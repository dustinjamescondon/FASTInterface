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
mass            = 3000000.0
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
# Run the non-added mass simulation with the given parameters
#.................................................
print('Simulating with the following parameters...')
print('---------------------------------------------')
proc1 = Popen(["../../bin/MassSpringDamper_Test_Win32_Release.exe" , str(disable_added_mass), str(simulation_time), str(timestep),
                str(mass), str(initial_disp), str(spring_coeff), str(damping_coeff), str(rpm), str(inflow_speed), nonadded_mass_output_fname], env = my_env)
print('---------------------------------------------')
print('Done!\n')

#-------------------------------------------------
# Run the added mass simulation with the given parameters
#.................................................
print('Simulating with the following parameters...')
print('---------------------------------------------')
proc2 = Popen(["../../bin/MassSpringDamper_Test_Win32_Release.exe", str(enable_added_mass), str(simulation_time), str(timestep),
                 str(mass), str(initial_disp), str(spring_coeff), str(damping_coeff), str(rpm), str(inflow_speed), added_mass_output_fname], env = my_env)
print('---------------------------------------------')
print('Done!\n')
#-------------------------------------------------

proc1.wait()
proc2.wait()
#-------------------------------------------------
# Plot the results from both simulation runs (with and without added mass)
#.................................................
plot.plot_output_files(nonadded_mass_output_fname + ".out", added_mass_output_fname + ".out")
#-------------------------------------------------
