import matplotlib.pyplot as plt
import csv

# first param is the output file from the simulation without added mass enabled, second is with
# added mass enabled
def plot_output_files(file_without_AM, file_with_AM):
    #-------------------------------------------------
    # Plot the results
    #-------------------------------------------------
    
    # without added mass
    time = []
    displacement = []
    velocity = []
    acc = []
    force = []
    
    # with added mass
    time_AM = []
    displacement_AM = []
    velocity_AM = []
    acc_AM = []
    force_AM = []
    
    params = []
    
    csvfile = open(file_without_AM, 'r')
    plots = csv.reader(csvfile, delimiter='\t')
    csvfile_AM =  open(file_with_AM, 'r')
    plots_AM = csv.reader(csvfile_AM, delimiter='\t')
        
    #  ignore the first row because it is the header
    params = next(plots)
    next(plots)
    params = next(plots_AM)
    
    #  read in all the columns, putting each in their respective lists
    for col in plots:
        time.append(float(col[0]))
        displacement.append(float(col[1]))
        velocity.append(float(col[2]))
        acc.append(float(col[3]))
        force.append(float(col[4]))
    
    for col in plots_AM:
        time_AM.append(float(col[0]))
        displacement_AM.append(float(col[1]))
        velocity_AM.append(float(col[2]))
        acc_AM.append(float(col[3]))
        force_AM.append(float(col[4]))
    
    csvfile.close()
    csvfile_AM.close()
        
    plt.figure(1)

    plt.subplot(221)
    # plot for the mass spring damper displacement and velocity
    plt.plot(time, displacement, linestyle=':', color='g', label='displacement')
    plt.plot(time_AM, displacement_AM, linestyle='-', color='b', label='displacement with added mass')
    plt.xlabel('Time')
    plt.ylabel('M')
    plt.title('Tip mass displacement')
    plt.legend(loc='upper right', shadow=True)

    plt.subplot(222)
    plt.plot(time, force, linestyle=':', color='g', label='force')
    plt.plot(time_AM, force_AM, linestyle='-', color='b', label='force with added mass')
    plt.xlabel('Time')
    plt.ylabel('Nm')
    plt.title('Aerodynamic force')
    plt.legend(loc='upper right', shadow=True)

    plt.subplot(223)
    plt.plot(time, velocity, linestyle=':', color='r', label='velocity')
    plt.plot(time_AM, velocity_AM, linestyle='-', color='y', label='velocity with added mass')
    plt.xlabel('Time')
    plt.ylabel('M/s')
    plt.title('Tip mass velocity')
    plt.legend(loc='upper right', shadow=True)

    plt.subplot(224)
    plt.plot(time, acc, linestyle=':', color='black', label='velocity')
    plt.plot(time_AM, acc_AM, linestyle='-', color='red', label='velocity with added mass')
    plt.xlabel('Time')
    plt.ylabel('M/s^2')
    plt.title('Tip mass acceleration')
    plt.legend(loc='upper right', shadow=True)
    
    plt.show()
    #--------------------------------------------------

def plot_output(output_filename):
    #-------------------------------------------------
    # Plot the results
    #-------------------------------------------------
    time = []
    displacement = []
    velocity = []
    rotor_disp = []
    rotor_vel = []
    gen_disp = []
    gen_vel = []
    params = []

    with open(output_filename, 'r') as csvfile:
        plots = csv.reader(csvfile, delimiter='\t')
        
        #  ignore the first row because it is the header
        params = next(plots)
        next(plots)

        #  read in all the columns, putting each in their respective lists
        for col in plots:
            time.append(float(col[0]))
            displacement.append(float(col[1]))
            velocity.append(float(col[2]))
            rotor_disp.append(float(col[3]))
            rotor_vel.append(float(col[4]))
            gen_disp.append(float(col[5]))
            gen_vel.append(float(col[6]))

        plt.figure(1)

        # plot for the mass spring damper displacement and velocity
        plt.subplot(121)
        plt.plot(time, displacement, linestyle='-', color='b', label='displacement')
        plt.plot(time, velocity, linestyle=':', color='g', label='velocity')
        plt.xlabel('Time')
        plt.ylabel('M and M/sec')
        plt.title('Mass spring damper')

        plt.legend(loc='upper right', shadow=True)

        # plot for the drivetrain angular displacement and velocity
        plt.subplot(122)
        plt.plot(time, rotor_disp, linestyle='-', color='r', label='rotor disp')
        plt.plot(time, rotor_vel, linestyle=':', color='y', label='rotor vel')
        plt.plot(time, gen_disp, linestyle='-', color='g', label='gen disp')
        plt.plot(time, gen_vel, linestyle=':', color='b', label='gen vel')
        plt.xlabel('Time')
        plt.ylabel('rad and rads/s')
        plt.title('Drive train shafts')
    
        plt.legend(loc='upper right', shadow=True)
        plt.show()
    #--------------------------------------------------
