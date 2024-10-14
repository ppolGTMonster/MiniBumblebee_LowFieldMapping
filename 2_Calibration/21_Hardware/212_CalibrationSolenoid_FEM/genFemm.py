######################################################################################################
#       Script to numerically calculate the solenoid's magnetic field using FEM simulation
#       
#       This script uses the open source software 
#           FEMM: https://www.femm.info/wiki/HomePage
#       and the Python interface 
#           pyFEMM: https://www.femm.info/wiki/pyFEMM
# 
#       Copyright (c) 2024 Pavel Povolni, Tuebingen, Germany
#       MIT LICENSED
#       Have fun guys!
#######################################################################################################



import femm
import math
import numpy as np
import os
import glob


################# FUNCTIONS ######################

def femm_drawCu(r,r_Cu,z):
    # Draw one circular Copper Wire at position r/z and with radius r_Cu

    femm.mi_drawarc(r-r_Cu,z,r+r_Cu,z,180,1)
    femm.mi_drawarc(r+r_Cu,z,r-r_Cu,z,180,1)
    femm.mi_addblocklabel(r, z)
    femm.mi_selectlabel(r, z)
    femm.mi_setblockprop('Copper', 0, 0, 'Coil',0,0,1)
    femm.mi_clearselected()
  
def femm_start(FEM_Cur,FOV_radius):
    # Start FEM-Simulation Tool FEMM and set it up

    #FEMM Model
    femm.openfemm()
    femm.newdocument(0) # 0 = magnetostatic
    femm.mi_probdef(0,'millimeters', 'axi', 1e-12) # freq, units, type, precision
    # Materials
    femm.mi_getmaterial('Air')
    femm.mi_getmaterial('Copper')

    # Circuits
    femm.mi_addcircprop('Coil', FEM_Cur, 1)


    # Add Sample Geometries
    femm.mi_drawarc(0, -FOV_radius, 0, FOV_radius, 180, 1) # r1,z1,r2,z2, angle, maxseg

    femm.mi_addblocklabel(FOV_radius/2, 0)
    femm.mi_selectlabel(FOV_radius/2, 0)
    femm.mi_setblockprop('Air')
    femm.mi_clearselected()

def femm_startCalc(FOV_radius,save_name,FEM_Cur):
    # Start one simulation round

    # Add surrounding Label
    femm.mi_addblocklabel(50, FOV_radius * 20)
    femm.mi_selectlabel(50, FOV_radius * 20)
    femm.mi_setblockprop('Air')
    femm.mi_clearselected()

    # Add boundary
    femm.mi_makeABC(7,5000,0,0,0)

    # Analyze
    femm.mi_zoomnatural()
    femm.mi_saveas(save_name)
    femm.mi_analyze()
    femm.mi_loadsolution()
    femm.mo_showdensityplot(1, 15e-3, 25e-3, 0, 'bmag')

def femm_getBCentre():
    #Reading the magnetic field values from the simulation result
    r1 = 0
    r2 = 40
    dr = 0.5
    z1 = -40
    z2 = 40
    dz = dr
    ni = int((r2 - r1) / dr + 2)
    nj = int((z2 - z1) / dz + 2)
  
    r_list = np.array([])
    z_list = np.array([])
    Br_list = np.array([])
    Bz_list = np.array([])

    for j in range(nj):
        for i in range(ni):
            r = r1 + i * dr
            z = z1 + j * dz
            Br, Bz = femm.mo_getb(r, z)
            
            r_list = np.append(r_list,r)
            z_list = np.append(z_list,z)
            Br_list = np.append(Br_list,Br)
            Bz_list = np.append(Bz_list,Bz)


    return Br_list,Bz_list,r_list,z_list

def femm_close():
    # Stopp FEMM Software
    femm.closefemm()
    
def femm_OneIteration(start_index,end_index, FEM_Cur, FOV_radius, z_Coil_list, R_Coil ,save_name):
    # Simulation of a “sub-coil”:
    # 1) Draw all copper wires
    # 2) Start the numerical calculation
    # 3) Read the calculated magnetic field values at the center
    # 4) Close FEMM again
    
    femm_start(FEM_Cur,FOV_radius)  
    for z_i in range(start_index,end_index): 
        z = z_Coil_list[z_i]
        z = round(z,2)
        r= R_Coil
        femm_drawCu(r,r_Cu,z)
    femm_startCalc(FOV_radius,save_name,FEM_Cur)
    Br_list,Bz_list,r_list,z_list = femm_getBCentre()
    femm_close()

    print("Finished: " + save_name)

    return Br_list,Bz_list,r_list,z_list 

def safeCSV(r,z,Br,Bz,name):
    #save the result for later analysis
    # Format: Coor R (mm) | Coor Z (mm) | Br (T) | Bz (T)
    # If you need B0 = sqrt(Br^2+Bz^2)

    data  = [r,z,Br,Bz]
    # Location for the CSV file
    file_path = name + ".csv"
    np.savetxt(file_path,np.column_stack(data),delimiter=',',fmt = '%.8f')





################# MAIN SCRIPT ######################


# Geometry specifications
d_Cu = 1.56 #mm, Diameter of wire including coating
r_Cu = d_Cu/2
n_Cu_Ly1 = 290 # Number of windings Layer 1, should be even here
n_Cu_Ly2 = 27 # Number of windigs Layer 2, take care on different counting start in Py(0) and Matlab (1)



R_Coil_1 = 80+d_Cu/2        #mm, Radius Layer 1 
R_Coil_2 = R_Coil_1 + d_Cu  #mm, Radius Layer 2

#Calculate z postitions of all Loops on both layers
z_temp_1 = (n_Cu_Ly1)/2
z_list_Ly1 = d_Cu * np.arange(-1*z_temp_1,z_temp_1+1) # inner coil, layer 1
z_list_Ly2_1 = d_Cu * (np.arange(0, n_Cu_Ly2) - z_temp_1) # outer coil, layer 2, negative z-side
z_list_Ly2_2 = d_Cu * (np.arange(-n_Cu_Ly2+1, 1) + z_temp_1) # outer coil, layer 2, positive z-side

# Current Flow
FEM_Cur = 1 # Ampere

# FOV
FOV_radius = 5 # Milimeter

# Delete old data
folder = ''
files_to_delete = glob.glob(os.path.join(folder, '*.csv')) + glob.glob(os.path.join(folder, '*.ans')) + glob.glob(os.path.join(folder, '*.fem')) 
for file in files_to_delete:
    try:
        os.remove(file)
    except:
        #nothing
        print('')


        
# Simulation
#Calculate the inner layer
#For some reason, FEMM sometimes crashes when there are too many windings. 
#To be on the safe side, the simulation was limited to 50 windings and a superposition of the individual results was calculated at the end in the center.
Br1_list,Bz1_list,r_list,z_list = femm_OneIteration(0,50, FEM_Cur, FOV_radius, z_list_Ly1, R_Coil_1 ,"simulation_coil_in1.fem")
safeCSV(r_list,z_list,Br1_list,Bz1_list,"simulation_coil_in1")
Br2_list,Bz2_list,_,_ = femm_OneIteration(51,100, FEM_Cur, FOV_radius, z_list_Ly1, R_Coil_1 ,"simulation_coil_in2.fem")
safeCSV(r_list,z_list,Br2_list,Bz2_list,"simulation_coil_in2")
Br3_list,Bz3_list,_,_ = femm_OneIteration(101,150, FEM_Cur, FOV_radius, z_list_Ly1, R_Coil_1 ,"simulation_coil_in3.fem")
safeCSV(r_list,z_list,Br3_list,Bz3_list,"simulation_coil_in3")
Br4_list,Bz4_list,_,_ = femm_OneIteration(151,200, FEM_Cur, FOV_radius, z_list_Ly1, R_Coil_1 ,"simulation_coil_in4.fem")
safeCSV(r_list,z_list,Br4_list,Bz4_list,"simulation_coil_in4")
Br5_list,Bz5_list,_,_ = femm_OneIteration(201,250, FEM_Cur, FOV_radius, z_list_Ly1, R_Coil_1 ,"simulation_coil_in5.fem") 
safeCSV(r_list,z_list,Br5_list,Bz5_list,"simulation_coil_in5")
Br6_list,Bz6_list,_,_ = femm_OneIteration(251,len(z_list_Ly1), FEM_Cur, FOV_radius, z_list_Ly1, R_Coil_1 ,"simulation_coil_in6.fem")
safeCSV(r_list,z_list,Br6_list,Bz6_list,"simulation_coil_in6")

BrIn = Br1_list + Br2_list + Br3_list + Br4_list + Br5_list + Br6_list
BzIn = Bz1_list + Bz2_list + Bz3_list + Bz4_list + Bz5_list + Bz6_list


# 2)  Calculate outer Coils
Br1_list,Bz1_list,_,_ = femm_OneIteration(0,len(z_list_Ly2_1), FEM_Cur, FOV_radius, z_list_Ly2_1, R_Coil_2 ,"simulation_coil_out1.fem")
safeCSV(r_list,z_list,Br1_list,Bz1_list,"simulation_coil_out1")
Br2_list,Bz2_list,_,_ = femm_OneIteration(0,len(z_list_Ly2_2), FEM_Cur, FOV_radius, z_list_Ly2_2, R_Coil_2 ,"simulation_coil_out2.fem")
safeCSV(r_list,z_list,Br2_list,Bz2_list,"simulation_coil_out2")

BrOut = Br1_list + Br2_list
BzOut = Bz1_list + Bz2_list

Br = BrIn + BrOut
Bz = BzIn + BzOut
safeCSV(r_list,z_list,Br,Bz,"Femm_Simulation")

