import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils

my_chain = ikpy.chain.Chain.from_urdf_file("./ar3.urdf")

target_position = [ 0.1, -0.2, 0.1]



print("The angles of each joints are : ", my_chain.inverse_kinematics(target_position))