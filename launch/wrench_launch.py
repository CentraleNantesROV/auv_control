from simple_launch import SimpleLauncher
from plankton_utils.time import is_sim_time
from launch.substitutions import Command

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    
    sl.declare_arg('namespace', default_value='bluerov2')
    
    with sl.group(ns=sl.arg('namespace')):
        sl.node('slider_publisher', 'slider_publisher', name='wrench_control',
                arguments=[sl.find('auv_control', 'wrench.yaml')])
        

    return sl.launch_description()
