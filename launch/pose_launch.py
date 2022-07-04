from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time=False)
            
    sl.declare_arg('namespace', default_value='bluerov2')
    
    with sl.group(ns=sl.arg('namespace')):
        
        sl.node('slider_publisher','slider_publisher',
                arguments = [sl.find('auv_control_teleop', 'pose_setpoint.yaml')])

    return sl.launch_description()
