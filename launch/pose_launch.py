from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time=False)
    ns = sl.declare_arg('namespace', default_value='bluerov2')
    
    with sl.group(ns=ns):
        
        sl.node('slider_publisher','slider_publisher',
                arguments = [sl.find('auv_control', 'pose_setpoint.yaml')])

    return sl.launch_description()
