    host = '192.168.12.1';
    port = 5353;
    command_pixhawk = ['ssh', host, 'bash', '/home/odroid/ros2_thrust_ws/start_uxrce.sh']
    command_flightstack = ['ssh', host, 'bash', '/home/odroid/ros2_thrust_ws/run_flightstack.sh']
    % start the pixhawk
    system(command_pixhawk)
    system(command_flightstack)