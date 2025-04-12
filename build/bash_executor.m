function [success] = bash_executor(n, gainsMatrix) 

    % % establish connection to the Odroid and ping it
    % % the code below was created with assistance from chatGPT
    % address = 12345;
    % port = 6000;
    % % Create a TCP/IP server object and specify the port number
    % server = tcpserver(address, port);
    % 
    % % Open the server
    % fopen(server);
    % 
    % % Wait for the client's connection and acknowledge
    % disp('Server: Waiting for client...');
    % fscanf(server);  % Wait for the client's initial message (handshake request)
    % disp('Server: Received handshake request.');
    % 
    % % Send acknowledgment message to client
    % fprintf(server, 'ACK: Handshake received by server');
    % 
    % % send the gains to the Odroid
    %z = [1; 2; 3; 4; 5; 6; 7; 8; 9];
    file = struct("KP_translational", ...
    struct("scaling_coeff", 1.0, "matrix", [14.0, 0.0, 0.0; ...
    0.0, 14.0, 0.0; ...
    0.0, 0.0, 25.0]), ...
    "KD_translational", ...
    struct("scaling_coeff", 1.0, "matrix", [14.0, 0.0, 0.0; ...
    0.0, 14.0, 0.0; ...
    0.0, 0.0, 25.0]), ...
    "KI_translational", ...
    struct("scaling_coeff", 1.0, "matrix", [14.0, 0.0, 0.0; ...
    0.0, 14.0, 0.0; ...
    0.0, 0.0, 25.0]), ...
    "KP_rotational", ...
    struct("scaling_coeff", 1.0, "matrix", diag(gainsMatrix(1:3))), ...
    'KD_rotational', ...
    struct('scaling_coeff', 1.0, 'matrix', diag(gainsMatrix(7:9))), ...
    "KI_rotational", struct("scaling_coeff", 1.0, "matrix", diag(gainsMatrix(4:6))))
    json = jsonencode(file);
    % convert the json into a readable file
    newFile = fopen('gains_pid_test.json', 'w');
    fwrite(newFile, json);
    fclose(newFile);
    host = '192.168.12.1';
    port = 5000;
    %filename = 'test_file.txt';
    filename = 'gains_pid_test.json';
    %processed_filename = 'received_file.txt';
    processed_filename = 'L2norm.json';
    n = 5; 
    password = "odroid";
    path = "~/ros2_thrust_ws/src/comms/tcp_server.cpp";
    user = "odroid";
    % copy the json over to the odroid using pscp
    cpycmd = sprintf("pscp -pw odroid gains_pid_test.json odroid@192.168.12.1:/home/odroid/ros2_thrust_ws/src/flightstack/params/control/pid");
    fakeBashCmd = sprintf("putty -m commands.txt odroid@192.168.12.1 -pw odroid");
    % get the L2.json from the odroid
    receivecmd = sprintf("pscp -pw odroid odroid@192.168.12.1:/home/odroid/ros2_thrust_ws/src/comms/L2norm.json .")
    %cmd2 = sprintf("./tcp_sever_bash.sh");
    system(cpycmd);
    system(fakeBashCmd);
    system(receivecmd);
    %system(cmd2);
    tcpClient = tcpclient(host, port);
    write(tcpClient, password, 'string');
    
    % fileID = fopen('test.txt', 'w');
    % fprintf(fileID, 'MATLAB sending a test file.\n');
    % fclose(fileID);
    % 
    % for i = 1:n
    %     fprintf("Iteration %d of %d\n", i, n);
    % %     %cmd = sprintf("plink -ssh odroid@192.168.12.1 -pw odroid ./ros2_thrust_ws/src/comms/tcp_server_bash.sh");
    % %     %system(cmd);
    %     tcpClient = tcpclient(host, port);
    %     write(tcpClient, password, 'string');
    % %     % Send file
    % %     fileID = fopen(filename, 'rb');
    % %     fileData = fread(fileID);
    % %     fclose(fileID);
    % % % 
    %     % write(tcpClient, fileData, "uint8");
    %     % pause(0.1);
    %     % write(tcpClient, uint8('EOF'), "uint8"); % Send EOF marker
    % % 
    % %     fprintf("File sent: %s\n", filename);
    % % 
    % %     % Wait until data starts arriving
    %     fprintf("Waiting for processed file...\n");
    %     % while tcpClient.NumBytesAvailable == 0
    %     %    pause(0.1);  % Short pause to avoid high CPU usage
    %     % end
    % % 
    %     % Receive processed file
    %     fprintf("%6.4f", tcpClient.NumBytesAvailable)
    %     receivedData = [];
    %     while true
    %         dataChunk = read(tcpClient, tcpClient.NumBytesAvailable, "uint8");
    %         if ~isempty(dataChunk)
    %             if length(dataChunk) >= 3 && all(dataChunk(end-2:end) == uint8('EOF'))
    %                 dataChunk = dataChunk(1:end-3); % Remove EOF marker
    %                 receivedData = [receivedData; dataChunk]; %#ok<AGROW>
    %                 break;
    %             end
    %             receivedData = [receivedData; dataChunk]; %#ok<AGROW>
    %         end
    %         success = 1;
    %     end
    % % 
    %     % Save received file
    %     fileID = fopen(processed_filename, 'wb');
    %     fwrite(fileID, receivedData);
    %     fclose(fileID);
    % % 
    %     fprintf("Processed file received and saved as: %s\n", processed_filename);
    % end
    
    % clear tcpClient;

    % ssh into the odroid
    % command_pixhawk = ['ssh', host, 'bash', '/home/odroid/ros2_thrust_ws/start_uxrce.sh']
    % command_flightstack = ['ssh', host, 'bash', '/home/odroid/ros2_thrust_ws/run_flightstack.sh']
    % % start the pixhawk
    % system(command_pixhawk)
    % system(command_flighstack)
    success = 1;
end