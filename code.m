function code()
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
        
        motorVelocity=0.05
        motorForce=70
        
        L(1) = Link([0 10 0 -pi/2]);
        L(2) = Link([0 0 0 -pi/2]);
        L(3) = Link([0 20 0 -pi/2]);
        L(4) = Link([0 0 0 pi/2]);
        L(5) = Link([0 19 0 -pi/2]);
        L(6) = Link([0 0 0 pi/2]);

        robot = SerialLink(L);
        T = robot.fkine([0 0 0 0 0 0]);

        
        T.t = [15 15 20];
        qi = robot.ikine(T ,[0 0 0 ], 'rlimit',1000)
        robot.plot(qi)

        th1 = qi(1);
        th2 = pi-qi(2);
        th3 = qi(3);
        th4 = qi(4);
        th5 = qi(5);
        th6 = qi(6);

        [returnCode, joint1] = vrep.simxGetObjectHandle(clientID, 'LBR4p_joint1', vrep.simx_opmode_blocking);
        [returnCode, joint2] = vrep.simxGetObjectHandle(clientID, 'LBR4p_joint2', vrep.simx_opmode_blocking);
        [returnCode, joint3] = vrep.simxGetObjectHandle(clientID, 'LBR4p_joint3', vrep.simx_opmode_blocking);
        [returnCode, joint4] = vrep.simxGetObjectHandle(clientID, 'LBR4p_joint4', vrep.simx_opmode_blocking);
        [returnCode, joint5] = vrep.simxGetObjectHandle(clientID, 'LBR4p_joint5', vrep.simx_opmode_blocking);
        [returnCode, joint6] = vrep.simxGetObjectHandle(clientID, 'LBR4p_joint6', vrep.simx_opmode_blocking);
        [returnCode, grib] = vrep.simxGetObjectHandle(clientID, 'RG2_openCloseJoint', vrep.simx_opmode_blocking);

        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint1, th1, vrep.simx_opmode_blocking);

        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint3, th3, vrep.simx_opmode_blocking);
        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint4, th4, vrep.simx_opmode_blocking);
        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint5, th5, vrep.simx_opmode_blocking);
        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint6, th6, vrep.simx_opmode_blocking);
        [returnCode] = vrep.simxSetIntegerSignal(clientID, grib, 1, vrep.simx_opmode_blocking);
        pause(0.5)
        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint2, th2, vrep.simx_opmode_blocking);

        % Gribber
        v =-motorVelocity
        [returnCode data] = vrep.simxGetIntegerSignal(clientID, 'RG2_open', vrep.simx_opmode_blocking)
        if (data && data ~= 0)
            v=motorVelocity
        end

        pause(1)

        [returnCode] = vrep.simxSetJointForce(clientID, grib, motorForce, vrep.simx_opmode_blocking);
        [returnCode] = vrep.simxSetJointTargetVelocity(clientID, grib, v, vrep.simx_opmode_blocking);

        pause(1)

        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint2, 0, vrep.simx_opmode_blocking);


        T.t = [15 15 0];
        qi = robot.ikine(T ,[0 0 0 ], 'rlimit',1000)
        robot.plot(qi)

        th1 = qi(1);
        %th2 = pi-qi(2);
        th3 = qi(3);
        th4 = qi(4);
        th5 = qi(5);
        th6 = th6 + pi-0.3;


        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint1, th1, vrep.simx_opmode_blocking);
        [returnCode]= vrep.simxSetJointTargetVelocity(clientID, joint1, 0.05,vrep.simx_opmode_blocking);

        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint3, th3, vrep.simx_opmode_blocking);
        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint4, th4, vrep.simx_opmode_blocking);
        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint5, th5, vrep.simx_opmode_blocking);
        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint6, th6, vrep.simx_opmode_blocking);
        [returnCode] = vrep.simxSetIntegerSignal(clientID, grib, 1, vrep.simx_opmode_blocking);

        pause(1);
        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint2, th2, vrep.simx_opmode_blocking);
        pause(1)

        [returnCode] = vrep.simxSetJointForce(clientID, grib, 0, vrep.simx_opmode_blocking);
        [returnCode] = vrep.simxSetJointTargetVelocity(clientID, grib, -v, vrep.simx_opmode_blocking);

        pause(1)
        
        [returnCode] = vrep.simxSetJointTargetPosition(clientID, joint2, 0, vrep.simx_opmode_blocking);

        vrep.simxFinish(-1);
    else
    disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!

    disp('Program ended');
end