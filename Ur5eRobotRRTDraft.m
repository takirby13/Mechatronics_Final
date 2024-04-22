%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
[ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 2;     % Traj duration (secs) 

    %% Load Gazebo and obtain poses
goHome('qr')
resetWorld
disp('Getting Robot and gCan1 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{24};         % gCan1 is cell {20}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
    %% Visualize Robot
    figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
show(ur5e,config,"Visuals","off","Collisions","on");
xticks(0:1:4);  % Set x-axis tick values from 0 to 3
yticks(0:1:4);  % Set y-axis tick values from 0 to 3
zticks(0:1:4);  % Set z-axis tick values from 0 to 3
hold on
for i = 1:length(env)
    show(env{i});
end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations

mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
mat_joint_traj(6) = 0.1;
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = config;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);
%% Visualize Path

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

for i = 1:interpStateTraj
    show(ur5e, interpStates(i,:),...
        "PreservePlot", false,...
        "Visuals","off",...
        "Collisions","on");
    title("Plan 1: MaxConnectionDistance = 0.3")
    drawnow;
end
%% %% 4. Create action client, message, populate ROS trajectory goal and send
    % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
    % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 
    %% Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
    % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');
    % [num_configs,~]=size(interpStates);
% for i = 1:num_configs
for i = 1:2:size(interpStates,1)
    traj_goal = convert2ROSPointVec(interpStates, ...
                                    rob_joint_names, ...
                                    interpStateTraj, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
    % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
end
    traj_result = traj_result.ErrorCode;

%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = path(end, :);
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner
% goHome('qz')
% type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
% strcmp(type,'gazebo')  
% models = getModels;                         % Extract gazebo model list
% model_name = models.ModelNames{6};          % greenbin is cell {6}  
% [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model
% 
% greenBin = [-0.5, -0.3654,-0.43, 0, 0, 0];
% place_pose = set_manual_goal(greenBin);
% 
% mat_traj = place_pose;
% [mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));

goalConfig = [2.19, 0.37, 1.76, -2.13, 0.00, 0.62]
planner=manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 1;
planner.ValidationDistance = 0.2;
planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,c] = size(interpStates);
hold off
%% visualize
% 
show(ur5e,config,"Visuals","off","Collisions","on");
hold on
for i = 1:length(env)
    show(env{i});
end

for i = 1:size(interpStates,1)
    show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
    title("Plan 5: Place the Can")
    drawnow;
    if i == (size(interpStates,1))
        view([80,7])
    end
end
%% %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
    % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);


%%  %% Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

    %% Set Grip Pos by default to pick / close gripper
    gripPos = 0.227; 
    gripPos = 0.23; % 0.225 for upright cans tends to slip. 

    % if nargin==0
    %     type = 'pick';
    % end
    % 
    % % Modify it if place (i.e. open)
    % if strcmp(type,'place')
    %     gripPos = 0;           
    % end

    %% Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    

    %% Clear grip_action_client: checking to see if this minimizes ROS network connection errors
    clear grip_action_client;


%% %% %% 4. Create action client, message, populate ROS trajectory goal and send
    % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
    % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 
    %% Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
    % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');
    % [num_configs,~]=size(interpStates);
% for i = 1:num_configs
for i = 1:2:size(interpStates,1)
    traj_goal = convert2ROSPointVec(interpStates, ...
                                    rob_joint_names, ...
                                    interpStateTraj, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
    % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
end
    traj_result = traj_result.ErrorCode;

%% %% %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
    % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);


%%  %% Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

    %% Set Grip Pos by default to pick / close gripper
    gripPos = 0.227; 
    gripPos = 0; % 0.225 for upright cans tends to slip. 

    % if nargin==0
    %     type = 'pick';
    % end
    % 
    % % Modify it if place (i.e. open)
    % if strcmp(type,'place')
    %     gripPos = 0;           
    % end

    %% Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qr')



