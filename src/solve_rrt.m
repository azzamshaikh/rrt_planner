clc;clear; close all

% ========================================================================
%                            Main Code 
% ========================================================================

% Load models and initialize a plot figure
[start_pose,goal_pose,transCase,counterShaft] = load_and_plot_models();

% Run RRT algorithm
goal = solveRRT(start_pose,goal_pose,transCase,counterShaft);

% Get path from solution
[node_path, pose_path] = returnPath(goal);
hold off

% Generate animation
create_animation(node_path,transCase,counterShaft)

% ========================================================================
%                             Functions
% ========================================================================

function update_plot(node,color)
    % Function to update the plot with a 3D point
    plot3(node.pose(1),node.pose(2),node.pose(3),'x',"Color",color)
end

function [start_pose,goal_pose,transCase,counterShaft] = load_and_plot_models()
    % Function to load and place the tranmission rigid body tree, the case,
    % and the countershaft
    % Returns the start and goal poses along with the collision objects of
    % the case and the counter shaft
    trans = createTransmissionRBT();
    start_pose = [0 820 480 0 0 0] ; % x y z r (about x) p (about y)  y (about z)
    goal_pose = [0 660 850 0 0 0];
    newTrans = offset_transmission(trans,start_pose);
    show(newTrans,"Collisions","on","Frames","on","Visuals","on");
    xlim([-300 300] )
    ylim([-100 850] )
    zlim([ 0 1000] )
    hold on
    transCase = createCase();
    counterShaft = createCounterShaft();
    plotcase(transCase);
    plotCounterShaft(counterShaft);
end

function goal = solveRRT(start_pose,goal_pose,transCase,counterShaft)
    % Initialization of the RRT
    start = create_node(start_pose); % Create a node out of the start pose
    goal = create_node(goal_pose);   % Create a node out of the goal pose
    update_plot(start,'r')           % Update the plot with the start node
    update_plot(goal,'b')            % Update the plot with the goal node
    node_list = [start];             % Create a new node list for the algorithm
    
    % Solving RRT
    for ii = 1:3000 % Run for a tree depth of 3000 iterations
        sampleNode = sample();          % Get a sample from the space
        update_plot(sampleNode,'g')     % Update the plot with the sample
        nearestNode = find_nearest_neighbor(sampleNode,node_list);          % Obtain the nearest neighbor
        newNode = apply_local_planner(nearestNode,sampleNode);              % Apply the local planner 
        isColliding = robotCollisionCheck(newNode,transCase,counterShaft);  % Run a collision check
        if isColliding  % If colliding, continue
            % disp('Collision detected')
            continue
        else            % If not colliding, node is valid
            update_plot(newNode,'r')    % Update the plot with the new node
            node_list(end+1) = newNode; % Add the new node to the end 
            line([nearestNode.pose(1) newNode.pose(1)],[nearestNode.pose(2) newNode.pose(2)],[nearestNode.pose(3) newNode.pose(3)],'Color','b') % Create a line between the new node and nearest node
            if Euclidean(newNode,goal) < 50 % If the goal is within 50 mm, the goal is reached
                disp('Reached goal')
                goal.parent = newNode;
                break
            end
        end    
    end
end

function create_animation(node_path,transCase,counterShaft)
    % Function for creating an animation of the motion
    visual = figure();
    xlim([-300 300] )
    ylim([-100 850] )
    zlim([ 0 1000] )
    for ii = 1:length(node_path)
        plotcase(transCase);
        hold on
        plotCounterShaft(counterShaft);
        tranmission_visual = collision_tranmission(node_path(ii));
        show(tranmission_visual,"Collisions","on","Visuals","on");
        hold on
        update_plot(node_path(ii),'g');
        if ii < length(node_path)-1
            line([node_path(ii).pose(1) node_path(ii+1).pose(1)],[node_path(ii).pose(2) node_path(ii+1).pose(2)],[node_path(ii).pose(3) node_path(ii+1).pose(3)],'Color','b');
        end
        xlim([-300 300] );
        ylim([-100 850] );
        zlim([ 0 1000] );
        saveas(visual,sprintf('RRT_Path_%d.png',ii));
        hold off
    end
    hold on
    for ii = 1:length(node_path)
        update_plot(node_path(ii),'g')
        if ii < length(node_path)-1
            line([node_path(ii).pose(1) node_path(ii+1).pose(1)],[node_path(ii).pose(2) node_path(ii+1).pose(2)],[node_path(ii).pose(3) node_path(ii+1).pose(3)],'Color','b')
        end
    end
    saveas(visual,sprintf('RRT_Path_%d.png',length(node_path)+1));
    hold off
end

function [node_path, pose_path] = returnPath(goal)
    % Function for obtaining the path sequence
    node_path = [goal];
    pose_path = [goal.pose];
    parent = goal.parent;
    while isstruct(parent)
        node_path(end+1) = parent;
        pose_path = [pose_path; parent.pose];
        parent = parent.parent;
    end
    node_path = flip(node_path);
    pose_path = flip(pose_path);
end


function node = create_node(pose)
    % Function for creating a node. A node is a struct with a position and
    % a parent index
    node.pose = pose;
    node.parent = -1;
end

function node = sample()
    % Function for obtaining a sample from the free space

    % Assign limits from where to select points
    xlimits = [-100 100];
    ylimits = [400 900];
    zlimits = [250 1000];
    roll_limits = [-pi/3 pi/3];
    
    function val = random_value_in_range(range)
        % Function for getting a random value from the range
        min = range(1);
        max = range(2);
        val = (max-min)*rand(1) + min;
    end

    % Get random points and return the sampled node
    rand_x = random_value_in_range(xlimits);
    rand_y = random_value_in_range(ylimits);
    rand_z = random_value_in_range(zlimits);
    rand_roll = random_value_in_range(roll_limits);
    pose = [rand_x, rand_y, rand_z, rand_roll, 0, 0];
    node = create_node(pose);
end

function node = find_nearest_neighbor(currentNode, node_list)
    % Function to find the nearest neighbor 
    distances_list = get_distance_list(currentNode, node_list); % Get a list of neighbors and their distances from the current node
    [min_val, index] = min(distances_list); % Find the smallest distance node
    node = node_list(index); % Return the node that is closest
end

function list = get_distance_list(currentNode, node_list)
    % Function to get a list of nodes that describe the position of a
    % node relative to the current node
    list = [];
    for ii = 1:length(node_list)
        euclid = Euclidean(currentNode, node_list(ii));
        list(ii) = euclid;
    end
end

function value = Euclidean(node1,node2)
    % Function to calculate the euclidean distance between two nodes
    xsquared = (node1.pose(1)-node2.pose(1))^2;
    ysquared = (node1.pose(2)-node2.pose(2))^2;
    zsquared = (node1.pose(3)-node2.pose(3))^2;
    value = sqrt(xsquared + ysquared + zsquared);
end

function node = apply_local_planner(nearest,sample)
    % Function to apply the local planner
    node = create_node([0 0 0 0 0 0]);              % Create an empty node
    motion = sample.pose - nearest.pose;            % Get the motion vector
    linear_ratio = 0.25;                            % Set a ratio 
    node.pose = nearest.pose + motion*linear_ratio; % Obtain the new node position 
    node.parent = nearest;                          % Assign the parent of the new node as the nearest node
end

function transmission = createTransmissionRBT()
    % Function to create a rigidbodytree of the tranmission
    transmission = rigidBodyTree;
    
    % Set DH params to assign offset values. Additional distances were
    % required to have bodies aligned properly
    dhparam = [0 pi/2 0 0,
               0 0 236/2 0,
               0 0 214+11 0,
               0 0 100+57 0,
               0 0 70+15 0,
               0 0 40+15 0];
    
    % Create body, joint, and collisions of the rigidbodytree
    body0 = rigidBody('body0');
    jnt0 = rigidBodyJoint('jnt0','revolute');
    setFixedTransform(jnt0,dhparam(1,:),"dh");
    body0.Joint = jnt0;
    addBody(transmission,body0,transmission.BaseName);
    
    body1 = rigidBody('body1');
    jnt1 = rigidBodyJoint('jnt1','fixed');
    setFixedTransform(jnt1,dhparam(2,:),"dh");
    body1.Joint = jnt1;
    addCollision(body1,"Cylinder",[72/2 236]);
    addBody(transmission,body1,"body0");
    
    body2 = rigidBody('body2');
    jnt2 = rigidBodyJoint('jnt2','fixed');
    setFixedTransform(jnt2,dhparam(3,:),"dh");
    body2.Joint = jnt2;
    addCollision(body2,"Cylinder",[240/2 214]);
    addBody(transmission,body2,'body1');
    
    body3 = rigidBody('body3');
    jnt3 = rigidBodyJoint('jnt3','fixed');
    setFixedTransform(jnt3,dhparam(4,:),"dh");
    body3.Joint = jnt3;
    addCollision(body3,"Cylinder",[180/2 100])
    addBody(transmission,body3,"body2")
    
    body4 = rigidBody('body4');
    jnt4 = rigidBodyJoint('jnt4','fixed');
    setFixedTransform(jnt4,dhparam(5,:),"dh");
    body4.Joint = jnt4;
    addCollision(body4,"Cylinder",[239/2 70])
    addBody(transmission,body4,"body3")
    
    body5 = rigidBody('body5');
    jnt5 = rigidBodyJoint('jnt5','fixed');
    setFixedTransform(jnt5,dhparam(6,:),"dh");
    body5.Joint = jnt5;
    addCollision(body5,"Cylinder",[72/2 40])
    addBody(transmission,body5,"body4")
end

function new_trans = offset_transmission(transmission,pose)
    % Function to assign a new pose to the tranmission rigidbodytree
    new_trans = rigidBodyTree("DataFormat","column");
    rb = rigidBody('newBase');
    rb.Joint.setFixedTransform(trvec2tform(pose(1:3))*axang2tform([1 0 0 pose(4)])*axang2tform([0 1 0 pose(5)])*axang2tform([0 0 1 pose(6)]));
    new_trans.addBody(rb,'base')
    new_trans.addSubtree('newBase',transmission)
end

function dummy_trans = collision_tranmission(node)
    % Function to create a transmission for the collision check function
    transmission = createTransmissionRBT();
    dummy_trans = rigidBodyTree("DataFormat","column");
    rb = rigidBody('newBase');
    pose = node.pose;
    rb.Joint.setFixedTransform(trvec2tform(pose(1:3))*axang2tform([1 0 0 pose(4)])*axang2tform([0 1 0 pose(5)])*axang2tform([0 0 1 pose(6)]));
    dummy_trans.addBody(rb,'base')
    dummy_trans.addSubtree('newBase',transmission)
end

function bool = robotCollisionCheck(newNode,trans_case,counter_shaft)
    % Function to check for collisions between the tranmission and the
    % case/countershaft at a specific node
    transmission = collision_tranmission(newNode);
    % disp('case and main shaft')
    val = checkCollision(transmission,homeConfiguration(transmission),trans_case,"IgnoreSelfCollision","on");
    if val == 1
        bool = true;
        return
    end
    % disp('counter shaft and main shaft')
    val = checkCollision(transmission,homeConfiguration(transmission),counter_shaft,"IgnoreSelfCollision","on");
    if val == 1
        bool = true;
        return
    end
    bool = false;
end

function transmissionCase = createCase()
    % Function to create the transmission case
    rightWall = createWall(660-12.5);
    leftWall = createWall(0+12.5);
    base = createBase;
    leftSideWall = createSideWall(197.5);
    rightSideWall = createSideWall(-197.5);
    transmissionCase = {base,leftSideWall,rightSideWall};
    for ii = 1:length(rightWall)
        transmissionCase{end+1} = rightWall{ii};
    end
    for ii = 1:length(leftWall)
        transmissionCase{end+1} = leftWall{ii};
    end
end

function plotcase(transCase)
    % Function to plot the tranmission case
    for i = 1:length(transCase)
        wall = transCase{i};
        [ax,patchobj] = show(wall);
        patchobj.FaceAlpha = 0.25;
        hold on
    end
end

function base = createBase()
    % Function to create the base of the case
    base = collisionBox(420-50,660-50,25);
    base.Pose = [0 0 0 0;
                  0 0 0 660/2;
                  0 0 0 12.5;
                  0 0 0 1];
end

function sideWall = createSideWall(x_pose)
    % Function to create the sidewall of the case
    sideWall = collisionBox(25,660-50,650);
    sideWall.Pose = [0 0 0 x_pose;
                  0 0 0 610/2+25;
                  0 0 0 650/2;
                  0 0 0 1];
end


function wall = createWall(y_pose)
    % Function to create the walls with the shaft bearings
    bottom = collisionBox(420,25,170);
    bottom.Pose = [0 0 0 0;
                  0 0 0 y_pose;
                  0 0 0 85;
                  0 0 0 1]; % based on center of the object
    
    leftPanel = collisionBox(130,25,160);
    leftPanel.Pose = [0 0 0 145;
                     0 0 0 y_pose;
                     0 0 0 250;
                     0 0 0 1] ;
    
    rightPanel = collisionBox(130,25,160);
    rightPanel.Pose = [0 0 0 -145;
                     0 0 0 y_pose;
                     0 0 0 250;
                     0 0 0 1] ;
    
    middle = collisionBox(420,25,70);
    middle.Pose = [0 0 0 0;
                  0 0 0 y_pose;
                  0 0 0 365;
                  0 0 0 1];
    
    
    leftTopPanel = collisionBox(130,25,160);
    leftTopPanel.Pose = [0 0 0 145;
                     0 0 0 y_pose;
                     0 0 0 480;
                     0 0 0 1] ;
    
    rightTopPanel = collisionBox(130,25,160);
    rightTopPanel.Pose = [0 0 0 -145;
                     0 0 0 y_pose;
                     0 0 0 480;
                     0 0 0 1] ;
    
    top = collisionBox(420,25,90);
    top.Pose = [0 0 0 0;
                  0 0 0 y_pose;
                  0 0 0 605;
                  0 0 0 1];

    wall = {bottom,leftPanel,rightPanel,middle,leftTopPanel,rightTopPanel,top};

end

function plotCounterShaft(counterShaft)
    % Function to plot the countershaft
    for i = 1:length(counterShaft)
        cyl = counterShaft{i};
        [ax,patchobj] = show(cyl);
        patchobj.FaceAlpha = 0.5;
        hold on
    end
end

function counterShaft = createCounterShaft()
    % Function to create the countershaft
    start_pose_x = 0;
    start_pose_z = 250;

    rotation_matrix = axang2tform([1 0 0 pi/2]);
    
    part1_length = 60;
    part2_length = 50;
    part3_length = 168;
    part4_length = 50;
    part5_length = 40;
    part6_length = 90;
    part7_length = 76;
    part8_length = 50;
    part9_length = 76;
    
    part1_dia = 72;
    part2_dia = 280;
    part3_dia = 72;
    part4_dia = 279;
    part5_dia = 247;
    part6_dia = 160;
    part7_dia = 72;
    part8_dia = 200;
    part9_dia = 72;
    
    
    part1_y_pose = part1_length/2;
    part2_y_pose = part1_length + part2_length/2;
    part3_y_pose = part1_length + part2_length + part3_length/2;
    part4_y_pose = part1_length + part2_length + part3_length + part4_length/2;
    part5_y_pose = part1_length + part2_length + part3_length + part4_length + part5_length/2;
    part6_y_pose = part1_length + part2_length + part3_length + part4_length + part5_length + part6_length/2;
    part7_y_pose = part1_length + part2_length + part3_length + part4_length + part5_length + part6_length + part7_length/2;
    part8_y_pose = part1_length + part2_length + part3_length + part4_length + part5_length + part6_length + part7_length + part8_length/2;
    part9_y_pose = part1_length + part2_length + part3_length + part4_length + part5_length + part6_length + part7_length + part8_length + part9_length/2;
    
    
    part1 = collisionCylinder(part1_dia/2,part1_length);
    part1.Pose = trvec2tform([start_pose_x, part1_y_pose, start_pose_z])*rotation_matrix;
    
    part2 = collisionCylinder(part2_dia/2,part2_length);
    part2.Pose = trvec2tform([start_pose_x, part2_y_pose, start_pose_z])*rotation_matrix;
    
    part3 = collisionCylinder(part3_dia/2,part3_length);
    part3.Pose = trvec2tform([start_pose_x,part3_y_pose, start_pose_z])*rotation_matrix;
    
    part4 = collisionCylinder(part4_dia/2,part4_length);
    part4.Pose = trvec2tform([start_pose_x,part4_y_pose, start_pose_z])*rotation_matrix;
    
    part5 = collisionCylinder(part5_dia/2,part5_length);
    part5.Pose = trvec2tform([start_pose_x,part5_y_pose, start_pose_z])*rotation_matrix;
    
    part6 = collisionCylinder(part6_dia/2,part6_length);
    part6.Pose = trvec2tform([start_pose_x,part6_y_pose, start_pose_z])*rotation_matrix;
    
    part7 = collisionCylinder(part7_dia/2,part7_length);
    part7.Pose = trvec2tform([start_pose_x,part7_y_pose, start_pose_z])*rotation_matrix;
    
    part8 = collisionCylinder(part8_dia/2,part8_length);
    part8.Pose = trvec2tform([start_pose_x,part8_y_pose, start_pose_z])*rotation_matrix;
    
    part9 = collisionCylinder(part9_dia/2,part9_length);
    part9.Pose = trvec2tform([start_pose_x,part9_y_pose, start_pose_z])*rotation_matrix;
    
    counterShaft = {part1 ,part2 ,part3 ,part4 ,part5 ,part6 ,part7 ,part8 ,part9};
end


