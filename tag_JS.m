% Initialize Robotarium object
N = 5;
runner_positions = [1.2 -1.2 -1.2 1.2 ; 0.6 0.6 -0.6 -0.6 ; 0.6 2.4 -2.4 -0.6];
initial_positions = [generate_initial_conditions(1, 'Width', 1, 'Height', 0.7) runner_positions]
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_dynamics();

% Barrier certificates
collision_certificate = create_uni_barrier_certificate_with_obstacles('SafetyRadius', 0.05);
wall_certificate = create_uni_barrier_certificate_with_boundary('SafetyRadius', 0.05);

% Creating position error and controller
args = {'PositionError', 0.25, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();

% Initializing various arrays
dxi = zeros(2,N);
dxu = zeros(2,N);

activeRunners = [2 3 4 5];
tagged = [];
tagEnable = false;

% Preparing positions
x = r.get_poses();
r.step();

% Freeze tag game - goes until all runners have been tagged
while(length(tagged) ~= 4)
    
    minDist = 3;
    closest = 0;

    for i = activeRunners
        dist = norm(x(1:2, i)-x(1:2, 1));
        if(dist < minDist)
            minDist = dist;
            closest = i;
        end
    end

    movement_pts = generate_initial_conditions(N-1, 'Width', 3.2, 'Height', 2);
    change_pts = false;

    while(~init_checker(x(:, 1), x(:, closest)))
        x = r.get_poses();
        
        dxi(:, 1) = controller(x(1:2, 1), x(1:2, closest)) * 0.92;

        for k = activeRunners
            runDist = norm(x(1:2, k)-movement_pts(1:2, k-1));
            if(runDist < 0.1)
                movement_pts(:, k-1) = generate_initial_conditions(1, 'Width', 3.2, 'Height', 2);
            end
        end

        dxi(:, activeRunners) = controller(x(1:2, activeRunners), movement_pts(1:2, activeRunners - ones(1, length(activeRunners))));
        
        dxu = si_to_uni_dyn(dxi, x);
        dxu = collision_certificate(dxu, x, x(1:2, tagged));
        dxu = wall_certificate(dxu, x);
        if(tagEnable)
            for t = tagged
                dxu(:, t) = [0;0];
            end
        end
        
        r.set_velocities(1:N, dxu);
        r.step();
        
        minDist = 3;
        closest = 0;
        
        for i = activeRunners
            dist = norm(x(1:2, i)-x(1:2, 1));
            if(dist < minDist)
                minDist = dist;
                closest = i;
            end
        end
    end
    
    activeRunners(activeRunners==closest) = [];
    tagged = sort([tagged closest]);
    tagEnable = true;
end

r.debug();

