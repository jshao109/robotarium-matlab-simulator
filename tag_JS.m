%Initialize Robotarium object
N = 4;
runner_positions = generate_initial_conditions(N-1, 'Width', 0.7, 'Height', 1.5, 'Spacing', 0.3) + [0.8 0.8 0.8 ; zeros(2, N-1)];
initial_positions = [[-1.3 ; -0.7 ; -2.4] runner_positions];
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_dynamics();
% Single-integrator barrier certificates
uni_barrier_certificate = create_uni_barrier_certificate_with_boundary('SafetyRadius', 0.23);

%Creating controller and safety buffer
args = {'PositionError', 0.35, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();

x = r.get_poses();
r.step();

dxi = [zeros(2, N) ; x(3, :)];
dxu = [zeros(2, N) ; x(3, :)];

activeRunners = [2 3 4];
tagged = [];
tagEnable = false;

for j = 1:N-1
    
    minDist = 3;
    closest = 0;

    for i = activeRunners
        dist = norm(x(1:2, i)-x(1:2, 1));
        if(dist < minDist)
            minDist = dist;
            closest = i;
        end
    end

    movement_pts = generate_initial_conditions(N, 'Spacing', 0.5, 'Width', 2.5, 'Height', 1.5);
    change_pts = false;

    while(~init_checker(x(:, 1), x(:, closest)))
        x = r.get_poses();
        
        dxi(1:2, 1) = controller(x(1:2, 1), x(1:2, closest));

        for k = activeRunners
            runDist = norm(x(1:2, k)-movement_pts(1:2, k));
            if(runDist < 0.1)
                movement_pts(:, k) = generate_initial_conditions(1, 'Width', 2.5, 'Height', 1.5);
            end
        end

        dxi(1:2, activeRunners) = controller(x(1:2, activeRunners), movement_pts(1:2, activeRunners));

        dxu(1:2, :) = si_to_uni_dyn(dxi(1:2, :), x);
        dxu(1:2, :) = uni_barrier_certificate(dxu(1:2, :), x);
        
        if(tagEnable)
            for t = tagged
                dxu(:, t) = [0;0;0];
            end
        end
        
        r.set_velocities(1:N, dxu(1:2, :));
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

% Completed: game works! 
% TODO: better opponent movement?

r.debug();

