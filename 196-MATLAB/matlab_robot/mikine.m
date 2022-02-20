%SerialLink.ikine Inverse kinematics by optimization without joint limits
%
% Q = R.ikine(T) are the joint coordinates (1xN) corresponding to the robot
% end-effector pose T which is an SE3 object or homogenenous transform
% matrix (4x4), and N is the number of robot joints.
%
% This method can be used for robots with any number of degrees of freedom.
%
% Options::
% 'ilimit',L        maximum number of iterations (default 500)
% 'rlimit',L        maximum number of consecutive step rejections (default 100)
% 'tol',T           final error tolerance (default 1e-10)
% 'lambda',L        initial value of lambda (default 0.1)
% 'lambdamin',M     minimum allowable value of lambda (default 0)
% 'quiet'           be quiet
% 'verbose'         be verbose
% 'mask',M          mask vector (6x1) that correspond to translation in X, Y and Z, 
%                   and rotation about X, Y and Z respectively.
% 'q0',Q            initial joint configuration (default all zeros)
% 'search'          search over all configurations
% 'slimit',L        maximum number of search attempts (default 100)
% 'transpose',A     use Jacobian transpose with step size A, rather than
%                   Levenberg-Marquadt
%
% Trajectory operation::
%
% In all cases if T is a vector of SE3 objects (1xM) or a homogeneous
% transform sequence (4x4xM) then returns the joint coordinates
% corresponding to each of the transforms in the sequence.  Q is MxN where
function qt = mikine(robot, tr,mask, varargin)
%mask表示当自由度小于6时，忽略部分的关节，1表示激活，0表示忽略
    n = robot.n;
    
    TT = SE3.check(tr);
        
    %  set default parameters for solution
    opt.ilimit = 500;
    opt.rlimit = 100;
    opt.slimit = 100;
    opt.tol = 1e-10;
    opt.lambda = 0.1;
    opt.lambdamin = 0;
    opt.search = false;
    opt.quiet = false;
    opt.verbose = false;
    opt.mask = mask;
    opt.q0 = zeros(1, n);
    opt.transpose = NaN;
    
    [opt,args] = tb_optparse(opt, varargin);
    
    if opt.search
        % randomised search for a starting point
        
        opt.search = false;
        opt.quiet = true;
        %args = args{2:end};
        
        for k=1:opt.slimit
            for j=1:n
                qlim = robot.links(j).qlim;
                if isempty(qlim)
                    if robot.links(j).isrevolute
                        q(j) = rand*2*pi - pi;
                    else
                        error('For a prismatic joint, search requires joint limits');
                    end
                else
                    q(j) = rand*(qlim(2)-qlim(1)) + qlim(1);
                end
            end
            fprintf('Trying q = %s\n', num2str(q));
            
            q = robot.ikine2(tr, q, args{:}, 'setopt', opt);
            if ~isempty(q)
                qt = q;
                return;
            end
        end
        error('no solution found, are you sure the point is reachable?');
        qt = [];
        return
    end
    
    assert(numel(opt.mask) == 6, 'RTB:ikine:badarg', 'Mask matrix should have 6 elements');
    assert(n >= numel(find(opt.mask)), 'RTB:ikine:badarg', 'Number of robot DOF must be >= the same number of 1s in the mask matrix');
    W = diag(opt.mask);
    
    
    qt = zeros(length(TT), n);  % preallocate space for results
    tcount = 0;              % total iteration count
    rejcount = 0;            % rejected step count
    
    q = opt.q0;
    
    failed = false;
    revolutes = robot.isrevolute();
    
    for i=1:length(TT)
        T = TT(i);
        lambda = opt.lambda;

        iterations = 0;
        
        if opt.debug
            e = tr2delta(robot.fkine(q), T);
            fprintf('Initial:  |e|=%g\n', norm(W*e));
        end
        
        while true
            % update the count and test against iteration limit
            iterations = iterations + 1;
            if iterations > opt.ilimit
                if ~opt.quiet
                    warning('ikine: iteration limit %d exceeded (pose %d), final err %g', ...
                        opt.ilimit, i, nm);
                end
                failed = true;
                break
            end
            
            e = tr2delta(robot.fkine(q), T);
            
            % are we there yet
            if norm(W*e) < opt.tol
                break;
            end
            
            % compute the Jacobian
            J = jacobe(robot, q);
            
            JtJ = J'*W*J;
            
            if ~isnan(opt.transpose)
                % do the simple Jacobian transpose with constant gain
                dq = opt.transpose * J' * e;
            else
                % do the damped inverse Gauss-Newton with Levenberg-Marquadt
                dq = inv(JtJ + (lambda + opt.lambdamin) * eye(size(JtJ)) ) * J' * W * e;
                
                % compute possible new value of
                qnew = q + dq';
                
                % and figure out the new error
                enew = tr2delta(robot.fkine(qnew), T);
                
                % was it a good update?
                if norm(W*enew) < norm(W*e)
                    % step is accepted
                    if opt.debug
                        fprintf('ACCEPTED: |e|=%g, |dq|=%g, lambda=%g\n', norm(W*enew), norm(dq), lambda);
                    end
                    q = qnew;
                    e = enew;
                    lambda = lambda/2;
                    rejcount = 0;
                else
                    % step is rejected, increase the damping and retry
                    if opt.debug
                        fprintf('rejected: |e|=%g, |dq|=%g, lambda=%g\n', norm(W*enew), norm(dq), lambda);
                    end
                    lambda = lambda*2;
                    rejcount = rejcount + 1;
                    if rejcount > opt.rlimit
                        if ~opt.quiet
                            warning('ikine: rejected-step limit %d exceeded (pose %d), final err %g', ...
                                opt.rlimit, i, norm(W*enew));
                        end
                        failed = true;
                        break;
                    end
                    continue;  % try again
                end
            end
            
            
            % wrap angles for revolute joints
            k = (q > pi) & revolutes;
            q(k) = q(k) - 2*pi;
            
            k = (q < -pi) & revolutes;
            q(k) = q(k) + 2*pi;
            
            nm = norm(W*e);
            
            
        end  % end ikine solution for this pose
        qt(i,:) = q';
        tcount = tcount + iterations;
        if opt.verbose && ~failed
            fprintf('%d iterations\n', iterations);
        end
        if failed
            if ~opt.quiet
                warning('failed to converge: try a different initial value of joint coordinates');
            end
            qt = [];
        end
    end
    
    
    if opt.verbose && length(TT) > 1
        fprintf('TOTAL %d iterations\n', tcount);
    end
    
    
end
