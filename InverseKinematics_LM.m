%{ 
基于牛顿-拉普逊法和L-M法的逆向运动学求解器
%}

function err_norm = InverseKinematics_LM(to, Target)
% Levenberg-Marquardt, Chan-Lawrence, Sugihara's modification
% 添加关节运动限制
global uLINK

idx = FindRoute(to);
wn_pos = 1/0.3;
wn_ang = 1/(2*pi);
We = diag([wn_pos wn_pos wn_pos wn_ang wn_ang wn_ang]);
% Wn = eye(length(idx));

ForwardKinematics(1);
err = CalcVWerr(Target, uLINK(to));
Ek = err'*We*err;

for n = 1:20
  J  = CalcJacobian(idx);

  alpha = ones(length(idx),1);
  for i=1:length(idx)
      j = idx(i);
      if isfield(uLINK(j), 'qmin') && isfield(uLINK(j), 'qmax')
          q = uLINK(j).q;
          q_min = deg2rad(uLINK(j).qmin);
          q_max = deg2rad(uLINK(j).qmax);
          d_low = q - q_min;
          d_high = q_max - q;
          d = min(d_low, d_high);
          threshold = deg2rad(5); % Threshold to activate damping
          if d < threshold
             alpha(i) = 1 + (threshold - d)/threshold * 9; % Scale damping
          end
      end
  end

  Wn = diag(alpha) * (Ek + 0.002); % Apply adaptive damping

  Jh = J'*We*J + Wn;  %Hk + wn
  gerr = J'*We*err;    %gk
  dq   = Jh \ gerr;    %new
  
  MoveJointsWithLimits(idx, dq);
  ForwardKinematics(1);
  err = CalcVWerr(Target, uLINK(to));
  Ek2 = err'*We*err;
  if Ek2 < 1E-12
      break;
  elseif Ek2 < Ek
      Ek = Ek2;
  else
      MoveJointsWithLimits(idx, -dq);  % revert
      ForwardKinematics(1);
      break, 
  end
end

if nargout == 1 
    err_norm = norm(err);
end
