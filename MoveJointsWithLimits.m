%{
带关节运动范围限制的驱动程序
%}

function MoveJointsWithLimits(idx, dq)
global uLINK;
for i = 1:length(idx)
    j = idx(i);
    new_q = uLINK(j).q + dq(i);
    % Enforce joint limits
    if isfield(uLINK(j), 'qmin')
        new_q = max(new_q, deg2rad(uLINK(j).qmin));
    end
    if isfield(uLINK(j), 'qmax')
        new_q = min(new_q, deg2rad(uLINK(j).qmax));
    end
    uLINK(j).q = new_q;
end
end
