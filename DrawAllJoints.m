%{
绘制所有关节
%}


function DrawAllJoints(j)
global uLINK
radius    = 0.02;
len       = 0.06;
joint_col = 0;

if j ~= 0  
    if ~isempty(uLINK(j).vertex)
        vert = uLINK(j).R * uLINK(j).vertex;
        for k = 1:3
            vert(k,:) = vert(k,:) + uLINK(j).p(k); % adding x,y,z to all vertex
        end
%         tic;
        DrawPolygon(vert, uLINK(j).face,0);
%         fprintf('执行DrawPolygon时间: %.4f 秒\n', toc);
    end
    
    hold on
    
    i = uLINK(j).mother;
    if i ~= 0
%         tic;
        Connect3D(uLINK(i).p,uLINK(j).p,'k',2);
%         fprintf('执行Connect3D时间: %.4f 秒\n', toc);
    end
    DrawCylinder(uLINK(j).p, uLINK(j).R * uLINK(j).a, radius,len, joint_col);
    
    
    DrawAllJoints(uLINK(j).child);
    DrawAllJoints(uLINK(j).sister);
end

