%{
计算改进预览控制中，控制输入所需的增益
2007-General ZMP Preview Control for Bipedal Walking
Ks: 标量增益
Kx：反馈增益
G: 预测增益权重
%}

function [Ks,Kx,G] = improved_preview_control_params(A,B,C,Q,R,N_preview)

%%构造增广矩阵
CA = C*A;
CB = C*B;
A_tilde = [1 CA(1) CA(2) CA(3);
           0 A(1,1) A(1,2) A(1,3);
           0 A(2,1) A(2,2) A(2,3);
           0 A(3,1) A(3,2) A(3,3)];

B_tilde = [CB;
           B(1);
           B(2);
           B(3)];

C_tilde = [1 0 0 0];

%% 求解新的Riccati离散方程
[P_tilde,~,~] = dare(A_tilde,B_tilde,C_tilde'*Q*C_tilde,R);

K_tilde = (R+B_tilde'*P_tilde*B_tilde)\(B_tilde'*P_tilde*A_tilde);

% 参数分解
Ks = K_tilde(1,1);
Kx = K_tilde(1,2:end);

%% 计算预览增益
Ac_tilde = A_tilde-B_tilde*K_tilde;

G = zeros(1,N_preview);
G(1,1) = -Ks;

I_tilde = [1;0;0;0];
X_tilde = -Ac_tilde'*P_tilde*I_tilde;

for i =2:N_preview
    G(1,i) = (R+B_tilde'*P_tilde*B_tilde)\(B_tilde'*X_tilde);
    X_tilde = Ac_tilde'*X_tilde;

end
end

