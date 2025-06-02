%{
定义机器全身关节、连杆参数
reference--《人形机器人》梶田秀司
%}

clc;
clear;

global uLINK

ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

%% 盆骨
uLINK = struct('name','Pelvic'    , 'm', 0.49967447, 'sister', 0, 'child', 2, 'b',[0  0    0]','a',UZ,'q',0,'qmin',-180,'qmax',180);

%% 左腿
uLINK(2) = struct('name','LL1' , 'm',  1.46421655, 'sister', 8, 'child', 3, 'b',[-0.0004969764, 0.0968081154, -0.0462185834]'   ,'a',[0.0 -0.7071 0.7071]','q',0,'qmin',-180,'qmax',180);
uLINK(3) = struct('name','LL2' , 'm',  1.43511075, 'sister', 0, 'child', 4, 'b',[0.0486984662, 0.0371170153, -0.0369654141]'   ,'a',UX,'q',0,'qmin',-180,'qmax',180);
uLINK(4) = struct('name','LL3' , 'm',  1.24331784, 'sister', 0, 'child', 5, 'b',[-0.0487999997, -0.0000071281, -0.0710049157]'   ,'a',UZ,'q',0,'qmin',-180,'qmax',180);
uLINK(5) = struct('name','LL4' , 'm',  1.99248663, 'sister', 0, 'child', 6, 'b',[0.0000024076, -0.0286144461, -0.1547928217]' ,'a',UY,'q',0,'qmin',-180,'qmax',180);
uLINK(6) = struct('name','LL5' , 'm',  0.44373732, 'sister', 0, 'child', 7, 'b',[-0.0000023253, 0.0268038758, -0.3051470408]' ,'a',UY,'q',0,'qmin',-43,'qmax',25);
uLINK(7) = struct('name','LL6' , 'm',  0, 'sister', 0, 'child', 0, 'b',[0  0   0  ]' ,'a',UX,'q',0,'qmin',-13,'qmax',9);

%% 右腿
uLINK(8) = struct('name','RL1' , 'm', 1.46421655, 'sister', 14, 'child', 9, 'b',[-0.0005528253, -0.0969021090, -0.0461286249]'   ,'a',[0.0 0.7071 0.7071]','q',0,'qmin',-180,'qmax',180);
uLINK(9) = struct('name','RL2' , 'm',  1.43511075, 'sister', 0, 'child',10, 'b',[0.0491665000, -0.0370240855, -0.0370553726]'   ,'a',UX,'q',0,'qmin',-180,'qmax',180);
uLINK(10)= struct('name','RL3' , 'm',  1.24331784, 'sister', 0, 'child',11, 'b',[-0.0491999999, -0.0000023291, -0.0710049159]'   ,'a',UZ,'q',0,'qmin',-180,'qmax',180);
uLINK(11)= struct('name','RL4' , 'm',  1.99248663, 'sister', 0, 'child',12, 'b',[-0.0000012162, 0.0267356444, -0.1552407853]' ,'a',UY,'q',0,'qmin',-180,'qmax',180);
uLINK(12)= struct('name','RL5' , 'm',  0.44373732, 'sister', 0, 'child',13, 'b',[0.0000016414, -0.0315804320, -0.3046375595]' ,'a',UY,'q',0,'qmin',-43,'qmax',26);
uLINK(13)= struct('name','RL6' , 'm',  0, 'sister', 0, 'child', 0, 'b',[0  0   0  ]' ,'a',UX,'q',0,'qmin',-10,'qmax',12);

%% 核心躯干
uLINK(14)= struct('name','Waist' , 'm',  1.64200933, 'sister', 0, 'child',15, 'b',[-0.0004658069, -0.0000024949, 0.0881420269]' ,'a',UZ,'q',0,'qmin',-180,'qmax',180);
uLINK(15)= struct('name','Chest' , 'm',  2.68573919, 'sister', 0, 'child',16, 'b',[-0.0000266108, 0.0002500000, 0.0385000000]' ,'a',UY,'q',0,'qmin',-180,'qmax',180);

%% 左臂
uLINK(16) = struct('name','LA1' , 'm',  0.9448603, 'sister', 21, 'child',17, 'b',[0.0000000000, 0.1461000000, 0.1810000000]'   ,'a',UY,'q',0,'qmin',-180,'qmax',180);
uLINK(17)= struct('name','LA2' , 'm',  1.00127281, 'sister', 0, 'child',18, 'b',[0.0331000000, 0.0587000000, 0.0000000000]'   ,'a',UX,'q',0,'qmin',-180,'qmax',180);
uLINK(18)= struct('name','LA3' , 'm',  0.17656212, 'sister', 0, 'child',19, 'b',[-0.0326000000, 0.0000000000, -0.1302000000]' ,'a',UZ,'q',0,'qmin',-180,'qmax',180);
uLINK(19)= struct('name','LEL' , 'm',  0.25959783, 'sister', 0, 'child',20, 'b',[-0.0005000000, 0.0360000000, -0.0315000000]' ,'a',UY,'q',0,'qmin',-180,'qmax',180);
uLINK(20)= struct('name','LW' , 'm',  0.6630308, 'sister', 0, 'child', 0, 'b',[-0.0000000000, -0.0360000000, -0.1220000000  ]' ,'a',UZ,'q',0,'qmin',-180,'qmax',180);

%% 右臂
uLINK(21) = struct('name','RA1' , 'm',  0.9448603, 'sister', 0, 'child',22, 'b',[0.0000000000, -0.1485000000, 0.1810000000]'   ,'a',UY,'q',0,'qmin',-180,'qmax',180);
uLINK(22)= struct('name','RA2' , 'm',  1.00127281, 'sister', 0, 'child',23, 'b',[0.0328000000, -0.0567000000, 0.0000000000]'   ,'a',UX,'q',0,'qmin',-180,'qmax',180);
uLINK(23)= struct('name','RA3' , 'm',  0.17656212, 'sister', 0, 'child',24, 'b',[-0.0328000000, 0.0000000000, -0.1312000000]' ,'a',UZ,'q',0,'qmin',-180,'qmax',180);
uLINK(24)= struct('name','REL' , 'm',  0.25959783, 'sister', 0, 'child',25, 'b',[-0.0000000000, -0.0380000000, -0.0305000000]' ,'a',UY,'q',0,'qmin',-180,'qmax',180);
uLINK(25)= struct('name','RW' , 'm',  0.6630308, 'sister', 0, 'child', 0, 'b',[0.0000000000, 0.0380000000, -0.1220000000]' ,'a',UZ,'q',0,'qmin',-180,'qmax',180);

%% 创建足部模型
[uLINK(7).vertex,uLINK(7).face]   = MakeBox([0.2 0.1 0.02] ,[0.05  0.05 0.05]);    
[uLINK(13).vertex,uLINK(13).face] = MakeBox([0.2 0.1 0.02] ,[0.05  0.05 0.05]); 

%% 绑定母连杆
FindMother(1);

%% 建立连杆名称与ID之间的哈希图
for n=1:length(uLINK)
    eval([uLINK(n).name,'=',num2str(n),';']);
end

%% 指定盆骨的位姿，初始化所有关节与连杆
uLINK(Pelvic).p = [4.0786595856E-05, -5.8915467832E-03, 0.6473743781]'; % 指定盆骨在世界坐标系中的位置
uLINK(Pelvic).R = eye(3); % 初始状态的姿态
ForwardKinematics(1);

%% 指定骨盆的运动学参数，初始化所有关节与连杆
uLINK(Pelvic).v = [0 0 0]'; % 线速度
uLINK(Pelvic).w = [0 0 0]'; % 角速度
for n=1:length(uLINK)
    uLINK(n).dq     = 0;            % joitn speed   [rad/s]
end





