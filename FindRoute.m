%{
返回结构二叉树上指定节点的所有直系母节点编号
%}

function idx = FindRoute(to)
global uLINK

i = uLINK(to).mother;
if i == 1
    idx = [to];
else
    idx = [FindRoute(i) to];
end