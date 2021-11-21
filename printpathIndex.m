function [pathindex] = printpathIndex(ind1,ind2,path)
%Recursively return flody algorithm path
node=path(ind1,ind2);

if node==ind1
    pathindex=ind2;

    return;
end
[pathindex] = printpathIndex(ind1,node,path);
% xpath=[xpath node];
pathindex=[pathindex printpathIndex(node,ind2,path)];

end
