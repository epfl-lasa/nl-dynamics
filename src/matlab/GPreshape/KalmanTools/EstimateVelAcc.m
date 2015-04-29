

function [R,V,A]=EstimateVA_P(P,Q,R,dt)

dim = size(P,1);
% A is 3*dim X 3*dim
A = [];
for i=1:dim
    newRow = zeros(3*dim);
    newRow(i)=1;
    newRow(dim+i)=dt;
    newRow(2*dim+i)=dt^2/2;
    A = [A;newRow];
end








