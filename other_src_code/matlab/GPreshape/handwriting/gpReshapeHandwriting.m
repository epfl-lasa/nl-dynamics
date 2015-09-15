
function gpReshapeHandwriting(ind, ell,ty)
% SShapeM4, ZShapeM3, NShapeM3, WShapeM4
letters = {'SShapeM4','ZShapeM3','NShapeM3','WShapeM4'};
load(letters{ind})
%[Priors, Mu, Sigma,Data] = learnSEDSModel(dd,4);
figure(1);clf;

if(strcmp(ty,'SEDS'))
    interactiveHandWritingSEDS(dd,Priors,Mu,Sigma,ell)   
elseif(strcmp(ty,'LINEAR'))
    interactiveHandWritingLINEAR(dd,ell)
end
%save('WShapeM4','dd','Priors','Mu','Sigma')
end



function [Priors, Mu, Sigma,Data] = learnSEDSModel(dd,K)

dt = 0.1; %time step of demonstration
tol_cutting = 1; % a threshold on velocity that will be used for trimming demos
nDemos = size(dd,2);
trajDemos = {};
t = {};
for i = 1:nDemos 
    trajDemos{i} = dd{i}.pos;
    t{i} = dd{i}.t;
end
[x0 , xT, Data, index] = preprocess_demos(trajDemos,t,tol_cutting);
%K = 4; %Number of Gaussian funcitons
% A set of options that will be passed to the solver. Please type 
% 'doc preprocess_demos' in the MATLAB command window to get detailed
% information about each option.
options.tol_mat_bias = 10^-6;
options.perior_opt = 1;
options.mu_opt = 1;
options.sigma_x_opt = 1;
options.display = 1;
options.tol_stopping=10^-10;
options.max_iter = 200;
options.normalization = 1;
options.objective = 'mse';

[Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,K); %finding an initial guess for GMM's parameter
[Priors Mu Sigma]=SEDS_Solver(Priors_0,Mu_0,Sigma_0,Data,options); %running SEDS optimization solvere

end