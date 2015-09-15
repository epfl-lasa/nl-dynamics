function K = covTP(hyp, x, z, i)

% Squared Exponential covariance function with isotropic distance measure. The
% covariance function is parameterized as:
%
% k(x^p,x^q) = sf^2 * exp(-(x^p - x^q)'*inv(P)*(x^p - x^q)/2) 
%
% where the P matrix is ell^2 times the unit matrix and sf^2 is the signal
% variance. The hyperparameters are:
%
% hyp = [ log(ell)
%         log(sf)  ]
%
% For more help on design of covariance functions, try "help covFunctions".
%
% Copyright (c) by Carl Edward Rasmussen and Hannes Nickisch, 2010-09-10.
%
% See also COVFUNCTIONS.M.

if nargin<2, K = '2'; return; end                  % report number of parameters
if nargin<3, z = []; end                                   % make sure, z exists
xeqz = numel(z)==0; dg = strcmp(z,'diag') && numel(z)>0;        % determine mode

R = exp(hyp(1));                                 % characteristic length scale
                                         % signal variance

% precompute squared distances
if dg                                                               % vector kxx
  K = zeros(size(x,1),1);
else
  if xeqz                                                 % symmetric matrix Kxx
    K = sq_dist(x');
  else                                                   % cross covariances Kxz
    K = sq_dist(x',z');
  end
end


if nargin<4                                                        % covariances
    K = sqrt(K);
    K = 1/12*(2.*K.^3-3.*R.*K.^2+R.^3);
else                                                               % derivatives
  if i==1
    K = 0;%sf2*exp(-K/2).*K;
  elseif i==2
    K = 0;%2*sf2*exp(-K/2);
  else
    error('Unknown hyperparameter')
  end
end