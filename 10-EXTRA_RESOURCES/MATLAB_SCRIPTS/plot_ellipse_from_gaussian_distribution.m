function elpt = ellipsedata(covmat, center, numpoints, sigmarule, varargin)

%% Ellipsedata V1.001
%
% Construct data points of ellipses representing contour curves of Gaussian
% distributions with any covariance and mean value.
%
%% Example
%
% In this example, the funcion ellipsedata constructs three ellipses of 100
% points each representing the contour curves corresponding to standard deviations
% of 1, 2 and 3 for a Gaussian distribution with covariance matrix given by
% [4,1;1,1] and mean value given by [3,3].
%
% elpt=ellipsedata([4,1;1,1],[3,3],100,[1,2,3]);
%
% The results can be plot as follows
%
% plot(elpt(:,1:2:end),elpt(:,2:2:end));
%
%% Input arguments
%
%   covmat:
%       Covariance matrix of a bivariate Gaussian distribution. Must be of
%       size 2x2, symmetric and positive definite. If the format is not
%       correct, an error is triggered. If the matrix is not symmetric, it
%       is symmetrized by adding its transpose and dividing by 2.
%
%   center:
%       The center (mean value) of the bivariate Gaussian distribution. If
%       the format is not correct, it is set to [0,0].
%
%   numpoints:
%       The number of points that each ellipse will be composed of. Must be
%       a positive integer number. If it is not numeric or positive, it is
%       set to 100. If it is not integer, it is converted to integer using
%       the function ceil.
%
%   sigmarule:
%       Vector of real numbers indicating the proportion of standard
%       deviation surrounded by each ellipse.
%
%   varargin (later assigned to "zeroprecision"):
%       A real number indicating the maximum difference after which two
%       numbers are considered different. This value is used for assessing
%       whether covmat is symmetric. If not specified, it is set to 1E-12
%
%% Output arguments
%
%   elpt:
%       Matrix in which each consecutive pair of columns represent an
%       ellipse corresponding to a different value of sigmarule, in the
%       order they were given in the input.
%
%% Version control
%
% V1.001: Changes are: (1) Previous version trigger an error if the matrix was not
% symmetric. In this version, if the matrix is not symmetric, it is
% symmetrized. (2) The last input argument is assigned to a variable called
% "zeroprecision" which controls the extent to which two numbers are
% considered different or equal. This value is used to assess whether
% covmat is symmetric or not.
%
%
%% Please, report any bugs to Hugo.Eyherabide@cs.helsinki.fi
%
% Copyright (c) 2014, Hugo Gabriel Eyherabide, Department of Mathematics
% and Statistics, Department of Computer Science and Helsinki Institute
% for Information Technology, University of Helsinki, Finland.
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions 
% are met:
% 
% 1. Redistributions of source code must retain the above copyright 
% notice, this list of conditions and the following disclaimer.
% 
% 2. Redistributions in binary form must reproduce the above copyright 
% notice, this list of conditions and the following disclaimer in the 
% documentation and/or other materials provided with the distribution.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
% HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
% SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
% TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
% OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
% OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
% OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


%% Check format of initial parameters
warningmessage=@(varname)warning(['Format of "' varname '" incorrect. Setting "' varname '" to default.']);

if isempty(varargin) || ~isnumeric(varargin{1}) || length(varargin{1})~=1,
    zeroprecision=1E-12;
else
    zeroprecision=varargin{1};
end

if zeroprecision<0, zeroprecision=-zeroprecision; end

if ~isnumeric(covmat) || size(covmat,1)~=size(covmat,2) || det(covmat)<0, 
    error('The argument "covmat" is not a covariance matrix');
end

if abs(covmat(1,2)-covmat(2,1))>zeroprecision,
    warning('The matrix "covmat" is not symmetric, and it has been symmetrized by adding its transpose and divided by 2');
    covmat=(covmat+covmat')/2;
end

if ~isnumeric(center) || length(center)~=2,
    warningmessage('center');
    center=[0;0];
end

if ~isnumeric(numpoints) || length(numpoints)~=1 || numpoints<1,
    warningmessage('numpoints');
    numpoints=100;
end

if ~isnumeric(sigmarule),
    warningmessage('sigmarule');
    sigmarule=3;
end


%% Calculations start here

% Converting input arguments into column vectors
center=center(:)';
sigmarule=sigmarule(:)';
numpoints=ceil(numpoints);

% Calculate the orthonormal eigenvectors of the covariance matrix (directions of the ellipses' principal axes)
% and the eigenvalues of the covariance matrix (length of the ellipses' principal axes).
[U, Lambdas] = eig(covmat);
Lambdas = diag(Lambdas).^.5;

% Chooses points
theta = linspace(0, 2*pi, numpoints)';

% Construct ellipse
elpt = [cos(theta), sin(theta)]*diag(Lambdas)*U';
numsigma = length(sigmarule);
elpt = repmat(elpt, 1, numsigma).*repmat(sigmarule(floor(1:.5:numsigma+.5)), numpoints, 1);
elpt = elpt + repmat(center, numpoints, numsigma);
end

