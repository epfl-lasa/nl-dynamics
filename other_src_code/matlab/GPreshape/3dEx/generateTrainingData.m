function data = generateTrainingData(type,varargin)
if(nargin>1)
    x0 = varargin{1};
else
    x0 = [4;3;-7];
end
if(nargin>2)
    rad = varargin{2};
else
    rad = 4;
end
if(strcmp(type,'escargot'))
    data = generateEscargotTrainingData(x0,rad);
elseif(strcmp(type,'circle'))
    data = generateCircularTrainingData(x0,rad);
elseif(strcmp(type,'spiral'))
    data = generateSpiralTrainingData(x0,rad);
end



end



function data = generateCircularTrainingData(x0,rad)
nData = 100;
t = linspace(0,2,nData);

z = repmat(x0(3),size(t));

x = x0(1)+rad.*sin(2*pi*t);
y = x0(2)+rad.*cos(2*pi*t);

zd = zeros(size(z));
xd = 2*pi*rad.*cos(2*pi*t)
yd = -2*pi*rad.*sin(2*pi*t);


data = [x;y;z;xd;yd;zd;t];

end

function data = generateSpiralTrainingData(x0,rad)
nData = 100;
t = linspace(0,2,nData);


z = x0(3)+t;

x = x0(1)+rad.*sin(2*pi*t);
y = x0(2)+rad.*cos(2*pi*t);

zd = ones(size(z));
xd = 2*pi*rad.*cos(2*pi*t)
yd = -2*pi*rad.*sin(2*pi*t);


data = [x;y;z;xd;yd;zd;t];

end

function data=generateEscargotTrainingData(x0,rad)
nData = 100;
t = linspace(0,2,nData);



z = x0(3)+2*t;

x = x0(1)+(1+t).*rad.*sin(2*pi*t);
y = x0(2)+(1+t).*rad.*cos(2*pi*t);

zd = 2*ones(size(z));
xd = (1+t).*2*pi*rad.*cos(2*pi*t) + rad.*sin(2*pi*t);
yd = -(1+t).*2*pi*rad.*sin(2*pi*t)+rad.*cos(2*pi*t);


data = [x;y;z;xd;yd;zd;t];
end