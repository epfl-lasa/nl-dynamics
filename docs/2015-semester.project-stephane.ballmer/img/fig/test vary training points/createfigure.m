function createfigure(X1, Y1, X2, Y2, X3, Y3, X4, Y4, X5, Y5, X6, Y6, X7, Y7)
% Create figure
figure('PaperSize',[20.99999864 29.69999902]);

% Create axes
axes1 = axes;
box(axes1,'on');
hold(axes1,'on');

% Create ylabel
ylabel('computation time (ms)','Margin',2);

% Create xlabel
xlabel('number of train points','Margin',2);

% Create plot
plot(X1,Y1,'ZDataSource','','DisplayName','continuous GPR, smoothing: 1',...
    'Color',[1 0 0]);

% Create plot
plot(X2,Y2,'ZDataSource','','DisplayName','continuous GPR, smoothing: 0.5',...
    'Color',[0 0 0]);

% Create plot
plot(X3,Y3,'ZDataSource','','DisplayName','continuous GPR, smoothing: 0.2',...
    'Color',[1 0 1]);

% Create plot
plot(X4,Y4,'ZDataSource','','DisplayName','continuous GPR, smoothing: 0.1',...
    'Color',[0.87058824300766 0.490196079015732 0]);

% Create plot
plot(X5,Y5,'ZDataSource','','DisplayName','continuous GPR, smoothing: 0.05',...
    'Color',[0 0 1]);

% Create plot
plot(X6,Y6,'ZDataSource','','DisplayName','continuous GPR, smoothing: 0.02',...
    'Color',[0.494117647409439 0.184313729405403 0.556862771511078]);

plot(X7,Y7,'ZDataSource','','DisplayName','discrete GPR',...
    'Color',[0 0.498039215803146 0]);

% Create legend
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.144236111111111 0.711780316141686 0.15875 0.17283950617284],...
    'EdgeColor',[0 0 0],...
    'FontSize',9);

