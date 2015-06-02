function [ ] = provideDemonstrations(varargin)

    global demonstration_data;
    demonstration_data = [];
    
    % Get words the person will demonstrate
    words = parse_args(varargin);
    for w = words
        disp(w)
    end
    
    figHandle = figure('name', 'Demonstrations');
    axes()
    dynamics = @(x) originalDynamicsLINEAR(x);
    setup_global_demo_figure(figHandle, dynamics, words);
    
end

function ret = demonstrationsCallback(h, e, words)
    disp(['Click callback on global demonstrations: ', get(gcf, 'selectiontype')])

    if(strcmp(get(gcf,'SelectionType'),'alt'))
        startDemonstration();
    elseif(strcmp(get(gcf,'SelectionType'),'normal'))
        saveDemonstrations(words);
    end
end

function ret = saveDemonstrations(words)
    global demonstration_data;    
    for w = words
        savePointsForWord(demonstration_data, w)
    end
end

function ret = savePointsForWord(data, word)
    word = char(word);
    fname = sprintf('data/%s.dat', word);
    save(fname, 'data')
    
    fprintf(1, 'Saved %d points to %s\n', size(data,2), fname)
end

function ret = startDemonstration()
    disp('started demonstration')
    set(gcf,'WindowButtonUpFcn',@stopDemonstration);
    set(gcf,'WindowButtonMotionFcn',@recordPoint);
end

function ret = stopDemonstration(h,e)
    disp('stopped demonstration')
    set(gcf,'WindowButtonMotionFcn',[]);
    %processNewData();
    %updateStreamlines(h);
    
    
    set(gcf,'WindowButtonUpFcn',[]);
    
    global demonstration_data;
    fprintf(1, 'Have %d demo data\n', size(demonstration_data, 2));
end

function ret = recordPoint(h,e)
    global demonstration_data;
    x = get(gca,'Currentpoint');
    x = x(1,1:2)';
    demonstration_data(:, end+1) = x;
    hold on
    plot(x(1),x(2),'r.')
end


function setup_global_demo_figure(figureHandle, dynamics, words)

    ax = findall(figureHandle,'type','axes');

    nX = 100;
    x = linspace(-200,200,nX);
    y = linspace(-200,200,nX);
    [xM, yM] = meshgrid(x,y);
    X = [xM(:)';yM(:)'];
    Xd = dynamics(X);
    hs = streamslice(ax, xM,yM,reshape(Xd(1,:),nX,nX),reshape(Xd(2,:),nX,nX),0.5);
    
    set(figureHandle, 'WindowButtonDownFcn', @(h,e)demonstrationsCallback(h,e, words));
end

function words = parse_args(args)

    words = {};
    for w = args
        words(end+1) = w;
    end    
end

    