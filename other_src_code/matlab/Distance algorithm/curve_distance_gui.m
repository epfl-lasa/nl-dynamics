

% -This program create a gui in order to test 3 different point-to-spline
% distance algorithm : brute force, dichotomy and an analytic way. 

% -It's design to see in all case which one is the most accurate and see the
% computational cost.

% -click on the plot the change the testing point (red cercle), you will se
% the result by the black cross, green cercle and magenta plus. You can
% stay clicked, the program handle motion capture.

% -click on the button and click on the plot 4 times to choose which point
% the spline got to fit (to change the curve)

% -you can change the parameter of the algorithm by editting the editbox on
% the right (number of iteration and precision factor) and tab enter

% -you can get every position and distance by a right click on the plot.
% (displayed on matlab terminal)


% --------------- graphical function

%initialisation function
function init()
    clear all
    close all
    
    fig = figure('Position', [100, 100, 900, 700]);
    set(fig, 'resize', 'off')
    set(gcf,'UserData',[])
    s={};
    s.coefx=[];
    s.coefy=[];
    s.newcoord=[[0,0];[0,0];[0,0];[0,0]];
    s.selectingNewCoord = 0;
    s.it = 5000;
    s.epsilon = 0.0000001;
    s.t_brute = 0;
    s.t_analy = 0;
    s.t_dicho = 0;
    
    set(gcf,'WindowButtonDownFcn',[])
    set(gcf, 'WindowButtonDownFcn', @mouseDownCB);
    set(gcf,'WindowButtonUpFcn',@mouseUpCB);
    set(gcf,'WindowButtonMotionFcn',[]);
    
    %1 if you want to ask init data, 0 for default value
    ask = 0;
    
    if ask
        disp('for x axis : a.t^3 + b.t^2 + c.t + d');
        s.coefx(1) = input('enter a: ');
        s.coefx(2) = input('enter b: ');
        s.coefx(3) = input('enter c: ');
        s.coefx(4) = input('enter d: ');
        
        disp('for y axis : a.t^3 + b.t^2 + c.t + d');
        s.coefy(1) = input('enter a: ');
        s.coefy(2) = input('enter b: ');
        s.coefy(3) = input('enter c: ');
        s.coefy(4) = input('enter d: ');
    else
        s.coefx = [6.10, -9.73, 4.34, 0.10];
        s.coefy = [-1.57, 2.65, -0.23, 0.07];
    end
    
    s.t = linspace (0,1,500);
    s.fx = @(t) s.coefx(1).*t.^3+s.coefx(2).*t.^2+s.coefx(3).*t+s.coefx(4);
    s.fy = @(t) s.coefy(1).*t.^3+s.coefy(2).*t.^2+s.coefy(3).*t+s.coefy(4);
    
    s.xs = 0;
    s.ys = 0;
    
    s.axis_max = max([s.fx(0), s.fx(1), s.fy(0), s.fy(1)]);
    s.axis_min = min([s.fx(0), s.fx(1), s.fy(0), s.fy(1)]);
    
    set(gcf,'UserData',s)
    update();
end

%update graphics and recall distance algorithm
function update()
    %getting user data
    s = get(gcf, 'UserData');
    
    %clear everything
    clf; 
    h=subplot(1,1,1);
    set(h, 'Units', 'pixels', 'position', [50, 50, 770, 600]);
    hold on;
    
    %set axis to be squared
    axis([0, 1, 0, 1]);
    
    plot(s.fx(s.t),s.fy(s.t), 'b', s.xs, s.ys, 'ro')
    hold on

    %disp(sprintf('\n\nperforming brute force algorithm with %d iteration', it));
    tic
    s.t_brute = bruteforce([0, 1], s.fx, s.fy, s.xs, s.ys, s.it);
    brute_time = toc;

    %disp(sprintf('\nperforming dichotomic algorithm until a difference of %.7f', epsilon));
    tic
    s.t_dicho = dichotomic([0, 1], s.fx, s.fy, s.xs, s.ys, s.epsilon);
    dicho_time = toc;

    %disp(sprintf('\nperforming analysis algorithm'));
    tic
    s.t_analy = analysis([0,1], s.xs, s.ys, s.coefx, s.coefy);
    analy_time = toc;
    
    set(gcf, 'UserData', s);

    plot(s.fx(s.t_brute), s.fy(s.t_brute), 'go', 'lineWidth', 2, 'markersize', 10)
    %mystr = sprintf('bruteforce algo d:%f  x:%f  y:%f', dist(s.t_brute, f(s.t_brute), xs,ys), s.t_brute, f(s.t_brute));
    %text(s.t_brute, f(s.t_brute), strcat('\leftarrow', mystr))
    hold on

    plot(s.fx(s.t_dicho), s.fy(s.t_dicho), 'kx', 'lineWidth', 2, 'markersize', 10)
    %mystr = sprintf('dichotomic algo d:%f  x:%f  y:%f', dist(s.t_dicho, f(s.t_dicho), xs,ys), s.t_dicho, f(s.t_dicho));
    %text(s.t_dicho, f(s.t_dicho), strcat('\leftarrow', mystr))
    hold on

    plot(s.fx(s.t_analy), s.fy(s.t_analy), 'm+', 'lineWidth', 2, 'markersize', 10)
    %mystr = sprintf('analysis algo d:%f  x:%f  y:%f  t:%f', dist(t_anal, f(t_anal), xs,ys), t_anal, f(t_anal), t_anal);
    %text(t_anal, f(t_anal), strcat('\leftarrow', mystr))
    
    if s.t_analy < 0
        t = linspace (0, s.t_analy, 500);
        plot (s.fx(t), s.fy(t), 'k:')
    elseif s.t_analy > 1
        t = linspace (1, s.t_analy, 500);
        plot (s.fx(t), s.fy(t), 'k:')
    end
    
    leg=legend('curve', 'test point', 'brute force result', 'dichotomic result', 'analytic result');
    set(leg,'location','southeastoutside');
    hold off
    
    %PUSH BUTTON
    uicontrol('Style'              ,'pushbutton',...
              'String'             ,'set new curve',...
              'Units'              , 'pixels', ...
              'Position'           ,[700 600 150 50],...
              'Callback'           ,@newCurveCB);

    %SPLINE X
    uicontrol('Style'              ,'text',...
              'HorizontalAlignment','left', ...
              'String'             ,sprintf('spline x = %.2f .t^3+ %.2f .t^2+ %.2f .t+ %.2f',s.coefx),...
              'Units'              , 'pixels', ...
              'Fontsize'           , 7, ...
              'Position'           ,[660 550 240 18]);

    %SPLINE Y
    uicontrol('Style'              ,'text', ...
              'HorizontalAlignment','left', ...
              'String'             ,sprintf('spline y = %.2f .t^3+ %.2f .t^2+ %.2f .t+ %.2f',s.coefy),...
              'Units'              , 'pixels', ...
              'Fontsize'           , 7, ...
              'Position'           ,[660 532 240 18]);

    %COMPUTATIONAL COST LABEL
    uicontrol('Style'              ,'text', ...
              'HorizontalAlignment','left', ...
              'String'             ,sprintf('brute force:\ndichotomy:\nanalytic:\n'),...
              'Units'              , 'pixels', ...
              'Position'           ,[660 450 110 50]);

    %COMPUTATIONAL COST VALUE
    uicontrol('Style'              ,'text', ...
              'HorizontalAlignment','left', ...
              'String'             ,sprintf('%f s\n%f s\n%f s\n',brute_time, dicho_time, analy_time),...
              'Units'              , 'pixels', ...
              'Position'           ,[770 450 80 50]);

    %EDIT ALGO LABEL
    uicontrol('Style'              ,'text', ...
              'HorizontalAlignment','left', ...
              'String'             ,sprintf('brute force:\ndichotomy:'),...
              'Units'              , 'pixels', ...
              'Position'           ,[660 400 110 33]);
 
    %EDIT BRUTE FORCE
    uicontrol('Style'              ,'edit', ...
              'HorizontalAlignment','left', ...
              'String'             ,sprintf('%d', s.it),...
              'Units'              ,'pixels', ...
              'Position'           ,[770 415 100 16],...
              'Callback'           ,@forceItCb);
          
    %EDIT DICHOTOMIC
    uicontrol('Style'              ,'edit', ...
              'HorizontalAlignment','left', ...
              'String'             ,sprintf('%.7f', s.epsilon),...
              'Units'              ,'pixels', ...
              'Position'           ,[770 399 100 16],...
              'Callback'           ,@dichoEpsCB);
          
    %MOST ACCURATE ALGO
    db = dist(s.fx(s.t_brute), s.fy(s.t_brute), s.xs, s.ys);
    dd = dist(s.fx(s.t_dicho), s.fy(s.t_dicho), s.xs, s.ys);
    da = dist(s.fx(s.t_analy), s.fy(s.t_analy), s.xs, s.ys);
    
    str = '';
    if min([db, dd, da]) == db
        str = 'brute force';
    elseif min([db, dd, da]) == dd
        str = 'dichotomic';
    elseif min([db, dd, da]) == da
        str = 'analytic';
    end
    
    uicontrol('Style'              ,'text', ...
              'HorizontalAlignment','left', ...
              'String'             ,strcat(str, ' algorithm is the most accurate'),...
              'Units'              , 'pixels', ...
              'Position'           ,[660 330 230 40]);
end

function makeNewCurve()
    s = get(gcf, 'UserData');
    
    %spline fit at for point : [x1,x2,x3,x4], corresponding to the folowing t point: t=[0, 1/3, 2/3, 1]
    A = [[0,0,0,1];[(1/3)^3,(1/3)^2,1/3,1];[(2/3)^3,(2/3)^2,2/3,1];[1,1,1,1]];
    
    s.coefx = (A\s.newcoord(:,1))';
    s.coefy = (A\s.newcoord(:,2))';
    
    s.fx = @(t) s.coefx(1).*t.^3+s.coefx(2).*t.^2+s.coefx(3).*t+s.coefx(4);
    s.fy = @(t) s.coefy(1).*t.^3+s.coefy(2).*t.^2+s.coefy(3).*t+s.coefy(4);
    
    set(gcf, 'UserData', s);
    
    update();
end



% ------------------ call back function

function forceItCb(h,~)
    s = get(gcf, 'UserData');
    s.it = str2num(get(h, 'String'));
    set(gcf, 'UserData', s);
    
    if (s.it >= 100000)
        warndlg('Be carefull, computation time might be very low for high value of iteration','!! Warning !!')
    end
    
    update();
end

function dichoEpsCB(h,~)
    s = get(gcf, 'UserData');
    s.epsilon = str2double(get(h, 'String'));
    set(gcf, 'UserData', s);
    
    if (s.epsilon <= 1e-20)
        warndlg('Be carefull, computation time might be very low for low value of epsilon','!! Warning !!')
    end
    
    update();
end

function newCurveCB(~,~)
    disp('Click 4 times on the graph')
    
    s = get(gcf, 'UserData');
    s.selectingNewCoord = 1;
    set(gcf, 'UserData', s);
end

function mouseDownCB(~, ~)
    disp(['Click callback on main figure: ', get(gcf,'SelectionType')])
    
    %check if it is a leftclick
    if(strcmp(get(gcf,'SelectionType'),'normal'))
        %get all data
        s=get(gcf,'UserData');

        %get click coordinate
        x = get(gca,'Currentpoint');
        x = x(1,1:2);
        
        %if we are in a 'choosing a new curve' mode
        if s.selectingNewCoord > 0
            if s.selectingNewCoord == 1
                clf; 
                h=subplot(1,1,1);
                set(h, 'Units', 'pixels', 'position', [50, 50, 600, 600]);
                hold on;
                axis([0, 1, 0, 1]);
            end
            
            disp(sprintf('click number %d', s.selectingNewCoord))

            s.newcoord(s.selectingNewCoord,:) = x;
            plot(x(1), x(2), 'ko');
            text(x(1)+0.1,x(2),sprintf('%d/4', s.selectingNewCoord))
            s.selectingNewCoord = s.selectingNewCoord +1;
            
            set(gcf,'UserData',s);
            
            %check if the 4points were clicked
            if s.selectingNewCoord == 5
                s.selectingNewCoord = 0;
                set(gcf,'UserData',s);
                
                makeNewCurve();
            end
            
        %if we just click on the graph to get new point
        else
            set(gcf,'WindowButtonMotionFcn',@mouseMotionCB);
            
            s.xs=x(1);
            s.ys=x(2);

            update();
            
            set(gcf,'UserData',s);
        end
    % check for a middle click
    elseif (strcmp(get(gcf,'SelectionType'),'extend'))
        s = get(gcf, 'UserData');
        s.coefx = [6.10, -9.73, 4.34, 0.10];
        s.coefy = [-1.57, 2.65, -0.23, 0.07];
        s.fx = @(t) s.coefx(1).*t.^3+s.coefx(2).*t.^2+s.coefx(3).*t+s.coefx(4);
        s.fy = @(t) s.coefy(1).*t.^3+s.coefy(2).*t.^2+s.coefy(3).*t+s.coefy(4);
        set(gcf, 'UserData',s);
        update();
    % check right click
    elseif(strcmp(get(gcf,'SelectionType'),'alt'))
        clc;
        s = get(gcf, 'UserData');
        
        disp('xs')
        s.xs
        disp('ys')
        s.ys
        
        disp('x brute force')
        s.fx(s.t_brute)
        disp('y brute force')
        s.fy(s.t_brute)
        disp('distance brute force')
        dist(s.fx(s.t_brute), s.fy(s.t_brute), s.xs, s.ys)
        
        disp('x dichotomy')
        s.fx(s.t_dicho)
        disp('y dichotomy')
        s.fy(s.t_dicho)
        disp('distance dichotomy')
        dist(s.fx(s.t_dicho), s.fy(s.t_dicho), s.xs, s.ys)
        
        disp('x analytic')
        s.fx(s.t_analy)
        disp('y analytic')
        s.fy(s.t_analy)
        disp('distance analytic')
        dist(s.fx(s.t_analy), s.fy(s.t_analy), s.xs, s.ys)
    end
end

function mouseUpCB(~, ~)
    %desactivate mouse motion
    set(gcf,'WindowButtonMotionFcn',[]);
end

function mouseMotionCB(~, ~)
    %get all data
    s=get(gcf,'UserData');
    
    %get mouse coord
    x = get(gca,'Currentpoint');
    x = x(1,1:2);
    
    %set new coord for (x*, y*)
    s.xs=x(1);
    s.ys=x(2);

    %save change
    set(gcf,'UserData',s);
    
    %update graph
    update();
end



% -------------------- calculs functions

function d = dist(x1,y1,x2,y2)
    d = sqrt((x1-x2).^2 + (y1-y2).^2);
end

function t_r=bruteforce(t, fx, fy, xs, ys, it)
    x = linspace(t(1), t(2), it);
    
    distmin = dist (fx(t(1)), fy(t(1)), xs,ys);
    t_r = t(1);
    
    for i = x
        distance = dist(fx(i), fy(i), xs, ys);
        
        if distance < distmin
            distmin = distance;
            t_r=i;
        end
    end
end

function t_r=dichotomic(t, fx, fy, xs, ys, epsilon)
    p_1=t(1);
    p_2=t(2);
    
    while abs(p_1-p_2) > epsilon
        d_1 = dist(fx(p_1), fy(p_1), xs,ys);
        d_2 = dist(fx(p_2), fy(p_2), xs,ys);
        new = (p_1 + p_2) / 2;
        
        if d_1 > d_2
            p_1 = new;
        else
            p_2 = new;
        end
    end
    
    t_r = p_1;

end

function t_r = analysis(t, xs,ys, coefx, coefy)
    % equ to resolve : a.t^5 + b.t^4+c.t^3+d.t^2+e.t+f = 0
    
    AA=[coefx(1), coefy(1)];
    BB=[coefx(2), coefy(2)];
    CC=[coefx(3), coefy(3)];
    DD=[coefx(4), coefy(4)];
    x =[xs, ys];
    
    a=0; b=0; c=0; d=0; e=0; f=0;

    for i=1:2
        A = AA(i);
        B = BB(i);
        C = CC(i);
        D = DD(i);
        mx = x(i);
        
        a = a + 6*A^2;
        b = b + 10*A*B;
        c = c + 8*A*C + 4*B^2;
        d = d + 6*A*D + 6*B*C - 6*A*mx;
        e = e + 4*B*D + 2*C^2 - 4*mx*B;
        f = f + 2*C*D - 2*mx*C;
    end

    value = roots([a,b,c,d,e,f]);
    rvalue = [];
    
    for i = 1:5
        if imag(value(i)) == 0% && real(value(i))>=0 && real(value(i))<=1
            rvalue = [rvalue; real(value(i))];
        end
    end

    if size(rvalue, 1) == 1
        t_r = rvalue(1);
    else
       fx = @(t) coefx(1).*t.^3+coefx(2).*t.^2+coefx(3).*t+coefx(4);
       fy = @(t) coefy(1).*t.^3+coefy(2).*t.^2+coefy(3).*t+coefy(4);
       
       t_r = rvalue(1);
       dmin = dist(fx(t_r), fy(t_r), xs, ys);
       
       for i=2:size(rvalue, 1)
           d = dist(fx(rvalue(i)), fy(rvalue(i)), xs,ys);
           if (d<dmin)
               dmin=d;
               t_r=rvalue(i);
           end
       end
    end
    
end

