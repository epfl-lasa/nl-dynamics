function analyzeError(pos)
    %check data smoothing
%     splinesCompare(pos, 1/10, 1)
    
    %check graph of smoothing error(alpha)
    val=[];
    f=@(i) i;
    int=linspace(0,1,10000/size(pos,2));

    for i=int
        val = [val, splinesCompare(pos,f(i), 0)];
    end
    
    figure
    plot(f(int), val);
    xlabel('alpha')
    ylabel('error')
end

function ret = smoothData(nbPoint, alpha)
% return a vector of the index of the new point 
    if (nbPoint*alpha<2)
        ret=[1,nbPoint];
    else
        ret=round(linspace(1,nbPoint,nbPoint*alpha));
    end
end

function ret = errorCalc(x1,y1,x2,y2)
    ret=sqrt((x1-x2).^2+(y1-y2).^2);
end

function err = splinesCompare (pos, alpha, plotParameter)
    s=smoothData(size(pos,2), alpha);
    
    %getting 2 splines
    spline1=computeTrajectory(pos, 1);
    spline2=computeTrajectory(pos, alpha);
    
    t=linspace(0,1, spline1.nbPoints*30);

    %plot spline
    if (plotParameter == 1)
        figure
        plot(ppval(spline1.vect(1), t),ppval(spline1.vect(2), t),'r', 'lineWidth', 1.5);
        hold on;
        plot(pos(1,:),pos(2,:),'ro');                         
        hold on;
        plot(ppval(spline2.vect(1), t), ppval(spline2.vect(2), t), 'b');                    
        hold on;
        plot(pos(1,s),pos(2,s), 'bo');                      
        hold off;
        
        xlabel('x')
        ylabel('y')
        
        %figure for the error graph
        figure
    end
    
    %error calculation
    err=0;
    for i=1:size(spline2.vect(1).breaks, 2)-1
        
        taBegin=spline1.vect(1).breaks(s(i));
        taEnd  =spline1.vect(1).breaks(s(i+1));
        tbBegin=spline2.vect(1).breaks(i);
        tbEnd  =spline2.vect(1).breaks(i+1);
        
        f=@(x,y) errorCalc(ppval(spline1.vect(1), x),ppval(spline1.vect(2), x),ppval(spline2.vect(1), y),ppval(spline2.vect(2), y));
        
        %draw error graph
        if (plotParameter == 1)
            ta=linspace(taBegin, taEnd, spline1.nbPoints/alpha);
            tb=linspace(tbBegin, tbEnd, spline1.nbPoints/alpha);
            
            plot(ta,f(ta,tb), 'r');
            hold on
        end
        
        %integral calculation
        err=err+int_trap(f, taBegin, taEnd, tbBegin, tbEnd, 100);
        
    end
    
    if (plotParameter == 1)
        hold off
        xlabel('error between curve')
        ylabel('t')
    end
end

function res = int_trap(f,a,b,c,d,M)
% f is an f(x,y) function, x going to [a,b] and y going to [c,d] and M the
% number of of subintervals

    h1=(b-a)/M;
    x1=linspace(a,b,M);
    x2=linspace(c,d,M);

    % Integrale
    res = h1/2 * ( f(a,c) + f(b,d) ) + h1 * sum( f(x1(2:M), x2(2:M)) );
    
% Model inspired by the function trap, whitch is made to calculate integrale 
% with trapezoidale quadratique formula from the course numerical analysis 
% given by Fabio Nobile.
end

function splinesData = computeTrajectory(pos, alpha)
    %smooth and return splines data (using matlab splines)

    values=smoothData(size(pos,2), alpha);
    x=transpose(pos(1,values));
    y=transpose(pos(2,values));

    nbPoint = size(x,1);
    
    % INSERTED BY KLAS: %
    % less confusing parameterization
    t = linspace(0,1,nbPoint);
    % replace tx AND ty by t below and replace ttx and tty in plot function
    % by linspace(0,1,nbPoints)
    % END KLAS%
    
    %spline data calculation
    splinesData.vect=[spline(t,x) spline(t,y)];
    splinesData.nbPoints = nbPoint;
end


