%posible errors in this code: warnings
%other posible error: for some values of inputs cant find interception
%because of the minimal error

%determine the curvature on a point, using the following formula:
%curvature = [f''(x)]/[1+(f'(x))^2]^(3/2)

%hold off %to delete , just for testing ASDA

x=linspace(0,Vx*Tm,10000);
error = 0.05; %error for the interpolation

%a=5; %to delete , just for testing ASDA
%b=-10;%to delete , just for testing ASDAA
%c=2;%to delete , just for testing SADASD
 
%polynomial equation
f = a*x.^2 + b*x + c;
%df = 2*a*x + b;
%ddf = 2*a

%curvature equation
%fcurve = ddf/((1+df.^2).^(3/2));

%set starting point SETTTTTTTTTTTTTTTTTTTTTTTTTTT
yo = c;
xo = 0;

%get df on the initial point
dfinit = 2*a*xo+b;

%get the relative yaw between the road and the car
yawDev = arctan(dfinit);

%setup curvature vector
curvature = zeros(predHorizon, 1);

%if a is equal to zero there is no curvature
if a ~= 0

    %get the estimated curvature for all the steps
    for i = 1:predHorizon

        %circle equation
        fcirclep = yo + sqrt((Vx*timeSample)^2 - (x-xo).^2);
        fcirclen = yo - sqrt((Vx*timeSample)^2 - (x-xo).^2);

        %plot(x,f)%to delete , just for testing ASDA
        %hold on%to delete , just for testing ASDA
        %plot(x,fcirclep)%to delete , just for testing ASDA
        %plot(x,fcirclen)%to delete , just for testing ASDA

        %find first intersection with an error of 'error'
        Intersectionsp=find(abs(f-fcirclep)<=(error),1,'last');
        Intersectionsn=find(abs(f-fcirclen)<=(error),1,'last');
        X_Valuesp=x(Intersectionsp);
        X_Valuesn=x(Intersectionsn);

        %find which side of the circle the equation intersected
        if isempty(X_Valuesn)
            xnext = X_Valuesp;
        elseif isempty(X_Valuesp)
            xnext = X_Valuesn;   

        %intercepted on both sides, see derivative
        else
            if (2*a*xo + b) >= 0
                xnext = X_Valuesp;
            else 
                xnext = X_Valuesn;
            end
        end
        ynext = a*xnext^2 + b*xnext + c;

        %calculate curvature
        curvature(i) = (2*abs(a))/((1+(2*a*xo+b)^2)^(3/2));

        %see te direction of curvature
        if (2*a*xo+b) < 0
            curvature(i) = -curvature(i);
        end
        %next starting point
        xo=xnext;
        yo=ynext;
    end
end