disp('Model is still running!')
tic; 
out = sim('normalControl');
steerAng = out.steerAng;
executionTime=toc
disp('Model is finished.')