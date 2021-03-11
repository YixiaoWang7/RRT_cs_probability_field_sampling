function [pathnew]=PathOptimal(path,ob,deltad,vertices)
    ci=1;%current index
    ti=length(path);%temp index
    pathnew=path(ci);
    while 1
        %from the end ti to the start ci
        %if line ti-ci is free, add ti to the new path. And ci=ti
        %if not, decrease the ti to approach ci
        if isFree(ob,vertices(path(ci),:),vertices(path(ti),:),deltad)
            pathnew=[path(ti) pathnew];
            ci=ti;
            if ci==length(path)
                break
            end
            ti=length(path);
        else
            ti=ti-1; 
        end
    end
end