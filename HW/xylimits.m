function [lim_x,lim_y]=xylimits(sigma,wd,zeta)

if ~isempty(sigma)
    lim_x=max(sigma);
    if ~isempty(wd)
        lim_y=max(wd);
    else
        if ~isempty(zeta)
                if min(zeta)>=1 || min(zeta)==0
                    lim_y=lim_x;
                else
                    wn=max(sigma)/min(zeta);
                    lim_y=ceil(wn*sqrt(1-min(zeta)^2));
                end
        else
            lim_y=lim_x;
        end
    end
else
    if ~isempty(wd)
        lim_y=max(wd);
        if ~isempty(zeta)
                if max(zeta)>=1 || max(zeta)==0
                    lim_x=lim_y;
                else
                    wn=max(wd)/sqrt(1-max(zeta)^2);
                    lim_x=ceil(wn*max(zeta));
                end
        else
            lim_x=lim_y;
        end
    else
        lim_y=5;
        lim_x=5;
    end
end

end