function [design_region]=region(sigma,wd,zeta,T,s_z)

switch s_z
    case 'z'
        x_mesh=linspace(-1,1);
        y_mesh=linspace(-1,1);
        [X_mesh,Y_mesh]=meshgrid(x_mesh,y_mesh);
        [Theta_mesh,R_mesh]=cart2pol(X_mesh,Y_mesh);
        % Sigma
        if ~isempty(sigma)
            boolean_sigma_max=R_mesh>exp(-max(sigma)*T);
            boolean_sigma_min=R_mesh<exp(-min(sigma)*T);
            if numel(sigma)==2
                boolean_sigma=boolean_sigma_max & boolean_sigma_min;
            elseif numel(sigma)==1
                boolean_sigma=boolean_sigma_max;
            end
        else
            boolean_sigma=1;
        end

        % Wd
        if ~isempty(wd)
            if numel(wd)==1
                wd(2)=0;
            end
            boolean_wd_max=Theta_mesh<max(wd)*T ;
            boolean_wd_min=Theta_mesh>min(wd)*T;
            boolean_wd=boolean_wd_max & boolean_wd_min;
        else 
            boolean_wd=1;
        end

        % zeta
        if ~isempty(zeta)
            Sigma_zeta_max=Theta_mesh/T/tan(acos(max(zeta)));
            Sigma_zeta_min=Theta_mesh/T/tan(acos(min(zeta)));
            R_zeta_max=exp(-Sigma_zeta_max*T);
            R_zeta_min=exp(-Sigma_zeta_min*T);
            boolean_zeta_max=R_mesh>R_zeta_max;
            boolean_zeta_min=R_mesh<R_zeta_min;
            if numel(zeta)==2
                boolean_zeta=boolean_zeta_min & boolean_zeta_max;
            elseif numel(zeta)==1
                boolean_zeta=boolean_zeta_min;
            end
        else
            boolean_zeta=1;
        end
        boolean=boolean_zeta & boolean_wd & boolean_sigma;
        design_region=R_mesh(boolean).*cos(Theta_mesh(boolean))...
                     +1i*R_mesh(boolean).*sin(Theta_mesh(boolean));
        
        indeces=find(imag(design_region)<0);
        a=zeros(size(design_region));
        a(indeces)=design_region(indeces);
        if ~isempty(indeces)
           design_region=design_region-a; 
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case 's'
        n=30;
        [lim_x,lim_y]=xylimits(sigma,wd,zeta);
        x_mesh=linspace(-lim_x,0.1,n*lim_x);
        y_mesh=linspace(-0.1,lim_y,n*lim_y);
        [X_mesh,Y_mesh]=meshgrid(x_mesh,y_mesh);
        [Theta_mesh,R_mesh]=cart2pol(X_mesh,Y_mesh);

        % Sigma
        if ~isempty(sigma)
            if numel(sigma)==1
                sigma(2)=0;
            end
            boolean_sigma_max=X_mesh>-max(sigma);
            boolean_sigma_min=X_mesh<-min(sigma);
            boolean_sigma=boolean_sigma_max & boolean_sigma_min;
        else
            boolean_sigma=1;
        end

        % Wd
        if ~isempty(wd)
            if numel(wd)==1
                wd(2)=0;
            end
            boolean_wd_max=Y_mesh<max(wd);
            boolean_wd_min=Y_mesh>min(wd);
            boolean_wd=boolean_wd_max & boolean_wd_min;
        else
            boolean_wd=1;
        end

        % Zeta
        if ~isempty(zeta)
            theta_max=pi-acos(max(zeta));
            theta_min=pi-acos(min(zeta));
            boolean_zeta_max=Theta_mesh>theta_min;
            boolean_zeta_min=Theta_mesh<theta_max;
            if numel(zeta)==2
                boolean_zeta=boolean_zeta_min & boolean_zeta_max;
            elseif numel(zeta)==1
                boolean_zeta=boolean_zeta_max;
            end
        else
            boolean_zeta=1;
        end

        boolean=boolean_wd & boolean_sigma & boolean_zeta;
        design_region=X_mesh(boolean)+1i*Y_mesh(boolean);
end 
end