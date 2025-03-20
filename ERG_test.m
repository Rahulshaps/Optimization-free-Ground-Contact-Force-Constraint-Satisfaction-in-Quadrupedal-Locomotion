function [xwd_ref,hr,hw]=ERG_test(J_r,d_r,s_r,xr,xw,a)
    Jr=s_r*J_r;
    dr=s_r*d_r;
    hr=Jr*xr+dr;
    hw=Jr*xw+dr;
    
    vr=0;
    vt=0;
    vn=0;
    Cr=[];
    
    if min(hw)>=0 || min(hr)>=0
%         1
        vr=a*(xr-xw);
    end
    
    if min(hw)>=0 && min(hr)<0
%         2
        nc=length(hr);
        for k=1:nc
            if hr(k)<0
                Cr=[Cr;Jr(k,:)];
            end
        end
        Nr=null(Cr);
        [~,n]=size(Nr);
        for k=1:n
            nk=Nr(:,k)/norm(Nr(:,k));
            vt=vt+a*nk*nk'*(xr-xw);
        end
    end    
    
    if min(hw)<0 && min(hr)<0
%         3
        [~,kmin]=min(hw);
        nk=Jr(kmin,:)/norm(Jr(kmin,:));
        if hr(kmin)>=hw(kmin)
            vn=a*nk*nk'*(xr-xw);
        else
            vn=-a*nk*nk'*(xr-xw);
        end
    end
    
    xwd_ref=vr+vn+vt;
end