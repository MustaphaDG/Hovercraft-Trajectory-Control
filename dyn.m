
function [dotX] = dyn(t, X, lambda_1,lambda_2,lambda_3,tmps,u_r,v_r)


  % récuperation des variables d'etat
  u = X(1,:);  v = X(2,:); r = X(3,:);
  epsilon = 0.00000001;
  
  u_r = interp1(tmps,u_r,t);
  v_r = interp1(tmps,v_r,t);
  du_r = 1 - (u_r)^2;
  dv_r = 1 - (v_r)^2;
  ddv_r = -2*v_r*dv_r;

  % calcul des commandes
  tau_u = du_r -lambda_1.*(u-u_r)- v.*r;
  tau_r = -(1./(u + epsilon)).*( ddv_r - lambda_2.*(v-v_r) - lambda_3.*(- u.*r - dv_r) + v.*(r.^2) + tau_u.*r);

  % equations du systeme

  dotu = v.*r + tau_u;
  dotv = -u.*r;
  dotr = tau_r;

  % renvoi d'arguments 
  dotX = [dotu dotv dotr]';
end 
