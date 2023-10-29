
function dy=mpc_diff(t,y)
global u_store
dy=[0];
dy=9.2*u_store(10)-5*y;