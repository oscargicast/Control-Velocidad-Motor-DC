clear all; close all; clc
% -------------------------------------------------------------------------
% Cargando la DATA
% -------------------------------------------------------------------------
data41 = load('../data/motor_gate.lvm');
T = 1/30; % Tiempo de Muestreo 
y1=data41(:,4);
u1=data41(:,2);
% -------------------------------------------------------------------------
% Identificación ARX
% -------------------------------------------------------------------------
data=iddata(y1,u1,T);
th=arx(data,[1 1 1]);
present(th)
thc=d2c(th);
[num,den]=tfdata(thc);
Gp=tf(num,den);
[Pp,Zp]=pzmap(Gp);         
% ------------------------------------------------------------------------
% Especificaciones de Diseño
% ------------------------------------------------------------------------
ts=1;       % Tiempo de Establecimiento
Mp=0.08;     % Máximo Sobrepaso
% ------------------------------------------------------------------------
% Polos deseados
% ------------------------------------------------------------------------
zeta=-log(Mp)/sqrt((log(Mp))^2+pi^2);
wn=4.6/(zeta*ts);
s1=-zeta*wn+1j*wn*sqrt(1-zeta^2);
sd=s1;
pdi=imag(sd); 
pdr=real(sd);
% ------------------------------------------------------------------------
% Ploteo de polos y ceros Planta + Controlador
% ------------------------------------------------------------------------
plot(real(Pp),imag(Pp),'x','MarkerSize',14) % polos planta
hold
plot(real(Zp),imag(Zp),'o','MarkerSize',14) % ceros planta
plot(pdr,pdi,'xr','MarkerSize',14) % polos deseados
plot([-18000 1],[0 0],'k') % ejex
plot([0 0],[-2 8],'k') % ejey
% ------------------------------------------------------------------------
% Angulos hacia los polos deseados
% ------------------------------------------------------------------------
theta1=(pi-atan(abs(pdi/pdr)))*180/pi;     % polo del PID (s=0)
theta2=atan(abs(pdi)/abs(Pp-pdr))*180/pi;  % polo de la planta
% ------------------------------------------------------------------------
% Fase positiva que deben aportar los ceros del PID
theta_c=-180+(theta1+theta2);
% ------------------------------------------------------------------------
% Calculo de a y b (Condicion de Fase)
theta_a=0.1*theta_c;
a=pdr-pdi/tan((180-theta_a)*pi/180);
theta_b=theta_c; 
b=pdr-pdi/tan((180-theta_b)*pi/180);
plot(a,0,'om','MarkerSize',14) % ceros PID (s=-a)
plot(b,0,'om','MarkerSize',14) % ceros PID (s=-b)
plot(0,0,'xm','MarkerSize',14) % polos PID
axis([-2000 200 -5 10])
% ------------------------------------------------------------------------
% Calculo de la Ganancia K (Condicion de Magnitud)
% ------------------------------------------------------------------------
Gc=tf(conv([1 abs(a)],[1 abs(b)]),[1 0]);
FLA=series(Gc,Gp);
K=rlocfind(FLA,sd); 
% Ganancias Kp, Ki, Kd del controlador continuo
Kp=K*abs(a+b); 
Ki=K*abs(a*b); 
Kd=K;       
KPID=[Kp Ki Kd];
% ------------------------------------------------------------------------
% PID No Interactivo GPID(s)=Kp*(1+1/(Ti*s)+Td*s)
% ------------------------------------------------------------------------
Ti=Kp/Ki; 
Td=Kd/Kp; 
Gc=K*Gc;
L=series(Gc,Gp);
H=L/(L+1);
% Ploteos
figure
t=0:0.001:2;
u=ones(size(t));
yp=lsim(H,u,t);
plot(t,u,'k','LineWidth',2)
hold
plot(t,yp,'r','LineWidth',2)
axis([0 2 0 1.2])
xlabel('\bf t(seg)')
h=ylabel('\bf y(t)');
set(h,'Rotation',pi/2)
grid

% -------------------------------------------------------------------------
% Re-diseño por tustin del Control en Tiempo Discreto
% -------------------------------------------------------------------------
[num,den]=tfdata(Gp);
tau=(1/den{1}(2));
T=tau/5;
[Nt,Dt] = tfdata(Gc,'v');
Nt = poly2sym(Nt,'s');
Dt = poly2sym(Dt,'s');
syms z
Gdt = Nt/Dt;
Gdt = subs(Gdt,{'s'},(2*(z-1))/(T*(z+1)));
Gdt = simplify(Gdt);
Gdt = vpa(Gdt,4); 
[NDt DDt] = numden(Gdt);
NDt = sym2poly(NDt);
DDt = sym2poly(DDt);
% -------------------------------------------------------------------------
% FT del Controlador digital D(z)
% -------------------------------------------------------------------------
GDt = tf(NDt,DDt,T);
printsys(NDt,DDt,'z')
[Np,Dp]=tfdata(Gp,'v');
datos=[Np Dp NDt DDt];
% -------------------------------------------------------------------------
% Coeficientes para lectura de LabVIEW
% -------------------------------------------------------------------------
save '../data/coef_rcnew.lvm' datos -ascii -tabs


