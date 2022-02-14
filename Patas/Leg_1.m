%%
clear all
close all
clc

%Definir longitud de los eslabones
L1 = 0;
L2 = 10;
L3 = 15;
L = L1+L2+L3;
h =2;
y0=0;
x0=0; 

%Posicion inicial
x = 0;
y =12;
z =-13;
[q1,q2,q3] = IK(h,x0,y0,L1,L2,L3,x,y,z);

%Puntos intermedio y final del movimiento de la pierna 
via = [4 12 -5;8 30 -13 ];


%Obtener trayectoria con mstraj
%             path vel-eje time origen  paso t_acel
traj = mstraj(via, [1 1 1], [], [x,y,z], 0.1, 2);
%plot3(traj(:,1),traj(:,2),traj(:,3))
length(traj)
%Simulacion pierna en movimiento
for i=1:length(traj)
    [q1(i),q2(i),q3(i)] = IK(h,x0,y0,L1,L2,L3,traj(i,1),traj(i,2),traj(i,3)); 
    if q2(i) == -1000
        q2(i)
        errordlg('La posicion deseada es inalcanzable para el brazo','lkasdj')
        break
    else
        figure(1)
        p0 = eye(4)*transl(x0,y0,h);
        p1 = trotz(q1(i))*transl(L1,0,0);   
        p2 =troty(q2(i))*transl(L2,0,0);    
        p3 = troty(q3(i))*transl(L3,0,0);
        p1 = p0*p1;
        p21 = p1*p2;
        p31 = p21*p3;
        pos = [ p0(1:3,4) p1(1:3,4) p21(1:3,4) p31(1:3,4)];
        plot3(pos(1,:),pos(2,:),pos(3,:),'b-o') %Graficar posicion del robot
        grid on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        hold on
        axis([-L L -L L -L L])
        view(137,48)
        h2 = findobj('Color','blue');   %Eliminar 
        delete(h2)
        plot3(pos(1,:),pos(2,:),pos(3,:),'b-o')
        x1(i) = p31(1,4);
        y1(i) = p31(2,4);
        z1(i) = p31(3,4);
        plot3(x1,y1,z1,'k','LineWidth',3)
        title('Robot')
        movieVector(i) = getframe; %Animacion
    end
end


%% Perfil de movimiento

%vel
v1 = diff(q1);
v2 = diff(q2);
v3 = diff(q3);

%acel 
a1 = diff(v1);
a2 = diff(v2);
a3 = diff(v3);

%yerk
y1 = diff(a1);
y2 = diff(a2);
y3 = diff(a3);

figure
t1 = 0:1/length(q1):1;
subplot(411)
grid on
plot(t1(1,1:length(q1)),q1)
title('Perfil de posicion articulacion 1')
xlabel('tiempo (s)')
ylabel('posicion (rad)')
subplot(412)
grid on
plot(t1(1,1:length(v1)),v1)
title('Perfil de velocidad articulacion 1')
xlabel('tiempo (s)')
ylabel('velocidad (rad/s)')
subplot(413)
grid on
plot(t1(1,1:length(a1)),a1)
title('Perfil de aceleracion articulacion 1')
xlabel('tiempo (s)')
ylabel('aceleracion (rad/s^2)')
subplot(414)
grid on
plot(t1(1,1:length(y1)),y1)
title('Perfil de Jerk articulacion 1')
xlabel('tiempo (s)')
ylabel('jerk (rad/s^3)')

figure
t2 = 0:1/length(q2):1;
subplot(411)
plot(t2(1,1:length(q2)),q2)
grid on
title('Perfil de posicion articulacion 2')
xlabel('tiempo (s)')
ylabel('posicion (rad)')
subplot(412)
grid on
plot(t2(1,1:length(v2)),v2)
title('Perfil de velocidad articulacion 2')
xlabel('tiempo (s)')
ylabel('velocidad (rad/s)')
subplot(413)
grid on
plot(t2(1,1:length(a2)),a2)
title('Perfil de aceleracion articulacion 2')
xlabel('tiempo (s)')
ylabel('aceleracion (rad/s^2)')
subplot(414)
grid on
plot(t1(1,1:length(y2)),y2)
title('Perfil de Jerk articulacion 2')
xlabel('tiempo (s)')
ylabel('jerk (rad/s^3)')

figure
t3 = 0:1/length(q3):1;
subplot(411)
grid on
plot(t3(1,1:length(q3)),q3)
title('Perfil de posicion articulacion 3')
xlabel('tiempo (s)')
ylabel('posicion (rad)')
subplot(412)
grid on
plot(t3(1,1:length(v3)),v3)
title('Perfil de velocidad articulacion 3')
xlabel('tiempo (s)')
ylabel('velocidad (rad/s)')
subplot(413)
grid on
plot(t3(1,1:length(a3)),a3)
title('Perfil de aceleracion articulacion 3')
xlabel('tiempo (s)')
ylabel('aceleracion (rad/s^2)')
subplot(414)
grid on
plot(t1(1,1:length(y3)),y3)
title('Perfil de Jerk articulacion 3')
xlabel('tiempo (s)')
ylabel('jerk (rad/s^3)')
