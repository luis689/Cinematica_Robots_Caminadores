% UNIVERSIDAD NACIONAL DE COLOMBIA
% DEPARTAMENTO DE INGENIERIA MECÁNICA Y MECATRÓNICA
% 17/12/2021
% CAMINTA LINEAL PARA UN ROBOT CUADRUPEDO
% LUIS ANTONIO ZULUAGA RAMIREZ
%%
clear all
clc
clf


%Parametros pierna robotica
%Longitud eslabones 
L1 = 0;       
L2 = 1;       
L3 = 1;  
L = L1+L2+L3;

% Dimensiones del cuerpo del cuadrupedo
a1 = 1;          % Arista 1, distancia en X
a2 = 1;          % Arista 2, distancia en Y
lim_h_i = L/10;   % Liminte inferior de altura
lim_h_u = 4*L/10; %Limite superior de altura
h = 0.5;          %Altura cuerpo
O_body = [0,0,0]; % Ubicacion inicial centro del cuerpo





%h = [linspace(1,0.2,50),linspace(0.2,1,25)];          % Altura del cuerpo
%rot = [linspace(0,30,50),linspace(30,0,25)];

% Ubicacion de cada pata
% Patas del lado izquierdo numero impar
% Patas del lado derecho numero par
lim_a1_sup = O_body(1)+a1/2;
lim_a1_inf = O_body(1)-a1/2;
lim_a2_sup = O_body(2)+a2/2;
lim_a2_inf = O_body(2)-a2/2;

ol1 = -0.5;
ol3 = 0.5;
ol2 = -0.5;
ol4 = 0.5;

%Definiciones cinematicas, espacio cuadrado de trabajo para mover la pierna
Rx = 4*sqrt(L^2-h^2)/5; 
Ry = Rx;



%Vectorees para ubicar cada pierna en su posicion inicia
%compuesto por 6 datos, Ubicacion de la  base, y ubicacion del extremo.
%       [x0,       y0, z0,      x,          y,    z]
leg1 = [lim_a1_inf,ol1,h,-Rx/2+lim_a1_inf,-Ry/2+ol1,0];
leg3 = [lim_a1_inf,ol3,h,-Rx/2+lim_a1_inf,Ry/2+ol3,0];
leg2 = [lim_a1_sup,ol2,h,Rx/2+lim_a1_sup,-Ry/2+ol2,0];
leg4 = [lim_a1_sup,ol4,h,Rx/2+lim_a1_sup,Ry/2+ol4,0];

%Vectores en cada eje coordenado para graficar poligono del cuerpo
X_body = [lim_a1_sup,lim_a1_sup,lim_a1_inf,lim_a1_inf,lim_a1_sup];
Y_body = [lim_a2_inf,lim_a2_sup,lim_a2_sup,lim_a2_inf,lim_a2_inf];
Z_body = [h,h,h,h,h];

% theta1 = atan2d(ol1,lim_a1_inf)
% theta2 = atan2d(ol2,lim_a1_sup)
% theta3 = atan2d(ol3,lim_a1_inf)
% theta4 = atan2d(ol4,lim_a1_sup)

%Cinematica inversa para obtener las rotaciones de las articulaciones en la
%posicion inicial
[q1_l1,q2_l1,q3_l1] = IK(leg1(3),leg1(1),leg1(2),L1,L2,L3,leg1(4),leg1(5),leg1(6));
[q1_l2,q2_l2,q3_l2] = IK(leg2(3),leg2(1),leg2(2),L1,L2,L3,leg2(4),leg2(5),leg2(6));
[q1_l3,q2_l3,q3_l3] = IK(leg3(3),leg3(1),leg3(2),L1,L2,L3,leg3(4),leg3(5),leg3(6));
[q1_l4,q2_l4,q3_l4] = IK(leg4(3),leg4(1),leg4(2),L1,L2,L3,leg4(4),leg4(5),leg4(6));

% Se utiliza cinematica directa para poder graficar las piernas. Se
% obitienen las coordendas para cada articulacion
[p1,pos1] = FK_1(L1,L2,L3,q1_l1,q2_l1,q3_l1,leg1(1:3),0);
[p2,pos2] = FK_1(L1,L2,L3,q1_l2,q2_l2,q3_l2,leg2(1:3),0);
[p3,pos3] = FK_1(L1,L2,L3,q1_l3,q2_l3,q3_l3,leg3(1:3),0);
[p4,pos4] = FK_1(L1,L2,L3,q1_l4,q2_l4,q3_l4,leg4(1:3),0);


%Vectores para graficar espacios de trabajo de cada pierna
ws_l1 = [lim_a1_inf,lim_a1_inf,lim_a1_inf-Rx,lim_a1_inf-Rx,lim_a1_inf;
         ol1,ol1-Ry,ol1-Ry,ol1,ol1;
         0,0,0,0,0];
ws_l2 = [lim_a1_sup,lim_a1_sup,lim_a1_sup+Rx,lim_a1_sup+Rx,lim_a1_sup;
         ol2,ol2-Ry,ol2-Ry,ol2,ol2;
         0,0,0,0,0];
ws_l3 = [lim_a1_inf,lim_a1_inf,lim_a1_inf-Rx,lim_a1_inf-Rx,lim_a1_inf;
         ol3,ol3+Ry,ol3+Ry,ol3,ol3;
         0,0,0,0,0];
ws_l4 = [lim_a1_sup,lim_a1_sup,lim_a1_sup+Rx,lim_a1_sup+Rx,lim_a1_sup;
         ol4,ol4+Ry,ol4+Ry,ol4,ol4;
         0,0,0,0,0];


%Graficar poligono de soporte
poligono = [pos1(1,4),pos3(1,4),pos4(1,4),pos2(1,4),pos1(1,4);
            pos1(2,4),pos3(2,4),pos4(2,4),pos2(2,4),pos1(2,4);
            0,0,0,0,0];

%Graficar Cuerpo en posicion inicial
plot3(X_body,Y_body,Z_body)
patch(X_body,Y_body,Z_body,'red','FaceAlpha',.5)
grid on
hold on
ylim([-2 10])
xlim([-3 3])
zlim([-0.5 2])
xlabel('X')
ylabel('Y')
zlabel('Z')

%Graficar piernas
plot3(pos1(1,:),pos1(2,:),pos1(3,:),'b-o','LineWidth',2) %Graficar posicion del robot
plot3(pos2(1,:),pos2(2,:),pos2(3,:),'b-o','LineWidth',2) %Graficar posicion del robot
plot3(pos3(1,:),pos3(2,:),pos3(3,:),'b-o','LineWidth',2) %Graficar posicion del robot
plot3(pos4(1,:),pos4(2,:),pos4(3,:),'b-o','LineWidth',2) %Graficar posicion del robot

% Graficar espacio de trabajo
plot3(ws_l1(1,:),ws_l1(2,:),ws_l1(3,:),'k--','LineWidth',0.5)
plot3(ws_l2(1,:),ws_l2(2,:),ws_l2(3,:),'k--','LineWidth',0.5)
plot3(ws_l3(1,:),ws_l3(2,:),ws_l3(3,:),'k--','LineWidth',0.5)
plot3(ws_l4(1,:),ws_l4(2,:),ws_l4(3,:),'k--','LineWidth',0.5)

plot3(poligono(1,:),poligono(2,:),poligono(3,:),'g','LineWidth',1.5)

% Angulos azimutal y de elevacion para la visualizacion de la animacion.
azimut = -150;
elevation = 60;
view(azimut,elevation)

%Calculos para simular caminata periodica discontinua

%% Puntos de la trayectoria de movimiento

for i=1:3
    % =========================================================================
    % ========================= LEG 1 =========================================
    % =========================================================================
    
    
    
    p_f_l1 = [lim_a1_inf-Rx/2,ol1,0];
    p_i_l1 = [(p_f_l1(1)+pos1(1,4))/2,(p_f_l1(2)+pos1(2,4))/2,h/4];
    via_leg1 = [p_i_l1;p_f_l1];
    %Obtener trayectoria con mstraj
    %             path vel-eje time origen  paso t_acel
    traj = mstraj(via_leg1, [1 1 1], [], [pos1(1,4),pos1(2,4),pos1(3,4)], 0.8, 2);
    % leg1 = [lim_a1_inf,ol1,h,traj(i,1),traj(i,2),traj(i,3)];
    
    
    
    for i = 1:length(traj)
        clf
    
        leg1(4) = traj(i,1);
        leg1(5) = traj(i,2);
        leg1(6) = traj(i,3);
        [q1_l1(i),q2_l1(i),q3_l1(i)] = IK(leg1(3),leg1(1),leg1(2),L1,L2,L3,leg1(4),leg1(5),leg1(6));
        [q1_l2(i),q2_l2(i),q3_l2(i)] = IK(leg2(3),leg2(1),leg2(2),L1,L2,L3,leg2(4),leg2(5),leg2(6));
        [q1_l3(i),q2_l3(i),q3_l3(i)] = IK(leg3(3),leg3(1),leg3(2),L1,L2,L3,leg3(4),leg3(5),leg3(6));
        [q1_l4(i),q2_l4(i),q3_l4(i)] = IK(leg4(3),leg4(1),leg4(2),L1,L2,L3,leg4(4),leg4(5),leg4(6));
    
    
        [p1,pos1(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l1(i),q2_l1(i),q3_l1(i),leg1(1:3),0);
        [p2,pos2(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l2(i),q2_l2(i),q3_l2(i),leg2(1:3),0);
        [p3,pos3(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l3(i),q2_l3(i),q3_l3(i),leg3(1:3),0);
        [p4,pos4(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l4(i),q2_l4(i),q3_l4(i),leg4(1:3),0);
        %Graficar Cuerpo en posicion inicial
        plot3(X_body,Y_body,Z_body)
        patch(X_body,Y_body,Z_body,'red','FaceAlpha',.5)
        grid on
        hold on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        ylim([-2 10])
        xlim([-2 3])
        zlim([-0.5 2])
        
        %Graficar piernas
        plot3(pos1(1,(i-1)*4+1:(i-1)*4+4),pos1(2,(i-1)*4+1:(i-1)*4+4),pos1(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos2(1,(i-1)*4+1:(i-1)*4+4),pos2(2,(i-1)*4+1:(i-1)*4+4),pos2(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos3(1,(i-1)*4+1:(i-1)*4+4),pos3(2,(i-1)*4+1:(i-1)*4+4),pos3(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos4(1,(i-1)*4+1:(i-1)*4+4),pos4(2,(i-1)*4+1:(i-1)*4+4),pos4(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        
%         % Graficar espacio de trabajo
%         plot3(ws_l1(1,:),ws_l1(2,:),ws_l1(3,:),'k--','LineWidth',0.5) 
%         plot3(ws_l2(1,:),ws_l2(2,:),ws_l2(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l3(1,:),ws_l3(2,:),ws_l3(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l4(1,:),ws_l4(2,:),ws_l4(3,:),'k--','LineWidth',0.5)
        if pos1(3,(i-1)*4+4) > 0
            poligono = [pos3(1,(i-1)*4+4),pos4(1,(i-1)*4+4),pos2(1,(i-1)*4+4),pos3(1,(i-1)*4+4);
                        pos3(2,(i-1)*4+4),pos4(2,(i-1)*4+4),pos2(2,(i-1)*4+4),pos3(2,(i-1)*4+4);
                        pos3(3,(i-1)*4+4),pos4(3,(i-1)*4+4),pos2(3,(i-1)*4+4),pos3(3,(i-1)*4+4)];
            
        else
            poligono = [pos1(1,(i-1)*4+4),pos3(1,(i-1)*4+4),pos4(1,(i-1)*4+4),pos2(1,(i-1)*4+4),pos1(1,(i-1)*4+4);
                        pos1(2,(i-1)*4+4),pos3(2,(i-1)*4+4),pos4(2,(i-1)*4+4),pos2(2,(i-1)*4+4),pos1(2,(i-1)*4+4);
                        pos1(3,(i-1)*4+4),pos3(3,(i-1)*4+4),pos4(3,(i-1)*4+4),pos2(3,(i-1)*4+4),pos1(3,(i-1)*4+4)];
        end
        
        plot3(poligono(1,:),poligono(2,:),poligono(3,:),'g','LineWidth',1.5)
        view(azimut,elevation)
        
        movieVector1(i) = getframe; %Animacion
    end
    
    % =========================================================================
    % ========================= LEG 3 =========================================
    % =========================================================================
    
    p_f_l3 = [lim_a1_inf-Rx/2,ol3+Ry,0];
    p_i_l3 = [(p_f_l3(1)+pos3(1,4))/2,(p_f_l3(2)+pos3(2,4))/2,h/4];
    via_leg3 = [p_i_l3;p_f_l3];
    %Obtener trayectoria con mstraj
    %             path vel-eje time origen  paso t_acel
    traj = mstraj(via_leg3, [1 1 1], [], [pos3(1,4),pos3(2,4),pos3(3,4)], 0.8, 2);
    % leg1 = [lim_a1_inf,ol1,h,traj(i,1),traj(i,2),traj(i,3)];
    for i = 1:length(traj)
        clf
    
        leg3(4) = traj(i,1);
        leg3(5) = traj(i,2);
        leg3(6) = traj(i,3);
        [q1_l1(i),q2_l1(i),q3_l1(i)] = IK(leg1(3),leg1(1),leg1(2),L1,L2,L3,leg1(4),leg1(5),leg1(6));
        [q1_l2(i),q2_l2(i),q3_l2(i)] = IK(leg2(3),leg2(1),leg2(2),L1,L2,L3,leg2(4),leg2(5),leg2(6));
        [q1_l3(i),q2_l3(i),q3_l3(i)] = IK(leg3(3),leg3(1),leg3(2),L1,L2,L3,leg3(4),leg3(5),leg3(6));
        [q1_l4(i),q2_l4(i),q3_l4(i)] = IK(leg4(3),leg4(1),leg4(2),L1,L2,L3,leg4(4),leg4(5),leg4(6));
    
    
        [p1,pos1(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l1(i),q2_l1(i),q3_l1(i),leg1(1:3),0);
        [p2,pos2(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l2(i),q2_l2(i),q3_l2(i),leg2(1:3),0);
        [p3,pos3(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l3(i),q2_l3(i),q3_l3(i),leg3(1:3),0);
        [p4,pos4(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l4(i),q2_l4(i),q3_l4(i),leg4(1:3),0);
        %Graficar Cuerpo en posicion inicial
        plot3(X_body,Y_body,Z_body)
        patch(X_body,Y_body,Z_body,'red','FaceAlpha',.5)
        grid on
        hold on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        ylim([-2 10])
        xlim([-2 3])
        zlim([-0.5 2])
        
        %Graficar piernas
        plot3(pos1(1,(i-1)*4+1:(i-1)*4+4),pos1(2,(i-1)*4+1:(i-1)*4+4),pos1(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos2(1,(i-1)*4+1:(i-1)*4+4),pos2(2,(i-1)*4+1:(i-1)*4+4),pos2(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos3(1,(i-1)*4+1:(i-1)*4+4),pos3(2,(i-1)*4+1:(i-1)*4+4),pos3(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos4(1,(i-1)*4+1:(i-1)*4+4),pos4(2,(i-1)*4+1:(i-1)*4+4),pos4(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        
%         % Graficar espacio de trabajo
%         plot3(ws_l1(1,:),ws_l1(2,:),ws_l1(3,:),'k--','LineWidth',0.5) 
%         plot3(ws_l2(1,:),ws_l2(2,:),ws_l2(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l3(1,:),ws_l3(2,:),ws_l3(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l4(1,:),ws_l4(2,:),ws_l4(3,:),'k--','LineWidth',0.5)
    
        if pos3(3,(i-1)*4+4) > 0.001
            poligono = [pos1(1,(i-1)*4+4),pos4(1,(i-1)*4+4),pos2(1,(i-1)*4+4),pos1(1,(i-1)*4+4);
                        pos1(2,(i-1)*4+4),pos4(2,(i-1)*4+4),pos2(2,(i-1)*4+4),pos1(2,(i-1)*4+4);
                        pos1(3,(i-1)*4+4),pos4(3,(i-1)*4+4),pos2(3,(i-1)*4+4),pos1(3,(i-1)*4+4)];
            
        else
            poligono = [pos1(1,(i-1)*4+4),pos3(1,(i-1)*4+4),pos4(1,(i-1)*4+4),pos2(1,(i-1)*4+4),pos1(1,(i-1)*4+4);
                        pos1(2,(i-1)*4+4),pos3(2,(i-1)*4+4),pos4(2,(i-1)*4+4),pos2(2,(i-1)*4+4),pos1(2,(i-1)*4+4);
                        pos1(3,(i-1)*4+4),pos3(3,(i-1)*4+4),pos4(3,(i-1)*4+4),pos2(3,(i-1)*4+4),pos1(3,(i-1)*4+4)];
        end
    
        
        
        plot3(poligono(1,:),poligono(2,:),poligono(3,:),'g','LineWidth',1.5)
        
        view(azimut,elevation)
        movieVector2(i) = getframe; %Animacion
    end
    
    % =========================================================================
    % ========================= BODY =========================================
    % =========================================================================
    
    
    traj_body = linspace(O_body(2),O_body(2)+Ry/2,10);
    for i = 1:length(traj_body)
        clf
        
        O_body = [0,traj_body(i),h];
    
        lim_a1_sup = O_body(1)+a1/2;
        lim_a1_inf = O_body(1)-a1/2;
        lim_a2_sup = O_body(2)+a2/2;
        lim_a2_inf = O_body(2)-a2/2;
        
        ol1 = lim_a2_inf;
        ol3 = lim_a2_sup;
        ol2 = lim_a2_inf;
        ol4 = lim_a2_sup;
        
        
        
        
        %Vectorees para ubicar cada pierna en su posicion inicia
        %compuesto por 6 datos, Ubicacion de la  base, y ubicacion del extremo.
        %       [x0,       y0, z0,      x,          y,    z]
        leg1 = [lim_a1_inf,ol1,h,leg1(4:6)];
        leg3 = [lim_a1_inf,ol3,h,leg3(4:6)];
        leg2 = [lim_a1_sup,ol2,h,leg2(4:6)];
        leg4 = [lim_a1_sup,ol4,h,leg4(4:6)];
        
        %Vectores en cada eje coordenado para graficar poligono del cuerpo
        X_body = [lim_a1_sup,lim_a1_sup,lim_a1_inf,lim_a1_inf,lim_a1_sup];
        Y_body = [lim_a2_inf,lim_a2_sup,lim_a2_sup,lim_a2_inf,lim_a2_inf];
        Z_body = [h,h,h,h,h];
    
    
        [q1_l1(i),q2_l1(i),q3_l1(i)] = IK(leg1(3),leg1(1),leg1(2),L1,L2,L3,leg1(4),leg1(5),leg1(6));
        [q1_l2(i),q2_l2(i),q3_l2(i)] = IK(leg2(3),leg2(1),leg2(2),L1,L2,L3,leg2(4),leg2(5),leg2(6));
        [q1_l3(i),q2_l3(i),q3_l3(i)] = IK(leg3(3),leg3(1),leg3(2),L1,L2,L3,leg3(4),leg3(5),leg3(6));
        [q1_l4(i),q2_l4(i),q3_l4(i)] = IK(leg4(3),leg4(1),leg4(2),L1,L2,L3,leg4(4),leg4(5),leg4(6));
    
    
        [p1,pos1(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l1(i),q2_l1(i),q3_l1(i),leg1(1:3),0);
        [p2,pos2(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l2(i),q2_l2(i),q3_l2(i),leg2(1:3),0);
        [p3,pos3(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l3(i),q2_l3(i),q3_l3(i),leg3(1:3),0);
        [p4,pos4(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l4(i),q2_l4(i),q3_l4(i),leg4(1:3),0);
        %Graficar Cuerpo en posicion inicial
        plot3(X_body,Y_body,Z_body)
        patch(X_body,Y_body,Z_body,'red','FaceAlpha',.5)
        grid on
        hold on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        ylim([-2 10])
        xlim([-2 3])
        zlim([-0.5 2])
        
        %Graficar piernas
        plot3(pos1(1,(i-1)*4+1:(i-1)*4+4),pos1(2,(i-1)*4+1:(i-1)*4+4),pos1(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos2(1,(i-1)*4+1:(i-1)*4+4),pos2(2,(i-1)*4+1:(i-1)*4+4),pos2(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos3(1,(i-1)*4+1:(i-1)*4+4),pos3(2,(i-1)*4+1:(i-1)*4+4),pos3(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos4(1,(i-1)*4+1:(i-1)*4+4),pos4(2,(i-1)*4+1:(i-1)*4+4),pos4(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        
%         % Graficar espacio de trabajo
%         plot3(ws_l1(1,:),ws_l1(2,:),ws_l1(3,:),'k--','LineWidth',0.5) 
%         plot3(ws_l2(1,:),ws_l2(2,:),ws_l2(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l3(1,:),ws_l3(2,:),ws_l3(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l4(1,:),ws_l4(2,:),ws_l4(3,:),'k--','LineWidth',0.5)
        
        plot3(poligono(1,:),poligono(2,:),poligono(3,:),'g','LineWidth',1.5)
        view(azimut,elevation)
       
        movieVector3(i) = getframe; %Animacion
    end
    
    % =========================================================================
    % ========================= LEG 2 =========================================
    % =========================================================================
    
    p_f_l2 = [lim_a1_sup+Rx/2,ol2,0];
    p_i_l2 = [(p_f_l2(1)+pos2(1,4))/2,(p_f_l2(2)+pos2(2,4))/2,h/4];
    via_leg2 = [p_i_l2;p_f_l2];
    %Obtener trayectoria con mstraj
    %             path vel-eje time origen  paso t_acel
    traj = mstraj(via_leg2, [1 1 1], [], [pos2(1,4),pos2(2,4),pos2(3,4)], 0.8, 2);
    % leg1 = [lim_a1_inf,ol1,h,traj(i,1),traj(i,2),traj(i,3)];
    for i = 1:length(traj)
        clf
    
        leg2(4) = traj(i,1);
        leg2(5) = traj(i,2);
        leg2(6) = traj(i,3);
        [q1_l1(i),q2_l1(i),q3_l1(i)] = IK(leg1(3),leg1(1),leg1(2),L1,L2,L3,leg1(4),leg1(5),leg1(6));
        [q1_l2(i),q2_l2(i),q3_l2(i)] = IK(leg2(3),leg2(1),leg2(2),L1,L2,L3,leg2(4),leg2(5),leg2(6));
        [q1_l3(i),q2_l3(i),q3_l3(i)] = IK(leg3(3),leg3(1),leg3(2),L1,L2,L3,leg3(4),leg3(5),leg3(6));
        [q1_l4(i),q2_l4(i),q3_l4(i)] = IK(leg4(3),leg4(1),leg4(2),L1,L2,L3,leg4(4),leg4(5),leg4(6));
    
    
        [p1,pos1(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l1(i),q2_l1(i),q3_l1(i),leg1(1:3),0);
        [p2,pos2(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l2(i),q2_l2(i),q3_l2(i),leg2(1:3),0);
        [p3,pos3(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l3(i),q2_l3(i),q3_l3(i),leg3(1:3),0);
        [p4,pos4(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l4(i),q2_l4(i),q3_l4(i),leg4(1:3),0);
        %Graficar Cuerpo en posicion inicial
        plot3(X_body,Y_body,Z_body)
        patch(X_body,Y_body,Z_body,'red','FaceAlpha',.5)
        grid on
        hold on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        ylim([-2 10])
        xlim([-2 3])
        zlim([-0.5 2])
        
        %Graficar piernas
        plot3(pos1(1,(i-1)*4+1:(i-1)*4+4),pos1(2,(i-1)*4+1:(i-1)*4+4),pos1(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos2(1,(i-1)*4+1:(i-1)*4+4),pos2(2,(i-1)*4+1:(i-1)*4+4),pos2(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos3(1,(i-1)*4+1:(i-1)*4+4),pos3(2,(i-1)*4+1:(i-1)*4+4),pos3(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos4(1,(i-1)*4+1:(i-1)*4+4),pos4(2,(i-1)*4+1:(i-1)*4+4),pos4(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        
%         % Graficar espacio de trabajo
%         plot3(ws_l1(1,:),ws_l1(2,:),ws_l1(3,:),'k--','LineWidth',0.5) 
%         plot3(ws_l2(1,:),ws_l2(2,:),ws_l2(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l3(1,:),ws_l3(2,:),ws_l3(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l4(1,:),ws_l4(2,:),ws_l4(3,:),'k--','LineWidth',0.5)
    
        if pos2(3,(i-1)*4+4) > 0.001
            poligono = [pos1(1,(i-1)*4+4),pos4(1,(i-1)*4+4),pos3(1,(i-1)*4+4),pos1(1,(i-1)*4+4);
                        pos1(2,(i-1)*4+4),pos4(2,(i-1)*4+4),pos3(2,(i-1)*4+4),pos1(2,(i-1)*4+4);
                        pos1(3,(i-1)*4+4),pos4(3,(i-1)*4+4),pos3(3,(i-1)*4+4),pos1(3,(i-1)*4+4)];
            
        else
            poligono = [pos1(1,(i-1)*4+4),pos3(1,(i-1)*4+4),pos4(1,(i-1)*4+4),pos2(1,(i-1)*4+4),pos1(1,(i-1)*4+4);
                        pos1(2,(i-1)*4+4),pos3(2,(i-1)*4+4),pos4(2,(i-1)*4+4),pos2(2,(i-1)*4+4),pos1(2,(i-1)*4+4);
                        pos1(3,(i-1)*4+4),pos3(3,(i-1)*4+4),pos4(3,(i-1)*4+4),pos2(3,(i-1)*4+4),pos1(3,(i-1)*4+4)];
        end
    
        
        
        plot3(poligono(1,:),poligono(2,:),poligono(3,:),'g','LineWidth',1.5)
        
        view(azimut,elevation)
        movieVector4(i) = getframe; %Animacion
    end
    
    % =========================================================================
    % ========================= LEG 4 =========================================
    % =========================================================================
    
    p_f_l4 = [lim_a1_sup+Rx/2,ol4+Ry,0];
    p_i_l4 = [(p_f_l4(1)+pos4(1,4))/2,(p_f_l4(2)+pos4(2,4))/2,h/4];
    via_leg4 = [p_i_l4;p_f_l4];
    %Obtener trayectoria con mstraj
    %             path vel-eje time origen  paso t_acel
    traj = mstraj(via_leg4, [1 1 1], [], [pos4(1,4),pos4(2,4),pos4(3,4)], 0.8, 2);
    % leg1 = [lim_a1_inf,ol1,h,traj(i,1),traj(i,2),traj(i,3)];
    for i = 1:length(traj)
        clf
    
        leg4(4) = traj(i,1);
        leg4(5) = traj(i,2);
        leg4(6) = traj(i,3);
        [q1_l1(i),q2_l1(i),q3_l1(i)] = IK(leg1(3),leg1(1),leg1(2),L1,L2,L3,leg1(4),leg1(5),leg1(6));
        [q1_l2(i),q2_l2(i),q3_l2(i)] = IK(leg2(3),leg2(1),leg2(2),L1,L2,L3,leg2(4),leg2(5),leg2(6));
        [q1_l3(i),q2_l3(i),q3_l3(i)] = IK(leg3(3),leg3(1),leg3(2),L1,L2,L3,leg3(4),leg3(5),leg3(6));
        [q1_l4(i),q2_l4(i),q3_l4(i)] = IK(leg4(3),leg4(1),leg4(2),L1,L2,L3,leg4(4),leg4(5),leg4(6));
    
    
        [p1,pos1(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l1(i),q2_l1(i),q3_l1(i),leg1(1:3),0);
        [p2,pos2(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l2(i),q2_l2(i),q3_l2(i),leg2(1:3),0);
        [p3,pos3(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l3(i),q2_l3(i),q3_l3(i),leg3(1:3),0);
        [p4,pos4(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l4(i),q2_l4(i),q3_l4(i),leg4(1:3),0);
        %Graficar Cuerpo en posicion inicial
        plot3(X_body,Y_body,Z_body)
        patch(X_body,Y_body,Z_body,'red','FaceAlpha',.5)
        grid on
        hold on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        ylim([-2 10])
        xlim([-2 3])
        zlim([-0.5 2])
        
        %Graficar piernas
        plot3(pos1(1,(i-1)*4+1:(i-1)*4+4),pos1(2,(i-1)*4+1:(i-1)*4+4),pos1(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos2(1,(i-1)*4+1:(i-1)*4+4),pos2(2,(i-1)*4+1:(i-1)*4+4),pos2(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos3(1,(i-1)*4+1:(i-1)*4+4),pos3(2,(i-1)*4+1:(i-1)*4+4),pos3(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos4(1,(i-1)*4+1:(i-1)*4+4),pos4(2,(i-1)*4+1:(i-1)*4+4),pos4(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        
%         % Graficar espacio de trabajo
%         plot3(ws_l1(1,:),ws_l1(2,:),ws_l1(3,:),'k--','LineWidth',0.5) 
%         plot3(ws_l2(1,:),ws_l2(2,:),ws_l2(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l3(1,:),ws_l3(2,:),ws_l3(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l4(1,:),ws_l4(2,:),ws_l4(3,:),'k--','LineWidth',0.5)
    
        if pos4(3,(i-1)*4+4) > 0.001
            poligono = [pos1(1,(i-1)*4+4),pos2(1,(i-1)*4+4),pos3(1,(i-1)*4+4),pos1(1,(i-1)*4+4);
                        pos1(2,(i-1)*4+4),pos2(2,(i-1)*4+4),pos3(2,(i-1)*4+4),pos1(2,(i-1)*4+4);
                        pos1(3,(i-1)*4+4),pos2(3,(i-1)*4+4),pos3(3,(i-1)*4+4),pos1(3,(i-1)*4+4)];
            
        else
            poligono = [pos1(1,(i-1)*4+4),pos3(1,(i-1)*4+4),pos4(1,(i-1)*4+4),pos2(1,(i-1)*4+4),pos1(1,(i-1)*4+4);
                        pos1(2,(i-1)*4+4),pos3(2,(i-1)*4+4),pos4(2,(i-1)*4+4),pos2(2,(i-1)*4+4),pos1(2,(i-1)*4+4);
                        pos1(3,(i-1)*4+4),pos3(3,(i-1)*4+4),pos4(3,(i-1)*4+4),pos2(3,(i-1)*4+4),pos1(3,(i-1)*4+4)];
        end
    
        
        
        plot3(poligono(1,:),poligono(2,:),poligono(3,:),'g','LineWidth',1.5)
        
        view(azimut,elevation)
        movieVector5(i) = getframe; %Animacion
    end
    
    % =========================================================================
    % ========================= BODY =========================================
    % =========================================================================
    
    
    traj_body = linspace(O_body(2),O_body(2)+Ry/2,10);
    for i = 1:length(traj_body)
        clf
        
        O_body = [0,traj_body(i),h];
    
        lim_a1_sup = O_body(1)+a1/2;
        lim_a1_inf = O_body(1)-a1/2;
        lim_a2_sup = O_body(2)+a2/2;
        lim_a2_inf = O_body(2)-a2/2;
        
        ol1 = lim_a2_inf;
        ol3 = lim_a2_sup;
        ol2 = lim_a2_inf;
        ol4 = lim_a2_sup;
        
        
        
        
        %Vectorees para ubicar cada pierna en su posicion inicia
        %compuesto por 6 datos, Ubicacion de la  base, y ubicacion del extremo.
        %       [x0,       y0, z0,      x,          y,    z]
        leg1 = [lim_a1_inf,ol1,h,leg1(4:6)];
        leg3 = [lim_a1_inf,ol3,h,leg3(4:6)];
        leg2 = [lim_a1_sup,ol2,h,leg2(4:6)];
        leg4 = [lim_a1_sup,ol4,h,leg4(4:6)];
        
        %Vectores en cada eje coordenado para graficar poligono del cuerpo
        X_body = [lim_a1_sup,lim_a1_sup,lim_a1_inf,lim_a1_inf,lim_a1_sup];
        Y_body = [lim_a2_inf,lim_a2_sup,lim_a2_sup,lim_a2_inf,lim_a2_inf];
        Z_body = [h,h,h,h,h];
    
    
        [q1_l1(i),q2_l1(i),q3_l1(i)] = IK(leg1(3),leg1(1),leg1(2),L1,L2,L3,leg1(4),leg1(5),leg1(6));
        [q1_l2(i),q2_l2(i),q3_l2(i)] = IK(leg2(3),leg2(1),leg2(2),L1,L2,L3,leg2(4),leg2(5),leg2(6));
        [q1_l3(i),q2_l3(i),q3_l3(i)] = IK(leg3(3),leg3(1),leg3(2),L1,L2,L3,leg3(4),leg3(5),leg3(6));
        [q1_l4(i),q2_l4(i),q3_l4(i)] = IK(leg4(3),leg4(1),leg4(2),L1,L2,L3,leg4(4),leg4(5),leg4(6));
    
    
        [p1,pos1(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l1(i),q2_l1(i),q3_l1(i),leg1(1:3),0);
        [p2,pos2(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l2(i),q2_l2(i),q3_l2(i),leg2(1:3),0);
        [p3,pos3(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l3(i),q2_l3(i),q3_l3(i),leg3(1:3),0);
        [p4,pos4(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l4(i),q2_l4(i),q3_l4(i),leg4(1:3),0);
        %Graficar Cuerpo en posicion inicial
        plot3(X_body,Y_body,Z_body)
        patch(X_body,Y_body,Z_body,'red','FaceAlpha',.5)
        grid on
        hold on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        ylim([-2 10])
        xlim([-2 3])
        zlim([-0.5 2])
        
        %Graficar piernas
        plot3(pos1(1,(i-1)*4+1:(i-1)*4+4),pos1(2,(i-1)*4+1:(i-1)*4+4),pos1(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos2(1,(i-1)*4+1:(i-1)*4+4),pos2(2,(i-1)*4+1:(i-1)*4+4),pos2(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos3(1,(i-1)*4+1:(i-1)*4+4),pos3(2,(i-1)*4+1:(i-1)*4+4),pos3(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos4(1,(i-1)*4+1:(i-1)*4+4),pos4(2,(i-1)*4+1:(i-1)*4+4),pos4(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        
%         % Graficar espacio de trabajo
%         plot3(ws_l1(1,:),ws_l1(2,:),ws_l1(3,:),'k--','LineWidth',0.5) 
%         plot3(ws_l2(1,:),ws_l2(2,:),ws_l2(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l3(1,:),ws_l3(2,:),ws_l3(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l4(1,:),ws_l4(2,:),ws_l4(3,:),'k--','LineWidth',0.5)
        
        plot3(poligono(1,:),poligono(2,:),poligono(3,:),'g','LineWidth',1.5)
        view(azimut,elevation)
        
        movieVector6(i) = getframe; %Animacion
    end
    i = i+1;

%     movieVector(i) = getframe; %Animacion
end
% =========================================================================
    % ========================= LEG 1 =========================================
    % =========================================================================
    
    
    
    p_f_l1 = [lim_a1_inf-Rx/2,ol1-Ry/2,0];
    p_i_l1 = [(p_f_l1(1)+pos1(1,4))/2,(p_f_l1(2)+pos1(2,4))/2,h/4];
    via_leg1 = [p_i_l1;p_f_l1];
    %Obtener trayectoria con mstraj
    %             path vel-eje time origen  paso t_acel
    traj = mstraj(via_leg1, [1 1 1], [], [pos1(1,4),pos1(2,4),pos1(3,4)], 0.5, 2);
    % leg1 = [lim_a1_inf,ol1,h,traj(i,1),traj(i,2),traj(i,3)];
    
    
    
    for i = 1:length(traj)
        clf
    
        leg1(4) = traj(i,1);
        leg1(5) = traj(i,2);
        leg1(6) = traj(i,3);
        [q1_l1(i),q2_l1(i),q3_l1(i)] = IK(leg1(3),leg1(1),leg1(2),L1,L2,L3,leg1(4),leg1(5),leg1(6));
        [q1_l2(i),q2_l2(i),q3_l2(i)] = IK(leg2(3),leg2(1),leg2(2),L1,L2,L3,leg2(4),leg2(5),leg2(6));
        [q1_l3(i),q2_l3(i),q3_l3(i)] = IK(leg3(3),leg3(1),leg3(2),L1,L2,L3,leg3(4),leg3(5),leg3(6));
        [q1_l4(i),q2_l4(i),q3_l4(i)] = IK(leg4(3),leg4(1),leg4(2),L1,L2,L3,leg4(4),leg4(5),leg4(6));
    
    
        [p1,pos1(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l1(i),q2_l1(i),q3_l1(i),leg1(1:3),0);
        [p2,pos2(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l2(i),q2_l2(i),q3_l2(i),leg2(1:3),0);
        [p3,pos3(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l3(i),q2_l3(i),q3_l3(i),leg3(1:3),0);
        [p4,pos4(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l4(i),q2_l4(i),q3_l4(i),leg4(1:3),0);
        %Graficar Cuerpo en posicion inicial
        plot3(X_body,Y_body,Z_body)
        patch(X_body,Y_body,Z_body,'red','FaceAlpha',.5)
        grid on
        hold on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        ylim([-2 10])
        xlim([-2 3])
        zlim([-0.5 2])
        
        %Graficar piernas
        plot3(pos1(1,(i-1)*4+1:(i-1)*4+4),pos1(2,(i-1)*4+1:(i-1)*4+4),pos1(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos2(1,(i-1)*4+1:(i-1)*4+4),pos2(2,(i-1)*4+1:(i-1)*4+4),pos2(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos3(1,(i-1)*4+1:(i-1)*4+4),pos3(2,(i-1)*4+1:(i-1)*4+4),pos3(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos4(1,(i-1)*4+1:(i-1)*4+4),pos4(2,(i-1)*4+1:(i-1)*4+4),pos4(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        
%         % Graficar espacio de trabajo
%         plot3(ws_l1(1,:),ws_l1(2,:),ws_l1(3,:),'k--','LineWidth',0.5) 
%         plot3(ws_l2(1,:),ws_l2(2,:),ws_l2(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l3(1,:),ws_l3(2,:),ws_l3(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l4(1,:),ws_l4(2,:),ws_l4(3,:),'k--','LineWidth',0.5)
        if pos1(3,(i-1)*4+4) > 0
            poligono = [pos3(1,(i-1)*4+4),pos4(1,(i-1)*4+4),pos2(1,(i-1)*4+4),pos3(1,(i-1)*4+4);
                        pos3(2,(i-1)*4+4),pos4(2,(i-1)*4+4),pos2(2,(i-1)*4+4),pos3(2,(i-1)*4+4);
                        pos3(3,(i-1)*4+4),pos4(3,(i-1)*4+4),pos2(3,(i-1)*4+4),pos3(3,(i-1)*4+4)];
            
        else
            poligono = [pos1(1,(i-1)*4+4),pos3(1,(i-1)*4+4),pos4(1,(i-1)*4+4),pos2(1,(i-1)*4+4),pos1(1,(i-1)*4+4);
                        pos1(2,(i-1)*4+4),pos3(2,(i-1)*4+4),pos4(2,(i-1)*4+4),pos2(2,(i-1)*4+4),pos1(2,(i-1)*4+4);
                        pos1(3,(i-1)*4+4),pos3(3,(i-1)*4+4),pos4(3,(i-1)*4+4),pos2(3,(i-1)*4+4),pos1(3,(i-1)*4+4)];
        end
        
        plot3(poligono(1,:),poligono(2,:),poligono(3,:),'g','LineWidth',1.5)
        view(azimut,elevation)
        
        movieVector7(i) = getframe; %Animacion
    end
    
    % =========================================================================
    % ========================= LEG 3 =========================================
    % =========================================================================
    
    p_f_l3 = [lim_a1_inf-Rx/2,ol3+Ry/2,0];
    p_i_l3 = [(p_f_l3(1)+pos3(1,4))/2,(p_f_l3(2)+pos3(2,4))/2,h/4];
    via_leg3 = [p_i_l3;p_f_l3];
    %Obtener trayectoria con mstraj
    %             path vel-eje time origen  paso t_acel
    traj = mstraj(via_leg3, [1 1 1], [], [pos3(1,4),pos3(2,4),pos3(3,4)], 0.5, 2);
    % leg1 = [lim_a1_inf,ol1,h,traj(i,1),traj(i,2),traj(i,3)];
    for i = 1:length(traj)
        clf
    
        leg3(4) = traj(i,1);
        leg3(5) = traj(i,2);
        leg3(6) = traj(i,3);
        [q1_l1(i),q2_l1(i),q3_l1(i)] = IK(leg1(3),leg1(1),leg1(2),L1,L2,L3,leg1(4),leg1(5),leg1(6));
        [q1_l2(i),q2_l2(i),q3_l2(i)] = IK(leg2(3),leg2(1),leg2(2),L1,L2,L3,leg2(4),leg2(5),leg2(6));
        [q1_l3(i),q2_l3(i),q3_l3(i)] = IK(leg3(3),leg3(1),leg3(2),L1,L2,L3,leg3(4),leg3(5),leg3(6));
        [q1_l4(i),q2_l4(i),q3_l4(i)] = IK(leg4(3),leg4(1),leg4(2),L1,L2,L3,leg4(4),leg4(5),leg4(6));
    
    
        [p1,pos1(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l1(i),q2_l1(i),q3_l1(i),leg1(1:3),0);
        [p2,pos2(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l2(i),q2_l2(i),q3_l2(i),leg2(1:3),0);
        [p3,pos3(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l3(i),q2_l3(i),q3_l3(i),leg3(1:3),0);
        [p4,pos4(:,(i-1)*4+1:(i-1)*4+4)] = FK_1(L1,L2,L3,q1_l4(i),q2_l4(i),q3_l4(i),leg4(1:3),0);
        %Graficar Cuerpo en posicion inicial
        plot3(X_body,Y_body,Z_body)
        patch(X_body,Y_body,Z_body,'red','FaceAlpha',.5)
        grid on
        hold on
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        ylim([-2 10])
        xlim([-2 3])
        zlim([-0.5 2])
        
        %Graficar piernas
        plot3(pos1(1,(i-1)*4+1:(i-1)*4+4),pos1(2,(i-1)*4+1:(i-1)*4+4),pos1(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos2(1,(i-1)*4+1:(i-1)*4+4),pos2(2,(i-1)*4+1:(i-1)*4+4),pos2(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos3(1,(i-1)*4+1:(i-1)*4+4),pos3(2,(i-1)*4+1:(i-1)*4+4),pos3(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        plot3(pos4(1,(i-1)*4+1:(i-1)*4+4),pos4(2,(i-1)*4+1:(i-1)*4+4),pos4(3,(i-1)*4+1:(i-1)*4+4),'b-o','LineWidth',2) %Graficar posicion del robot
        
%         % Graficar espacio de trabajo
%         plot3(ws_l1(1,:),ws_l1(2,:),ws_l1(3,:),'k--','LineWidth',0.5) 
%         plot3(ws_l2(1,:),ws_l2(2,:),ws_l2(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l3(1,:),ws_l3(2,:),ws_l3(3,:),'k--','LineWidth',0.5)
%         plot3(ws_l4(1,:),ws_l4(2,:),ws_l4(3,:),'k--','LineWidth',0.5)
    
        if pos3(3,(i-1)*4+4) > 0.001
            poligono = [pos1(1,(i-1)*4+4),pos4(1,(i-1)*4+4),pos2(1,(i-1)*4+4),pos1(1,(i-1)*4+4);
                        pos1(2,(i-1)*4+4),pos4(2,(i-1)*4+4),pos2(2,(i-1)*4+4),pos1(2,(i-1)*4+4);
                        pos1(3,(i-1)*4+4),pos4(3,(i-1)*4+4),pos2(3,(i-1)*4+4),pos1(3,(i-1)*4+4)];
            
        else
            poligono = [pos1(1,(i-1)*4+4),pos3(1,(i-1)*4+4),pos4(1,(i-1)*4+4),pos2(1,(i-1)*4+4),pos1(1,(i-1)*4+4);
                        pos1(2,(i-1)*4+4),pos3(2,(i-1)*4+4),pos4(2,(i-1)*4+4),pos2(2,(i-1)*4+4),pos1(2,(i-1)*4+4);
                        pos1(3,(i-1)*4+4),pos3(3,(i-1)*4+4),pos4(3,(i-1)*4+4),pos2(3,(i-1)*4+4),pos1(3,(i-1)*4+4)];
        end
    
        
        
        plot3(poligono(1,:),poligono(2,:),poligono(3,:),'g','LineWidth',1.5)
        view(azimut,elevation)
        
        movieVector8(i) = getframe; %Animacion
    end

%%

% --------------- Codigo para guardar el video---------------
% video = VideoWriter('Cuadrupedo','MPEG-4');
% video.FrameRate = 20;
% movieVector = [movieVector1,movieVector2,movieVector3,movieVector4,movieVector5,movieVector6];
% 
% open(video);
% writeVideo(video,movieVector);
% close(video)
