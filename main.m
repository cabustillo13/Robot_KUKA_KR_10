clc, clear, close all;
%=========================================================================%
fprintf('###########################################\n')
fprintf('#               TRABAJO FINAL             #\n')
fprintf('###########################################\n\n')
%=========================================================================%

%{
       NOTAS:
___________________________________________________________________________

Como la función seriallink.plot3d(), no tomaba la opción para cambiar
el color de los eslabones (posiblemente por un error en el código de Peter
Corke), se optó por harcodear su código, específicamente el scritp:
    WINDOWS:    C:\Program Files\polyspace\R2020b\toolbox\rtb\@SerialLink\plot3d.m
    LINUX:      /usr/local/MATLAB/R2018a/toolbox/rtb/@SerialLink/
donde se comentó la línea:
          % C = [C; colorname(c{1})];
y se la reemplazó por:
            C = [0.8500, 0.3250, 0.0980;
                 0.8500, 0.3250, 0.0980;
                 0.8500, 0.3250, 0.0980;
                 0.8500, 0.3250, 0.0980;
                 0.8500, 0.3250, 0.0980;
                 0.8500, 0.3250, 0.0980];
___________________________________________________________________________
En el código de la función jtraj de Peter Corke se cambió la escala de las
gráficas QD y QDD, modificando (en la línea 54) el valor de la variable 
tscal de 1 a 20.
___________________________________________________________________________
%}

%% DEFINICIÓN DEL ROBOT:

% Parámetros de DENAVIT-HARTENBERG:
dh = [%  θ(rad) d(m)   a(m)   α(rad) σ(bool)
        0.000  0.400  0.025  -pi/2  0
        0.000  0.000  0.560  0.000  0
        0.000  0.000  0.035  -pi/2  0
        0.000  0.515  0.000   pi/2  0
        0.000  0.000  0.000  -pi/2  0
        0.000  0.080  0.000  0.000  0      ];
R = SerialLink(dh,'name','KR10 R1100 sixx');

% LÍMITES ARTICULARES del robot:
R.qlim = [-170, 170;            %q1
          -190,  45;            %q2
          -210,  66;            %q3
          -185, 185;            %q4
          -120, 120;            %q5
          -350, 350]*(pi/180);  %q6
% fprintf('Los Limites Articulares listados desde θ1 hasta θ6 son:\n');
% disp(R.qlim*(180/pi))

% POSICIÓN INICIAL del robot [Espacio Articular]:
q_ini = [0,-135,45,0,0,0]*(pi/180);
R.offset = [0 0 0 0 0 0]*(pi/180);

R.base = transl(0, 0, 0);
R.tool = transl(0, 0, 0); %Lente de camara a 5 cm en Zextremo.  (0, 0, 0.05)
Tool = R.tool.double;
Base = R.base.double;

%PATH ABSOLUTO Windows:
PATH = 'C:\Users\cabus\Downloads\TF_BustilloPerez\KR10_R1100_sixx';
%PATH RELATIVO:
% PATH='.\KR10_R1100_sixx';
%PATH ABSOLUTO Linux:
% PATH='/home/rodri/Downloads/Robótica 1/PROYECTO/TF_BustilloPerez/KR10_R1100_sixx';

%Cubo:
CUBO = [0.8  0.16 0.43;   %Vértice 1
        0.8 -0.16 0.43;   %Vértice 2
        1    0.16 0.43;   %Vértice 3
        1   -0.16 0.43;   %Vértice 4
        0.8  0.16 0.87;   %Vértice 5
        0.8 -0.16 0.87;   %Vértice 6
        1    0.16 0.87;   %Vértice 7
        1   -0.16 0.87];  %Vértice 8

%% DEFINICIÓN DE PUNTOS PARA EL CÁLCULO DE LA TRAYECTORIA:
%Se definen 20 puntos característicos de la tarea.

%Vértice Inferior Izquierdo del Cubo:
T_InfIzq=transl(CUBO(1,:)'+[-0.1;0;0]);
T_InfIzq=T_InfIzq*troty(pi/2)*trotz(pi);

%Matriz de Posición Inicial:
T0=cinematicaDirecta(R,q_ini);
T0=T0.double;

%Maniobra de seguridad para iniciar la tarea:
T01=T0*transl(0,-0.15,0);
T02=T01*transl(-0.4,0,0);

%Se comienza la tarea en el Vértice Inferior Izquierdo:
T1=T_InfIzq*transl(0.01,0,0);%subir 1cm desde esa posición
T2=T1*transl(0,0.3,0); %recorrer de izquierda a derecha
T3=T2*transl(0.07,0,0); %subir 7cm
T4=T3*transl(0,-0.3,0); %recorrer de derecha a izquierda
T5=T4*transl(0.07,0,0); %subir 7cm
T6=T5*transl(0,0.3,0);%recorrer de izquierda a derecha
T7=T6*transl(0.07,0,0); %subir 7cm
T8=T7*transl(0,-0.3,0); %recorrer de derecha a izquierda
T9=T8*transl(0.07,0,0); %subir 7cm
T10=T9*transl(0,0.3,0); %recorrer de izquierda a derecha
T11=T10*transl(0.07,0,0); %subir 7cm
T12=T11*transl(0,-0.3,0); %recorrer de derecha a izquierda
T13=T12*transl(0.07,0,0); %subir 7cm
T14=T13*transl(0,0.3,0); %recorrer de izquierda a derecha

T15=T14*transl(0.05,0,0); %Maniobra de Seguridad

T016=T15*transl(0,-0.15,-0.2);

TT=[T0 T01 T02 T1 T2 T3 T4 T5 T6 T7 T8 T9 T10 T11 T12 T13 T14 T15 T016 T0]; %Matriz de 4x80
%Así, se definen 20 puntos -> 19 tramos.
P = size(TT,2)/4; %P: Cantidad de puntos definidos = 20


%% CÁLCULO DE TRAYECTORIA:
fprintf('Cálculo de trayectoria para tarea de inspección\n');
fprintf('mediante el uso de jtraj y ctraj\n\n')

qq=q_ini';

ROBOT.dh = dh;
ROBOT.qlim = R.qlim;
ROBOT.q = q_ini;
ROBOT.offset = R.offset;
ROBOT.base = R.base;
ROBOT.tool = R.tool;

c=1;
for i=1:P-1
    sols_CinInv_J=cinematicaInversa(ROBOT, TT(:,c+4*i:c+4*i+3) );
%     ROBOT.q = sols_CinInv_J(:,1)'; %Debido a que los 17 puntos se
     %encuentran bastante alejados, si se descomenta esta línea, la
     %CinInv nos devuelve otra solución que aunque sea la más cercana
     %al punto anterior, no es la deseada.
    qq=[qq sols_CinInv_J(:,1)];
end

%--------------------------------------------------------------------------
%USO DE LAS FUNCIONES JTRAJ y CTRAJ:

N=24; %Discretización de puntos en la interpolación.
%En cada uno de los 19 tramos, se obtienen 22 (N menos los dos puntos
%extremos) puntos interiores al interpolar, con lo que se terminan
%obteniendo N*(P-1)=24*19=456 puntos de discretización en la trayectoria
%completa de la tarea.

QQJ=[];
QQJv=[];
QQJa=[];
c=1;
f=4;
TTC=[];
for i=1:P-1
    if i<=3 || i>=P-2 %JTRAJ
        [Q, QD, QDD]=jtraj( qq(:,i)' , qq(:,i+1)' , N );
        QQJ=[QQJ;Q];
        QQJv=[QQJv;QD];
        QQJa=[QQJa;QDD];
%         QQJv=diff(QQJ);
%         QQJa=diff(QQJv);
        c=c+4;
        f=f+4;
    elseif i>=4 && i<=P-2 %CTRAJ
        TTC(:,:,i*N-(N-1):i*N)=ctraj(  TT(:,c:f) , TT(:,c+4:f+4) , N  ); %size(TTC,3)=456-2*N=408
        c=c+4;
        f=f+4;
    end
end
TTC(:,:,1:3*N) = []; %size(TTC,3)=456-5*N=336  TTC es un arreglo de 4x4x336
%--------------------------------------------------------------------------

QQC=[];
%Se calcula la cinematica inversa para las 20 T, que describen el camino:
for i=1:size(TTC,3)
    sols_CinInv_C=cinematicaInversa(ROBOT, TTC(:,:,i));
    ROBOT.q = sols_CinInv_C(:,1)';
    QQC=[QQC; sols_CinInv_C(:,1)'];
end

%Se concatena QQJ con QQC:
QQ = [ QQJ( 1:size(QQJ,1)-2*N , :) ; QQC ; QQJ(size(QQJ,1)-(2*N-1):size(QQJ,1),:) ];

% q149 = QQ(149,:)*(180/pi)
% q150 = QQ(150,:)*(180/pi)
% q151 = QQ(151,:)*(180/pi)

figure(1)
COLORES = 'Colores propios, ver nota al inicio';
R.plot3d(q_ini, 'path',PATH, 'workspace',[-1 1.5 -1 1 -0 1.5], 'color',COLORES, 'trail',{'r'});
hold on;
graficarCubo(CUBO,'green');
pause();
%R.plot3d(QQ, 'path',PATH, 'workspace',[-1 1.5 -1 1 -0 1.5], 'color',COLORES, 'delay',0.01);


%% GRÁFICAS DE POSICIONES, VELOCIDADES Y ACELERACIONES ARTICULARES:
fprintf('A continuación se muestran las gráficas de posiciones, velocidades y ')
fprintf('aceleraciones articulares.\n\n')
fprintf('Presione una tecla para continuar\n');
pause();

QQCv=diff(QQC);  %Velocidades Articulares
QQCa=diff(QQCv); %Aceleraciones Articulares

% size(QQJ,1)  %120
% size(QQJv,1) %120
% size(QQJa,1) %120
% size(QQC,1)  %336
% size(QQCv,1) %335 %Se obtiene un elemento menos, debido al uso de la función diff
% size(QQCa,1) %334 %Se obtiene un elemento menos, debido al uso de la función diff

figure(2)
subplot(3,1,1)
qplot(QQ);
title('Posiciones Articulares');
subplot(3,1,2)
QQv = [ QQJv(1:size(QQJv,1)-2*N,:) ; QQCv ; QQJv(size(QQJv,1)-(2*N-1):size(QQJv,1),:) ];
% QQv = [ [0,0,0,0,0,0] ; QQv];
qplot(QQv);
title('Velocidades Articulares');
subplot(3,1,3)
QQa = [ QQJa(1:size(QQJa,1)-2*N,:) ; QQCa ; QQJa(size(QQJa,1)-(2*N-1):size(QQJa,1),:) ];
qplot(QQa);
title('Aceleraciones Articulares');

