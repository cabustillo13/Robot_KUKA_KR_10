% %=======================================================================%
%                  CINEMÁTICA INVERSA Kuka KR 10 R1100 sixx     
% %=======================================================================%
%Esta función recibe la posición Cartesiana inicial del robot (junto a como
%es el robot),y la posición y orientación objetivo deseada. Y devuelve 8 
%soluciones (cantidad máx. posible, considerando que los límites
%articulares de q(6) son +-180°) ordenadas en las 8 columnas de la matriz 
%qq (en rads), junto a dos vectores de flags que nos indican si la posición 
%objetivo para alguna solución se encuentra fuera del ET debido a la 
%extensión máx del brazo o a los límites articulares.


%% USO DE LA FUNCIÓN CINEMÁTICA INVERSA [Del Espacio Cartesiano al Espacio Articular]:
%flag_EMB: significa flag de Espacio de Trabajo, considerando solo la 
%Extensión Máxima del Brazo (y no los límites articulares).
%% Función de CINEMÁTICA INVERSA:
function [qq_ordenadas, flags_EMB, flags_LimArt] = cinematicaInversa(ROBOT, T_objetivo)
  % Se simplifica el problema al dejar de lado la matriz de base y de herramienta:
  T_objetivo = invHomog(ROBOT.base.double) * T_objetivo * invHomog(ROBOT.tool.double);

  % POSICIÓN DE LA MUÑECA:
  d6 = ROBOT.dh(6,2); %d6=0.080
  p04 = T_objetivo(1:3,4) - d6 * T_objetivo(1:3,3);
  %Se grafica la trama de la muñenca:
%   T04 = [ 0    1    0    p04(1)
%           0    0    1    p04(2)
%           1    0    0    p04(3)
%           0    0    0    1     ]; %T04 = T_muneca. También se puede obtener multiplicando: T03*T34, en donde T34=R.links(4).A(0).double;..
%   trplot(T04,'color','c');        %..como se hizo en el script Visualizacion_de_las_tramas, pero sería más costoso.
  
  % Cálculo de q1:
  %  -> 2 valores posibles
  q1 = calcular_q1(p04);
  qq(1,1:4) = [1 1 1 1]*q1(1);
  qq(1,5:8) = [1 1 1 1]*q1(2);
  
  % Cálculo de q2:
  %  -> 2 valores posibles para cada q1
  [q21,flag_EMB_1] = calcular_q2(ROBOT,q1(1),p04);
  [q22,flag_EMB_2] = calcular_q2(ROBOT,q1(2),p04);
  qq(2,:) = [q21(1) q21(1) q21(2) q21(2) q22(1) q22(1) q22(2) q22(2)];
  
  % Cálculo de q3:
  %  -> 1 valor posible para cada par (q1,q2)
  [q311] = calcular_q3(ROBOT,q1(1),q21(1),p04);
  [q312] = calcular_q3(ROBOT,q1(1),q21(2),p04);
  [q321] = calcular_q3(ROBOT,q1(2),q22(1),p04);
  [q322] = calcular_q3(ROBOT,q1(2),q22(2),p04);
  qq(3,:) = [q311 q311 q312 q312 q321 q321 q322 q322];
  
  %% Cálculo de q4, q5 y q6:
  %  -> 2 conjuntos de valores para cada terna (q1,q2,q3)
  for i=0:2:6
      q1 = qq(1,i+1); %Al iterar creamos un vector q1 con los sus 8 valores, para las 8 posible soluciones.
      q2 = qq(2,i+1);
      q3 = qq(3,i+1);
      [q4,q5,q6] = calcular_qm(ROBOT,q1,q2,q3,T_objetivo);
      qq(4:6,i+1:i+2) = [q4;q5;q6];
  end
  
  %% Se elimina el offset:
  qq = qq - ROBOT.offset' * ones(1,8);

  %% CONSIDERACIÓN DEL ESPACIO DE TRABAJO CON LOS LÍMITES ARTICULARES:
  % Se recorre la matriz qq de soluciones, columna por columna, evaluando cada
  %una de las 6 variables articulares:
  % Como es común que una posición angular de una θ calculada, en realidad
  %se encuentre dentro de los límites articulares, y que solo el ángulo se 
  %encuente girado 360°. Por ello, en el primer if, se suma o resta 360° si
  %si fuera el caso, para asegurarse de no descartar erroneamente una
  %solución exacta.
  % Luego, si aún así el θ se encuentra fuera de sus límites, entonces en
  %el segundo if anidado, se le cambia su valor por el valor de su límite 
  %(superior o inferior) más cercano:
  flags_LimArt=zeros(1,8);
  for j = 1:length(qq(1,:)) %length(qq(1,:))= 8
      for i = 1:length(qq(:,1)) %length(qq(:,1))= 6
          if  qq(i,j) < ROBOT.qlim(i,1)  ||  ROBOT.qlim(i,2) < qq(i,j) %Si θ se encuentra fuera de sus límites articulares:
              if qq(i,j) < ROBOT.qlim(i,1) %Se asegura si en realidad sí puede encontrarse dentro de su límite articular, al girarlo 360°:
                  qq_aux = qq(i,j); %Se guarda el valor, por si no logra entrar dentro de los líms. articulares.
                  qq(i,j) = qq(i,j) + 2*pi;
              elseif ROBOT.qlim(i,2) < qq(i,j)
                  qq_aux = qq(i,j);
                  qq(i,j) = qq(i,j) - 2*pi;
              end
              %____________________________________________________________________________________________________________________________
              if  qq(i,j) < ROBOT.qlim(i,1)  ||  ROBOT.qlim(i,2) < qq(i,j) %Se vuelve a revisar si θ está fuera de su límite, en tal caso:
                  flags_LimArt(j)=1; %Para luego colocar la solución al final al ordenarlarlas.
                  %%% SE LE COLOCA EL VALOR QUE SE HABÍA CALCULADO:
                  qq(i,j) = qq_aux;
                  %%% SE LE COLOCA EL VALOR DEL LÍM. ARTICULAR MÁS CERCANO:
                  if qq(i,j) < 0,  qq(i,j) = qq(i,j) + 2*pi; end %Se pasan todos los ángulos a positivo para facilitar luego las restas.
                  LimInfPositivo = (ROBOT.qlim(i,1) + 2*pi);     %
                  LimSupPositivo = ROBOT.qlim(i,2);              %
                  if  abs( qq(i,j)-LimInfPositivo )  <  abs( qq(i,j)-LimSupPositivo ) %Restamos los ángulos para ver cuál es el más cercano.
                      qq(i,j) = ROBOT.qlim(i,1); %qq(i,j) = LimInf(Negativo);
                  else
                      qq(i,j) = ROBOT.qlim(i,2); %qq(i,j) = LimSup;
                  end
              end
              %____________________________________________________________________________________________________________________________
          end
      end
  end


  %% CONSIDERACIÓN DEL ESPACIO DE TRABAJO CON LA EXTENSIÓN MÁXIMA DEL BRAZO:
  flags_EMB=zeros(1,8);
  if flag_EMB_1 == 0 &&  flag_EMB_2 == 1
      flags_EMB(1:4)=1; %Para luego colocar la solución al final al ordenarlarlas.
  elseif flag_EMB_1 == 1 &&  flag_EMB_2 == 0
      flags_EMB(5:8)=1; %Para luego colocar la solución al final al ordenarlarlas.
  elseif flag_EMB_1 == 0  &&  flag_EMB_2 == 0
      flags_EMB=ones(1,8);
  end
  
  %% Se ordenan las soluciones desde la distancia articular más cercana a la más lejana:
  disArticulares=zeros(1,8);
  num=[1,2,3,4,5,6,7,8];
  
  for i=1:8
      disArticulares(i)=sum( abs(qq(:,i)'-ROBOT.q) ); %Se calcula la Distancia Articular RESPECTO A LA POSICIÓN
  end                                                 %ARTICULAR ROBOT.q (¡QUE POR DEFAULT ES [0,0,0,0,0,0] !)

  ordenamiento=[num;disArticulares;flags_EMB;flags_LimArt]; %num: número de solución.
  %Al realizar la cin. inv., las 8 soluciónes quedan ordenadas con la
  %numeración num (del 1 al 8), {de la 1er fila de la matriz
  %ordenamiento}.
  %Entonces, primero se ordenan las columnas de menor a mayor según las
  %distancias articulares {2da fila}. Luego se reordenan colocando al final
  %las soluciones que se encuentren fuera del ET por exceder la longitud 
  %máxima del brazo {3er fila}. Y finalmente, se reordenan colocando al
  %final las soluciones que se encuentren fuera del ET por exceder los 
  %límites articulares {4ta fila}:
  [~,data]=sort(ordenamiento(2,:));  %por ej. se obtiene:   data = 2  1  3  5  6  8  7  4
  ordenamiento=ordenamiento(:,data); %aquí se reordenan las columnas de la matriz ordenamiento

  [~,data]=sort(ordenamiento(3,:));
  ordenamiento=ordenamiento(:,data);
  
  [~,data]=sort(ordenamiento(4,:));
  ordenamiento=ordenamiento(:,data);
  
  qq_ordenadas=[];
  for i=1:8
      qq_ordenadas=[qq_ordenadas qq(:,ordenamiento(1,i))];
  end
  flags_EMB = ordenamiento(3,:);
  flags_LimArt = ordenamiento(4,:);
end

%% CÁLCULO de q1,q2,q3:
function [q1] = calcular_q1(p04)
  q1(1) = atan2(p04(2), p04(1));
  if q1(1) > 0, q1(2) = q1(1) - pi; else, q1(2) = q1(1) + pi; end %Se obtiene el valor q1(2) opuesto al valor de q1(1):
end

function [q2,flag_EMB] = calcular_q2(ROBOT, q1, p04)
  T01 = A_dh(ROBOT.dh(1,:), q1); %Esta función creada es lo mismo que usar el toolbox:  T01=R.links(1).A(q1).double;
  
  p14 = invHomog(T01)*[p04; 1];
  
  B = atan2(p14(2), p14(1));
  r = sqrt(p14(1)^2 + p14(2)^2);
  a2 = ROBOT.dh(2,3); %a2=0.560
  d4 = ROBOT.dh(4,2); %d4=0.515
  a3 = ROBOT.dh(3,3); %a3=0.035
  D4 = sqrt(a3^2 + d4^2);
  G = acos( (a2^2 + r^2 - D4^2) / (2*a2*r) ); % Si el ángulo G da un número imaginário, significa que no hay solución
  %es decir, que el objetivo no se encuentra dentro del Espacio de Trabajo ¡Diestro!. [Aunque tal vez sí pueda tener
  %alguna solución válida en el Espacio de Trabajo Alcanzable].
  q2(1) = B - real(G); %Si G llegara a ser imagario, entonces solo usamos la parte.. 
  q2(2) = B + real(G); %..real para obtener el punto más cercano al objetivo.
  if isreal(G), flag_EMB = 1; else, flag_EMB = 0; end
end

function [q3] = calcular_q3(ROBOT, q1, q2, p04)
  T01 = A_dh(ROBOT.dh(1,:), q1);      %NOTA: T1 es lo mismo que decir: T01
  T02 = T01 * A_dh(ROBOT.dh(2,:), q2); %T02 = T01 * T12
  p24 = invHomog(T02)*[p04; 1];

  H = atan2(p24(2), p24(1));
  d4 = ROBOT.dh(4,2); %d4=0.515
  a3 = ROBOT.dh(3,3); %a3=0.035
  I = atan2(d4, a3);  %I es siempre una constante, para un mismo robot: I=1.5029(=86.1..°);
  
  q3 = H - I;
end


%% CÁLCULO de q4,q5,q6:
function [q4,q5,q6] = calcular_qm(ROBOT,q1,q2,q3,T_objetivo)
  T01 = A_dh(ROBOT.dh(1,:),q1);
  T12 = A_dh(ROBOT.dh(2,:),q2);
  T23 = A_dh(ROBOT.dh(3,:),q3);
  T36 = invHomog(T23) * invHomog(T12) * invHomog(T01) * T_objetivo;
  %___________________________________________________________________________________________
  % SOLUCIÓN DEGENERADA: 
  %Al estar z3 y z5 alineados (y z6), entonces q4 y q6 producen el mismo giro, y por lo tanto podrían tener
  %infinitos valores combinados, ya que generan el mismo movimiento. Esto es a su vez una SINGULARIDAD!
  %Esta degeneración se cumple cuano q5=0 (q5=180 se descarta porque difícilmente se cumpla).
  %Por cuestión de utilidad práctica, se recurrió a dejar q4 con el valor que tenga previamente el robot, y a girar q6 lo
  %necesario para cumplir con la rotación del vector X, y así no tener infinitas soluciones.
  if abs(T36(3,3) - 1) < eps %*10000000000 %Si el versor de Z6 se encuentra prácticamente alineado con el eje Z3, entonces:
      q4(1) = ROBOT.q(4); %Aquí es donde se asume que: q4 = q4_anterior
      q5(1) = 0; %Pues claro, sino no se estaría en la solución degenerada.
      q6(1) = atan2(T36(2,1), T36(1,1)) - q4(1);%atan2(ComponenteYdelVersorXde6RespectoA3, ComponenteXdelVersorXde6RespectoA3) - theta4Inicial
      %El segundo conjunto de soluciones es igual al primero por ser un caso tan particular:
      q4(2) = q4(1);
      q5(2) = 0;
      q6(2) = q6(1);
      fprintf('\n ////////////////////////////////////// \n\n');
      
%       q6(1) = ROBOT.q(6);
%       q5(1) = 0;
%       q4(1) = atan2(T36(2,1), T36(1,1)) - q6(1);
% 
%       q6(2) = q6(1);
%       q5(2) = 0;
%       q4(2) = q4(1);
    %___________________________________________________________________________________________
  else
      % SOLUCIÓN NORMAL:
      q4(1) = atan2(-T36(2,3), -T36(1,3));%atan2(-ComponenteYdelVersorZde6RespectoA3, -ComponenteXdelVersorZde6RespectoA3)
                    %Nota: el - delante de un array simplemente cambia el signo a cada uno de sus elementos.
      if q4(1)>0, q4(2)=q4(1)-pi; else, q4(2)=q4(1)+pi; end %Se obtiene el valor q4(2) opuesto al valor de q4(1):
      q5 = zeros(1,2);
      q6 = zeros(1,2);
      for i=1:2
          T34 = A_dh(ROBOT.dh(4,:), q4(i));
          T46 = invHomog(T34) * T36;
          q5(i) = atan2(-T46(1,3), T46(2,3));%atan2(-ComponenteXdelVersorZde6RespectoA4, ComponenteYdelVersorZde6RespectoA4)

          T45 = A_dh(ROBOT.dh(5,:), q5(i));
          T56 = invHomog(T45) * T46;
          q6(i) = atan2(T56(2,1), T56(1,1));%atan2(ComponenteYdelVersorXde6RespectoA5, ComponenteXdelVersorXde6RespectoA5)
      end
  end
end





%% INVERSA DE MATRIZ HOMOGENEA (de forma más eficiente):
function iT = invHomog(T)
  iT = eye(4);
  iT(1:3, 1:3) = T(1:3, 1:3)';
  iT(1:3, 4) = - iT(1:3, 1:3) * T(1:3, 4);
end
%% Matriz de Denavit-Hartenberg
function T = A_dh(dh, q) %No se implementa el toolbox, sino que se optó por usar esta función que calcula cada matriz homogénea Ti_i+1
                         %Nomenclatura para los parámetros de Denhavit-Hartenberg: [theta d a alpha]
  s = dh(5);%dh es solo la fila i de la matriz dh completa, y guardamos en s el valor de la última columna, que indica si la articulación..
  if s      %..es prinsmático [con un 1] ó angular [con un 0]. Simplemente para tener una función más general.
      theta = dh(1);
      d = q;
  else
      theta = q;
      d = dh(2);
  end
  a = dh(3);
  alpha = dh(4);
  st = sin(theta);
  ct = cos(theta);
  sa = sin(alpha);
  ca = cos(alpha);
%   T = [ ct     -st      0     a
%         st*ca   ct*ca  -sa   -sa*d
%         st*sa   ct*sa   ca    ca*d
%          0       0       0     1   ]; %Del libro Craig ¿PORQUÉ NO FUNCIONA?
  T = [ ct  -st*ca   st*sa  a*ct
        st   ct*ca  -ct*sa  a*st
         0      sa      ca     d
         0       0       0     1]; %Del profe
end
%%

