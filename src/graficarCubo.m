
function graficarCubo(coord,color)
    %Hacer una inspección de barrido en un cubo a partir de las coordenadas 
    %definidas según el siguiente orden para cada cubo:
    %
    %    7-------8
    %   /|      /|
    %  / |     / |
    % 5--|----6  |
    % |  3----|--4
    % | /     | /
    % 1-------2

    for k = 1:length(coord)/8

      X = coord(8*(k-1)+1:8*(k-1)+8,1);
      Y = coord(8*(k-1)+1:8*(k-1)+8,2);
      Z = coord(8*(k-1)+1:8*(k-1)+8,3);

      %Los puntos de cada cara se ordenan según el sentido antihorario
      caras = [1 2 4 3; 5 6 8 7; 1 3 7 5; 2 4 8 6; 1 2 6 5; 3 4 8 7];

      %size(X) = [4 6]
      %- cada columna hace referencia a los puntos de un plano
      %- hay 4 elementos en cada columna que se refieren a las coordenadas x de
      %cada punto del plano

      X = [X(caras(1,:)) X(caras(2,:)) X(caras(3,:)) X(caras(4,:)) X(caras(5,:)) X(caras(6,:))];
      Y = [Y(caras(1,:)) Y(caras(2,:)) Y(caras(3,:)) Y(caras(4,:)) Y(caras(5,:)) Y(caras(6,:))];
      Z = [Z(caras(1,:)) Z(caras(2,:)) Z(caras(3,:)) Z(caras(4,:)) Z(caras(5,:)) Z(caras(6,:))];

      alpha = 0.4; %transparencia de la cara
      colour =color; %color de la cara

      fill3(X,Y,Z,colour,'FaceAlpha',alpha); %Se dibuja el cubo
%       axis equal

      hold on
    end
end