
%flag_posPermitida: Un 1 significa que sí está permitida, y un 0 que no.

function [transformada_homogenea, q, flag_posPermitida] = cinematicaDirecta(R,q)
    flag_posPermitida=1;
    for i=1:length(q)
        if q(i) < R.qlim(i,1) %Si q(i) es Menor que el límite inferior:
            flag_posPermitida=0;
            q(i)=R.qlim(i,1);
        elseif R.qlim(i,2) < q(i) %Si q(i) es Mayor que el límite superior:
            flag_posPermitida=0;
            q(i)=R.qlim(i,2);
        end
        transformada_homogenea = R.fkine(q);
    end
    if flag_posPermitida == 0
        fprintf("La posición articular solicitada no es posible\n");
        fprintf("El vector de posición articular mas cercano es:\n ");
        disp(q);
    end
end