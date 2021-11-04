# Robot KUKA KR 10 R1100 Sixx
Automatización del mantenimiento predictivo destinado a la inspección visual de piezas prismáticas por Computer Vision.

![Simulación](https://github.com/cabustillo13/Robot_KUKA_KR_10/blob/main/resources/Simulaci%C3%B3n.gif)

## Resumen:

Simulación de un brazo robótico inspector tipo serie, de 7 eslabones, 6 articulaciones y 6 grados de libertad, con una cámara como efector final. Con la finalidad de: inspeccionar el estado de las piezas de máquinas y estructuras; determinar cuándo es necesario realizarle un mantenimiento preventivo; o si ya no pueden seguir en operatividad y deba ser reemplazada. Como tarea específica para la realización del proyecto, se propuso realizar el control del estado de la soldadura de un tubo en operatividad. 

Como resultado se parametrizo: 
- La estructura con Denavit – Hartenberg. 
- Cinemática Directa e Inversa. 
- Relación de velocidades.
-  Planificación y generación de trayectoria.

## Recomendaciones:

- Colocar tu ```absolute path``` del 3D model en el ```main.m``` 
- Se redefinio 2 funciones del Toolbox de Peter Corke para este proyecto: ```plot3d.m```, ```jtraj.m``` y ```unitQuaternion.m```. Se deben cambiar esos archivos por los nuestros que se encuentran [acá](https://github.com/cabustillo13/Robot_KUKA_KR_10/tree/main/auxiliary_functions_toolbox).

#### plot3d.m 
Cambiar los colores asigandos aleatoriamente al robot, por colores emblemáticos del robot KUKA. <br>
PATH: ```C:\Program Files\Matlab\R2020a\toolbox\rtb\@SerialLink\plot3d.m```

#### jtraj.m 
Cambiar la escala para que las gráficas de jtraj y ctraj tengan concordancia. <br>
PATH: ```C:\Program Files\Matlab\R2020a\toolbox\rtb\jtraj.m ```

#### unitQuaternion.m 
Fix typo minor error. <br>
PATH: ```C:\Program Files\Matlab\R2020a\toolbox\smtb\unitQuaternion.m```

## Recursos:

- [Ver informe final](https://github.com/cabustillo13/Robot_KUKA_KR_10/blob/main/Informe%20Final.pdf)
- [Ver diapositivas finales](https://github.com/cabustillo13/Robot_KUKA_KR_10/blob/main/Presentaci%C3%B3n%20Final.pdf)
- [Modelo 3D - STL format](https://github.com/cabustillo13/Robot_KUKA_KR_10/tree/main/3D_model)
- [Más vídeos](https://github.com/cabustillo13/Robot_KUKA_KR_10/tree/main/resources)

## Integrantes:
- [Carlos Bustillo](https://github.com/cabustillo13) 
- [Rodrigo Pérez](https://github.com/RodriPerez2110)
