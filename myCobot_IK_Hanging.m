function q = myCobot_IK_Hanging(P, R, L)
    % IK Analítica para myCobot colgado (Ceiling Mount)
    % P: [x; y; z] del TCP relativo a la base colgada
    % R: Matriz de rotación deseada del TCP
    % L: Estructura con longitudes
    x = P(1); y = P(2); z = P(3);
    % 1. Posición del Centro de la Muñeca (Wrist Center - WC)
    % Retrocedemos d6 desde el target en la dirección del eje Z de la herramienta
    % Como es ceiling mount, z es negativo hacia abajo.
    approach = R(:, 3); % Eje Z deseado
    WC = P - L.d6 * approach;
    xc = WC(1); yc = WC(2); zc = WC(3);
    % 2. J1: Waist (Rotación base)
    q1 = atan2(yc, xc);
    % 3. J2 y J3: Problema Planar 2-Links
    % Proyectamos en el plano del brazo
    r = sqrt(xc^2 + yc^2);      % Radio horizontal
    % Altura efectiva desde el hombro. 
    % OJO: Si el robot cuelga, el hombro está en -d1 relativo a la base Z=0?
    % Asumimos Base en Z=0, Hombro baja d1.
    dz = zc - (-L.d1); % Diferencia de altura desde el hombro al WC
    % Distancia directa hombro-muñeca
    D = sqrt(r^2 + dz^2);
    % Ley de Cosenos
    % cos(beta) = (a2^2 + D^2 - a3^2) / (2*a2*D)
    cos_angle_sh = (L.a2^2 + D^2 - L.a3^2) / (2 * L.a2 * D);
    % Protección numérica
    if abs(cos_angle_sh) > 1
        warning('Target fuera de alcance (Singularidad)');
        cos_angle_sh = sign(cos_angle_sh);
    end
    angle_sh = acos(cos_angle_sh); % Ángulo interno triángulo
    % Ángulo de elevación del vector D
    angle_elev = atan2(dz, r);
    % q2 (Hombro): 
    % En configuración normal: 0 es vertical? O horizontal? 
    % Asumiendo 0 es vertical hacia abajo para myCobot:
    q2 = -(angle_elev + angle_sh); % Codo "afuera" o "atrás"
    % Nota: Aquí hay que jugar con los signos según la convención de ceros del URDF.
    % Generalmente myCobot vertical es q2=0.
    % q3 (Codo):
    cos_angle_elb = (L.a2^2 + L.a3^2 - D^2) / (2 * L.a2 * L.a3);
    if abs(cos_angle_elb) > 1, cos_angle_elb = sign(cos_angle_elb); end
    angle_elb = acos(cos_angle_elb);
    % El ángulo articular suele ser (pi - angulo_interno)
    q3 = pi - angle_elb; 
    % Revisar dirección de q3 según servomotores (+/-)
    % 4. Orientación (Muñeca Esférica)
    % Calculamos la rotación R0_3 (Base a Codo) con los q1, q2, q3 hallados
    % (Simplificación: Usamos una función de FK parcial rápida o matrices básicas)
    % Matriz de rotación de J3 relativa a Base:
    c1=cos(q1); s1=sin(q1);
    c23=cos(q2+q3); s23=sin(q2+q3); % Asumiendo ejes paralelos J2/J3
    % R03 aproximada para robot antropomórfico vertical
    % X apunta adelante, Y izquierda, Z arriba (local)
    R03 = [c1*c23, -c1*s23, -s1;
           s1*c23, -s1*s23,  c1;
           -s23,   -c23,     0]; 
    % Rotación necesaria en la muñeca: R3_6 = inv(R03) * R_target
    R36 = R03' * R;
    % Extraer Euler Z-Y-Z (o Z-Y-X dependiendo del robot) para J4, J5, J6
    % Para myCobot suele ser: J4(roll), J5(pitch), J6(roll) -> ZYZ
    % Solución ZYZ Euler:
    % q5 = atan2(sqrt(r31^2 + r32^2), r33)
    q5 = atan2(sqrt(R36(3,1)^2 + R36(3,2)^2), R36(3,3));
    if sin(q5) > 1e-4
        q4 = atan2(R36(2,3), R36(1,3));
        q6 = atan2(R36(3,2), -R36(3,1));
    else
        % Singularidad (Gimbal lock, ejes alineados)
        q4 = 0;
        q6 = atan2(-R36(1,2), R36(1,1));
    end
    q = [q1; q2; q3; q4; q5; q6];
    
end