clc, clear all, close all

p = [1; 3; -15];
ang = [-15; 90; -90]*pi/180;
ang = [179; 0; 91]*pi/180;

R1 = compute_R_angles(ang)

ang2 = compute_euler_angles_optimization(R1)*180/pi
R2 = compute_R_angles(ang2)


T = calc_T_pos_ang(p,ang)

[u,alf] = calc_T_extract_u_angle(T)

T2 = calc_T_u_angle(alf,u,p)

