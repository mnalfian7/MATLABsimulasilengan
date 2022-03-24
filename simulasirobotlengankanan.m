clear; clc;

%% Inisiasi Ukuran Lengan Robot %%
a1 = 25;   %ukuran panjang lengan atas 25 cm
a2 = 22.5;   %ukuran panjang lengan bawah 22,5 cm

%% Fungsi Robot "Upperlimb Exoskeleton %%
robot = SerialLink( [ Revolute('a', a1) Revolute('a', a2) ] ,'name', 'my robot'); %tepat dua joint
robot.offset=[-pi/2 0]; %jadikan dia posisi awal dari upperlimb exos

%%  Forward Kinematics %%

% Inisiasi konfigurasi matriks %
% dalam bentuk nilai radian %
qz = [0 0];                 %posisi awal q1=0 q2=0
qa = [1.5708 2.6180];     %gerakan mengapit q1=90 q2=150
qb = [0 1.5708];           %gerakan mengambil q1=0 q2=90
qc = [3.14159 0];          %gerakan mengangkat q1=180 q2=0
fk0 = robot.fkine(qz);
fk1 = robot.fkine(qa);
fk2 = robot.fkine(qb);
fk3 = robot.fkine(qc);

%% Inverse Kinematics %% Numerical Solution Ways!
%1. Gerakan Mengapit%
T1=transl(5.514,11.25,0); %(posisi x,y,z) didapat dari forward kinematics
ik1=robot.ikine(T1,[0 0], 'mask', [1 1 0 0 0 0]);

%2. Gerakan Mengambil%
T2=transl(22.5,25,0); %(posisi x,y,z) didapat dari forward kinematics
ik2=robot.ikine(T2,[0 0], 'mask', [1 1 0 0 0 0]);

%3. Gerakan Mengangkat%
T3=transl(0,47.5,0); %(posisi x,y,z) didapat dari forward kinematics
ik3=robot.ikine(T3,[0 0], 'mask', [1 1 0 0 0 0]);

%% Trajectory Planning %%
%jtraj(joint awal, joint akhir, banyaknya step);
tg1=jtraj(qz,qa,10);
tg2=jtraj(qz,qb,10);
tg3=jtraj(qz,qc,10);

%simulasi gerak --> robot.plot(tg)
%% Jacobian Matriks %%
% Mencari nilai matriks jacobianJ
J1=robot.jacobe(qa);
J2=robot.jacobe(qb);
J3=robot.jacobe(qc); 

% Membentuk matriks jacobian menjadi square matriks
Ja=[J1(1,1) J1(1,2); J1(2,1) J1(2,2)];
Jb=[J2(1,1) J2(1,2); J2(2,1) J2(2,2)];
Jc=[J3(1,1) J3(1,2); J3(2,1) J3(2,2)];

% Inverse matriks jacobian, syarat : (1) square matrix (2) determinan ≠ 0
Jai=inv(Ja);
Jbi=inv(Jb);
Jci=inv(Jc); %inf, karena matriks singular det = 0