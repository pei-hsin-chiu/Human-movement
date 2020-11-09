function y();
load DataQ1.mat;
%global M;
%-----------HW2策肈----------
cornersa=[0 508 0;464 508 0;464 0 0;0 0 0];
cornersb=[464 511 0;0 511 0;0 1019 0;464 1019 0];
corners=cat(3,cornersa,cornersb);

 Vfp2tc1=[-0.156 0.995 -43.574];
 Vfp2tc2=[0.195 1.142 -41.737];
 Vfp2tc=cat(3,Vfp2tc1,Vfp2tc2);
 [Rg2fp, Vg2fp] = fpRV(corners, Vfp2tc);
 
 Pz1=Vfp2tc(:,3,1);
 Pz2=Vfp2tc(:,3,2);
 Pz=cat(3, Pz1, Pz2);
 for n=4:146
 fpF_local1=vertcat((Fx1(n))',(Fy1(n))',(Fz1(n))');
 fpF_local2=vertcat((Fx2(n))',(Fy2(n))',(Fz2(n))');
 fpF_local=cat(3,fpF_local1,fpF_local2);
 fpM_local1=vertcat((Mx1(n))',(My1(n))',(Mz1(n))');
 fpM_local2=vertcat((Mx2(n))',(My2(n))',(Mz2(n))');
 fpM_local=cat(3,fpM_local1,fpM_local2);
 COP_local = fpCOP(fpF_local, fpM_local, Pz);
 [gCOP, fpFg, Rg2fp, Vg2fp] = ForcePlate(fpF_local, fpM_local, corners, Vfp2tc, Pz);
 [gCOP, nCOP, fpFg, Rg2fp, Vg2fp] = ForcePlateN(fpF_local, fpM_local, corners, Vfp2tc, Pz)
   h = figure('Visible', 'off');
   % h = figure(1);
    %dot
    plot3(gCOP(1),gCOP(2),gCOP(3),'ko');view(2);
   %frame
    x1=[464 0 0 464];
    y1=[0 0 508 508];
    patch(x1, y1,'white');
    x2=[464 0 0 464];
    y2=[511 511 1019 1019];
    patch(x2, y2,'white');
    alpha(0);
    saveas( h , [ 'n' , num2str( n ) , '.jpg' ] );
 end
 %-----------HW2策肈----------
 pic=dir('.\*.jpg');
%ゅン?秖:num
a=size(pic);
num=a(1);
for i=4:146
im(:,:,:,i)=imread(strcat('n' , num2str(i),'.jpg'));
imshow(im(:,:,:,i))
M(i-3)= getframe;
end
movie2avi(M,'problem3.avi','FPS',30)
  %----------------------------------
  %-----------HW2策肈(拜氮)----------
  %{
    1.,﹚穦Τキmoment
    2.COP(center of
    pressure)琌单把σ翴,ㄤ单ぇよz禸,z禸﹚Τmoment,ㄤ緇ㄢ禸moment0
    3.
    4.材そΑ⊿Τ矪瞶NaN计,穦瞷gimbal lock;材そΑ虑パTz,磷Pzだダ0硑ΘNaN拜肈
  %}
  %----------------------------------
end
function [Rg2fp, Vg2fp] = fpRV(corners, Vfp2tc)
X1=((corners(1,:,1)+corners(4,:,1))/2)-((corners(2,:,1)+corners(3,:,1))/2);
Y1=((corners(1,:,1)+corners(2,:,1))/2)-((corners(3,:,1)+corners(4,:,1))/2);
Z1=cross(X1,Y1);
Rg2fp1=[X1/norm(X1);Y1/norm(Y1);Z1/norm(Z1)];
X2=((corners(1, :, 2)+corners(4, :, 2))/2)-((corners(2, :, 2)+corners(3, :, 2))/2);
Y2=((corners(1, :, 2)+corners(2, :, 2))/2)-((corners(3, :, 2)+corners(4, :, 2))/2);
Z2=cross(X2,Y2);
Rg2fp2=[X2/norm(X2);Y2/norm(Y2);Z2/norm(Z2)];
Rg2fp=cat(3,Rg2fp1, Rg2fp2);
Vg2tc1=(((corners(1, :, 1)+(corners(4, :, 1)))/2)+((corners(2, :, 1)+(corners(3, :, 1)))/2))/2;
Vg2fp1=(Vg2tc1-Vfp2tc(:,1))';
Vg2tc2=(((corners(1, :, 2)+(corners(4, :, 2)))/2)+((corners(2, :, 2)+(corners(3, :, 2)))/2))/2;
Vg2fp2=(Vg2tc2-Vfp2tc(:,2))';
Vg2fp=cat(3,Vg2fp1, Vg2fp2);
end

function COP_local = fpCOP(fpF_local, fpM_local, Pz)
Px=((Pz*fpF_local(1,:,1))-fpM_local(2,:,1))/(fpF_local(3,:,1));
Py=((Pz*fpF_local(2,:,1))+fpM_local(1,:,1))/(fpF_local(3,:,1));
COP_local=vertcat(Px,Py,Pz);
end

function [gCOP, fpFg, Rg2fp, Vg2fp] = ForcePlate(fpF_local, fpM_local, corners, Vfp2tc, Pz)
X1=((corners(1,:,1)+corners(4,:,1))/2)-((corners(2,:,1)+corners(3,:,1))/2);
Y1=((corners(1,:,1)+corners(2,:,1))/2)-((corners(3,:,1)+corners(4,:,1))/2);
Z1=cross(X1,Y1);
Rg2fp1=[X1/norm(X1);Y1/norm(Y1);Z1/norm(Z1)];
X2=((corners(1, :, 2)+corners(4, :, 2))/2)-((corners(2, :, 2)+corners(3, :, 2))/2);
Y2=((corners(1, :, 2)+corners(2, :, 2))/2)-((corners(3, :, 2)+corners(4, :, 2))/2);
Z2=cross(X2,Y2);
Rg2fp2=[X2/norm(X2);Y2/norm(Y2);Z2/norm(Z2)];
Rg2fp=cat(3,Rg2fp1, Rg2fp2);
Vg2tc1=(((corners(1, :, 1)+(corners(4, :, 1)))/2)+((corners(2, :, 1)+(corners(3, :, 1)))/2))/2;
Vg2fp1=(Vg2tc1-Vfp2tc(:,1))';
Vg2tc2=(((corners(1, :, 2)+(corners(4, :, 2)))/2)+((corners(2, :, 2)+(corners(3, :, 2)))/2))/2;
Vg2fp2=(Vg2tc2-Vfp2tc(:,2))';
Vg2fp=cat(3,Vg2fp1, Vg2fp2);
Pz1=Vfp2tc(:,3,1);
Pz2=Vfp2tc(:,3,2);
Pz=Pz1;
for n=4:146
    Px1=((Pz*fpF_local(1,:,1))-fpM_local(2,:,1))/(fpF_local(3,:,1));
    Px2=((Pz*fpF_local(1,:,2))-fpM_local(2,:,2))/(fpF_local(3,:,2));
    Py1=((Pz*fpF_local(2,:,1))+fpM_local(1,:,1))/(fpF_local(3,:,1));
    Py2=((Pz*fpF_local(2,:,2))+fpM_local(1,:,2))/(fpF_local(3,:,2));
    COP_local1=vertcat(Px1,Py1,Pz1);
    COP_local2=vertcat(Px2,Py2,Pz2);
    gCOP1=Vg2fp1+Rg2fp1*COP_local1;
    gCOP2=Vg2fp2+Rg2fp2*COP_local2;
    gCOP=cat(3, gCOP1, gCOP2);
    fpFg1=(fpF_local(:,1))'*(Rg2fp1)';
    fpFg2=(fpF_local(:,2))'*(Rg2fp2)';
    fpFg=cat(3, fpFg1, fpFg2);
end
end

function [gCOP, nCOP, fpFg, Rg2fp, Vg2fp] = ForcePlateN(fpF_local, fpM_local, corners, Vfp2tc, Pz)
X1=((corners(1,:,1)+corners(4,:,1))/2)-((corners(2,:,1)+corners(3,:,1))/2);
Y1=((corners(1,:,1)+corners(2,:,1))/2)-((corners(3,:,1)+corners(4,:,1))/2);
Z1=cross(X1,Y1);
Rg2fp1=[X1/norm(X1);Y1/norm(Y1);Z1/norm(Z1)];
X2=((corners(1, :, 2)+corners(4, :, 2))/2)-((corners(2, :, 2)+corners(3, :, 2))/2);
Y2=((corners(1, :, 2)+corners(2, :, 2))/2)-((corners(3, :, 2)+corners(4, :, 2))/2);
Z2=cross(X2,Y2);
Rg2fp2=[X2/norm(X2);Y2/norm(Y2);Z2/norm(Z2)];
Rg2fp=cat(3,Rg2fp1, Rg2fp2);
Vg2tc1=(((corners(1, :, 1)+(corners(4, :, 1)))/2)+((corners(2, :, 1)+(corners(3, :, 1)))/2))/2;
Vg2fp1=(Vg2tc1-Vfp2tc(:,1))';
Vg2tc2=(((corners(1, :, 2)+(corners(4, :, 2)))/2)+((corners(2, :, 2)+(corners(3, :, 2)))/2))/2;
Vg2fp2=(Vg2tc2-Vfp2tc(:,2))';
Vg2fp=cat(3,Vg2fp1, Vg2fp2);
Pz1=Pz(:,:,1);
Pz2=Pz(:,:,2);
Pz=Pz1;


    Px1=((Pz1*fpF_local(1,:,1))-fpM_local(2,:,1))/(fpF_local(3,:,1));
    Px2=((Pz2*fpF_local(1,:,2))-fpM_local(2,:,2))/(fpF_local(3,:,2));
    Py1=((Pz1*fpF_local(2,:,1))+fpM_local(1,:,1))/(fpF_local(3,:,1));
    Py2=((Pz2*fpF_local(2,:,2))+fpM_local(1,:,2))/(fpF_local(3,:,2));
    COP_local1=vertcat(Px1,Py1,Pz1);
    COP_local2=vertcat(Px2,Py2,Pz2);
    gCOP1=Vg2fp1+Rg2fp1*COP_local1;
    gCOP2=Vg2fp2+Rg2fp2*COP_local2;
    nCOP=cat(3, gCOP1, gCOP2);
    fpFg1=(fpF_local(:,1))'*(Rg2fp1)';
    fpFg2=(fpF_local(:,2))'*(Rg2fp2)';
    fpFg=cat(3, fpFg1, fpFg2);
fpF_local;
    fpF_global1=Rg2fp(:,:,1)*fpF_local(:,:,1);
    fpF_global2=Rg2fp(:,:,2)*fpF_local(:,:,2);
    fpF_global=fpF_global1+fpF_global2;
   
    fpM_global1=cross(Vg2fp(:,1), fpF_global1)+Rg2fp(:,:,1)*fpM_local(:,:,1);
    fpM_global2=cross(Vg2fp(:,2), fpF_global2)+Rg2fp(:,:,2)*fpM_local(:,:,2);
    fpM_global=fpM_global1+fpM_global2;
   Px= (Pz1*fpF_global(1)-fpM_global(2))/(fpF_global(3));
   Py= (Pz1*fpF_global(2)+fpM_global(1))/(fpF_global(3));
   gCOP=[Px; Py; Pz1];
   

    

 
end
