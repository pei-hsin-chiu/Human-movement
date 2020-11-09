function y()
load DataQ1.mat;
%-----------HW1習題一function1----------
Pcoord=cat(3,RASI, LASI, RPSI);
side='R'|'r';
Mkstr=vertcat('RASI', 'LASI', 'RPSI');
[Rg2p, Vg2p, Pcoord_local]=CoordPelvis(Pcoord, side, Mkstr);
rRg2p=Rg2p;
rVg2p=Vg2p;
Pcoord=cat(3,LASI, RASI, LPSI);
side='L'|'l';
Mkstr=vertcat('LASI', 'RASI', 'LPSI');
[Rg2p, Vg2p, Pcoord_local]=CoordPelvis(Pcoord, side, Mkstr);
lRg2p=Rg2p;
lVg2p=Vg2p;  
%------------------------function2-----------
Tcoord=cat(3,RTRO, RLFC, RMFC);
side='R'|'r';
Mkstr=vertcat('RTRO', 'RLFC', 'RMFC');
[Rg2t, Vg2t, Tcoord_local]=CoordThigh(Tcoord, side, Mkstr);
rRg2t=Rg2t;
rVg2t=Vg2t;
Tcoord=cat(3,LTRO, LLFC, LMFC);
side='L'|'l';
Mkstr=vertcat('LTRO', 'LLFC', 'LMFC');
[Rg2t, Vg2t, Tcoord_local]=CoordThigh(Tcoord, side, Mkstr);
lRg2t=Rg2t;
lVg2t=Vg2t;
%------------------------function3-----------
Scoord=cat(3,RSHA, RMMA, RLMA, RTT);
side='R'|'r';
Mkstr=vertcat('RSHA','RMMA','RLMA', 'RTT ');
[Rg2s, Vg2s, Scoord_local]=CoordShank(Scoord, side, Mkstr);
rRg2s=Rg2s;
rVg2s=Vg2s;
Scoord=cat(3,LSHA, LMMA, LLMA, LTT);
side='L'|'l';
Mkstr=vertcat('LSHA','LMMA','LLMA','LTT ');
[Rg2s, Vg2s, Scoord_local]=CoordShank(Scoord, side, Mkstr);
lRg2s=Rg2s;
lVg2s=Vg2s;
%------------------------function4-----------
Fcoord=cat(3,RFOO, RTOE, RHEE);
side='R'|'r';
Mkstr=vertcat('RFOO', 'RTOE', 'RHEE');
[Rg2f, Vg2f, Fcoord_local]=CoordFoot(Fcoord, side, Mkstr);
rRg2f=Rg2f;
rVg2f=Vg2f;
Fcoord=cat(3,LFOO, LTOE, LHEE);
side='L'|'l';
Mkstr=vertcat('LFOO', 'LTOE', 'LHEE');
[Rg2f, Vg2f, Fcoord_local]=CoordFoot(Fcoord, side, Mkstr);
lRg2f=Rg2f;
lVg2f=Vg2f;
%----------------------------------
%-----------HW1習題二----------
Pcoord=cat(3,RASI, LASI, RPSI, LASI, RASI, LPSI);
Tcoord=cat(3,RTRO, RLFC, RMFC, LTRO, LLFC, LMFC);
Scoord=cat(3,RSHA, RMMA, RLMA, RTT, LSHA, LMMA, LLMA, LTT);
Fcoord=cat(3,RFOO, RTOE, RHEE, LFOO, LTOE, LHEE);
P_global=cat(3, Pcoord, Tcoord, Scoord, Fcoord);
P_global_old=P_global;
Rg2l=cat(3, Rg2p, Rg2t, Rg2s, Rg2f);
size(Rg2l);
Vg2l_before=cat(2, Vg2p', Vg2t', Vg2s', Vg2f');
Vg2l=Vg2l_before';
P_local=CoordG2L(Rg2l, Vg2l, P_global);
P_global=CoordL2G(Rg2l, Vg2l, P_local);
P_global_new=P_global;

isequal(P_global_new(:,:,1), P_global_old(:,:,1))

%-------------HW1問答題---------------------
%1.甲乙丙丁己
%----------------------------------------------
end
function[Rg2p, Vg2p, Pcoord_local]=CoordPelvis(Pcoord, side, Mkstr)
C=zeros(159,3);%V的(159,3)矩陣
B=zeros(3,3,159);%R的(3,3,159)矩陣
A=zeros(3,3,159);%Pcoord的(3,3,159)矩陣
for n=1:159
if side=='R'
zrp=(Pcoord(n,:,1)-Pcoord(n,:,2))/norm(Pcoord(n,:,1)-Pcoord(n,:,2));
yrp=(cross(zrp, (Pcoord(n,:,1)-Pcoord(n,:,3))))/norm(cross(zrp, (Pcoord(n,:,1)-Pcoord(n,:,3))));
xrp=cross(yrp, zrp);
Rg2p_matrix=cat(2, xrp', yrp', zrp');
B(:,:,n)=Rg2p_matrix;
Rg2p=B;
Vg2p_matrix=Pcoord(n,:,1);
C(n,:)=Vg2p_matrix;
Vg2p=C;
Pcoord_local_RASI=(Rg2p(:,:,n))'*(Pcoord(n,:,1)-Vg2p(n,:))';
Pcoord_local_LASI=(Rg2p(:,:,n))'*(Pcoord(n,:,2)-Vg2p(n,:))';
Pcoord_local_RPSI=(Rg2p(:,:,n))'*(Pcoord(n,:,3)-Vg2p(n,:))';
Pcoord_local_matrix=cat(2,Pcoord_local_RASI, Pcoord_local_LASI, Pcoord_local_RPSI);
A(:,:,n)=Pcoord_local_matrix';
Pcoord_local=A;
else
zrp=(Pcoord(n,:,2)-Pcoord(n,:,1))/norm(Pcoord(n,:,2)-Pcoord(n,:,1));
yrp=(cross(zrp, (Pcoord(n,:,2)-Pcoord(n,:,3))))/norm(cross(zrp, (Pcoord(n,:,2)-Pcoord(n,:,3))));
xrp=cross(yrp, zrp);
Rg2p_matrix=cat(2, xrp', yrp', zrp');
B(:,:,n)=Rg2p_matrix;
Rg2p=B;
Vg2p_matrix=Pcoord(n,:,3);
C(n,:)=Vg2p_matrix;
Vg2p=C;
Pcoord_local_RASI=(Rg2p(:,:,n))'*(Pcoord(n,:,2)-Vg2p(n,:))';
Pcoord_local_LASI=(Rg2p(:,:,n))'*(Pcoord(n,:,1)-Vg2p(n,:))';
Pcoord_local_LPSI=(Rg2p(:,:,n))'*(Pcoord(n,:,3)-Vg2p(n,:))';
Pcoord_local_matrix=cat(2,Pcoord_local_RASI, Pcoord_local_LASI, Pcoord_local_LPSI);
A(:,:,n)=Pcoord_local_matrix';
Pcoord_local=A;
end
end
end

function[Rg2t, Vg2t, Tcoord_local]=CoordThigh(Tcoord, side, Mkstr)
C=zeros(159,3);%V的(159,3)矩陣
B=zeros(3,3,159);%R的(3,3,159)矩陣
A=zeros(3,3,159);%Tcoord的(3,3,159)矩陣
for n=1:159
if side=='R'
zrt=(Tcoord(n,:,2)-Tcoord(n,:,3))/norm(Tcoord(n,:,2)-Tcoord(n,:,3));
xrt=(cross((Tcoord(n,:,1)-Tcoord(n,:,2)), zrt))/norm(cross((Tcoord(n,:,1)-Tcoord(n,:,2)), zrt));
yrt=cross(zrt, xrt);
Rg2t_matrix=cat(2, xrt', yrt', zrt');
B(:,:,n)=Rg2t_matrix;
Rg2t=B;
Vg2t_matrix=Tcoord(n,:,1);
C(n,:)=Vg2t_matrix;
Vg2t=C;
Tcoord_local_RTRO=(Rg2t(:,:,n))'*(Tcoord(n,:,1)-Vg2t(n,:))';
Tcoord_local_RLFC=(Rg2t(:,:,n))'*(Tcoord(n,:,2)-Vg2t(n,:))';
Tcoord_local_RMFC=(Rg2t(:,:,n))'*(Tcoord(n,:,3)-Vg2t(n,:))';
Tcoord_local_matrix=cat(2,Tcoord_local_RTRO, Tcoord_local_RLFC, Tcoord_local_RMFC);
A(:,:,n)=Tcoord_local_matrix';
Tcoord_local=A;
else
zlt=(Tcoord(n,:,2)-Tcoord(n,:,3))/norm(Tcoord(n,:,2)-Tcoord(n,:,3));
xlt=(cross((Tcoord(n,:,1)-Tcoord(n,:,2)), zlt))/norm(cross((Tcoord(n,:,1)-Tcoord(n,:,2)), zlt));
ylt=cross(zlt, xlt);
Rg2t_matrix=cat(2, xlt', ylt', zlt');
B(:,:,n)=Rg2t_matrix;
Rg2t=B;
Vg2t_matrix=Tcoord(n,:,1);
C(n,:)=Vg2t_matrix;
Vg2t=C;
Tcoord_local_LTRO=(Rg2t(:,:,n))'*(Tcoord(n,:,1)-Vg2t(n,:))';
Tcoord_local_LLFC=(Rg2t(:,:,n))'*(Tcoord(n,:,2)-Vg2t(n,:))';
Tcoord_local_LMFC=(Rg2t(:,:,n))'*(Tcoord(n,:,3)-Vg2t(n,:))';
Tcoord_local_matrix=cat(2,Tcoord_local_LTRO, Tcoord_local_LLFC, Tcoord_local_LMFC);
A(:,:,n)=Tcoord_local_matrix';
Tcoord_local=A;
end
end
end

function[Rg2s, Vg2s, Scoord_local]=CoordShank(Scoord, side, Mkstr)
C=zeros(159,3);%V的(159,3)矩陣
B=zeros(3,3,159);%R的(3,3,159)矩陣
A=zeros(4,3,159);%Scoord的(4,3,159)矩陣
for n=1:159
if side=='R'
xrs=(cross((Scoord(n,:,1)-Scoord(n,:,2)), (Scoord(n,:,3)-Scoord(n,:,2))))/norm(cross((Scoord(n,:,1)-Scoord(n,:,2)), (Scoord(n,:,3)-Scoord(n,:,2))));
zrs=(cross(xrs, (Scoord(n,:,4)-0.5*(Scoord(n,:,2)+Scoord(n,:,3)))))/norm(cross(xrs, (Scoord(n,:,4)-0.5*(Scoord(n,:,2)+Scoord(n,:,3)))));
yrs=cross(zrs, xrs);
Rg2s_matrix=cat(2, xrs', yrs', zrs');
B(:,:,n)=Rg2s_matrix;
Rg2s=B;
Vg2s_matrix=Scoord(n,:,4);
C(n,:)=Vg2s_matrix;
Vg2s=C;
Scoord_local_RSHA=(Rg2s(:,:,n))'*(Scoord(n,:,1)-Vg2s(n,:))';
Scoord_local_RMMA=(Rg2s(:,:,n))'*(Scoord(n,:,2)-Vg2s(n,:))';
Scoord_local_RLMA=(Rg2s(:,:,n))'*(Scoord(n,:,3)-Vg2s(n,:))';
Scoord_local_RTT=(Rg2s(:,:,n))'*(Scoord(n,:,4)-Vg2s(n,:))';
Scoord_local_matrix=cat(2,Scoord_local_RSHA, Scoord_local_RMMA, Scoord_local_RLMA, Scoord_local_RTT);
A(:,:,n)=Scoord_local_matrix';
Scoord_local=A;
else
xls=(cross((Scoord(n,:,1)-Scoord(n,:,2)), (Scoord(n,:,3)-Scoord(n,:,2))))/norm(cross((Scoord(n,:,1)-Scoord(n,:,2)), (Scoord(n,:,3)-Scoord(n,:,2))));
zls=(cross(xls, (Scoord(n,:,4)-0.5*(Scoord(n,:,2)+Scoord(n,:,3)))))/norm(cross(xls, (Scoord(n,:,4)-0.5*(Scoord(n,:,2)+Scoord(n,:,3)))));
yls=cross(zls, xls);
Rg2s_matrix=cat(2, xls', yls', zls');
B(:,:,n)=Rg2s_matrix;
Rg2s=B;
Vg2s_matrix=Scoord(n,:,4);
C(n,:)=Vg2s_matrix;
Vg2s=C;
Scoord_local_LSHA=(Rg2s(:,:,n))'*(Scoord(n,:,1)-Vg2s(n,:))';
Scoord_local_LMMA=(Rg2s(:,:,n))'*(Scoord(n,:,2)-Vg2s(n,:))';
Scoord_local_LLMA=(Rg2s(:,:,n))'*(Scoord(n,:,3)-Vg2s(n,:))';
Scoord_local_LTT=(Rg2s(:,:,n))'*(Scoord(n,:,4)-Vg2s(n,:))';
Scoord_local_matrix=cat(2,Scoord_local_LSHA, Scoord_local_LMMA, Scoord_local_LLMA, Scoord_local_LTT);
A(:,:,n)=Scoord_local_matrix';
Scoord_local=A;
end
end
end

function[Rg2f, Vg2f, Fcoord_local]=CoordFoot(Fcoord, side, Mkstr)
C=zeros(159,3);%V的(159,3)矩陣
B=zeros(3,3,159);%R的(3,3,159)矩陣
A=zeros(3,3,159);%Tcoord的(3,3,159)矩陣
for n=1:159
if side=='R'
xrf=(0.5*((Fcoord(n,:,1)+Fcoord(n,:,2)))-Fcoord(n,:,3))/norm(0.5*((Fcoord(n,:,1)+Fcoord(n,:,2)))-Fcoord(n,:,3));
yrf=(cross(xrf, (Fcoord(n,:,1)-Fcoord(n,:,2))))/norm(cross(xrf, (Fcoord(n,:,1)-Fcoord(n,:,2))));
zrf=cross(xrf, yrf);
Rg2f_matrix=cat(2, xrf', yrf', zrf');
B(:,:,n)=Rg2f_matrix;
Rg2f=B;
Vg2f_matrix=Fcoord(n,:,3);
C(n,:)=Vg2f_matrix;
Vg2f=C;
Fcoord_local_RFOO=(Rg2f(:,:,n))'*(Fcoord(n,:,1)-Vg2f(n,:))';
Fcoord_local_RTOE=(Rg2f(:,:,n))'*(Fcoord(n,:,2)-Vg2f(n,:))';
Fcoord_local_RHEE=(Rg2f(:,:,n))'*(Fcoord(n,:,3)-Vg2f(n,:))';
Fcoord_local_matrix=cat(2,Fcoord_local_RFOO, Fcoord_local_RTOE, Fcoord_local_RHEE);
A(:,:,n)=Fcoord_local_matrix';
Fcoord_local=A;
else
xlf=(0.5*((Fcoord(n,:,1)+Fcoord(n,:,2)))-Fcoord(n,:,3))/norm(0.5*((Fcoord(n,:,1)+Fcoord(n,:,2)))-Fcoord(n,:,3));
ylf=(cross(xlf, (Fcoord(n,:,1)-Fcoord(n,:,2))))/norm(cross(xlf, (Fcoord(n,:,1)-Fcoord(n,:,2))));
zlf=cross(xlf, ylf);
Rg2f_matrix=cat(2, xlf', ylf', zlf');
B(:,:,n)=Rg2f_matrix;
Rg2f=B;
Vg2f_matrix=Fcoord(n,:,3);
C(n,:)=Vg2f_matrix;
Vg2f=C;
Fcoord_local_LFOO=(Rg2f(:,:,n))'*(Fcoord(n,:,1)-Vg2f(n,:))';
Fcoord_local_LTOE=(Rg2f(:,:,n))'*(Fcoord(n,:,2)-Vg2f(n,:))';
Fcoord_local_LHEE=(Rg2f(:,:,n))'*(Fcoord(n,:,3)-Vg2f(n,:))';
Fcoord_local_matrix=cat(2,Fcoord_local_LFOO, Fcoord_local_LTOE, Fcoord_local_LHEE);
A(:,:,n)=Fcoord_local_matrix';
Fcoord_local=A;
end
end
end

function P_local=CoordG2L(Rg2l, Vg2l, P_global)
A=zeros(159,3,6);%P_local的(159,3,6)矩陣
B=zeros(159,3,6);%T_local的(159,3,6)矩陣
C=zeros(159,3,8);%S_local的(159,3,8)矩陣
D=zeros(159,3,6);%F_local的(159,3,6)矩陣
for n=1:159
Pcoord_local_RASI=(Rg2l(:,:,n))'*(P_global(n,:,1)-Vg2l(n,:))';
Pcoord_local_LASI=(Rg2l(:,:,n))'*(P_global(n,:,2)-Vg2l(n,:))';
Pcoord_local_RPSI=(Rg2l(:,:,n))'*(P_global(n,:,3)-Vg2l(n,:))';
Pcoord_local_LPSI=(Rg2l(:,:,n))'*(P_global(n,:,6)-Vg2l(n,:))';
Pcoord_local_matrix=cat(3, Pcoord_local_RASI, Pcoord_local_LASI, Pcoord_local_RPSI, Pcoord_local_LASI, Pcoord_local_RASI,Pcoord_local_LPSI);
A(n,:,:)=Pcoord_local_matrix;
Pcoord_local=A;
end
for m=160:318
Tcoord_local_RTRO=(Rg2l(:,:,m))'*(P_global(m-159,:,7)-Vg2l(m,:))';
Tcoord_local_RLFC=(Rg2l(:,:,m))'*(P_global(m-159,:,8)-Vg2l(m,:))';
Tcoord_local_RMFC=(Rg2l(:,:,m))'*(P_global(m-159,:,9)-Vg2l(m,:))';
Tcoord_local_LTRO=(Rg2l(:,:,m))'*(P_global(m-159,:,10)-Vg2l(m,:))';
Tcoord_local_LLFC=(Rg2l(:,:,m))'*(P_global(m-159,:,11)-Vg2l(m,:))';
Tcoord_local_LMFC=(Rg2l(:,:,m))'*(P_global(m-159,:,12)-Vg2l(m,:))';
Tcoord_local_matrix=cat(3,Tcoord_local_RTRO, Tcoord_local_RLFC, Tcoord_local_RMFC, Tcoord_local_LTRO, Tcoord_local_LLFC, Tcoord_local_LMFC);
B(m-159,:,:)=Tcoord_local_matrix;
Tcoord_local=B;
end
for k=319:477
Scoord_local_RSHA=(Rg2l(:,:,k))'*(P_global(k-318,:,13)-Vg2l(k,:))';
Scoord_local_RMMA=(Rg2l(:,:,k))'*(P_global(k-318,:,14)-Vg2l(k,:))';
Scoord_local_RLMA=(Rg2l(:,:,k))'*(P_global(k-318,:,15)-Vg2l(k,:))';
Scoord_local_RTT=(Rg2l(:,:,k))'*(P_global(k-318,:,16)-Vg2l(k,:))';
Scoord_local_LSHA=(Rg2l(:,:,k))'*(P_global(k-318,:,17)-Vg2l(k,:))';
Scoord_local_LMMA=(Rg2l(:,:,k))'*(P_global(k-318,:,18)-Vg2l(k,:))';
Scoord_local_LLMA=(Rg2l(:,:,k))'*(P_global(k-318,:,19)-Vg2l(k,:))';
Scoord_local_LTT=(Rg2l(:,:,k))'*(P_global(k-318,:,20)-Vg2l(k,:))';
Scoord_local_matrix=cat(3,Scoord_local_RSHA, Scoord_local_RMMA, Scoord_local_RLMA, Scoord_local_RTT, Scoord_local_LSHA, Scoord_local_LMMA, Scoord_local_LLMA, Scoord_local_LTT);
C(k-318,:,:)=Scoord_local_matrix;
Scoord_local=C;
end
for i=478:636
Fcoord_local_RFOO=(Rg2l(:,:,i))'*(P_global(i-477,:,21)-Vg2l(i,:))';
Fcoord_local_RTOE=(Rg2l(:,:,i))'*(P_global(i-477,:,22)-Vg2l(i,:))';
Fcoord_local_RHEE=(Rg2l(:,:,i))'*(P_global(i-477,:,23)-Vg2l(i,:))';
Fcoord_local_LFOO=(Rg2l(:,:,i))'*(P_global(i-477,:,24)-Vg2l(i,:))';
Fcoord_local_LTOE=(Rg2l(:,:,i))'*(P_global(i-477,:,25)-Vg2l(i,:))';
Fcoord_local_LHEE=(Rg2l(:,:,i))'*(P_global(i-477,:,26)-Vg2l(i,:))';
Fcoord_local_matrix=cat(3,Fcoord_local_RFOO, Fcoord_local_RTOE, Fcoord_local_RHEE, Fcoord_local_LFOO, Fcoord_local_LTOE, Fcoord_local_LHEE);
D(i-477,:,:)=Fcoord_local_matrix;
Fcoord_local=D;
end
P_local=cat(3, Pcoord_local, Tcoord_local, Scoord_local, Fcoord_local);
end

function P_global=CoordL2G(Rg2l, Vg2l, P_local)
A=zeros(159,3,6);%P_local的(159,3,6)矩陣
B=zeros(159,3,6);%T_local的(159,3,6)矩陣
C=zeros(159,3,8);%S_local的(159,3,8)矩陣
D=zeros(159,3,6);%F_local的(159,3,6)矩陣
for n=1:159
Pcoord_global_RASI=Rg2l(:,:,n)*P_local(n,:,1)'+Vg2l(n,:)';
Pcoord_global_LASI=Rg2l(:,:,n)*P_local(n,:,2)'+Vg2l(n,:)';
Pcoord_global_RPSI=Rg2l(:,:,n)*P_local(n,:,3)'+Vg2l(n,:)';
Pcoord_global_LPSI=Rg2l(:,:,n)*P_local(n,:,6)'+Vg2l(n,:)';
Pcoord_global_matrix=cat(3, Pcoord_global_RASI, Pcoord_global_LASI, Pcoord_global_RPSI, Pcoord_global_LASI, Pcoord_global_RASI,Pcoord_global_LPSI);
A(n,:,:)=Pcoord_global_matrix;
Pcoord_global=A;
end
for m=160:318
Tcoord_global_RTRO=Rg2l(:,:,m)*P_local(m-159,:,7)'+Vg2l(m,:)';
Tcoord_global_RLFC=Rg2l(:,:,m)*P_local(m-159,:,8)'+Vg2l(m,:)';
Tcoord_global_RMFC=Rg2l(:,:,m)*P_local(m-159,:,9)'+Vg2l(m,:)';
Tcoord_global_LTRO=Rg2l(:,:,m)*P_local(m-159,:,10)'+Vg2l(m,:)';
Tcoord_global_LLFC=Rg2l(:,:,m)*P_local(m-159,:,11)'+Vg2l(m,:)';
Tcoord_global_LMFC=Rg2l(:,:,m)*P_local(m-159,:,12)'+Vg2l(m,:)';
Tcoord_global_matrix=cat(3,Tcoord_global_RTRO, Tcoord_global_RLFC, Tcoord_global_RMFC, Tcoord_global_LTRO, Tcoord_global_LLFC, Tcoord_global_LMFC);
B(m-159,:,:)=Tcoord_global_matrix;
Tcoord_global=B;
end
for k=319:477
Scoord_global_RSHA=Rg2l(:,:,k)*P_local(k-318,:,13)'+Vg2l(k,:)';
Scoord_global_RMMA=Rg2l(:,:,k)*P_local(k-318,:,14)'+Vg2l(k,:)';
Scoord_global_RLMA=Rg2l(:,:,k)*P_local(k-318,:,15)'+Vg2l(k,:)';
Scoord_global_RTT=Rg2l(:,:,k)*P_local(k-318,:,16)'+Vg2l(k,:)';
Scoord_global_LSHA=Rg2l(:,:,k)*P_local(k-318,:,17)'+Vg2l(k,:)';
Scoord_global_LMMA=Rg2l(:,:,k)*P_local(k-318,:,18)'+Vg2l(k,:)';
Scoord_global_LLMA=Rg2l(:,:,k)*P_local(k-318,:,19)'+Vg2l(k,:)';
Scoord_global_LTT=Rg2l(:,:,k)*P_local(k-318,:,20)'+Vg2l(k,:)';
Scoord_global_matrix=cat(3,Scoord_global_RSHA, Scoord_global_RMMA, Scoord_global_RLMA, Scoord_global_RTT, Scoord_global_LSHA, Scoord_global_LMMA, Scoord_global_LLMA, Scoord_global_LTT);
C(k-318,:,:)=Scoord_global_matrix;
Scoord_global=C;
end
for i=478:636
Fcoord_global_RFOO=Rg2l(:,:,i)*P_local(i-477,:,21)'+Vg2l(i,:)';
Fcoord_global_RTOE=Rg2l(:,:,i)*P_local(i-477,:,22)'+Vg2l(i,:)';
Fcoord_global_RHEE=Rg2l(:,:,i)*P_local(i-477,:,23)'+Vg2l(i,:)';
Fcoord_global_LFOO=Rg2l(:,:,i)*P_local(i-477,:,24)'+Vg2l(i,:)';
Fcoord_global_LTOE=Rg2l(:,:,i)*P_local(i-477,:,25)'+Vg2l(i,:)';
Fcoord_global_LHEE=Rg2l(:,:,i)*P_local(i-477,:,26)'+Vg2l(i,:)';
Fcoord_global_matrix=cat(3,Fcoord_global_RFOO, Fcoord_global_RTOE, Fcoord_global_RHEE, Fcoord_global_LFOO, Fcoord_global_LTOE, Fcoord_global_LHEE);
D(i-477,:,:)=Fcoord_global_matrix;
Fcoord_global=D;
end
P_global=cat(3, Pcoord_global, Tcoord_global, Scoord_global, Fcoord_global);
end
