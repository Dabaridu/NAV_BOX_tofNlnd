TS=.1;
PHIS=0.;
A0=400000.;
A1=-6000.;
A2=-16.1;
XH=0.;
XDH=0.;
SIGNOISE=1000.;
ORDER=2;
T=0.;
S=0.;
H=.001;
PHI=[1 TS ;0 1];
P=[99999999 0;0 999999999];
IDNP=eye(ORDER);
Q=zeros(ORDER);
RMAT=SIGNOISE^2;
Q(1,1)=TS*TS*TS*PHIS/3.;
Q(1,2)=.5*TS*TS*PHIS;
Q(2,1)=Q(1,2);
Q(2,2)=PHIS*TS;
HMAT=[1 0];
HT=HMAT';
PHIT=PHI';
count=0;
for T=0:TS:30
    PHIP=PHI*P;
    PHIPPHIT=PHIP*PHIT;
    M=PHIPPHIT+Q;
    HM=HMAT*M;
    HMHT=HM*HT; 
    HMHTR=HMHT+RMAT;
    HMHTRINV=inv(HMHTR);
    MHT=M*HT;
    K=MHT*HMHTRINV;
    KH=K*HMAT;
    IKH=IDNP-KH;
    P=IKH*M;
    XNOISE=SIGNOISE*randn;
    X=A0+A1*T+A2*T*T;
    XD=A1+2*A2*T;
    XS=X+XNOISE;
    RES=XS-XH-TS*XDH+16.1*TS*TS;
    XH=XH+XDH*TS-16.1*TS*TS+K(1,1)*RES;
    XDH=XDH-32.2*TS+K(2,1)*RES;
    SP11=sqrt(P(1,1));
    SP22=sqrt(P(2,2));
    XHERR=X-XH;
    XDHERR=XD-XDH;
    SP11P=-SP11;
    SP22P=-SP22;
    count=count+1;
    ArrayT(count)=T;
    ArrayX(count)=X;
    ArrayXH(count)=XH;
    ArrayXD(count)=XD;
    ArrayXDH(count)=XDH;
    ArrayXHERR(count)=XHERR;
    ArraySP11(count)=SP11;
    ArraySP11P(count)=SP11P;
    ArrayXDHERR(count)=XDHERR;
    ArraySP22(count)=SP22;
    ArraySP22P(count)=SP22P;
end
figure
plot(ArrayT,ArrayXHERR,ArrayT,ArraySP11,ArrayT,ArraySP11P),grid
legend('Error','Upper Bound','Lower Bound')
xlabel('Time (Sec)')
ylabel('Error in Estimate of Altitude (Ft)')
axis([0 30 -1500 1500])
figure
plot(ArrayT,ArrayXDHERR,ArrayT,ArraySP22,ArrayT,ArraySP22P),grid
legend('Error','Upper Bound','Lower Bound')
xlabel('Time (Sec)')
ylabel('Error in Estimate of Velocity (Ft/Sec)')
axis([0 30 -200 200])
clc
output=[ArrayT; ArrayX; ArrayXH; ArrayXD; ArrayXDH]';
save datfil output -ascii
output=[ArrayT; ArrayXHERR; ArraySP11; ArraySP11P; ArrayXDHERR; ArraySP22; ArraySP22P]';
save covfil output -ascii
disp 'simulation finished'