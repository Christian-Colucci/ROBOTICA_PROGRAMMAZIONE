d=[0.525 0 0 0.86 0 0.1]
teta= [0 -1.5708 0 0 0 0]
alfa= [1.5708 0 1.5708 -1.5708 1.5708 0]
a=[0.15 0.79 0.15 0 0 0]

T01=trvec2tform([0 0 d(1)])*rotm2tform(rotz(teta(1)))*trvec2tform([a(1) 0 0])*rotm2tform(rotx(alfa(1)));
T12=trvec2tform([0 0 d(2)])*rotm2tform(rotz(teta(2)))*trvec2tform([a(2) 0 0])*rotm2tform(rotx(alfa(2)));
T23=trvec2tform([0 0 d(3)])*rotm2tform(rotz(teta(3)))*trvec2tform([a(3) 0 0])*rotm2tform(rotx(alfa(3)));
T34=trvec2tform([0 0 d(4)])*rotm2tform(rotz(teta(4)))*trvec2tform([a(4) 0 0])*rotm2tform(rotx(alfa(4)));
T45=trvec2tform([0 0 d(5)])*rotm2tform(rotz(teta(5)))*trvec2tform([a(5) 0 0])*rotm2tform(rotx(alfa(5)));
T56=trvec2tform([0 0 d(6)])*rotm2tform(rotz(teta(6)))*trvec2tform([a(6) 0 0])*rotm2tform(rotx(alfa(6)));

T46=T45*T56
T36=T34*T46
T26=T23*T36
T16=T12*T26
T06=T01*T16

t0=tform2trvec(T06);
R0=tform2rotm(T06);

eul0=rotm2eul(R0);
axis_angle0=rotm2axang(R0);

t1=tform2trvec(T16);
R1=tform2rotm(T16);

eu11=rotm2eul(R1);
axis_angle1=rotm2axang(R1);

t2=tform2trvec(T26);
R2=tform2rotm(T26);

eul2=rotm2eul(R2);
axis_angle2=rotm2axang(R2)

t3=tform2trvec(T36)
R3=tform2rotm(T36)

eul3=rotm2eul(R3);
axis_angle3=rotm2axang(R3)

t4=tform2trvec(T46)
R4=tform2rotm(T46)

eul4=rotm2eul(R4);
axis_angle4=rotm2axang(R4)

t5=tform2trvec(T56)
R5=tform2rotm(T56)

eul5=rotm2eul(R5);
axis_angle5=rotm2axang(R5)


