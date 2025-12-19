function PlotRefFrames()

figure
ax=axes('DataAspectRatio',[1 1 1],'View',[37.5 30]);
hold(ax,'on')
xlabel('X'),ylabel('Y'),zlabel('Z')
axis padded
grid on
title('Exercise 6 - Frame Tree visualization','FontSize',20)
view(37.5,30)

%% global frame
Tg=eye(4);
plotFrame(Tg,'{Global}',ax);

%% frame 0 wrt global
r0=1/100*[431.5;0;0];
R0=eye(3);
T0=homog(R0,r0);
plotFrame(T0,'0',ax);

%% frame 1 wrt frame 0
r1_0=1/100*[0;0;175];
R1_0=eye(3);
T1=T0*homog(R1_0,r1_0);
plotFrame(T1,'1',ax);

%% frame 2 wrt frame 1
r2_1=1/100*[0;0;98];
R2_1=[-1 0 0;0 0 1;0 1 0];
T2=T1*homog(R2_1,r2_1);
plotFrame(T2,'2',ax);

%% frame 3 wrt frame 2
r3_2=1/100*[105;0;0];
R3_2=[0 0 1;0 1 0;-1 0 0];
T3=T2*homog(R3_2,r3_2);
plotFrame(T3,'3',ax);

%% frame 4 wrt frame 3
r4_3=1/100*[0;145.5;326.5];
R4_3=[0 0 -1;0 -1 0;-1 0 0];
T4=T3*homog(R4_3,r4_3);
plotFrame(T4,'4',ax);

%% frame 5 wrt frame 4
r5_4=1/100*[35;0;0];
R5_4=[0 0 1;-1 0 0;0 -1 0];
T5=T4*homog(R5_4,r5_4);
plotFrame(T5,'5',ax);

%% frame 6 wrt frame 5
r6_5=1/100*[0;0;385];
R6_5=[0 1 0;0 0 1;1 0 0];
T6=T5*homog(R6_5,r6_5);
plotFrame(T6,'6',ax);

%% frame 7 wrt frame 6
r7_6=1/100*[153;0;0];
R7_6=[0 0 1;1 0 0;0 1 0];
T7=T6*homog(R7_6,r7_6);
plotFrame(T7,'7',ax);

end

function T=homog(R,r)
T=[R r;0 0 0 1];
end

function plotFrame(T,name,ax)
o=T(1:3,4);
x=o+T(1:3,1);
y=o+T(1:3,2);
z=o+T(1:3,3);

plot3(ax,[o(1) x(1)],[o(2) x(2)],[o(3) x(3)],'r','LineWidth',1.5)
plot3(ax,[o(1) y(1)],[o(2) y(2)],[o(3) y(3)],'g','LineWidth',1.5)
plot3(ax,[o(1) z(1)],[o(2) z(2)],[o(3) z(3)],'b','LineWidth',1.5)

text(x(1),x(2),x(3),['X_' name],'FontAngle','italic')
text(y(1),y(2),y(3),['Y_' name],'FontAngle','italic')
text(z(1),z(2),z(3),['Z_' name],'FontAngle','italic')
end
