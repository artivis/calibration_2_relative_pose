function Func_CameraDisplay(rot_cam,Translation_Camera,CameraSize,CameraEdge,CameraColor)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Created by Sang Ly and Cedric Demonceaux
%%%%% Last modified 2009-11-10

%%%%% Rotation (3x3 matrix) and Translation (3-vector)
%%%%% of the current camera with respect to original coordinate system
%%%%% X_w = R*X_c + T
%%%%% CameraSize: half of the camera body length
%%%%% CameraEdge: line width of the camera body
%%%%% CameraColor: 3-vector RGB color of the camera
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r = CameraSize;
t = Translation_Camera;
R = rot_cam;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% Corners of the camera in the camera coordinate system %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
X1 = [r,-r,-2*r]';
X2 = [r,r,-2*r]';
X3 = [-r,-r,-2*r]';
X4 = [-r,r,-2*r]';
 
X5 = [r,-r,2*r]';
X6 = [r,r,2*r]';
X7 = [-r,-r,2*r]';
X8 = [-r,r,2*r]';
 
X9 = [1.5*r,-1.5*r,3*r]';
X10 = [1.5*r,1.5*r,3*r]';
X11 = [-1.5*r,-1.5*r,3*r]';
X12 = [-1.5*r,1.5*r,3*r]';

%%%%% Points for the camera axes
X13 = [0 0 0]';
X14 = [0 0 6*r]';
X15 = [0 6*r 0]';
X16 = [6*r 0 0]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% Corners of the camera in the world coordinate system %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


X1 = R*X1+t;
X2 = R*X2+t;
X3 = R*X3+t;
X4 = R*X4+t;
X5 = R*X5+t;
X6 = R*X6+t;
X7 = R*X7+t;
X8 = R*X8+t;
X9 = R*X9+t;
X10 = R*X10+t;
X11 = R*X11+t;
X12 = R*X12+t;

X13 = R*X13+t;
X14 = R*X14+t;
X15 = R*X15+t;
X16 = R*X16+t;

line([X1(1) ;X2(1)],[X1(2) ;X2(2)],[X1(3);X2(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X1(1) ;X3(1)],[X1(2) ;X3(2)],[X1(3);X3(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X4(1) ;X3(1)],[X4(2) ;X3(2)],[X4(3);X3(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X2(1) ;X4(1)],[X2(2) ;X4(2)],[X2(3);X4(3)],'LineWidth',CameraEdge,'color',CameraColor);

line([X5(1) ;X6(1)],[X5(2) ;X6(2)],[X5(3);X6(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X5(1) ;X7(1)],[X5(2) ;X7(2)],[X5(3);X7(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X8(1) ;X7(1)],[X8(2) ;X7(2)],[X8(3);X7(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X6(1) ;X8(1)],[X6(2) ;X8(2)],[X6(3);X8(3)],'LineWidth',CameraEdge,'color',CameraColor);

line([X1(1) ;X5(1)],[X1(2) ;X5(2)],[X1(3);X5(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X2(1) ;X6(1)],[X2(2) ;X6(2)],[X2(3);X6(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X3(1) ;X7(1)],[X3(2) ;X7(2)],[X3(3);X7(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X4(1) ;X8(1)],[X4(2) ;X8(2)],[X4(3);X8(3)],'LineWidth',CameraEdge,'color',CameraColor);

line([X9(1) ;X10(1)],[X9(2) ;X10(2)],[X9(3);X10(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X9(1) ;X11(1)],[X9(2) ;X11(2)],[X9(3);X11(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X12(1) ;X11(1)],[X12(2) ;X11(2)],[X12(3);X11(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X10(1) ;X12(1)],[X10(2) ;X12(2)],[X10(3);X12(3)],'LineWidth',CameraEdge,'color',CameraColor);

line([X5(1) ;X9(1)],[X5(2) ;X9(2)],[X5(3);X9(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X6(1) ;X10(1)],[X6(2) ;X10(2)],[X6(3);X10(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X7(1) ;X11(1)],[X7(2) ;X11(2)],[X7(3);X11(3)],'LineWidth',CameraEdge,'color',CameraColor);
line([X8(1) ;X12(1)],[X8(2) ;X12(2)],[X8(3);X12(3)],'LineWidth',CameraEdge,'color',CameraColor);

% 'b' Z axis
line([X13(1) ;X14(1)],[X13(2) ;X14(2)],[X13(3);X14(3)],'LineWidth',2*CameraEdge,'color','b');
% 'g' Y axis
line([X13(1) ;X15(1)],[X13(2) ;X15(2)],[X13(3);X15(3)],'LineWidth',2*CameraEdge,'color','g');
% 'r' X axis
line([X13(1) ;X16(1)],[X13(2) ;X16(2)],[X13(3);X16(3)],'LineWidth',2*CameraEdge,'color','r');

axis equal;