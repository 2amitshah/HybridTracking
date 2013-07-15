function plotPolaris(plothandle, H_OT_to_EMT, Y)

if ~exist('plothandle','var')
    plothandle = figure;
end

if ~exist('H_OT_to_EMT', 'var')
    load(which('H_OT_to_EMT.mat'));
end

% Load transformation matrix and defines rotation & translation
if ~exist('Y', 'var')
    Y = polaris_to_aurora([],H_OT_to_EMT);
end

figure(plothandle)
% Polaris base
polarisBaseVertices = [
    75, -35, 35; % Upper part
    
   -75, -35, 35; % Right 'circle'
   -79, -35, 26;
   -82, -35, 17;
   -84, -35, 8;
   -85, -35, 0;
   -84, -35, -8;
   -82, -35, -17;
   -79, -35, -26;
   -75, -35, -35;
   
    75, -35, -35; % Right 'circle'
    79, -35, -26;
    82, -35, -17;
    84, -35, -8;
    85, -35, 0;
    84, -35, 8;
    82, -35, 17;
    79, -35, 26;
   
    75, 35, 35; % Lower part part
    
   -75, 35, 35; % Right 'circle'
   -79, 35, 26;
   -82, 35, 17;
   -84, 35, 8;
   -85, 35, 0;
   -84, 35, -8;
   -82, 35, -17;
   -79, 35, -26;
   -75, 35, -35;
   
    75, 35, -35; % Right 'circle'
    79, 35, -26;
    82, 35, -17;
    84, 35, -8;
    85, 35, 0;
    84, 35, 8;
    82, 35, 17;
    79, 35, 26; ];

%rotation
distanceEMT_to_OT = Y(1:3,4)';
rotEMT_to_OT = Y(1:3,1:3);
rotationX90 = [1 0 0; 0 0 -1; 0 1 0];
rotationZ90 = [0 -1 0; 1 0 0; 0 0 1];
polarisBaseRotated = (rotationZ90*rotationX90*polarisBaseVertices');
polarisBaseVerticesRotated = (rotEMT_to_OT*polarisBaseRotated)';

%translation
polarisBaseVerticesTranslated = polarisBaseVerticesRotated + repmat(distanceEMT_to_OT,size(polarisBaseVertices,1),1);

polarisBaseFaces = [
    1 2 10 11; % Lower part
    2 3 9 10;
    3 4 8 9;
    4 5 7 8;
    5 6 6 7;
    
    18 1 11 12;
    17 18 12 13;
    16 17 13 14;
    15 16 14 15;
    
    19 20 28 29; % Upper part
    20 21 27 28;
    21 22 26 27;
    22 23 25 26;
    23 24 24 25;
    
    36 19 29 30;
    35 36 30 31;
    34 35 31 32;
    33 34 32 33;
    
    1 2 20 19;
    2 3 21 20;
    3 4 22 21;
    4 5 23 22;
    5 6 24 23;
    6 7 25 24;
    7 8 26 25;
    8 9 27 26;
    9 10 28 27;
    10 11 29 28;
    11 12 30 29;
    12 13 31 30;
    13 14 32 31;
    14 15 33 32;
    15 16 34 33;
    16 17 35 34;
    17 18 36 35;
    18 1 19 36;
];

grey = [0.7,0.7,0.7];
patch('Vertices', polarisBaseVerticesTranslated,'Faces',polarisBaseFaces,'FaceColor',grey);


% Center of cameras (rotated as well!)
centerCylinder1 = distanceEMT_to_OT + (rotEMT_to_OT*rotationZ90*rotationX90*[-45 -45 0]')';
centerCylinder2 = distanceEMT_to_OT + (rotEMT_to_OT*rotationZ90*rotationX90*[45 -45 0]')';


% Generate a cilynder of radius 25 and 20 points (camera)
[Xs, Ys, Zs]=cylinder([25],20);

% Height = 10 pix
Zs = 10.*Zs;
newXC = Ys; newYC = Zs; newZC = Xs;

% Rotation of cylinder to be aligned with polaris base
cylinderMatrix1 = [newXC(1,:); newYC(1,:); newZC(1,:)];
cylinderMatrix2 = [newXC(2,:); newYC(2,:); newZC(2,:)];
cylinderRotatedInitial1 = rotationZ90*rotationX90*cylinderMatrix1;
cylinderRotated1 = (rotEMT_to_OT*cylinderRotatedInitial1);
cylinderRotatedInitial2 = rotationZ90*rotationX90*cylinderMatrix2;
cylinderRotated2 = (rotEMT_to_OT*cylinderRotatedInitial2);
newXC = [cylinderRotated1(1,:); cylinderRotated2(1,:)];
newYC = [cylinderRotated1(2,:); cylinderRotated2(2,:)];
newZC = [cylinderRotated1(3,:); cylinderRotated2(3,:)];

% Creation of two cylinders and placement
newXC1 = newXC+centerCylinder1(1); newYC1 = newYC+centerCylinder1(2); newZC1 = newZC+centerCylinder1(3);
newXC2 = newXC+centerCylinder2(1); newYC2 = newYC+centerCylinder2(2); newZC2 = newZC+centerCylinder2(3);

% Plot of the two cylinders
camera1 = surf(newXC1, newYC1, newZC1);
camera2 = surf(newXC2, newYC2, newZC2);
set(camera1,'FaceColor',[0 0 1],'FaceAlpha',0.5);
set(camera2,'FaceColor',[0 0 1],'FaceAlpha',0.5);
fill3(newXC1(1,:),newYC1(1,:),newZC1(1,:),'y');
fill3(newXC2(1,:),newYC2(1,:),newZC2(1,:),'y');
fill3(newXC1(2,:),newYC1(2,:),newZC1(2,:),'y');
fill3(newXC2(2,:),newYC2(2,:),newZC2(2,:),'y');

% Tripod polaris. Divided in four sections (upper part, and three legs)
polarisTripodVertices = [-20 -20 35;
                        -20 20 35;
                        20 20 35;
                        20 -20 35;
                        -20 -20 -distanceEMT_to_OT(3);
                        -20 20 -distanceEMT_to_OT(3);
                        20 20 -distanceEMT_to_OT(3);
                        20 -20 -distanceEMT_to_OT(3)] ;
polarisTripod1LVertices = [polarisTripodVertices(5:8,:);
                        -200 60 -2*distanceEMT_to_OT(3);
                        -200 100 -2*distanceEMT_to_OT(3);
                        -160 100 -2*distanceEMT_to_OT(3);
                        -160 60 -2*distanceEMT_to_OT(3)];
                        
polarisTripod2LVertices = [polarisTripodVertices(5:8,:);
                        160 60 -2*distanceEMT_to_OT(3);
                        160 100 -2*distanceEMT_to_OT(3);
                        200 100 -2*distanceEMT_to_OT(3);
                        200 60 -2*distanceEMT_to_OT(3)];
                    
polarisTripod3LVertices = [polarisTripodVertices(5:8,:);
                        -20 -200 -2*distanceEMT_to_OT(3);
                        -20 -160 -2*distanceEMT_to_OT(3);
                        20 -160 -2*distanceEMT_to_OT(3);
                        20 -200 -2*distanceEMT_to_OT(3)];
                    
                    
polarisTripodVertices = polarisTripodVertices + repmat(distanceEMT_to_OT,size(polarisTripodVertices,1),1);
polarisTripod1LVertices = polarisTripod1LVertices + repmat(distanceEMT_to_OT,size(polarisTripod1LVertices,1),1);
polarisTripod2LVertices = polarisTripod2LVertices + repmat(distanceEMT_to_OT,size(polarisTripod2LVertices,1),1);
polarisTripod3LVertices = polarisTripod3LVertices + repmat(distanceEMT_to_OT,size(polarisTripod3LVertices,1),1);
                    
polarisTripodFaces = [1 2 3 4; ...
                        2 6 7 3; ...
                        4 3 7 8; ...
                        1 5 8 4; ...
                        1 2 6 5; ...
                        5 6 7 8];
                    
                    
patch('Vertices', polarisTripodVertices,'Faces',polarisTripodFaces,'FaceColor',grey);
patch('Vertices', polarisTripod1LVertices,'Faces',polarisTripodFaces,'FaceColor',grey);
patch('Vertices', polarisTripod2LVertices,'Faces',polarisTripodFaces,'FaceColor',grey);
patch('Vertices', polarisTripod3LVertices,'Faces',polarisTripodFaces,'FaceColor',grey);

end