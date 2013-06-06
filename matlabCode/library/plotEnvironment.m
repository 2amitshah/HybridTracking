function [ plothandle ] = plotEnvironment()
close all;
Y = polaris_to_aurora;

distanceEMT_to_OT = Y(1:3,end)';


%% AURORA
auroraVertices = [     
                   250,  330,   0; % Upper face
                   
                   250, -330,   0; % First corner
                   236, -342,   0;
                   222, -352,   0;
                   208, -358,   0;
                   194, -366,   0;
                   180, -370,   0;
                   
                  -180, -370,   0; % Second corner
                  -194, -366,   0;
                  -208, -358,   0;
                  -222, -352,   0;
                  -236, -342,   0;
                  -250, -330,   0;
                  
                  -250,  330,   0; % Third corner
                  -236,  342,   0;
                  -222,  352,   0;
                  -208,  358,   0;
                  -194,  366,   0;
                  -180,  370,   0; 
                   
                   180,  370,   0; % Four corner
                   194,  366,   0;
                   208,  358,   0;
                   222,  352,   0;
                   236,  342,   0;
                   
                   250,  330, -50; % Upper face
                   
                   250, -330, -50; % First corner
                   236, -342, -50;
                   222, -352, -50;
                   208, -358, -50;
                   194, -366, -50;
                   180, -370, -50;
                   
                  -180, -370, -50; % Second corner
                  -194, -366, -50;
                  -208, -358, -50;
                  -222, -352, -50;
                  -236, -342, -50;
                  -250, -330, -50;
                  
                  -250,  330, -50; % Third corner
                  -236,  342, -50;
                  -222,  352, -50;
                  -208,  358, -50;
                  -194,  366, -50;
                  -180,  370, -50; 
                   
                   180,  370, -50; % Four corner
                   194,  366, -50;
                   208,  358, -50;
                   222,  352, -50;
                   236,  342, -50;     
                   ];
              
auroraFaces = [ 1 2 13 14; % Lower face
    
                2 3 12 13;
                3 4 11 12;
                4 5 10 11;
                5 6 9 10;
                6 7 8 9;
                
                14 15 24 1;
                15 16 23 24;
                16 17 22 23;
                17 18 21 22;
                18 19 20 21;
                
                25 26 37 38; % Upper face
                26 27 36 37;
                27 28 35 36;
                28 29 34 35;
                29 30 33 34;
                30 31 32 33;
                
                38 39 48 25;
                39 40 47 48;
                40 41 46 47;
                41 42 45 46;
                42 43 44 45;
                
                 1  2 26 25; % Laterals
                 2  3 27 26;
                 3  4 28 27;
                 4  5 29 28;
                 5  6 30 29;
                 6  7 31 30;
                 7  8 32 31;
                 8  9 33 32;
                 9 10 34 33;
                10 11 35 34;
                11 12 36 35;
                12 13 37 36;
                13 14 38 37;
                14 15 39 38;
                15 16 40 39;
                16 17 41 40;
                17 18 42 41;
                18 19 43 42;
                19 20 44 43;
                20 21 45 44;
                21 22 46 45;
                22 23 47 46;
                23 24 48 47;
                24  1 25 48;
];

plothandle = figure
axis([-1000 1000 -1000 1000 -1000 1000])
set(gca,'zdir','reverse')
grey = [0.7,0.7,0.7];
patch('Vertices',auroraVertices,'Faces',auroraFaces,'FaceColor',grey)
hold on


%% POLARIS



xc = distanceEMT_to_OT(1); yc = distanceEMT_to_OT(2); zc = distanceEMT_to_OT(3);

% Polaris base
polarisBaseVertices = [
    75, -35,  35; % Upper part
    
   -75, -35,  35; % Right 'circle'
   -79, -35,  26;
   -82, -35,  17;
   -84, -35,   8;
   -85, -35,   0;
   -84, -35,  -8;
   -82, -35, -17;
   -79, -35, -26;
   -75, -35, -35;
   
    75, -35, -35; % Right 'circle'
    79, -35, -26;
    82, -35, -17;
    84, -35,  -8;
    85, -35,   0;
    84, -35,   8;
    82, -35,  17;
    79, -35,  26;
   
    75,  35,  35; % Lower part part
    
   -75,  35,  35; % Right 'circle'
   -79,  35,  26;
   -82,  35,  17;
   -84,  35,   8;
   -85,  35,   0;
   -84,  35,  -8;
   -82,  35, -17;
   -79,  35, -26;
   -75,  35, -35;
   
    75,  35, -35; % Right 'circle'
    79,  35, -26;
    82,  35, -17;
    84,  35,  -8;
    85,  35,   0;
    84,  35,   8;
    82,  35,  17;
    79,  35,  26; ];

polarisBaseVertices = polarisBaseVertices + repmat(distanceEMT_to_OT,size(polarisBaseVertices,1),1);

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

patch('Vertices', polarisBaseVertices,'Faces',polarisBaseFaces,'FaceColor',grey);
xlabel('x');
ylabel('y');
zlabel('z');

% Center of cameras
centerCylinder1 = distanceEMT_to_OT + [-45 35 0];
centerCylinder2 = distanceEMT_to_OT + [45 35 0];

% Generate a cilynder of radius 25 and 20 points
[Xs, Ys, Zs]=cylinder([25],20); 
% Height = 10 pix
Zs = 10.*Zs;

% Coordinates transformation!!!! Create 2 cilynders
newXC1 = Ys+centerCylinder1(1); newYC1 = Zs+centerCylinder1(2); newZC1 = Xs+centerCylinder1(3);
newXC2 = Ys+centerCylinder2(1); newYC2 = Zs+centerCylinder2(2); newZC2 = Xs+centerCylinder2(3);


camera1 = surf(newXC1, newYC1, newZC1); 
camera2 = surf(newXC2, newYC2, newZC2); 
set(camera1,'FaceColor',[0 0 1],'FaceAlpha',0.5);
set(camera2,'FaceColor',[0 0 1],'FaceAlpha',0.5);
fill3(newXC1(1,:),newYC1(1,:),newZC1(1,:),'y');
fill3(newXC2(1,:),newYC2(1,:),newZC2(1,:),'y');
fill3(newXC1(2,:),newYC1(2,:),newZC1(2,:),'y');
fill3(newXC2(2,:),newYC2(2,:),newZC2(2,:),'y');

% Tripod polaris
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
                    
polarisTripodFaces =    [1 2 3 4; ... 
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

