function plotAuroraTable (plothandle)

if ~exist('plothandle','var') 
    plothandle = figure;
end

%% AURORA vertices and faces
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
                   
                   250,  330, 34; % Upper face
                   
                   250, -330, 34; % First corner
                   236, -342, 34;
                   222, -352, 34;
                   208, -358, 34;
                   194, -366, 34;
                   180, -370, 34;
                   
                  -180, -370, 34; % Second corner
                  -194, -366, 34;
                  -208, -358, 34;
                  -222, -352, 34;
                  -236, -342, 34;
                  -250, -330, 34;
                  
                  -250,  330, 34; % Third corner
                  -236,  342, 34;
                  -222,  352, 34;
                  -208,  358, 34;
                  -194,  366, 34;
                  -180,  370, 34; 
                   
                   180,  370, 34; % Four corner
                   194,  366, 34;
                   208,  358, 34;
                   222,  352, 34;
                   236,  342, 34;     
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
%%

% Plot the planar field generator in grey
figure(plothandle)
grey = [0.7,0.7,0.7];
hold on
patch('Vertices',auroraVertices,'Faces',auroraFaces,'FaceColor',grey)
hold off

% Plot a cube and obtain the object handles
sizeCube = 60;
centerCube = [185,265,-sizeCube];
cubeVertices = repmat(sizeCube,8,3).*...
               [0 0 0; 0 1 0; 1 1 0; 1 0 0;...
                0 0 1; 0 1 1; 1 1 1; 1 0 1] + repmat(centerCube,8,1);
    
cubeFaces = [1 2 3 4; 2 6 7 3; 4 3 7 8;...
             1 5 8 4; 1 2 6 5; 5 6 7 8];

hold on
cubeObj=patch('Faces', cubeFaces,'Vertices',cubeVertices);
hold off
set(cubeObj,'edgecolor','b','FaceLighting', 'gouraud','facecolor','w');

axis vis3d image

end