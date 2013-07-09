function plotAuroraVolume()
%% AURORA volume
xmax = 210;
ymax = 300;
stepx1 = 80;
stepx2 = 190;
stepy1 = 90;
stepy2 = 270;
height1 = -120;
height2 = -561;
height3 = -519;
height4 = -556;
height5 = -533;

auroraVolumeVertices = [
    -xmax, -ymax,height1; %1%lower plane
    -xmax, ymax, height1;
    xmax, -ymax, height1;
    xmax, ymax, height1;
    
    0,-ymax, height1; %5 points on axis
    0,ymax,height1;
    -xmax,0,height1;
    xmax,0,height1;
    
    -stepx1, -stepy2, height1; %9
    stepx1, -stepy2, height1;
    -stepx1, stepy2, height1;
    stepx1, stepy2, height1;
    
    -stepx2, stepy1, height1; %13
    stepx2, stepy1, height1;
    -stepx2, -stepy1, height1;
    stepx2, -stepy1, height1;
    
    0,-ymax, height3; %17 points on axis
    0,ymax,height3;
    -xmax,0,height2;
    xmax,0,height2;
    
    -stepx1, -stepy2, height5; %21
    stepx1, -stepy2, height5;
    -stepx1, stepy2, height5;
    stepx1, stepy2, height5;
    
    -stepx2, stepy1, height4; %25
    stepx2, stepy1, height4;
    -stepx2, -stepy1, height4;
    stepx2, -stepy1, height4;
    
    0,0,height1; %29
    0,0,height2; 
    
    -xmax, -ymax,height2; % %upper plane
    -xmax, ymax, height2;
    xmax, -ymax, height2;
    xmax, ymax, height2;

    ];
auroraVolumeFaces = [    
    8, 14,12, 6,11,13, 7;%lower face
    7, 15, 9, 5,10,16, 8; 
    
    20,26,24,18,23,25,19;%upper face
    19,27,21,17,22,28,20;
    
    %1,2,6,5;
    %3,4,8,7;
    %5,6,8,7;
    ];

auroraVolumeFacesSides = [
    8,14,26,20;
    14,12,24,26;
    12,6,18,24;
    6,11,23,18;
    11,13,25,23;
    13,7,19,25;
    7,15,27,19;
    15,9,21,27;
    9,5,17,21;
    5,10,22,17;
    10,16,28,22;
    16,8,20,28;
 
];
patch('Vertices', auroraVolumeVertices,'Faces',auroraVolumeFaces, 'FaceColor','r', 'FaceAlpha',.2, 'EdgeColor', 'none' );
patch('Vertices', auroraVolumeVertices,'Faces',auroraVolumeFacesSides, 'FaceColor','r', 'FaceAlpha',.2, 'EdgeColor', 'none' );
end

