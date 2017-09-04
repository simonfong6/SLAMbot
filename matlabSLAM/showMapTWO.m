
column = 50;
row = 50; 
A(1:100,1:100) = 8; 

update = showMap(column,row,A);
    for i = 1:40
        %Wait one second.
        pause(1);
        %Update map according to what uSonic sensors
        %see.
        %map = updateMap(x+i,y+i,map);
        %Display the updated map using new position
        %of car.
        update = showMap(column+i,row,A);
    end

function update = showMap(column, row, map)
%Takes in position of car and current map
%and displays the mapclea with the car on top.
%@param x X-coordinate of the car.
%@param y Y-coordinate of the car.
%@param map Current map of the world contains
%no information about the car.

    global UNKNOWN;
    global CARCRITICAL; % Any part of the car that physically touches the table
    global CARBODY;
    global SONIC;
    global TERRAIN;
    global NOOBJECT;
    global EDGE;
    global OBJECT;
    
    UNKNOWN = 8;        %White
    CARCRITICAL = 2;    %Red
    CARBODY = 1;        %Black
    SONIC = 3;          %Blue
    TERRAIN = 4;        %Green
    NOOBJECT = 7;       %Yellow
    EDGE = 5;           %Cyan
    OBJECT = 6;         %Magenta
    
    mapColor = [0,0,0
           1,0,0
           0,0,1
           0,1,0
           0,1,1
           1,0,1
           1,1,0
           1,1,1];
       
    colormap(mapColor);
    
    
    
    %Center Block
	map(row-1:row, column:column+1) = CARCRITICAL;
	
	%Left Wheel
	map(row-3,column-3:column-1) = CARCRITICAL;
	
	%Right Wheel
	map(row+2,column-3:column-1) = CARCRITICAL;
	
	%Body
	map(row-2:row+1,column-3:column-1) = CARBODY;
    map(row+1,column:column+1) = CARBODY;
    map(row-2,column:column+1) = CARBODY;
    map(row-2:row+1,column+2) = CARBODY;
    
    %Ultrasonic Sensors
    map(row-3,column+1:column+2) = SONIC;
    map(row+2,column+1:column+2) = SONIC;
    map(row-1:row,column+3) = SONIC;
    
    image(map);
    update = map;
end