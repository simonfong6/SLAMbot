%Map variables to colors.
global UNKNOWN;
global CARCRITICAL;
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


main()



function main();
    %Retrieve global variables
    global UNKNOWN;
    global CARCRITICAL;
    global CARBODY;
    global SONIC;
    global TERRAIN;
    global NOOBJECT;
    global EDGE;
    global OBJECT;
    
    %Maps numbers to colors
    %When 1 it is interpreted as 0,0,0 (Black)
    %When 2 it is interpreted as 1,0,0 (Red)
    %And so on
    mapColor = [0,0,0
           1,0,0
           0,0,1
           0,1,0
           0,1,1
           1,0,1
           1,1,0
           1,1,1];
    %Sets the color on
    %Used when image function interprets integers
    %as colors.
    colormap(mapColor);
    
    %Creates a map 100 x 100 map and sets all values
    %to UNKNOWK value.
    map(1:100,1:100) = UNKNOWN;
    
    %Sets starting coordinate of the car as 50,50
    %or center of image.
    x = 50;
    y = 50;
    
    %Sets blocks under car body as terrain.
    map = initMap(x,y,map);
    
    %Shows the map with car using image.
    showMap(x,y,map);
    
    %Move 40 blocks in positive X
    for i = 1:40
        %Wait one second.
        pause(1);
        %Update map according to what uSonic sensors
        %see.
        map = updateMap(x+i,y+i,map);
        %Display the updated map using new position
        %of car.
        showMap(x+i,y+i,map);
    end
end


%Takes in position of car and current map
%and displays the map with the car on top.
%@param x X-coordinate of the car.
%@param y Y-coordinate of the car.
%@param map Current map of the world contains
%no information about the car.
function showMap(x,y,map);
    global UNKNOWN;
    global CARCRITICAL;
    global CARBODY;
    global SONIC;
    global TERRAIN;
    global NOOBJECT;
    global EDGE;
    global OBJECT;
    
    %Center Block
	map(x:x+1,y:y+1) = CARCRITICAL;
	
	%Left Wheel
	map(x-3:x-1,y+3) = CARCRITICAL;
	
	%Right Wheel
	map(x-3:x-1,y-2) = CARCRITICAL;
	
	
	%Body
	map(x-3:x-1,y-1:y+2) = CARBODY;
    map(x:x+1,y-1) = CARBODY;
    map(x:x+1,y+2) = CARBODY;
    map(x+2,y-1:y+2) = CARBODY;
    
    %Ultrasonic Sensors
    map(x+1:x+2,y+3) = SONIC;
    map(x+1:x+2,y-2) = SONIC;
    map(x+3,y:y+1) = SONIC;
    
    
    
    % get the handle to the image
    hImg = image(flipud(transpose(map)));
    % get the handle to the parent axes
    hAxs = get(hImg,'Parent');
    % reverse the order of the y-axis tick labels
    yAxisTickLabels = get(hAxs, 'YTickLabel');
    set(hAxs,'YTickLabel',flipud(yAxisTickLabels));
   
  
end

%Sets the value of the blocks under the car body
%to be terrain.
function updatedMap = initMap(x,y,map);
    global UNKNOWN;
    global CARCRITICAL;
    global CARBODY;
    global SONIC;
    global TERRAIN;
    global NOOBJECT;
    global EDGE;
    global OBJECT;
    %Center Block
	map(x:x+1,y:y+1) = TERRAIN;
	
	%Left Wheel
	map(x-3:x-1,y+3) = TERRAIN;
	
	%Right Wheel
	map(x-3:x-1,y-2) = TERRAIN;
	
	
	%Body
	map(x-3:x-1,y-1:y+2) = TERRAIN;
    map(x:x+1,y-1) = TERRAIN;
    map(x:x+1,y+2) = TERRAIN;
    map(x+2,y-1:y+2) = TERRAIN;
    
    updatedMap = map;
end

%Change update the values of the blocks that the 
%uSonic sensors see.
function updatedMap = updateMap(x,y,map);
    global UNKNOWN;
    global CARCRITICAL;
    global CARBODY;
    global SONIC;
    global TERRAIN;
    global NOOBJECT;
    global EDGE;
    global OBJECT;
    POSX = 0;   %Positive X
    NEGX = 1;   %Negative X
    POSY = 2;   %Positive Y
    NEGY = 3;   %Negative Y
    
    
    if 1
        left = TERRAIN;
    else
        left = EDGE;
    end
    if 1
        right = TERRAIN;
    else
        right = EDGE;
    end
    if 1
        frontDown = TERRAIN;
    else
        frontDown = EDGE;
    end
    if 1
        n = 10;
        frontObject = NOOBJECT;
    end
    
    dir = POSX;
    %If we are facing in the postive X.
    if dir == POSX
        map(x+1:x+2,y+3) = left;
        map(x+1:x+2,y-2) = right;
        map(x+3,y:y+1) = frontDown;
        map(x+4:x+n,y:y+1) = frontObject;
    elseif dir == NEGX
        map(x-2:x-1,y+3) = left;
        map(x+1:x+2,y-2) = right;
        map(x+3,y:y+1) = frontDown;
        map(x+4:x+n,y:y+1) = frontObject;
    elseif dir == POSY
    elseif dir == NEGY
    end
    
    
    updatedMap = map;
end