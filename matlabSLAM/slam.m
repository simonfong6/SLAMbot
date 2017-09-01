WHITE = 8 %Unexlpored
RED = 2; %Car part that touches table
BLACK = 1; %Car body
BLUE = 3; %Ultra sonic sensors
GREEN = 4; %Terrain
YELLOW = 7; %No Object
CYAN = 5; %Ocean/Edge
MAGENTA = 6; %Object

A(1:100,1:100) = WHITE;
x = 50;
y = 50;

A = initMap(x,y,A);
showMap(x,y,A);

for i = 1:40
    pause(1);
    showMap(x+i,y,A);
end



function showMap(x,y,A)
    WHITE = 8 %Unexlpored
    RED = 2; %Car part that touches table
    BLACK = 1; %Car body
    BLUE = 3; %Ultra sonic sensors
    GREEN = 4; %Terrain
    YELLOW = 7; %No Object
    CYAN = 5; %Ocean/Edge
    MAGENTA = 6; %Object
    map = [0,0,0
           1,0,0
           0,0,1
           0,1,0
           0,1,1
           1,0,1
           1,1,0
           1,1,1];
    colormap(map);
    
    %Center Block
	A(x:x+1,y:y+1) = RED;
	
	%Left Wheel
	A(x-3:x-1,y+3) = RED;
	
	%Right Wheel
	A(x-3:x-1,y-2) = RED;
	
	
	%Body
	A(x-3:x-1,y-1:y+2) = BLACK;
    A(x:x+1,y-1) = BLACK;
    A(x:x+1,y+2) = BLACK;
    A(x+2,y-1:y+2) = BLACK;
    
    %Ultrasonic Sensors
    A(x+1:x+2,y+3) = BLUE;
    A(x+1:x+2,y-2) = BLUE;
    A(x+3,y:y+1) = BLUE;
    
    
    
    % get the handle to the image
    hImg = image(flipud(transpose(A)));
    % get the handle to the parent axes
    hAxs = get(hImg,'Parent');
    % reverse the order of the y-axis tick labels
    yAxisTickLabels = get(hAxs, 'YTickLabel');
    set(hAxs,'YTickLabel',flipud(yAxisTickLabels));
   
  
end

function matrix = initMap(x,y,A);
    GREEN = 4;  %Terrain
    %Center Block
	A(x:x+1,y:y+1) = GREEN;
	
	%Left Wheel
	A(x-3:x-1,y+3) = GREEN;
	
	%Right Wheel
	A(x-3:x-1,y-2) = GREEN;
	
	
	%Body
	A(x-3:x-1,y-1:y+2) = GREEN;
    A(x:x+1,y-1) = GREEN;
    A(x:x+1,y+2) = GREEN;
    A(x+2,y-1:y+2) = GREEN;
    
    matrix = A;
end