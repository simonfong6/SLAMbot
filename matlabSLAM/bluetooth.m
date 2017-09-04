%% Interfacting w/ HC-06 Bluetooth Module 
% Pair with your bluetooth device before continuing 
clear all

% Lists available Bluetooth devices
instrhwinfo('Bluetooth');

% Create a bluetooth variable and open it at channel 1
device = Bluetooth('HC-06', 1);

% Pair with device
fopen(device);
disp('Matlab paired with Bluetooth device');

%% Write command to HC-06/Arduino
% while(1)
% 
%         data = fscanf(device, '%s', 100);
%         dataparts = strsplit(data,',');
%         if strcmp(dataparts{1},'EVENT')
%             distance1 = str2num(dataparts{2});
%             distance2 = str2num(dataparts{3});
%             distance3 = str2num(dataparts{4});
%             distance4 = str2num(dataparts{5});
%             theta = str2num(dataparts{6});
%             speed = str2num(dataparts{7});
%             disp(data);
%         else
%             disp(data);
%         end
%         
% %     if ~isempty(read)
% %         cmd = input('> ', 's');
% %         disp(cmd);
% % %         if cmd == 'a'
% % %             break; 
% % %         end
% %         fprintf(device, '%c', cmd);
% %     end
% 
% end


% final = str2double(data);

% Matrix manipulation here


%% Clear Bluetooth Object when Finished
fclose(device);
clear('device')
