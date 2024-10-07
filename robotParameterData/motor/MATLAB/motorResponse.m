clear;

device = serialport("/dev/cu.usbmodem145457901", 500000);
data = zeros(1,500);

% if device.BytesAvailable
%     while true
%         msg = fscanf(device);
%         msg = str2num(msg);
%         if msg == "Setup Finished"
%             break;
%         end
%     end
% end

write(device, 'a', "char");
    

timer = tic;

count = 1;
while toc(timer) <= 10
    if device.BytesAvailable
        speed = strtrim(fscanf(device));
        speed = str2num(speed);
        data(count) = speed;
        count = count + 1;
%         pause(0.001);
    end
end

figure
plot(1:length(data), data);
xlabel('time (ms)')
ylabel('wheel speed (rad/s)')

clear device;
