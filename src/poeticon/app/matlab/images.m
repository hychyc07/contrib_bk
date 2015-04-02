% Copyright: (C) 2010 RobotCub Consortium
% Authors: Vadim Tikhanoff
% CopyPolicy: Released under the terms of the LGPLv2.1 or later, see
% LGPL.TXT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initialize YARP
clear
LoadYarp;
import yarp.BufferedPortImageRgb
import yarp.BufferedPortBottle
import yarp.Port
import yarp.Bottle
import yarp.Time
import yarp.ImageRgb
import yarp.Image
import yarp.PixelRgb
done=0;
speech=Bottle;
rep=Bottle;
%load poeticon_model;
load model-4-4-2011.mat;
%creating ports
port=BufferedPortBottle;%port for reading qquit signal
portImage=BufferedPortImageRgb; %port for reading image
portFilters=Port;%BufferedPortImageRgb; %port for sending image
portObjects=Port;
%first close the port just in case
port.close;
portImage.close;
portFilters.close;
portObjects.close;
disp('opening ports...');
port.open('/matlab/read');
disp('opened port /matlab/read');
pause(0.5);
portImage.open('/matlab/img:i');
disp('opened port /matlab/img:i');
pause(0.5);
portFilters.open('/matlab/img:o');
disp('opened port /matlab/img:o');
pause(0.5);
portObjects.open('/matlab/mask:o');
disp('opened port /matlab/mask:o');

disp('initializing...');
pause(2);
disp('done.');

while(~done)
    
     speech = port.read( false );
     if (sum(size(speech)) ~= 0)
         disp('received command: ');
         disp(speech);
         %checking for quit signal
         if (strcmp(speech.toString, 'quit'))
             done=1;
         else
             disp('getting a yarp image..');
             yarpImage=portImage.read(true);
             disp('got it..');
             h=yarpImage.height;
             w=yarpImage.width;
             tool=YarpImageHelper(h, w);
             tic
             IN = tool.getRawImg(yarpImage);
             TEST = reshape(IN, [h w 3]);
             COLOR=uint8(zeros(h, w, 3));
             r = cast(TEST(:,:,1),'uint8');
             g = cast(TEST(:,:,2),'uint8');
             b = cast(TEST(:,:,3),'uint8');
             COLOR(:,:,1)= r;
             COLOR(:,:,2)= g;
             COLOR(:,:,3)= b;
             
             %imtool(COLOR);
             disp('running label filters..');
             %im = double(imread('left/test2.jpg'))/255;%%using test image
             im = double(COLOR)/255;
             label = miximshow(1,1+1000,classify_image_scaled(im,model));
             rValMin=0.0; gValMin=0.0; bValMin=0.0;
             rValMax=0.0; gValMax=0.0; bValMax=0.0;
             %imshow([im label]);
             if (strcmp(speech.get(0), 'plate')) || (strcmp(speech.get(1), 'plate'))|| (strcmp(speech.get(2), 'plate'))
                disp('filtering plates') %yellowish new
                rValMin=0.85; gValMin=0.85; bValMin=0.48;
                rValMax=1.0; gValMax=1.0; bValMax=0.55;
             elseif (strcmp(speech.get(0), 'veg')) || (strcmp(speech.get(1), 'veg'))|| (strcmp(speech.get(2), 'veg'))
                disp('filtering vegetables') %yellow light new
                rValMin=0.85; gValMin=0.85; bValMin=0.60;
                rValMax=1.00; gValMax=1.00; bValMax=0.95;
             elseif (strcmp(speech.get(0), 'fork')) || (strcmp(speech.get(1), 'fork'))|| (strcmp(speech.get(2), 'fork'))
                disp('filtering fork')  %dark green new
                rValMin=0.01; gValMin=0.40; bValMin=0.01;
                rValMax=0.1; gValMax=0.60; bValMax=0.1;
             elseif (strcmp(speech.get(0), 'basket')) || (strcmp(speech.get(1), 'basket'))|| (strcmp(speech.get(2), 'basket'))
                disp('filtering baskets')%blue new
                rValMin=0.0001; gValMin=0.35; bValMin=0.80;
                rValMax=0.01; gValMax=0.55; bValMax=1.0;             
             elseif (strcmp(speech.get(0), 'cup')) || (strcmp(speech.get(1), 'cup'))|| (strcmp(speech.get(2), 'cup'))
                disp('filtering cups')%light green new
                rValMin=0.0001; gValMin=0.75; bValMin=0.05;
                rValMax=0.01; gValMax=0.90; bValMax=0.25;
             elseif (strcmp(speech.get(0), 'bowl')) || (strcmp(speech.get(1), 'bowl'))|| (strcmp(speech.get(2), 'bowl'))
                disp('filtering bowls')%light green new
                rValMin=0.001; gValMin=0.90; bValMin=0.40;
                rValMax=0.01; gValMax=1.0; bValMax=0.55;
             elseif (strcmp(speech.get(0), 'coke')) || (strcmp(speech.get(1), 'coke'))|| (strcmp(speech.get(2), 'coke'))
                disp('filtering coke')%dark purple new
                rValMin=0.40; gValMin=0.0001; bValMin=0.40;
                rValMax=0.55; gValMax=0.01; bValMax=0.55;
             elseif (strcmp(speech.get(0), 'octopus')) || (strcmp(speech.get(1), 'octopus'))|| (strcmp(speech.get(2), 'octopus'))
                disp('filtering octopus') %purple new
                rValMin=0.40; gValMin=0.0001; bValMin=0.85;
                rValMax=0.50; gValMax=0.01; bValMax=1.0;
             elseif (strcmp(speech.get(0), 'lego')) || (strcmp(speech.get(1), 'lego'))|| (strcmp(speech.get(2), 'lego'))
                disp('filtering lego') %brown new
                rValMin=0.40; gValMin=0.40; bValMin=0.0001;
                rValMax=0.50; gValMax=0.50; bValMax=0.01;
             else
                disp('nothing')
             end
              
             disp(rValMin);
             disp(gValMin);
             disp(bValMin);
             disp(rValMax);
             disp(gValMax);
             disp(bValMax);
             white=uint8(zeros(h, w, 3));
             sz=size(label);
             
             for i=1:sz(1)
                for j=1:sz(2)
                    if label(i,j,1)>rValMin && label(i,j,1)<rValMax && label(i,j,2)>gValMin && label(i,j,2)<gValMax && label(i,j,3)>bValMin && label(i,j,3)<bValMax
                       white(i,j,:)=1;
                    else
                        white(i,j,:)=0;
                    end
                end
             end
             [x,y] = ait_centroid(white);
             x
             y
             %figure;
             %imshow(white)
             hold on
             plot(x,y,'--rs','LineWidth',2)
             disp('sending filtered image to port');   
             img = yarp.ImageRgb();
             imgMask = yarp.ImageRgb();
             img.resize(320,240);
             imgMask.resize(320,240);
             img.zero();
             imgMask.zero();
             INMASK = reshape(white, [h*w*3 1]);
             OUT = reshape(label, [h*w*3 1]);
             tempImg = cast(OUT *255,'int16');
             tempMaskImg = cast(INMASK *255,'int16');
             img = tool.setRawImg(tempImg, h, w, 3);
             imgMask = tool.setRawImg(tempMaskImg, h, w, 3);
             portFilters.write(img);
             portObjects.write(imgMask);
             time = toc;
             fprintf('filters took %f seconds \n', time);
             %rep.clear;
             %rep.fromString('done');
             %port.write(rep);
         end
     end
    pause(0.01);
end
disp('Going to close the port');
portImage.close;
port.close;
portFilters.close;
portObjects.close;