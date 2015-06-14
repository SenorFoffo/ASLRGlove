clear all;
close all;
format long

t = tcpip('0.0.0.0', 1000, 'NetworkRole', 'server');
set(t, 'InputBufferSize', 512);
%set(t,'Timeout',.1);
set(t, 'Terminator', 'LF');
fopen(t);
figure(1)
%hold on
%axis manual
%axis([-100 100 -100 100 -100 100])
%while(true)
%while(true)
old_vector1=[0 0 0 0];
old_vector2=[0 0 0 0];
a=1;
b=2;
c=3;
while(true)
        data=fgetl(t);
        vector=str2num(data);
        if(length(vector)>5)
          %if(vector(1)==1)
           % old_vector1=vector;
          %else
           % old_vector2=vector;
          %end
          
          %data=fgetl(t);
          %vector=str2num(data);
          %if(length(vector)>3)
           % if(vector(1)==1)
            %  old_vector1=vector;
            %else
             % old_vector2=vector;
            %end
          %end
            axis([-100 100 -100 100 -100 100])
            quiver3(0,0,0,vector(1), vector(2),vector(3))
            if (vector(1)>10000)
                a
                vector(1)
            end
            if (vector(2)>10000)
                b
                vector(2)
            end
            if (vector(3)>10000)
                c
                vector(3)
            end
            axis([-100 100 -100 100 -100 100])
            hold on
            quiver3(0,0,0,vector(4), vector(5),vector(6))
            hold off
            legend('sensor thumb', 'sensor pinky');
       end
       flushinput(t);
       pause(0.05);
%end
end
fclose(t);
