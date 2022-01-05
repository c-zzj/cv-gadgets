fname = 'Schefflera';
video = VideoWriter(strcat(fname,'.mp4'),'MPEG-4');
video.FrameRate = 3;
open(video)

for i=7:13
    demo_optical_flow(fname,i,i+1);
    pause(.2);
    frame = getframe(gcf);
    writeVideo(video, frame);
    fprintf('Frame: %d\n',i)
end
close(video)