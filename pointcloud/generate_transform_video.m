function generate_transform_video(image_number, total_rotation, axis, total_translation, video_length, frame_rate, enhance)
% this function generates depth and rgb videos
unit_translation = [0; 0; 1];
pair = 'XZ';
if strcmp(axis,'y')
    unit_translation = [1; 0; 0];
    pair = 'YX';
elseif strcmp(axis, 'z')
    unit_translation = [0; 1; 0];
    pair = 'ZY';
end
num_frames = video_length * frame_rate;
omega = total_rotation / num_frames;
unit_translation = unit_translation * (total_translation / num_frames);

depth = VideoWriter(strcat('im',int2str(image_number),'_',pair,'_depth.mp4'), 'MPEG-4');
rgb = VideoWriter(strcat('im',int2str(image_number),'_',pair,'_rgb.mp4'),'MPEG-4');
depth.FrameRate = frame_rate;
rgb.FrameRate = frame_rate;

open(depth)
open(rgb)
for t=0:num_frames-1
    rotation = omega * t;
    translation = unit_translation * t;
    if enhance
        [depth_img, rgb_img] = compute_2D_projection_enhanced(image_number, rotation, axis, translation);
    else
        [depth_img, rgb_img] = compute_2D_projection(image_number, rotation, axis, translation);
    end
    writeVideo(depth, double(depth_img) / 65536) % convert to double
    writeVideo(rgb, rgb_img)
end
close(depth)
close(rgb)
end