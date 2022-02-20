%% This file is just a visualization of simulation. 
% No simulation process is calculated here. But you can adjust visual
% settings.
close all
clc

figure

quivers = quiver3([0.0; 0.0], [0.0; 0.0], [0.0; 0.0], [0.0; 0.0], [0.0; 0.0], [1.0; 1.0], 1.00, 'Color','b', 'LineWidth', 2.0);

% set z offset for arm if needed
[link1, verts1] = linkGeneration();
[link2, verts2] = linkGeneration();

%provide an offset for verts 2
z_offset = -0.1;
verts2 = bsxfun(@plus, verts2, [0.0, 0.0, z_offset]);

xlim([-2.25 2.25])
ylim([-2.25 2.25])
zlim([-2 2])

%set(gca,'xtick',[])
set(gca,'xticklabel',[])
%set(gca,'ytick',[])
set(gca,'yticklabel',[])
%set(gca,'ztick',[])
set(gca,'zticklabel',[])

camlight
campos([0.0 0.0 2]);
camorbit(-15,10,'camera')

axis equal

traj1 = animatedline('Marker','none', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);
traj2 = animatedline('Marker','none', 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.0);

%% Data loaded are:
% W: data from the output of VM control calculated by ode
th1 = W(:,1)';
th2 = W(:,2)';
th1_dot = W(:,3)';
th2_dot = W(:,4)';


for i=1:size(th1,2)
    [fr1, pos1, fr2, pos2] = planar_fk2(th1(i), th2(i));

    linkTransform(link1, verts1, fr1)
    linkTransform(link2, verts2, fr2)

    addpoints(traj1, pos1(1), pos1(2), pos1(3));
    addpoints(traj2, pos2(1), pos2(2), pos2(3)+z_offset);
    
    vec_w1 = sign(th1_dot(i))*0.1 + th1_dot(i);
    vec_w2 = sign(th2_dot(i))*0.1 + th2_dot(i);
    
    set(quivers, 'xdata', [0; pos1(1)],'ydata', [0; pos1(2)], 'wdata', [vec_w1; vec_w2]);

    axis equal
    xlim([-2.25 2.25])
    ylim([-2.25 2.25])
    zlim([-1 1])

    drawnow;

end
