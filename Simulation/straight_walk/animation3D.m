 function animation3D(t,z,parms,fps,steps,view_angle)


 if (parms.makeAVI)
    mov = VideoWriter('rimlesswheel.avi');
    open(mov);
    axis off
    set(gcf,'Color',[1,1,1])
 end
 
 
%%%%%% interpolate 
[mmm,nnn] = size(z);
if (steps<1)
    steps = 1; %for animation to work
end

t_temp = linspace(0,t(end),fps*steps);
for i=1:nnn
    z_temp(:,i) = interp1(t,z(:,i),t_temp);
end
z = z_temp;
t = t_temp;
%%%%%%%%%%%%%%


%theta = linspace(0,2*pi);
%l0 = parms.l0; %0.254;
b = parms.b; %4.31/2;
lbox = parms.lbox;
tbox = parms.tbox;
offsetz_box = parms.offsetz_box;
n = parms.n;
dth = 2*pi/n;
shaft = [0 0; 
        -b b];
bb = b-0.025; %pull in by 0.05 cm
% torso = [0 0 0 0;
%          -bb -bb bb bb;
%           0 -lbox -lbox 0];
[mm,nn] = size(shaft); 
%[mm2,nn2] = size(torso);

vertex(1,:) = [ tbox  bb -lbox];
vertex(2,:) = [ tbox -bb -lbox];
vertex(3,:) = [ tbox -bb  lbox];
vertex(4,:) = [ tbox  bb  lbox];
vertex(5,:) = [-tbox  bb -lbox];
vertex(6,:) = [-tbox -bb -lbox];
vertex(7,:) = [-tbox -bb  lbox];
vertex(8,:) = [-tbox  bb  lbox];
% vertex(1,:) = [ l  w -t];
% vertex(2,:) = [ l -w -t];
% vertex(3,:) = [ l -w  t];
% vertex(4,:) = [ l  w  t];
% vertex(5,:) = [-l  w -t];
% vertex(6,:) = [-l -w -t];
% vertex(7,:) = [-l -w  t];
% vertex(8,:) = [-l  w  t];

[mm3,nn3] = size(vertex');

face = [ 1 2 3 4 ;
         5 6 7 8 ;
         2 6 7 3 ;
         1 5 8 4 ;
         3 7 8 4 ;
         2 6 5 1 ];

     
qL = z(:,1);
qR = z(:,3);
max_coord = max(max(abs(z(:,9:10))));
for i=1:length(t)
    mid_x = z(i,9);
    mid_y = z(i,10);
    axis([mid_x-1 mid_x+1 mid_y-1 mid_y+1 -1 1]);
    %axis((max_coord+parms.l0)*[-1 1 -1 1 -1 1]); 
    patch(2*max_coord*[-1 1 1 -1],2*max_coord*[-1 -1 1 1],[0 0 0 0],'Facecolor','y','Edgecolor','y','FaceAlpha',0.3);
    
%     axis((max_coord+parms.l0)*[-1 1 -1 1 -1 1]); 
%     patch(2*max_coord*[-1 1 1 -1],10*[-1 -1 1 1],[0 0 0 0],'Facecolor','y','Edgecolor','y','FaceAlpha',0.3);
%         
    phi=z(i,11);
    R_phi = [cos(phi) -sin(phi); %rotation matrix
             sin(phi) cos(phi)];
    newbox = R_phi*shaft;
    
    %%%%% rotation by q2 about -y axis
     q2 = z(i,5);
     Ry_q2 = [cos(q2) 0 sin(q2); %rotation matrix
             0       1   0;
             -sin(q2) 0 cos(q2)];
     %%% rotation by phi about z axis
     Rz_phi = [cos(phi) -sin(phi) 0; %rotation matrix
               sin(phi) cos(phi) 0;
                0         0       1];

    
    offset = [z(i,9) z(i,10)]'; %offset is the point about which the circle will move
    offsetmat= repmat(offset,1,nn); %creates nn copies of offset to be added to the vertices of the box.
 
    newbox = newbox + offsetmat; %rotate + translate
    
    l = z(i,7);
    trans_zL = l*cos(qL(i));
    trans_zR = l*cos(qR(i));
    
    offset = [z(i,9) z(i,10) 0.5*(trans_zL+trans_zR)]';
    %offsetmat2 = repmat(offset,1,nn2);
    %torso_trans = offsetmat2 + Rz_phi*Ry_q2*torso;
    
    %offsetz = -3*2.54/100;
    offset = offset + Rz_phi*Ry_q2*[0 0 -offsetz_box]';
    offsetmat3 = repmat(offset,1,nn3);
    
    vertex_trans = offsetmat3 + Rz_phi*Ry_q2*vertex';
    vertex_trans = vertex_trans';
    
    for j=1:n
        l = z(i,7);
        AL = qL(i)-(j-1)*dth; 
        AR = qR(i)-(j-1)*dth;
        spoke_line_L = [l*sin(AL);
                             0];
        spoke_line_L_trans = R_phi*spoke_line_L;
        spoke_line_R = [l*sin(AR);
                             0];
        spoke_line_R_trans = R_phi*spoke_line_R;
        
        
        left_xline(j,:) = [newbox(1,1)  newbox(1,1)+spoke_line_L_trans(1)];
        left_yline(j,:) = [newbox(2,1) newbox(2,1)+spoke_line_L_trans(2)];
        left_zline(j,:) = trans_zL*[1 1] + [0 l*cos(AL)];
        
        right_xline(j,:) = [newbox(1,2)  newbox(1,2)+spoke_line_R_trans(1)];
        right_yline(j,:) = [newbox(2,2) newbox(2,2)+spoke_line_R_trans(2)];
        right_zline(j,:) = trans_zR*[1 1] +[0 l*cos(AR)];
    end
    
    h3=line(newbox(1,:),newbox(2,:),[trans_zL trans_zR],'Linewidth',3,'Color','k');
    %h4 = patch(torso_trans(1,:),torso_trans(2,:),torso_trans(3,:),'green');
    h4 = patch('Vertices',vertex_trans,'Faces',face,'FaceColor','green');
    for j=1:n
        left_h(j)=line(left_xline(j,:),left_yline(j,:),left_zline(j,:),'Color',[0.5 0.5 0.5],'Linewidth',3);
        right_h(j)=line(right_xline(j,:),right_yline(j,:),right_zline(j,:),'Color',[0.5 0.5 0.5],'Linewidth',3);
    end
    xlabel('x'); ylabel('y'); zlabel('z');
    AZ = view_angle(1);
    EL = view_angle(2);
    view(AZ,EL);
    if (parms.makeAVI)
        currFrame = getframe;
        writeVideo(mov,currFrame);
    end
    pause(0.05);
    if (i<length(t)) 
        delete(h3); 
        delete(h4);
        for j=1:n
            delete(left_h(j));
            delete(right_h(j));
        end
    end
end

if (parms.makeAVI)
  close(mov);
end