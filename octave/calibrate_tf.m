function trans = calibrate_tf(gps_points, map_points)
    local = zeros(size(gps_points));
    A = zeros([size(gps_points,1)*2, 4]);
    b = zeros(size(gps_points,1)*2 , 1);
    for i=1:size(gps_points,1)
      local(i,:) = gps_to_local(gps_points(1,:), gps_points(i,:));
      A (2*i - 1, 1) = local(i,1);
      A (2*i - 1, 2) = -local(i,2);
      A (2*i - 1, 3) = 1;
      A (2*i - 1, 4) = 0;
      
      A (2*i, 1) = local(i,2);
      A (2*i, 2) = local(i,1);
      A (2*i, 3) = 0;
      A (2*i, 4) = 1;
      
      b (2*i - 1) = map_points(i, 1);
      b (2*i) = map_points(i, 2);
      
      
    endfor
    A;
    b;
    
    a = inv(A'*A)*A'*b
    
    trans(1,1) = a(3);
    trans(1,2) = a(4);
    
    trans(1,3) = atan2(a(2),a(1))
    
endfunction