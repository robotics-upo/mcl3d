function v = gps_to_local(a, b)
    equatorial_shift = 111319.9;
    latitude_shift = 111111.1;
    diff = a - b
    center = 0.5*(a + b)
    v(1) = diff(2) * equatorial_shift * cos( center(1) * pi / 360);
    v(2) = diff(1) * latitude_shift;
endfunction