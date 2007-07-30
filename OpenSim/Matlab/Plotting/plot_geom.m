function plot_geom(geom)

lightdir = [1 0 0];
color = [0 0 1];

%close all;
figure;
hold on;
for i=1:geom.np
%for i=1:1
    poly = geom.poly{i};
%    poly(end+1) = poly(1);
%    plot3(geom.vertices(poly,1), geom.vertices(poly,2), geom.vertices(poly,3));
    N = sum(geom.normals(poly,:));
    N = N / norm(N);
    c = 0.5*(dot(N,lightdir)+1) * color;
    patch(geom.vertices(poly,1), geom.vertices(poly,2), geom.vertices(poly,3), c)
end
