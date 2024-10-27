figure; 
hold on;

for i = 1:100
    radius = rand * 3;      
    
    x = (rand * 20) - 10;  
    y= (rand * 20) - 10;
   
    color = rand(1, 3);  
    
    plotcircle(x, y, radius, color);
end

axis equal;
axis([-15 15 -15 15]);  
axis off;

hold off;  

