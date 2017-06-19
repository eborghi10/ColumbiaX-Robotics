A = [4,4];
#A = [-4,4];
#A = [4,-4];
#A = [-4,-4];

B = [0,0];

#stepSize = 0.5;
step_size = [0.5, 1];

v = A-B;

%Distance between A and B 
distancia = norm(v);

%# of points
#numPuntos = distancia/stepSize;
num_puntos = ceil(abs(v)./step_size);
max_num_puntos = max(num_puntos);

X = [];
Y = [];

% Matrix form of the for loop
m = (A-B)/(max_num_puntos-1);
b = (B*max_num_puntos-A)/(max_num_puntos-1);
T = [];

%loop over all points
for i=1:num_puntos
  #Solve the x,y position at a distance ini
  #The points are in the line xf = w*x + c
  #Where w and c are parameters fixed by the vectors
  #Idem for the yf points. yf = g*y+d
  
  #A(1) = w*numPuntos+c
  #B(1) = w*1 + c
  
  %{
  w = (A(1)-B(1))/(max_num_puntos-1);
  # c = B(1) - w;
  c = (B(1)*max_num_puntos - A(1))/(max_num_puntos-1);
  
  X = [X;w*i+c];
  
  g = (A(2)-B(2))/(max_num_puntos-1);
  #d = B(2) - g;
  d = (B(2)*max_num_puntos - A(2))/(max_num_puntos-1);
  
  Y = [Y;g*i+d];
  %}
  
  T=[T; m*i+b ];
  
endfor

X = T(:,1);
Y = T(:,2);

scatter(X,Y);
