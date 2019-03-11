xspace=0:1000;
yspace=0:1000;
x0=10;% initial coordinates
y0=10;
xf=950;%final coordinates
yf=950;
vmax=140;
vmin=10;
Change=50; %number of  changes
step=round((vmax-vmin)/Change);
probmut=0.001; %probability of mutation
% velocity=vmax:step:vmin; %velocities from 10 to 140 km/hr
velocity=linspace(10,140,Change);
N=10;%number of obstacles
%obstacle size
a=50;
b=50;
rot=randi([-180 180],Change); %rotation possibilities
% obstacle position
xobs=zeros(N,2);
yobs=zeros(N,2);
xobs(:,1)=randi([0 xspace(end)-a],N,1);%
yobs(:,1)=randi([0 yspace(end)-b],N,1);
xobs(:,2)=xobs(:,1)+a;
yobs(:,2)=yobs(:,1)+b;
% check for overlap
% for i=1:N
% xobs(i,1)    
% 
% 
% end
% plot obstacles
for i=1:N
x1=xobs(i,1);
x2=xobs(i,2);
y1=yobs(i,1);
y2=yobs(i,2);
% repeat first to close figure
xplot=[x1 x1 x2 x2 x1];
yplot=[y1 y2 y2 y1 y1];
plot(xplot,yplot)
hold on
end
%% Initial population
M=1;
xp=zeros(M,50);
yp=zeros(M,50);
%initial population on start coordinates
xp(:)=x0;
% xp(:,end)=xf;
yp(:)=y0;
% yp(:,end)=yf;
%% initial movements 
 for i=1:M 
     for k=2:Change
         ptx=xp(i,k-1)+velocity(k-1).*cos(pi/180*rot(i,k-1)); %initial position+velocity*angle
          pty=yp(i,k-1)+velocity(k-1).*sin(pi/180*rot(i,k-1));
          while sign(ptx)==-1 ||sign(pty)==-1 %avoid negative path 
          rot=randi([-180 180],Change);
          ptx=xp(i,k-1)+velocity(k-1).*cos(pi/180*rot(i,k-1)); 
          pty=yp(i,k-1)+velocity(k-1).*sin(pi/180*rot(i,k-1));
          end
          xp(i,k)=ptx;
          yp(i,k)=pty;          
     end
     plot(xp(i,:),yp(i,:),'--'); 
     xlim([0 1000]);
     ylim([0,1000]); 
     hold on; 
 end
 %% mutation
%  for i=1:Changes;
%  randN=rand()
%  if randN<probmut
%      xp(i)= ;
%      yp(i)= ;
%  end
%  
%  end
 
%%


% %  FALTA QUE NO SE EMPALMEN OBSTACULOS
% % FALTA QUE CUANDO CHOQUE PASE ALGO 
