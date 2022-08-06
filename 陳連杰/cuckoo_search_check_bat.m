% ======================================================== % 
% Files of the Matlab programs included in the book:       %
% Xin-She Yang, Nature-Inspired Metaheuristic Algorithms,  %
% Second Edition, Luniver Press, (2010).   www.luniver.com %
% ======================================================== %   
% Cuckoo Search for nonlinear constrained optimization     %
% Programmed by Xin-She Yang @ Cambridge University 2009   %
% Usage: cuckoo_spring(25000) or cuckoo_spring;            %

function [bestnest,fmin]=cuckoo_search_check(time)
clc
clear 
tic;
to=clock;
format long;%%%%�@ex:short,pi=3.1416;long,pi=3.141596548222
% help cuckoo_search_spring.m
if nargin<1,
    % Number of iteraions
    time=20;%%%  1080904 ���դh
   % time=20000;
end

%disp('Computing ... it may take a few minutes.');

%%%%%%%%%%%%%%  1080826 �ѼƳ]�w---�i�󦹤��q����(1080911)
% Number of nests (or different solutions)
%n=5;%%%%%  1080826 n=1,2....(Population No.)
n=10;%%%%%  1080904 n=1,2....(Population No.)

% Discovery rate of alien eggs/solutions
pa=0.25;%%%%%% 1080826 pa=local search/global search(����j�M�V�j�V�n)

% Simple bounds of the search domain
% Lower bounds and upper bounds
%%%%%%%%%%%%%  1080928 �@,�G,�T���ܼ�  %%%%%%%%
Lb=[0.8*20802.21 0.8*20802.21 0.8*20802.21];
Ub=[1.2*20802.21 1.2*20802.21 1.2*20802.21];
%%%%%%%%%%%%%%%%%%%%%%%% 1080826 ��l�ȳ]�w %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%�@1080911 �i�󦹬q�]�w�P�_����A�p�G��=0 ����z�׭ȩΪ�l�ȡ@
%%%%%%%%%%%%  else ������e���x�s��bestnest=nest �@�����N����l�� 

% Random initial solutions----1080826 �ǧ�-��l�ȳ]�w
%{
 for i=1:n,
 nest(i,:)=Lb+(Ub-Lb).*rand(size(Lb));
 end
%}
 
%nest=20802.2*abs(rand(1,1));%  1080823 �´�-��l�ȳ]�w

%nest=20802;%%%%%  1080911 �ǧ̭�-�z�׭ȽT�{�{�������T

%%%%%   1080924 load bestness data from above execute cucko  program
filename=['bestnest.dat'];
bestnest=load(filename);
%%%%%   1080930 �e�����q���x�s���G��,���J�i��
bestnest_1=load(filename);%%%%%�@1080930 �e�����q���x�s���G��,���J�i��
data4=[bestnest_1];
file_out=['bestnest_1.dat' ]; 
save(file_out,'data4','-ascii');  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get the current best
nest=bestnest;%%%%%  1081007 �ģ̌�
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fitness=10^10*ones(n,1);
[fmin,bestnest,nest,fitness]=get_best_nest(nest,nest,fitness);

N_iter=0;
%% Starting iterations
for t=1:time,
    % Generate new solutions (but keep the current best)
     new_nest=get_cuckoos(nest,bestnest,Lb,Ub);   %%%%%  1080911 �i�H���������I,�H�K�ˬd�� bestness�O�_���T�@
     [fnew,best,nest,fitness]=get_best_nest(nest,new_nest,fitness);
    % Update the counter
      N_iter=N_iter+n; 
    % Discovery and randomization
      new_nest=empty_nests(nest,Lb,Ub,pa) ;
    
    % Evaluate this solution
      [fnew,best,nest,fitness]=get_best_nest(nest,new_nest,fitness);
    % Update the counter again
      N_iter=N_iter+n;
    % Find the best objective so far  
    if fnew<fmin,
        fmin=fnew;
        bestnest=best ;
    end
    %%%%%%%%% 1080911 ���N�]�w�������,�i�H�N�]�w�μȰ�����
     L(t)=fmin; 
   
    if t>1
        B=diff(L);
        %{
    if L(t)>0
    
    if   abs(B(t-1))<1e-10 && B(t-1)~=0
               
        break
        
    end
        else 
        %}
     if   abs(B(t-1))<1e-4 && B(t-1)~=0
               
        break
        
    
    end
    end
   % end
    %%%%%%%%%%  1080911  ���N�]�w�������
    figure(1)
    plot(L,'LineWidth',3,'EraseMode','xor');
    drawnow
  %%%%%%%%%%%  1080929 %%%%%  
  %{
  data1=[L]
  file_out=['iteration.dat' ]; 
  save(file_out,'data1','-ascii');
  %}
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
end %% End of iterations
 %%%%%%%%%%%  1080929 %%%%%  
  %%%%%%%%%%  1080826�@ load old file ���G���  %%%%%%
   T=[]
  L  
  filename_1=['iteration.dat'];
 iteration_1=load(filename_1);
 T=[iteration_1;L']
 %%%%%    1080930  save file
  data3=[T]
  file_out=['iteration.dat' ]; 
  save(file_out,'data3','-ascii');
% x=bestnest;
%% Post-optimization processing
%% Display all the nests
% disp(strcat('Total number of iterations=',num2str(N_iter)));
%%%%%%%    10809111 save 'bestness' as next nest initial guress.

 fmin  %%%%%%  1080826�@ ���G���
 bestnest  %%%%�@1080826 ���G���
 %%%%%%%%%%%%%  1080827 store data  
 data1=[fmin]
 data2=[bestnest]
  file_out=['fmin.dat' ]; 
  save(file_out,'data1','-ascii');   
  file_out=['bestnest.dat' ]; 
  save(file_out,'data2','-ascii');  
   %abs((bestnest-bestnest_1)/bestnest)<1e-2%%%%%%  �۳]�z�׭Ȼ~�t��
  %%%%%%%%%%%  1080929  %%%%%%
 if  abs(bestnest-bestnest_1)<1e-4%%%%%%  �۳]�z�׭Ȼ~�t�� 1081007 �ǧ�
    delete d:\bbb %%%%%%%  1080929 �]�w�p�G���q���N�w�g�O�z�׭�,�h���ݤ��q���� 
    delete d:\aaa
 else
     delete d:\aaa %%%%%%%  1081004 �~�������q���N�@����ҳ]�w����
 end
 %%%%%%%%%%%%%%% ���J�e������ɶ� 1081005
 filename_2=['elaspedtime.dat'];
 t2=load(filename_2);
%%%%%%% �����ɶ�+�W�@���ɶ� 
t=etime(clock,to);
 data4=[t+t2];%%%%%   �����ɶ�+�W�@���ɶ��X�p�� 1081005
 file_out=['elaspedtime.dat' ]; 
  save(file_out,'data4','-ascii');
 toc;

%% --------------- All subfunctions are list below ------------------
%% Get cuckoos by ramdom walk
function nest=get_cuckoos(nest,best,Lb,Ub)
% Levy flights
n=size(nest,1);
% Levy exponent and coefficient
% For details, see equation (2.21), Page 16 (chapter 2) of the book
% X. S. Yang, Nature-Inspired Metaheuristic Algorithms, 2nd Edition, Luniver Press, (2010).
beta=3/2;
sigma=(gamma(1+beta)*sin(pi*beta/2)/(gamma((1+beta)/2)*beta*2^((beta-1)/2)))^(1/beta);

for j=1:n,
    s=nest(j,:);
    % This is a simple way of implementing Levy flights
    % For standard random walks, use step=1;
    %% Levy flights by Mantegna's algorithm
    u=randn(size(s))*sigma;
    v=randn(size(s));
    step=u./abs(v).^(1/beta);
  
    % In the next equation, the difference factor (s-best) means that 
    % when the solution is the best solution, it remains unchanged.     
    stepsize=0.01*step.*(s-best);
    % Here the factor 0.01 comes from the fact that L/100 should the typical
    % step size of walks/flights where L is the typical lenghtscale; 
    % otherwise, Levy flights may become too aggresive/efficient, 
    % which makes new solutions (even) jump out side of the design domain 
    % (and thus wasting evaluations).
    % Now the actual random walks or flights
    s=s+stepsize.*randn(size(s));
   % Apply simple bounds/limits
   nest(j,:)=simplebounds(s,Lb,Ub);
end


%% Find the current best nest
function [fmin,best,nest,fitness]=get_best_nest(nest,newnest,fitness)
% Evaluating all new solutions
for j=1:size(nest,1),
    fnew=fobj(newnest(j,:));
    if fnew<=fitness(j),
       fitness(j)=fnew;
       nest(j,:)=newnest(j,:);
    end
end
% Find the current best
[fmin,K]=min(fitness) ;
best=nest(K,:);

%% Replace some nests by constructing new solutions/nests
function new_nest=empty_nests(nest,Lb,Ub,pa)
% A fraction of worse nests are discovered with a probability pa
n=size(nest,1);
% Discovered or not -- a status vector
K=rand(size(nest))>pa;

% In real world, if a cuckoo's egg is very similar to a host's eggs, then 
% this cuckoo's egg is less likely to be discovered, thus the fitness should 
% be related to the difference in solutions.  Therefore, it is a good idea 
% to do a random walk in a biased way with some random step sizes.  
nestn1=nest(randperm(n),:);
nestn2=nest(randperm(n),:);
%% New solution by biased/selective random walks
stepsize=rand*(nestn1-nestn2);
new_nest=nest+stepsize.*K;
for j=1:size(new_nest,1)
    s=new_nest(j,:);
  new_nest(j,:)=simplebounds(s,Lb,Ub);  
end

% Application of simple constraints
function s=simplebounds(s,Lb,Ub)
  % Apply the lower bound
  ns_tmp=s;
  I=ns_tmp<Lb;
  ns_tmp(I)=Lb(I);
  
  % Apply the upper bounds 
  J=ns_tmp>Ub;
  ns_tmp(J)=Ub(J);
  % Update this new move 
  s=ns_tmp;

%% Spring desgn optimization -- objective function---- 1080826
function z=fobj(u)
u1=u(1);
u2=u(2);
u3=u(3);
G = 0.8*[20802.21] ;
g = [3467.035] ;%%%% 1080731  g=G/6
E = diag(g.*g) ;
sigma = M7_DOF_7dof_p_noise_B1(u1,u2,u3) ;%%%  call fuction program.
% The well-known spring design problem
z=(3*2000+2*2)*(log(sigma))+(0.5)*(u-G)*(inv(E))*((u-G)')+(3*2000/2)+(1+1) ;%%%%%%�@�������N��{��
z=z+getnonlinear(u);%%%%% confirm 1080904

function Z=getnonlinear(u)
Z=0;
% Penalty constant
lam=10^15;

% Inequality constraints
g=[];
% g(1)=1-u(2)^3*u(3)/(71785*u(1)^4);
% gtmp=(4*u(2)^2-u(1)*u(2))/(12566*(u(2)*u(1)^3-u(1)^4));
% g(2)=gtmp+1/(5108*u(1)^2)-1;
% g(3)=1-140.45*u(1)/(u(2)^2*u(3));
% g(4)=(u(1)+u(2))/1.5-1;

% No equality constraint in this problem, so empty;
geq=[];

% Apply inequality constraints
for k=1:length(g),
    Z=Z+ lam*g(k)^2*getH(g(k));
end
% Apply equality constraints
for k=1:length(geq),
   Z=Z+lam*geq(k)^2*getHeq(geq(k));
end

% Test if inequalities hold
% Index function H(g) for inequalities
function H=getH(g)
if g<=0,
    H=0;
else
    H=1;
end
% Index function for equalities
function H=getHeq(geq)
if geq==0,
   H=0;
else
   H=1;
end
% ----------------- end ------------------------------

