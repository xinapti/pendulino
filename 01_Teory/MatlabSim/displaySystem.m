function displaySystem(t_sim, frameRate, Xstate, U, par)
%displaySystem Summary of this function goes here
% X   = [Theta, Omega] vect
% U   = Mag_acc vect
% par = [L, Km]

[Theta, Omega] = deal(Xstate(:,1), Xstate(:,2));
[L, Km] = deal(par(1), par(2));

%Converti Stato angolare in Stato cartesiano
x = L *  sin(Theta);
y = L * -cos(Theta);
Fm_x = abs(U).*sign(-Theta) .* Km .* cos(Theta)./(1 + (L.*sin(Theta)).^2);

% Plotting results
figure(1)
clf
subplot(1,2,1)
plot(t_sim,Theta,'r-')
hold on
grid on
plot(t_sim,Omega,'b-')
plot(t_sim,Fm_x,'g-')
xlabel('time')
ylabel('rad°')
legend('angle','angular velocity', 'MagAcc')
% force the plot to have the same extent to the top and bottom
YL = get(gca, 'YLim');
maxlim = max(abs(YL));
set(gca, 'YLim', [-maxlim maxlim]);


subplot(1,2,2)
plot(x,y,'ko')
hold on
grid on
xlabel('x')
ylabel('y')
axis(gca, 'equal')

% Animation Plot
f = figure(2);
f.Position = [0 0 1080 1080];
clf
for i=1:length(t_sim)
    pendPlotFrame(f, [x(i),y(i)], Fm_x(i), [L, Km])
    grid on
    ylim([-(L+1),0])
    axis(gca,'square')
    daspect([1 1 1])
%     pause(1/frameRate)
    %if i~=length(t)
        %clf   
    %end   
    movieVector(i)=getframe;
end

myWriter=VideoWriter('Pendulum');
myWriter.FrameRate=frameRate; % Frame per second
myWriter.Quality=100;

open(myWriter);
writeVideo(myWriter,movieVector);
close(myWriter);

end