
clc;clear;close all
f = figure;
j = 1;
image = imshow('Screenshot 2024-11-04 174632.jpg');
hold on;
%%

while true
    try % ginput(number) throws error when 'ENTER' is pressed
        [xv(j), yv(j)] = ginput(1);
        plot(xv(j),yv(j),'ro','MarkerFaceColor','r','MarkerSize',3)

        if j > 1
            if mod(j,2) == 0
                plot(xv(j-1:j),yv(j-1:j),'LineWidth',2,'color','red') % polygon
            else
                
            end
            
        end

        j=j+1;

    catch
        break;
    end
    
    % In case something other than enter is pressed
    if f.CurrentCharacter > 0
        break;
    end

end
%%
indexes = 1:j-1;
index = indexes(logical(mod(indexes,2)));
index2 = indexes(~logical(mod(indexes,2)))

figure;
hold on
Outside = plot(xv(index), yv(index), 'Color', 'blue');
Inside = plot(xv(index2), yv(index2), 'Color', 'red');

% in = inpolygon(xq,yq,xv,yv);
% figure
% plot(xv,yv,'LineWidth',2,'color','red') % polygon
% axis equal
% hold on
% plot(xq(in),yq(in),'r+') % points inside
% plot(xq(~in),yq(~in),'bo') % points outside

