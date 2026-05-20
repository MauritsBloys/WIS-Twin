%% Save figure for use in Latex as eps or pdf

function [] = saveFigureEps(figureName, fontSize, usePdf)
% figureName    filename (without extension)
% fontSize      default 18
% usePdf        true | false save as pdf (default true)

% By putting this in a separate function it is easy to change the location
% or disable saving for all figures

    hold on;
    if nargin > 1
        set(findobj(gcf,'type','axes'),'FontSize', fontSize);
    else
        set(findobj(gcf,'type','axes'),'FontSize', 18); % default fontSize
    end
    
    if nargin < 3
        usePdf = true;
    end
        
    
   
    imageLocation = 'C:\Users\mauri\Downloads\BEP\Digital Twin\Figures';

    if exist(imageLocation, 'dir') 
        % My Latex editor (Texpad) has prblems live typesetting eps, 
        % therefore the default now is to save as pdf

        if usePdf
            % Script written by E Akbas (c) Aug 2010 used to remove margins from
            % PDF

            saveTightFigure(gcf, sprintf('%s/%s.pdf', imageLocation, figureName));
        else
            saveas(gcf,sprintf('%s/%s', imageLocation, figureName), 'epsc');
        end
    else
        disp("Folder to save image not found, skipping saveFigureEps.");
    end
end

