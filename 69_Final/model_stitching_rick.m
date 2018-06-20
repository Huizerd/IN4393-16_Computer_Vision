% model_stitching.m
%
% Combines all the 3D coordinate models of three and four consecutive
% images
%
% Input:
%   -   triple_models = 3D coordinates from three consecutive images
%   -   quad_models = 3D coordinates from four consecutive images
%
% Output:
%   -   complete_model  = complete 3D model
%   -   color           = colors of points in 3d model in normalized rgb.
% Authors: 
%   - Bas Buller 4166566
%   - Rick Feith 4218272

function [complete_model, color,quad_order,triple_order] = model_stitching_rick(triple_models, quad_models)
    % Find biggest four view model, set as starting point
    quad_order = [];
    triple_order = [];
    ind = 0;
    len = 0;
    for i = 1:max(size(quad_models))
        if(max(size(quad_models{i})) > len)
            len = size(quad_models{i,1},2);
            ind = i;
        end
    end


    sign = 0;
    function [top, bottom] = set_top_bottom(ind,sign,top,bottom)
    if sign == 0
        if (ind ~= 1 && ind ~= 19)
            top = ind+1;
            bottom = ind-1;
        elseif ind ==1
            top = 2;
            bottom = 19;
        else
            top = 1;
            bottom = 18;
        end
    elseif sign >0
        if top ~= 19
            top = top+1;   
        else
            top = 1;
        end
    elseif sign < 0
        if bottom~=1
            bottom = bottom -1;
        else
            bottom = 19;
        end
    end
    end

    
    [top,bottom]  =set_top_bottom(ind,sign,0,0);
    quad_order = [quad_order ind];
    triple_order = [triple_order ind top];
    
    for i = 2:max(size(quad_models))
        if (size(quad_models{top,1},2) >= size(quad_models{bottom,1},2))
            sign = 1;
            if (size(quad_models{top,1},2)==0)
                quad_order = [quad_order top]; 
                top = set_top_bottom(ind,sign, top,bottom);
                triple_order = [triple_order top];
                fprintf(strcat("Not enough matches to stitch models, lenght order: ", num2str(length(quad_order)), "\n"))
            else
                quad_order = [quad_order top];
                top = set_top_bottom(ind,sign, top,bottom);
                triple_order = [triple_order top];
            end
        elseif (size(quad_models{top,1},2) < size(quad_models{bottom,1},2))
            sign = -1;
            quad_order = [quad_order bottom];
            triple_order = [triple_order bottom];
            [~, bottom] = set_top_bottom(ind,sign,top,bottom);
           
        end
    end
    triple_order = triple_order(1:19);
    track_bottom = quad_order(1);
    
    % Preassign cell array for uodated models
     updated_triple_models = cell(max(size(triple_models)), 1);
     updated_quad_models = cell(max(size(quad_models)), 1);
     color = [];
    
%% Update first two views
    % First quad view model is not transformed
    updated_quad_models(quad_order(1)) = {quad_models{quad_order(1), 1}};
    direction = 1;
    % Assign temporary working variables
    quad = quad_models{quad_order(1), 1};
    triple = triple_models{triple_order(1),1};
    
    % Find matching points between three and four view models
    match_triple = triple_models{triple_order(1), 2};   
    match_quad = quad_models{quad_order(1), 2}(1:3, :);
%     size(match_triple)
%     size(match_quad)
    % save color values
    color_triple = triple_models{triple_order(1),3};
    color_quad = quad_models{quad_order(1),3};
        
    % Find intersection of points between four and three view
    [~, ~, IB] = intersect(match_quad', match_triple', 'rows', 'stable');
    if (size(quad') ~= size(triple(:,IB)'))
         fprintf("sizes mismatch")
    end
    
    % Procrustes with matching points between three and four view
    [~, ~, trans] = procrustes(quad', triple(:, IB)');
    
    % Apply transform to entire three view model and save in cell array
    new_triple = trans.b * triple' * trans.T + trans.c(1, :);
    updated_triple_models(triple_order(1)) = {new_triple'};

    
    % Final, complete 3D point model
    complete_model = [new_triple'];
    color = [color; color_triple];
    
    
%% Loop over remaining views
    % Loop over models to determine procrustes transforms, first fit three
    % view to four view, next fir four view to three view.
    for i = 2:length(quad_order)
        % Assign temporary working variables 
        fprintf(strcat("Appending triple model ", num2str(triple_order(i)), " to quad model ", num2str(quad_order(i-1)),", direction: ",num2str(direction),"\n"))
        triple = triple_models{triple_order(i),1};
        quad = updated_quad_models{quad_order(i-1),1};
        
        match_triple = triple_models{triple_order(i), 2};
        if direction == 1
            match_quad = quad_models{quad_order(i-1),2}(2:4,:);
        else
            match_quad = quad_models{quad_order(i-1),2}(1:3,:);
        end
        
        color_temp = triple_models{triple_order(i),3};
        
        % Find intersection of points between four and three view
        [~, ~, IB] = intersect(match_quad', match_triple', 'rows', 'stable');
        
         if (size(quad') ~= size(triple(:,IB)'))
             fprintf("sizes mismatch")
        end
        % Procrustes with matching points between three and four view
        [~, ~, trans] = procrustes(quad', triple(:, IB)');
   
        % Apply transform to entire three view model and save in cell array
        new_triple = trans.b * triple' * trans.T + trans.c(1, :);
        
        
        % Save models
        updated_triple_models(triple_order(i)) = {new_triple'};
        complete_model = [complete_model new_triple'];
        color = [color; color_temp];
        
        save temp
        
        if size(quad_models{quad_order(i)},2)>8
        % Reassign temporary working variables
            if quad_order(i)==triple_order(i)
                direction = 1;
                triple = new_triple';
                quad = quad_models{quad_order(i),1};

                match_quad = quad_models{quad_order(i), 2}(1:3, :);

                % Find intersection of points between four and three view
                [~, ~, IB] = intersect(match_quad', match_triple', 'rows', 'stable');

                if (size(quad') ~= size(triple(:,IB)'))
                     fprintf("sizes mismatch")
                end     

                % Procrustes with matching points between three and four view
                [~, new_quad,~ ] = procrustes( triple(:, IB)',quad');

                % Apply transform to entire quad view model and save in cell array
                updated_quad_models(quad_order(i)) = {new_quad'};

            elseif ((quad_order(i)+1)==track_bottom) || (quad_order(i)==19 && triple_order(i)==1);
                direction = 0;
                three_index  = find(triple_order==track_bottom);
                track_bottom = quad_order(i);
                triple = updated_triple_models{triple_order(three_index)};
                quad = quad_models{quad_order(i),1};

                match_quad = quad_models{quad_order(i), 2}(2:4, :);
                match_triple = triple_models{triple_order(three_index),2};
                % Find intersection of points between four and three view
                [~, ~, IB] = intersect(match_quad', match_triple', 'rows', 'stable');

                if (size(quad') ~= size(triple(:,IB)'))
                     fprintf("sizes mismatch")
                end     

                % Procrustes with matching points between three and four view
                [~, new_quad,~ ] = procrustes( triple(:, IB)',quad');

                % Apply transform to entire quad view model and save in cell array
                updated_quad_models(quad_order(i)) = {new_quad'};

            elseif (find(triple_order(1:i)==quad_order(i)))
                direction = 1;
                three_index = find(triple_order(1:i)==quad_order(i));
                triple = updated_triple_models{triple_order(three_index)};
                quad = quad_models{quad_order(i),1};

                match_quad = quad_models{quad_order(i), 2}(1:3, :);
                match_triple = triple_models{triple_order(three_index),2};
                % Find intersection of points between four and three view
                [~, ~, IB] = intersect(match_quad', match_triple', 'rows', 'stable');
                save temp
                if (size(quad') ~= size(triple(:,IB)'))
                     fprintf("sizes mismatch")
                end     

                % Procrustes with matching points between three and four view
                [~, new_quad,~ ] = procrustes( triple(:, IB)',quad');

                % Apply transform to entire quad view model and save in cell array
                updated_quad_models(quad_order(i)) = {new_quad'}; 
            else
                save temp
                fprintf("Unknown scenaria for procrustes joining. \n")
                break
            end
        else
            fprintf(strcat("Quad model is empty or to sparse. ",num2str(i) ," models have been joined before termination. \n"))
            break
        end
        
        
       save updated_quad updated_quad_models 
end
end




















