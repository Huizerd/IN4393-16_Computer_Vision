function [top, bottom] = set_top_bottom(ind, sign, top, bottom)

% First time
if sign == 0
    
    if ind ~= 1 && ind ~= 19
        top = ind + 1;
        bottom = ind - 1;
    elseif ind == 1
        top = 2;
        bottom = 19;
    else
        top = 1;
        bottom = 18;
    end
    
elseif sign > 0
    
    if top ~= 19
        top = top + 1;   
    else
        top = 1;
    end
    
elseif sign < 0
    
    if bottom ~= 1
        bottom = bottom - 1;
    else
        bottom = 19;
    end
    
end
end