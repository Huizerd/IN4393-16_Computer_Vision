function F = imageDerivatives(image, sigma, type)

G = gaussian(sigma);

if type == 'x'
    F = conv2(image, gaussianDer(G, sigma), 'same');
elseif type == 'y'
    F = conv2(image, gaussianDer(G, sigma)', 'same');
elseif type == 'xx'
    F = conv2(image, gaussianDerDer(G, sigma), 'same');
elseif type == 'yy'
    F = conv2(image, gaussianDerDer(G, sigma)', 'same');
elseif type == 'xy'
    F = conv2(gaussianDer(G, sigma), gaussianDer(G, sigma), image, 'same');
elseif type == 'yx'
    F = conv2(gaussianDer(G, sigma), gaussianDer(G, sigma), image, 'same');

end
end