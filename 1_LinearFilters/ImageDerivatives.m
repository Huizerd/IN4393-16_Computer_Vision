function F = ImageDerivatives(image, sigma, type)

G = gaussian(sigma);

if type == 'x'
    F = normalConv(image, gaussianDer(G, sigma));
elseif type == 'y'
    F = normalConv(image, gaussianDer(G, sigma)');
elseif type == 'xx'
    F = normalConv(image, gaussianDer(gaussianDer(G, sigma), sigma));
elseif type == 'yy'
    F = normalConv(image, gaussianDer(gaussianDer(G, sigma)', sigma)');
elseif type == 'xy'
    F = normalConv(image, gaussianDer(gaussianDer(G, sigma), sigma)');
elseif type == 'yx'
    F = normalConv(image, gaussianDer(gaussianDer(G, sigma)', sigma));
end
end