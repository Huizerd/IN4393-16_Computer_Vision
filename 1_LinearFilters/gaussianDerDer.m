function Gdd = gaussianDerDer(G, sigma)

sz = floor(3*sigma + 0.5);
x = linspace(-3*sigma, 3*sigma, (2*sz+1));

Gdd = (-(sigma*sigma) + (x.*x)) ./ (sigma*sigma*sigma*sigma) .* G;

end