function Gd = gaussianDer(G, sigma)

x = linspace(-3*sigma, 3*sigma, 9);

Gd = -x ./ (sigma*sigma) .* G;

end