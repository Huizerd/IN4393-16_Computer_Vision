% Correct??

function G = gaussian(sigma)

x = linspace(-3*sigma, 3*sigma, 9);

G = 1 / (sigma * sqrt(2*pi)) * exp(-x.*x / (2*sigma*sigma));

G = G/sum(G);

end