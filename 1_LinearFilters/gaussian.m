% Correct??

function G = gaussian(sigma)

sz = floor(3*sigma + 0.5);
x = linspace(-3*sigma, 3*sigma, (2*sz+1));

G = 1 / (sigma * sqrt(2*pi)) * exp(-x.*x / (2*sigma*sigma));

G = G/sum(G);

end