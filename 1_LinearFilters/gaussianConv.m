function imOut = gaussianConv(image, sigma_x, sigma_y)

u = gaussian(sigma_x);
v = gaussian(sigma_y);

imOut = conv2(u, v, image, 'same');

end