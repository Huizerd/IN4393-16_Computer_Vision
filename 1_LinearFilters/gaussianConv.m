function imOut = gaussianConv(image, sigma_x, sigma_y)

band1 = image(:,:,1);
band2 = image(:,:,2);
band3 = image(:,:,3);

u = gaussian(sigma_x);
v = gaussian(sigma_y);

band1_out = conv2(u, v, band1, 'same');
band2_out = conv2(u, v, band2, 'same');
band3_out = conv2(u, v, band3, 'same');

imOut = cat(3, band1_out, band2_out, band3_out);

end