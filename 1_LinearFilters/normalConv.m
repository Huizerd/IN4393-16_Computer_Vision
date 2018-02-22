function imOut = normalConv(image, G)

band1 = image(:,:,1);
band2 = image(:,:,2);
band3 = image(:,:,3);

band1_out = conv2(band1, G, 'same');
band2_out = conv2(band2, G, 'same');
band3_out = conv2(band3, G, 'same');

imOut = cat(3, band1_out, band2_out, band3_out);

end