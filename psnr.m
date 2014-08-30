function p=psnr(a,b)
    imga=imread(a);
    imgb=imread(b);
    MAX=255;
    [h,w]=size(imga);
    mes=sum(sum((imga-imgb).^2))/(h*w);
    p=20*log10(MAX/sqrt(mes));
    p=sum(p)/3;
    
    