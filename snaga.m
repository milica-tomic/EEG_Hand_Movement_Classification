% SPEKTRALNA GUSTINA SNAGE 
function [gustina_snage, f] = snaga(signal)   

Fs = 160;                               
T = 1/Fs;                   
L = size(signal,1);           % duzina signala
t = (0:L-1)*T;        

Y = fft(signal,[]);       % broj tacaka je jednak duzini signala 
amp = abs(Y./L).^2;       % kvadriranje vrednosti dvostranog amplitudskog spektra 
%figure, stem(amp);
%title('Dvostrani periodogram')
%xlabel('Tacke');
%ylabel('Snaga')
 
amp = amp(1:(L/2)+1,:);            % odbacivanje negativnih frekvencija
amp(2:end-1,:) = 2*amp(2:end-1,:); % izracunavanje spektralne snage jednostranog spektra
gustina_snage = amp/(Fs/L);        % podela sa rezolucijom frekvencijske ose 
% -> dobija se snaga po odbirku 

f = Fs*(0:(L/2))/L;                        % frekvencijska osa 
%figure, stem(f, gustina_snage)   
%title('Jednostrani periodogram')
%ylabel('Spektralna gustina snage [V^2/Hz]')
%xlabel('f [Hz]')
    
end 






 