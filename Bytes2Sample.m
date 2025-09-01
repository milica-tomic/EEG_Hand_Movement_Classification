function CH1_vrednost = Bytes2Sample( CH1_bajtovi )

% Constants for voltage calculations
 Vref = 4.5; % AD converter reference voltage
 Gain = 24; % Gain of the instrumentation amplifier
 CH01_Data = CH1_bajtovi .* ([2^16; 2^8; 1]); %pravimo neozna?eni 24-bitni decimalni ekvivalent
 % CH01 is row with 3 elements, byte type. Each element is one byte received for CH01 from Smarting.
 CH01_Out = sum(CH01_Data); %pravimo neozna?eni 24-bitni decimalni ekvivalent
 if(CH01_Out > hex2dec('007FFFFF')) %ako je ve?e od (2^24)/2 - 1
 CH01_Out = CH01_Out - hex2dec('1000000'); %oduzmi 2^24
 end
 scaleFactor = (Vref / (2 ^ 23 - 1)) / Gain; %da bismo dobili napon, množimo kvantom A/D konvertora; usput i delimo
%poja?anjem Gain
 to_uV = 1e+6; %konvertujemo iz V u uV
 % ***** Converted EEG data into uV *****
 CH1_vrednost = CH01_Out * scaleFactor * to_uV;


end

