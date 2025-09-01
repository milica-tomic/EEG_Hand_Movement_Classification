function GyroX_Out = Bytes2Sample1(CH01)

GyroX_Data = CH01 .* ([2^8; 1]);
GyroX_Out = sum(GyroX_Data);
if(GyroX_Out > hex2dec('7FFF'))
    GyroX_Out = GyroX_Out - hex2dec('10000');
end

end

