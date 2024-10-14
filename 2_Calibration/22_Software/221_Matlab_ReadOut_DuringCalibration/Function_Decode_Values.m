function [Bx,By,Bz,Temp,newData] = Function_Decode_Values(COM_line)
% Function to decode the read String from UART/Serial


H1 = extractBetween(COM_line, "H1;","|");
H2 = extractBetween(COM_line, "H2;","|");
H3 = extractBetween(COM_line, "H3;","|");
H4 = extractBetween(COM_line, "H4;","|");
H5 = extractBetween(COM_line, "H5;","|");
H6 = extractBetween(COM_line, "H6;","|");
H7 = extractBetween(COM_line, "H7;","|");
H8 = extractAfter(COM_line, "H8;");

Bx = zeros(8,1);
By = zeros(8,1);
Bz = zeros(8,1);
Temp = zeros(8,1);
newData = zeros(8,1);

getValues(H1,1);
getValues(H2,2);
getValues(H3,3);
getValues(H4,4);
getValues(H5,5);
getValues(H6,6);
getValues(H7,7);
getValues(H8,8);

    function [] = getValues(text, index)
        Bx(index) = str2double(extractBetween(text,"Bx=","("));
        By(index) = str2double(extractBetween(text,"By=","("));
        Bz(index) = str2double(extractBetween(text,"Bz=","("));
        Temp(index) = str2double(extractBetween(text,"Temp=",";"));
        newData(index) = str2double(extractAfter(text,"ND="));
    end
end
