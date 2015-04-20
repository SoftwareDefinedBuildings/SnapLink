function values = readValuesFromTxt(filename)
%% IO function
    try
        values = textscan(urlread(filename),'%f');
    catch
        fid = fopen(filename,'r');
        values = textscan(fid,'%f');
        fclose(fid);
    end
    values = values{1};
end
