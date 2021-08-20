s = serial('COM2', 'BaudRate', 9600, 'Terminator', 'CR/LF');
fopen(s);
i = 0;
while(i<1000)
    read = fscanf(s,'%d');
    disp(read);
end
fclose(s);