clc,
xdel(winsid())

    //get data from file
    data_raw = read_csv("/home/serveur/catkin_ws/data/test_linear_speed_2015-11-12_17-53-11");
    data = evstr(data_raw) ;
    cmd = data(:,1)' ;
    speed_x = data(:,6)' ;
    size_c = size(cmd);
    size_c = size_c(2);
    disp("size/2")
    disp(size_c)
    disp(round(size_c/2))
    cmd(round(size_c/2)+1)=0.00001;
    speed_x(round(size_c/2)+1) = 0.0;
    // compute error 
    err_speed_x = (speed_x - cmd)./cmd ; 
    mean_error_x = mean(err_speed_x)
    mean_error_x_array = ones(1,size_c)*mean_error_x;
    disp(mean_error_x)
    figure
    //xlabel("cmd"")
    plot(cmd,mean_error_x_array,"r+--")
    plot(cmd,err_speed_x,"b+--")
    
    figure
    plot(cmd,cmd,"r+--")
    plot(cmd,speed_x,"b+--")

    data_raw = read_csv("/home/serveur/catkin_ws/data/test_linear_speed_2015-11-12_17-50-34");
    data = evstr(data_raw) ;
    cmd = data(:,1)' ;
    speed_x = data(:,6)' ;
    size_c = size(cmd);
    size_c = size_c(2);
    disp("size/2")
    disp(size_c)
    disp(round(size_c/2))
    cmd(round(size_c/2)+1)=0.00001;
    speed_x(round(size_c/2)+1) = 0.0;

    // compute error 
    err_speed_x = (speed_x - cmd)./cmd ; 
    mean_error_x = mean(err_speed_x)
    mean_error_x_array = ones(1,size_c)*mean_error_x;
    disp(mean_error_x)
    figure
    //xlabel(cmd)
    plot(cmd,mean_error_x_array,"r+--")
    plot(cmd,err_speed_x,"b+--")
    
    figure
    plot(cmd,cmd,"r+--")
    plot(cmd,speed_x,"b+--")
    
    data_raw = read_csv("/home/serveur/catkin_ws/data/test_linear_speed_2015-11-12_17-47-06");
    data = evstr(data_raw) ;
    cmd = data(:,1)' ;
    speed_x = data(:,6)' ;
    size_c = size(cmd);
    size_c = size_c(2);
    disp("size/2")
    disp(size_c)
    disp(round(size_c/2))
    cmd(round(size_c/2)+1)=0.00001;
    speed_x(round(size_c/2)+1) = 0.0;

    // compute error 
    err_speed_x = (speed_x - cmd)./cmd ; 
    mean_error_x = mean(err_speed_x)
    mean_error_x_array = ones(1,size_c)*mean_error_x;
    disp(mean_error_x)
    figure
    //xlabel("cmd")
    plot(cmd,mean_error_x_array,"r+--")
    plot(cmd,err_speed_x,"b+--")
    
    figure
    plot(cmd,cmd,"r+--")
    plot(cmd,speed_x,"b+--")