clc


    //get data from file
    data_raw = read_csv("/home/serveur/catkin_ws/data/test_linear_speed_2015-11-13_17-42-31");
    data = evstr(data_raw) ;
    cmd = data(:,1)' ;
    speed_x = data(:,6)' ;
    dx = data(:,3)
    size_c = size(cmd);
    size_c = size_c(2);
    // compute error 
    err_speed_x = (speed_x - cmd)./cmd ; 
    err_speed_x(size_c/2+1) = 0.0;
    mean_error_x = mean(err_speed_x)
    mean_error_x_array = ones(1,size_c)*mean_error_x;
    disp(mean_error_x)
    figure
    plot(cmd,mean_error_x_array,"r+--")
    plot(cmd,err_speed_x,"b+--")
    xgrid(0)
    figure
    plot(cmd,cmd,"r+--")
    plot(cmd,speed_x,"b+--")
    xgrid(0)
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////

    data_raw = read_csv("/home/serveur/catkin_ws/data/test_linear_speed_2015-11-13_17-59-40");
    data = evstr(data_raw) ;
    cmd = data(:,1)' ;
    speed_x = data(:,6)' ;
    size_c = size(cmd);
    size_c = size_c(2);
    // compute error 
    err_speed_x = (speed_x - cmd)./cmd ; 
    err_speed_x(size_c/2+1) = 0.0;
    mean_error_x = mean(err_speed_x)
    mean_error_x_array = ones(1,size_c)*mean_error_x;
    disp(mean_error_x)
    figure
    plot(cmd,mean_error_x_array,"r+--")
    plot(cmd,err_speed_x,"b+--")    
    xgrid(0)
    figure
    plot(cmd,cmd,"r+--")
    plot(cmd,speed_x,"b+--")
    xgrid(0)
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////

    data_raw = read_csv("/home/serveur/catkin_ws/data/test_linear_speed_2015-11-13_17-60-19");
    data = evstr(data_raw) ;
    cmd = data(:,1)' ;
    speed_x = data(:,6)' ;
    size_c = size(cmd);
    size_c = size_c(2);
    // compute error 
    err_speed_x = (speed_x - cmd)./cmd ; 
    err_speed_x(size_c/2+1) = 0.0;
    mean_error_x = mean(err_speed_x)
    mean_error_x_array = ones(1,size_c)*mean_error_x;
    disp(mean_error_x)
    figure
    plot(cmd,mean_error_x_array,"r+--")
    plot(cmd,err_speed_x,"b+--")    
    xgrid(0)
    figure
    plot(cmd,cmd,"r+--")
    plot(cmd,speed_x,"b+--")
    xgrid(0)
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////

