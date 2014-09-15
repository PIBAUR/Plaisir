#include <unistd.h>
#include <stdio.h>
#include <signal.h>

int main(){
    printf("hello_world\n\n");
    
    pid_t pid1;
    pid_t pid2;
    pid_t pid3;
    pid_t pid4;
    pid1 = fork();
    
    switch(pid1){
        // fils 1 :
        case -1: //erreur...
            printf("error\n\n");
            break;
        case 0://fils : exec launch setup
            sleep(1);
            printf("1111111111111111111111111111111111111111111111111\n");
            printf("1111111111111111111111111111111111111111111111111\n");
            printf("1111111111111111111111111111111111111111111111111\n");
            printf("1111111111111111111111111111111111111111111111111\n");
            printf("1111111111111111111111111111111111111111111111111\n");
            printf("1111111111111111111111111111111111111111111111111\n");
            execlp("roslaunch", "roslaunch", "test_suivi", "setup.launch", NULL);
            while(1){
                printf("fils 1 running...\n");
                sleep(1);
            }
            
            break;
        default: //pere : nouveau fils
            pid2 = fork();
            switch(pid2){
                //fils 2 :
                case -1: //erreur
                    printf("error\n\n");
                    break;
                case 0: //fils : exec launch map
                    sleep(10);
                    printf("2222222222222222222222222222222222222222222222222\n");
                    printf("2222222222222222222222222222222222222222222222222\n");
                    printf("2222222222222222222222222222222222222222222222222\n");
                    printf("2222222222222222222222222222222222222222222222222\n");
                    printf("2222222222222222222222222222222222222222222222222\n");
                    printf("2222222222222222222222222222222222222222222222222\n");
                    execlp("roslaunch", "roslaunch", "test_suivi", "map.launch", NULL);
                    while(1){
                        printf("fils 2 running...\n");
                        sleep(1);
                    }
                    break;
                default://pere : attends entree pour kill fils 1 & 2
                    printf("WAIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIT\n");
                    printf("=================================================\n");
                    printf("=================================================\n");
                    printf("=================================================\n");
                    printf("=================================================\n");
                    printf("=================================================\n");
                                        
                    pid3 = fork();
                    switch(pid3){
                        //fils 3 :
                        case -1: //erreur
                            printf("error\n\n");
                            break;
                        case 0: //fils : exec courbe bezier
                            sleep(10);
                            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                            printf("=================================================\n");
                            printf("=================================================\n");
                            printf("=================================================\n");
                            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                            printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
                            execlp("roslaunch", "roslaunch", "test_suivi", "bezier.launch", NULL);
                            while(1){
                                printf("fils 3 running...\n");
                                sleep(10);
                            }
                            break;
                        default://pere : attends entree pour kill fils 1 & 2
                            
                            pid4 = fork();
                            switch(pid4){
                                case -1: //erreur
                                    printf("error\n\n");
                                    break;
                                case 0: 
                                    sleep(10);
                                    printf("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF\n");
                                    printf("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII\n");
                                    printf("NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN\n");
                                    execlp("roslaunch", "roslaunch", "test_suivi", "command.launch", NULL);
                                
                                    break;
                                default :
                            printf("wait for 3...\n");
                            wait(pid4);
                            
                            kill(pid1,SIGTERM);
                            kill(pid2,SIGTERM);
                            kill(pid3,SIGTERM);
                            wait(pid3);
                            wait(pid2);
                            wait(pid1);
                            printf("=================================================\n");
                            printf("======================END!!!=====================\n");
                            printf("=================================================\n");
                        }
                        break;
                    }
                break;
            }
        break;
    }

    
    return 0;
}
