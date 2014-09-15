#include <unistd.h>
#include <stdio.h>
#include <signal.h>

int main(){
    printf("hello_world\n\n");
    
    pid_t pid1;
    pid_t pid2;
    pid_t pid3;
    pid1 = fork();
    
    switch(pid1){
        // fils 1 :
        case -1: //erreur...
            printf("error\n\n");
            break;
        case 0://fils : exec launch cartographie_pere
            sleep(1);
            printf("=================================================\n");
            printf("===========NEATO LAUNCH CARTO FILS 1=============\n");
            printf("=================================================\n");
            execlp("roslaunch", "roslaunch", "md25", "cartographie_pere.launch", NULL);
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
                case 0: //fils : exec launch cartographie_fils
                    sleep(10);
                    printf("=================================================\n");
                    printf("==========NEATO LAUNCH CARTO FILS 2==============\n");
                    printf("=================================================\n");
                    execlp("roslaunch", "roslaunch", "md25", "cartographie_fils.launch", NULL);
                    while(1){
                        printf("fils 2 running...\n");
                        sleep(1);
                    }
                    break;
                default://pere : attends entree pour kill fils 1 & 2
                    printf("=================================================\n");
                    printf("===================Fils crées====================\n");
                    printf("=========APPUYER <<ENTREE>> POUR QUITTER=========\n");
                    printf("=================================================\n");
                    char c = '\0';
                    while(c!='\n'){
                        c = getchar();
                    }
                    
                    pid3 = fork();
                    switch(pid3){
                        //fils 3 :
                        case -1: //erreur
                            printf("error\n\n");
                            break;
                        case 0: //fils : exec rosrun map_server map_saver
                            sleep(1);
                            printf("=================================================\n");
                            printf("===================SAVE MAP======================\n");
                            printf("=================================================\n");
                            execlp("rosrun", "rosrun", "map_server", "map_saver", "-f", "/home/serveur/catkin_ws/maps/last_map", NULL);
                            printf("fils 3 running...\n");
                            break;
                        default://pere : attends entree pour kill fils 1 & 2
                            printf("wait for 3...\n");
                            wait(pid3);
                            printf("map save done...\n");
                            kill(pid1,SIGTERM);
                            kill(pid2,SIGTERM);
                            wait(pid2);
                            wait(pid1);
                            printf("=================================================\n");
                            printf("======================END!!!=====================\n");
                            printf("=================================================\n");
                        break;
                    }
                break;
            }
        break;
    }
    //execlp("roslaunch", "roslaunch", "neato_node", "cartographie.launch", NULL);
    
    return 0;
}