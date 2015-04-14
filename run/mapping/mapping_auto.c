#include <unistd.h>
#include <stdio.h>
#include <signal.h>

int main(){
    printf("=================================================\n");
    printf("====================START MAPPING================\n");
    printf("=================================================\n");

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
            execlp("roslaunch", "roslaunch", "robot", "carto_step1.launch", NULL);
            while(1){
                printf("fils 1 running...\n");
                sleep(10);
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
                    execlp("roslaunch", "roslaunch", "robot", "carto_step2.launch", NULL);
                    while(1){
                        printf("fils 2 running...\n");
                        sleep(10);
                    }
                    break;
                default://pere : attends entree pour kill fils 1 & 2
                    printf("=================================================\n");
                    printf("===================Fils crées====================\n");
                    printf("=========APPUYER <<ENTREE>> POUR QUITTER=========\n");
                    printf("=================================================\n");

                    pid3 = fork();
                    switch(pid3){
                        //fils 3 :
                        case -1: //erreur
                            printf("error\n\n");
                            break;
                        case 0: //fils : exec rosrun map_server map_saver
                            sleep(20);
                            printf("=================================================\n");
                            printf("==========SIMPLE EXPLORATION CONTROLLER========\n");
                            printf("=================================================\n");
                            execlp("rosrun", "rosrun", "hector_exploration_controller", "simple_exploration_controller", NULL);
                            printf("fils 3 running...\n");
                            while(1){
                                printf("fils 3 running...\n");
                                sleep(10);
                            }
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
    return 0;
}
