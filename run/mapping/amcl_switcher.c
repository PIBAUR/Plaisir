#include <unistd.h>
#include <stdio.h>
#include <signal.h>

int main(int argc, char **argv){
    printf("=================================================\n");
    printf("====================START AMCL_SwITCHER================\n");
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
            printf("===========READ AMCL_POSE=============\n");
            printf("=================================================\n");
            char robot_arg[256];
            snprintf(robot_arg, sizeof robot_arg, "robot:=%s", argv[1]);
            execlp("rosrun", "rosrun", "amcl_test", "__init__.py", NULL);
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
                    printf("==========LAUNCH NEW AMCL_PARAM==============\n");
                    printf("=================================================\n");
                    execlp("roslaunch", "roslaunch", "robot", "server_localisation_switched.launch", "robot:=01", NULL);
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
                            printf("==========TRIGGER PUBLISH INIT POSE========\n");
                            printf("=================================================\n");

                            execlp("rostopic", "rostopic", "pub", "robot01/publish_trigger", "std_msgs/Bool",
                            		"true", NULL);
                            printf("fils 3 running...\n");
                            while(1){
                                printf("fils 3 running...\n");
                                sleep(10);
                            }
                            break;
                        default://pere : attends entree pour kill fils 1 & 2
                            printf("wait for 3...\n");
                            sleep(40);

                            wait(pid2);
                            //kill(pid1,SIGTERM);
                            //wait(pid1);
                            kill(pid3,SIGTERM);
                            wait(pid3);
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
