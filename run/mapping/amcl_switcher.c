#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>

int main(int argc, char **argv){
    printf("=================================================\n");
    printf("====================START AMCL_SwITCHER================\n");
    printf("=================================================\n");

    //add those following lines for the second robot and others

    char *robot_id =NULL;
    robot_id = malloc(argc * sizeof(char*));
    if (argc != 2)
      {
        printf("Erreur read argument. \n");

       }
    else
       {

        printf("1er argument: %s\n", argv[1]);
        //const char robot_id_chaine[] =(const char)*argv[1];


       }
    //*********************
    char robot_arg[256];
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
                    snprintf(robot_arg, sizeof robot_arg, "robot:=%s", argv[1]);
                    printf("1er argument: %s\n",robot_arg);
                    execlp("roslaunch", "roslaunch", "robot", "server_localisation_switched.launch", robot_arg, NULL);
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
                            snprintf(robot_arg, sizeof robot_arg, "robot%s/publish_trigger", argv[1]);
                            printf("2eme argument: %s\n",robot_arg);
                            execlp("rostopic", "rostopic", "pub", robot_arg, "std_msgs/Bool",
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
