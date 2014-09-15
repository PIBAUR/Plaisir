#include <unistd.h>
#include <stdio.h>

int main(){
    printf("hello_world\n\n");
    
    pid_t pid;
    
    pid = fork();
    
    switch(pid){
        case -1:
            printf("error\n\n");
            break;
        case 0:
            sleep(10);
            printf("=================================================\n");
            printf("=================NEATO LAUNCH====================\n");
            printf("=================================================\n");
            execlp("roslaunch", "roslaunch", "neato_node", "cartographie_fils.launch", NULL);
            break;
        default:
            printf("=================================================\n");
            printf("=================NEATO LAUNCH====================\n");
            printf("=================================================\n");
            execlp("roslaunch", "roslaunch", "neato_node", "cartographie_pere.launch", NULL);
            wait(NULL);
            printf("=================================================\n");
            printf("======================END!!!=====================\n");
            printf("=================================================\n");
        break;
    }
    //execlp("roslaunch", "roslaunch", "neato_node", "cartographie.launch", NULL);
    
    return 0;
}
