#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <signal.h>
#include <ctype.h>
#include <pthread.h>

#define SM_COMPONENT 1
#define FE_COMPONENT 2

extern char *optarg;
extern int optind;

char *progName;
int smPID[4] = {0};
int fePID[4] = {0};
int doStop = 0;

void Usage(int err);
int parseInput(char *buf);
int spawn(const unsigned int instance, const int component, int *pids);
int pkill(const unsigned int instance, const int component, int *pid);
int daemon_main(void);
void *kill_thread(void *arg);

struct thread_data {
	pthread_t thread_id;
	int component;
	int instance;
	int *pid;
};

/**
 * Print program usage information
 * 
 * @author gomezchr (3/16/2015)
 * 
 * @param err Program exit code
 */
void Usage(int err){
	fprintf(stderr, "Usage: %s -D\n", progName);
	fprintf(stderr, "       %s [-i instance] [sm|fe] start|stop|restart|halt\n", progName);
	exit(err);
}

/**
 * Handle Linux signals, specifically SIGTERM.
 * 
 * @author gomezchr (3/16/2015)
 * 
 * @param signo Caught signal
 */
void sig_handler(int signo){
	if(signo == SIGTERM){
		doStop = 1;
		raise(SIGINT);	// Send signal to self to stop any blocking I/O
	}
}

/**
 * pthread function to kill an instance of an FM component.
 * 
 * @author gomezchr (3/16/2015)
 * 
 * @param arg Pointer to thread_data structure.
 * 
 * @return void* NULL
 */
void *kill_thread(void *arg){
	struct thread_data *data = arg;
	pkill(data->instance, data->component, data->pid);
	return NULL;
}

/**
 * Function to parse values passed through named pipe to the 
 * daemon. 
 * 
 * @author gomezchr (3/16/2015)
 * 
 * @param buf Pointer to the string extracted from the named 
 *  		  pipe
 * 
 * @return int Returns 1 if the daemon recieved a command to 
 *  	   stop.
 */
int parseInput(char *buf){
	char *token;
	int i, c;
	struct thread_data *threads = NULL;
	pthread_attr_t attr;
	token = strtok(buf, " ");
	if(token == NULL){
		fprintf(stderr, "Received invalid input. Ignoring.\n");
		return 0;
	}
	if(strncmp(token, "stop", 4) == 0){
		token = strtok(NULL, " ");
		if(token == NULL) token = "a";
		switch(token[0]){
		case '0':	// Kill both SM and FE for this instance number.
			token = strtok(NULL, " ");
			if(token == NULL){
				fprintf(stderr, "Received invalid input. Ignoring.\n");
				return 0;
			}
			i = atoi((const char*)token);
			if(i >= 4 || i < 0){
				fprintf(stderr, "Received invalid input. Ignoring.\n");
				return 0;
			}
			threads = calloc(2, sizeof(struct thread_data));
			if(threads == NULL){
				fprintf(stderr, "Failed to allocate thread struct.\n");
				return 0;
			}
			pthread_attr_init(&attr);
			pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
			threads[0].instance = i;
			threads[0].component = SM_COMPONENT;
			threads[0].pid = &smPID[i];
			pthread_create(&threads[0].thread_id, &attr, &kill_thread, &threads[0]);
			threads[1].instance = i;
			threads[1].component = FE_COMPONENT;
			threads[1].pid = &fePID[i];
			pthread_create(&threads[1].thread_id, &attr, &kill_thread, &threads[1]);
			pthread_attr_destroy(&attr);
			free(threads);
			break;
		case '1':	// SM and FE are treated the same so this prevents code redundancy
		case '2':
			c = atoi((const char*)token);
			token = strtok(NULL, " ");
			if(token == NULL){
				fprintf(stderr, "Received invalid input. Ignoring.\n");
				return 0;
			}
			i = atoi((const char*)token);
			pkill(i, c, c == SM_COMPONENT ? smPID : fePID);
			break;
		case 'a':	// Kill all running instances.
			threads = calloc(8, sizeof(struct thread_data));
			if(threads == NULL){
				fprintf(stderr, "Failed to allocate thread struct.\n");
				return 0;
			}
			pthread_attr_init(&attr);
			pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
			for(i = 0; i < 4; ++i){
				threads[i * 2].instance = i;
				threads[i * 2].component = SM_COMPONENT;
				threads[i * 2].pid = &smPID[i];
				pthread_create(&threads[i * 2].thread_id, &attr, &kill_thread, &threads[i * 2]);
				threads[(i * 2) + 1].instance = i;
				threads[(i * 2) + 1].component = FE_COMPONENT;
				threads[(i * 2) + 1].pid = &fePID[i];
				pthread_create(&threads[(i * 2) + 1].thread_id, &attr, &kill_thread, &threads[(i * 2) + 1]);
			}
			pthread_attr_destroy(&attr);
			free(threads);
			return 1;
		default:
			fprintf(stderr, "Unknown component value. (%d)\n", token[0]);
		}
		return 0;
	}
	if(strncmp(token, "start", 5) == 0){
		token = strtok(NULL, " ");
		if(token == NULL){
			fprintf(stderr, "Received invalid input. Ignoring.\n");
			return 0;
		}
		switch(token[0]){
		case '0':
			token = strtok(NULL, " ");
			if(token == NULL){
				fprintf(stderr, "Received invalid input. Ignoring.\n");
				return 0;
			}
			i = atoi((const char*)token);
			if(spawn(i, SM_COMPONENT, smPID) == -1) return 0;
			spawn(i, FE_COMPONENT, fePID);
			break;
		case '1':
		case '2':
			c = atoi((const char*)token);
			token = strtok(NULL, " ");
			if(token == NULL){
				fprintf(stderr, "Received invalid input. Ignoring.\n");
				return 0;
			}
			i = atoi((const char*)token);
			spawn(i, c, c == SM_COMPONENT ? smPID : fePID);
			break;
		default:
			fprintf(stderr, "Unknown component value.\n");
		}
	}
	if(strncmp(token, "restart", 7) == 0){
		token = strtok(NULL, " ");
		if(token == NULL){
			fprintf(stderr, "Received invalid input. Ignoring.\n");
			return 0;
		}
		switch(token[0]){
		case '0':
			token = strtok(NULL, " ");
			if(token == NULL){
				fprintf(stderr, "Received invalid input. Ignoring.\n");
				return 0;
			}
			i = atoi((const char*)token);
			if(pkill(i, SM_COMPONENT, smPID) == 0)
				spawn(i, SM_COMPONENT, smPID);
			if(pkill(i, FE_COMPONENT, fePID) == 0)
				spawn(i, FE_COMPONENT, fePID);
			break;
		case '1':
		case '2':
			c = atoi((const char*)token);
			token = strtok(NULL, " ");
			if(token == NULL){
				fprintf(stderr, "Received invalid input. Ignoring.\n");
				return 0;
			}
			i = atoi((const char*)token);
			if(pkill(i, c, c == SM_COMPONENT ? smPID : fePID) == 0)
				spawn(i, c, c == SM_COMPONENT ? smPID : fePID);
			break;
		default:
			fprintf(stderr, "Unknown component value.\n");
		}
	}
	return 0;
}

/**
 * Spawns an instance of the FM component.
 * 
 * @author gomezchr (3/16/2015)
 * 
 * @param instance Instance number the component belongs to 
 * @param component Integer representation of the component 
 *  				(1 = SM, 2 = FE)
 * @param pids Pointer to list of active PIDs. This function 
 *  		   will assign pids[instance] to the newly spawned
 *  		   PID.
 * 
 * @return int PID spawned or -1 if an error has occurred
 */
int spawn(const unsigned int instance, const int component, int *pids){
	int pid;
	char prog[25], name[6];
	if(instance >= 4){
		fprintf(stderr, "Invalid instance number.\n");
		return -1;
	}
	if(pids[instance] != 0 && kill(pids[instance], 0) == 0){
		fprintf(stderr, "Instance %d of %s is already running.\n", instance, component == SM_COMPONENT ? "SM": "FE");
		return -1;
	}
	switch(pid = fork()){
	case 0:
		sprintf(prog, "/opt/opafm/runtime/%s", component == SM_COMPONENT ? "sm" : "fe");
		sprintf(name, "%s_%d", component == SM_COMPONENT ?"sm" : "fe", instance);
		execlp(prog, prog, "-e", name, NULL);
		return 0;
	case -1:
		fprintf(stderr, "Failed to start %s for instance %d.\n", component == SM_COMPONENT ? "SM" : "FE", instance);
		return -1;
	default:
		fprintf(stdout, "Started instance %d of %s.\n", instance, component == SM_COMPONENT ? "SM" : "FE");
		if(pid > 0) pids[instance] = pid;
		return pid;
	}
}

/**
 * Kills instance of FM component 
 * 
 * @author gomezchr (3/16/2015)
 * 
 * @param instance Instance number the component belongs to
 * @param component Integer representation of the component 
 *  				(1 = SM, 2 = FE)
 * @param pid Pointer to the active PID to kill. This function 
 *  		   will assign pid to 0 if process was properly
 *  		   terminated.
 * 
 * @return int Returns 0 if process was properly terminated, 
 *  	   otherwise -1 on error.
 */
int pkill(const unsigned int instance, const int component, int *pid){
	int status;
	if(instance >= 4){
		fprintf(stderr, "Invalid instance number.\n");
		return -1;
	}
	if(*pid != 0 && kill(*pid, 0) == 0){
		kill(*pid, SIGTERM);								// Here we send a SIGTERM to the PID specified.
		sleep(component == SM_COMPONENT ? 3 : 1); 			// Wait 3 seconds for SM to die from SIGTERM, only 1 sec for FE
		waitpid((pid_t)*pid, &status, WUNTRACED | WNOHANG);	// Reap the children if they're already dead. Because zombies are bad, mmkay.
		if(!WIFEXITED(status) && kill(*pid, 0) == 0){		// Checks for insubordinate children
			kill(*pid, SIGKILL);							// Order a hit on them
			sleep(component == SM_COMPONENT ? 3 : 1);		// Allow 1-3 seconds for murder
			waitpid((pid_t)*pid, &status, WUNTRACED | WNOHANG);	// Reap
			if(!WIFEXITED(status) && kill(*pid, 0) == 0){		// Report all survivors
				fprintf(stderr, "Failed to kill %s from instance %d\n", component == SM_COMPONENT ? "SM" : "FE", instance);
				return -1;
			}
		}
		*pid = 0;	// Reset PID to 0 if process terminated successfully
		fprintf(stdout, "Stopped instance %d of %s.\n", instance, component == SM_COMPONENT ? "SM" : "FE");
		return 0;
	} else if (kill(*pid, 0) != 0) {
		*pid = 0;	//PID not running or doesn't exist.
	}
	fprintf(stderr, "Instance %d of %s not running.\n", instance, component == SM_COMPONENT ? "SM" : "FE");
	return 0;
}

/**
 * Daemon main loop. Sets up named pipe and captures commands 
 * given through named pipe. 
 *  
 * Can be stopped with "stop" command or with SIGTERM. 
 * 
 * @author gomezchr (3/16/2015)
 * 
 * @return int Returns negative integer on error.
 */
int daemon_main(){
	int fd;
	char *pipe = "/var/run/opafmd";
	char buf[128] = {0};
	unlink(pipe); // cleanup any bad states
	if(mkfifo(pipe, 0660) == -1){
		fprintf(stderr, "Failed to create pipe...\n");
		return(-1);
	}
	while (!doStop){
		if((fd = open(pipe, O_RDONLY)) == -1){
			fprintf(stderr, "Failed to open pipe for reading.\n");
			return(-2);
		}
		while(read(fd, buf, 127) != 0){
			doStop = parseInput(buf);
		}
		close(fd);
	}
	unlink(pipe);		// Destroy pipe
	pthread_exit(0);	// Allows any running background threads to terminate properly.
}

int main(int argc, char* argv[]) {
	char opts[] = "Di:";
	char c;
	int isDaemon = 0;
	char *cmd = NULL, *comp = NULL, inst[2] = {0};
	progName = argv[0];
	if(argc < 2)
		Usage(-1);
	while((c = getopt(argc, argv, opts)) != -1) {
		switch (c) {
		case 'D':
			isDaemon = 1;
			break;
		case 'i':
			memcpy(inst, optarg, 1);
			break;
		case '?':
			if (optopt == 'c')
				fprintf(stderr, "Missing argument for option -%c\n", optopt);
			else if (isprint(optopt))
				fprintf(stderr, "Unknown option -%c\n", optopt);
			Usage(0);
			break;
		}
	}
	if(optind < argc && isDaemon){
		fprintf(stderr, "Cannot start daemon with additional arguments.\n");
		Usage(-2);
	}
	while (optind < argc) {
		char *arg = argv[optind++];
		switch(arg[0]){
		case 's':
			switch(arg[1]){
			case 't':
				if(strcmp(arg, "start"))
					cmd = "stop";
				else
					cmd = "start";
				break;
			case 'm':
				comp = "sm";
				break;
			default:
				fprintf(stderr, "Unknown parameter %s\n", arg);
				Usage(-1);
			}
			break;
		case 'r':
			if (strcmp(arg, "restart")) {
				fprintf(stderr, "Unknown parameter %s\n", arg);
				Usage(-1);
			} else {
				cmd = "restart";
			}
			break;
		case 'f':
			if (strcmp(arg, "fe")) {
				fprintf(stderr, "Unknown parameter %s\n", arg);
				Usage(-1);
			} else {
				comp = "fe";
			}
			break;
		case 'h':
			if (strcmp(arg, "halt")){
				fprintf(stderr, "Unknown parameter %s\n", arg);
				Usage(-1);
			} else {
				if((isDaemon = open("/var/run/opafmd", O_WRONLY)) == -1){	//re-using isDaemon because reasons.
					fprintf(stderr, "Failed to open pipe to daemon.\nMake sure daemon is running and you have permission to speak with it.");
					exit(-2);
				}
				write(isDaemon, "stop", 4);	// write stop command without parameters to named pipe to kill it softly. With our words.
				close(isDaemon);
				exit(0);
			}
		default:
			fprintf(stderr, "Unknown parameter %s\n", arg);
			Usage(-1);
		}
	}
	if(isDaemon){
		// Here we fork the daemon and if successful...
		switch(fork()){
		case 0:
			// daemon inits it's main loop
			return daemon_main();
		case -1:
			fprintf(stderr, "Failed to fork.\n");
			return -1;
		default:
			// and the parent calls the control script to start up defaults.
			sleep(1);
			char *args[] = {"/opt/opafm/bin/opafmctrl.sh", "start", NULL};
			execvp(args[0], args);
			return 0;
		}
	} else {
		char prog[] = "/opt/opafm/bin/opafmctrl.sh";
		if(inst[0] != 0){
			if(comp != NULL)
				execlp(prog, prog, cmd, "-i", inst, comp, NULL);	// Call opafmctl with instance number and component,
			else
				execlp(prog, prog, cmd, "-i", inst, NULL);			// only with instance number,
		} else {
			if(comp != NULL)
				execlp(prog, prog, cmd, comp, NULL);				// only with component,
			else
				execlp(prog, prog, cmd, NULL);						// or without additional arguments.
		}
	}
	return 0;
}
