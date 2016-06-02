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
#include <errno.h>

#define SM_COMPONENT 1
#define FE_COMPONENT 2
#define FM_NUM_COMPONENTS 2
#define FM_MAX_INSTANCES 4

#define FM_XML_CONFIG "/etc/sysconfig/opafm.xml"
#define OPAXMLEXTRACT "/opt/opafm/etc/opaxmlextract"
#define OPAXMLEXTRACT_RESTART_PARAMS "-H -e Common.Shared.StartupRetries -e Common.Shared.StartupStableWait -e Common.Sm.StartupRetries -e Common.Sm.StartupStableWait -e Common.Fe.StartupRetries -e Common.Fe.StartupStableWait -e Fm.Shared.StartupRetries -e Fm.Shared.StartupStableWait -e Fm.Sm.StartupRetries -e Fm.Sm.StartupStableWait -e Fm.Fe.StartupRetries -e Fm.Fe.StartupStableWait -X"
#define OPAXMLEXTRACT_INSTANCE_PARAMS "-H -e Fm.Shared.Start -e Fm.Sm.Start -e Fm.Fe.Start -X"
#define OPAXMLEXTRACT_COMMON_PARAMS "-H -e Common.Sm.Start -e Common.Fe.Start -X"

extern char *optarg;
extern int optind;

struct thread_data {
	pthread_t thread_id;
	int component;
	int instance;
	int *pid;
};

struct restartCounter_t {
	struct componentCounter_t {
		time_t lastUpdate;
		unsigned int counter;
	} components[2];
};

struct daemonConfig_t {
	struct globalConfig_t {
		unsigned int defaultFmMaxRestarts[FM_NUM_COMPONENTS];
		unsigned int defaultFmRestartGracePeriod[FM_NUM_COMPONENTS];
	} global;
	struct instanceConfig_t {
		int enabled;
		struct componentConfig_t {
			int enabled;
			unsigned int maxRetries;
			unsigned int retryTimeout;
		} component[FM_NUM_COMPONENTS];
	} instance[FM_MAX_INSTANCES];
} config = {
	{				//config.global
		{5},		//config.global.defaultFmMaxRestart
		{10*60}		//config.global.defaultFmRestartGracePeriod
	},
	{				//config.instance[*]
		{
			0,		//config.instance[*].enabled
			{{0}}   //config.instance[*].component[*]
		}
	}
};

char *progName;
char *nullEnv[] = {0};
int smPID[FM_MAX_INSTANCES] = {0};
int fePID[FM_MAX_INSTANCES] = {0};
struct restartCounter_t instanceRestarts[FM_MAX_INSTANCES] = {};
int doStop = 0;

const char *componentToString(const unsigned int component);
const char *componentToExecName(const unsigned int component);
int *componentToPIDArray(const unsigned int component);
void reloadSMs(void);
void Usage(int err);
int checkRestarts(const unsigned int instance, const unsigned int component);
void sig_handler(int signo);
int parseInput(char *buf);
int spawn(const unsigned int instance, const int component, int *pids);
int pkill(const unsigned int instance, const int component, int *pid);
void updateInstances(void);
void parseRestartConfig(FILE *fd);
void parseInstanceConfig(FILE *fd, int isCommon);
int loadConfig(void);
int daemon_main(void);
void *kill_thread(void *arg);

/**
 * Returns string representation of component.
 * 
 * @param component 
 * 
 * @return const char* 
 */
const char *componentToString(const unsigned int component){
	switch (component){
	case SM_COMPONENT:
		return "SM";
	case FE_COMPONENT:
		return "FE";
	default:
		return "";
	}
}

/**
 * Returns string representation of component's executable.
 * 
 * @param component 
 * 
 * @return const char* 
 */
const char *componentToExecName(const unsigned int component){
	switch (component){
	case SM_COMPONENT:
		return "sm";
	case FE_COMPONENT:
		return "fe";
	default:
		return "";
	}
}

/**
 * Returns associated PID array for component.
 * 
 * @param component 
 * 
 * @return int* 
 */
int *componentToPIDArray(const unsigned int component){
	switch (component){
	case SM_COMPONENT:
		return smPID;
	case FE_COMPONENT:
		return fePID;
	default:
		return NULL;
	}
}

/**
 * Sends SIGHUP to all running SM instances.
 */
void reloadSMs(void){
	int inst = 0;
	for (inst = 0; inst < FM_MAX_INSTANCES; ++inst){
		if (smPID[inst])
			kill(smPID[inst], SIGHUP);
	}
}

/**
 * Print program usage information
 * 
 * @param err Program exit code
 */
void Usage(int err){
	fprintf(stderr, "Usage: %s -D\n", progName);
	fprintf(stderr, "       %s [-i instance] [sm|fe] start|stop|restart|halt\n", progName);
	exit(err);
}

/**
 * Checks restart counter specified to see if the counter has 
 * exceeded the threshold over a the course of it's grace 
 * period. 
 * 
 * @param instance 
 * @param component 
 * 
 * @return int 0 if counter has not surpassed threshold, 1 
 *  	   otherwise
 */
int checkRestarts(const unsigned int instance, const unsigned int component){
	struct componentCounter_t *count = &instanceRestarts[instance].components[component - 1];
	int retryTimeout = config.instance[instance].component[component - 1].retryTimeout;
	int maxRetries = config.instance[instance].component[component - 1].maxRetries;
	time_t now = time(NULL);

	if ((now - count->lastUpdate) > (retryTimeout ? retryTimeout : config.global.defaultFmRestartGracePeriod[component - 1])){
		count->counter = 1;
	} else {
		count->counter += 1;
	}
	count->lastUpdate = (1 > 0 ? now : count->lastUpdate);
	return count->counter > (maxRetries ? maxRetries : config.global.defaultFmMaxRestarts[component - 1]) ? 1 : 0;
}

/**
 * Handle Linux signals, specifically SIGTERM, SIGHUP and 
 * SIGCHLD. 
 * 
 * @param signo Caught signal
 */
void sig_handler(int signo){
	int i = 0, pid = 0, stat = 0;
	unsigned int instance = 0, component = 0;
	if(signo == SIGTERM){
		doStop = 1;
	} else if (signo == SIGHUP){
		loadConfig();
		updateInstances();
		reloadSMs();
	} else if (signo == SIGCHLD){
		if(doStop)
			return;
		pid = wait(&stat);
		if (pid == 0 || stat == 0)
			return;
		for(i = 0; i < 4; ++i){
			if(smPID[i] == pid){
				component = SM_COMPONENT;
				instance = i;
				break;
			} else if (fePID[i] == pid){
				component = FE_COMPONENT;
				instance = i;
				break;
			}
		}
		if(component == 0){
			return; //Failed to find child in PID table, bad signal or aborted child.
		}
		if(checkRestarts(instance, component)){
			//Component has failed 5 times.
			doStop = 2;
			return;
		}
		if (pkill(instance, component, &((componentToPIDArray(component))[instance])) == 0){
			spawn(instance, component, componentToPIDArray(component));
		} else {
			//Failed to kill child. Might be a zombie.
			doStop = 3;
		}
	}
	return;   /// add file
}

/**
 * pthread function to kill an instance of an FM component.
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
			threads = calloc(FM_NUM_COMPONENTS, sizeof(struct thread_data));
			if(threads == NULL){
				fprintf(stderr, "Failed to allocate thread struct.\n");
				return 0;
			}
			pthread_attr_init(&attr);
			pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
			for(c = 0; c < FM_NUM_COMPONENTS; ++c){
				threads[c].instance = i;
				threads[c].component = c + 1;
				threads[c].pid = &((componentToPIDArray(c + 1))[i]);
				pthread_create(&threads[c].thread_id, &attr, &kill_thread, &threads[c]);
			}
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
			pkill(i, c, componentToPIDArray(c));
			break;
		case 'a':	// Kill all running instances.
			threads = calloc(FM_MAX_INSTANCES * FM_NUM_COMPONENTS, sizeof(struct thread_data));
			if(threads == NULL){
				fprintf(stderr, "Failed to allocate thread struct.\n");
				return 0;
			}
			pthread_attr_init(&attr);
			pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
			for(i = 0; i < FM_MAX_INSTANCES; ++i){
				for(c = 0; c < FM_NUM_COMPONENTS; ++c){
					threads[(i * FM_NUM_COMPONENTS) + c].instance = i;
					threads[(i * FM_NUM_COMPONENTS) + c].component = c + 1;
					threads[(i * FM_NUM_COMPONENTS) + c].pid = &((componentToPIDArray(c + 1))[i]);
					pthread_create(&threads[(i * FM_NUM_COMPONENTS) + c].thread_id, &attr, &kill_thread, &threads[(i * FM_NUM_COMPONENTS) + c]);
				}
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
			spawn(i, SM_COMPONENT, smPID);
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
			spawn(i, c, componentToPIDArray(c));
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
			if(pkill(i, c, componentToPIDArray(c)) == 0)
				spawn(i, c, componentToPIDArray(c));
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
	if (instance >= 4){
		fprintf(stderr, "Invalid instance number.\n");
		return -1;
	}
	if (pids[instance] != 0 && kill(pids[instance], 0) == 0){
		fprintf(stderr, "Instance %d of %s is already running.\n", instance, componentToString(component));
		return -1;
	}
	if (!config.instance[instance].component[component - 1].enabled){
		fprintf(stderr, "Instance %d of %s is not enabled. Please enable instance in %s\n", instance, componentToString(component), FM_XML_CONFIG);
		return -1;
	}
	switch(pid = fork()){
	case 0:
		sprintf(prog, "/opt/opafm/runtime/%s", componentToExecName(component));
		sprintf(name, "%s_%d", componentToExecName(component), instance);
		execle(prog, prog, "-e", name, NULL, nullEnv);
		return 0;
	case -1:
		fprintf(stderr, "Failed to start %s for instance %d.\n", componentToString(component), instance);
		return -1;
	default:
		fprintf(stdout, "Started instance %d of %s.\n", instance, componentToString(component));
		if(pid > 0) pids[instance] = pid;
		return pid;
	}
}

/**
 * Kills instance of FM component 
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
				fprintf(stderr, "Failed to kill %s from instance %d\n", componentToString(component), instance);
				return -1;
			}
		}
		*pid = 0;	// Reset PID to 0 if process terminated successfully
		instanceRestarts[instance].components[component - 1].lastUpdate = 0;	// Make sure to reset restart timer to prevent manual stop/restart from counting against component
		fprintf(stdout, "Stopped instance %d of %s.\n", instance, componentToString(component));
		return 0;
	} else if (*pid != 0) {
		*pid = 0;	//PID not running or doesn't exist.
	}
	fprintf(stderr, "Instance %d of %s not running.\n", instance, componentToString(component));
	return 0;
}

/**
 * Parse extracted XML config values for instance parameters
 * 
 * @param pd IO stream to extracted XML values.
 */
void parseInstanceConfig(FILE *pd, int isCommon){
	char buf[512] = {0};
	char *token = NULL;
	int val = 0, inst = 0, comp = 0;
	if(isCommon){
		while(fscanf(pd, "%s", buf) > 0){
			token = strtok(buf, ";");
                        if (token == NULL)
                                break;
                        val = atoi(token);
			switch (token - buf){
			case 0:
				config.instance[0].component[SM_COMPONENT - 1].enabled = val;
				config.instance[1].component[SM_COMPONENT - 1].enabled = val;
				config.instance[2].component[SM_COMPONENT - 1].enabled = val;
				config.instance[3].component[SM_COMPONENT - 1].enabled = val;
				token = strtok(NULL, ";");
				break;
                        case 1:
				config.instance[0].component[FE_COMPONENT - 1].enabled = val;
				config.instance[1].component[FE_COMPONENT - 1].enabled = val;
				config.instance[2].component[FE_COMPONENT - 1].enabled = val;
				config.instance[3].component[FE_COMPONENT - 1].enabled = val;
				token = strtok(NULL, ";");
				break;
			}
		}
	} else {
		while(fscanf(pd, "%s", buf) > 0){
			token = strtok(buf, ";");
			if (token == NULL)
				break;
			val = atoi(token);
			switch (val){
			case 0:
				config.instance[inst].enabled = 0;
				break;
			case 1:
				config.instance[inst].enabled = 1;
				token = strtok(NULL, ";");
				if (token == NULL){
					break;
				} else {
 					val = atoi(token);
					comp = (token - buf) - 2;
 					config.instance[inst].component[comp++].enabled = val;
					while ((token = strtok(NULL, ";")) != NULL){
						val = atoi(token);
						config.instance[inst].component[comp++].enabled = val;
					}
 				}
 			}
			++inst;
 		}
	}
}

/**
 * Checks config and starts/stop instances according to config.
 */
void updateInstances(void) {
	int inst, comp;
	for (inst = 0; inst < FM_MAX_INSTANCES; ++inst){
		if (config.instance[inst].enabled){
			for (comp = 1; comp < FM_NUM_COMPONENTS + 1; ++comp){
				if (config.instance[inst].component[comp - 1].enabled){
					if (!(componentToPIDArray(comp))[inst])
						spawn(inst, comp, componentToPIDArray(comp));
				} else {
					if ((componentToPIDArray(comp))[inst])
						pkill(inst, comp, &((componentToPIDArray(comp))[inst]));
				}
			}
		} else {
			for (comp = 1; comp < FM_NUM_COMPONENTS + 1; ++comp){
				if ((componentToPIDArray(comp))[inst])
					pkill(inst, comp, &((componentToPIDArray(comp))[inst]));
			}
		}
	}
}

/**
 * Parse extracted XML config values for automatic restart 
 * parameters. 
 *
 * @param pd IO stream to extracted XML values.
 */
void parseRestartConfig(FILE *pd){
	char buf[512] = {0};
	char *token = NULL;
	int cnt = 0, instSm = 0, instFe = 0, val = 0;
	while(fscanf(pd, "%s", buf) > 0){
		token = strtok(buf, ";");
		if (token == NULL)
			break;
		cnt = token - buf;
		switch(cnt){ //The following case statements are based on the number of leading ';'
		case 0://Common.Shared.StartupRetries & Common.Shared.StartupStableWait
			val = atoi(token);
			config.global.defaultFmMaxRestarts[0] = val;
			config.global.defaultFmMaxRestarts[1] = val;
			token = strtok(NULL, ";");
			if (token == NULL)
				break;
			val = atoi(token);
			config.global.defaultFmRestartGracePeriod[0] = val * 60;
			config.global.defaultFmRestartGracePeriod[1] = val * 60;
			break;
		case 2://Common.Sm.StartupRetries & Common.Sm.StartupStableWait
			val = atoi(token);
			config.global.defaultFmMaxRestarts[0] = val;
			token = strtok(NULL, ";");
			if (token == NULL)
				break;
			val = atoi(token);
			config.global.defaultFmRestartGracePeriod[0] = val * 60;
			break;
		case 4://Common.Fe.StartupRetries & Common.Fe.StartupStableWait
			val = atoi(token);
			config.global.defaultFmMaxRestarts[1] = val;
			token = strtok(NULL, ";");
			if (token == NULL)
				break;
			val = atoi(token);
			config.global.defaultFmRestartGracePeriod[1] = val * 60;
			break;
		case 6://Fm.Shared.StartupRetries & Fm.Shared.StartupStableWait
			val = atoi(token);
			config.instance[instSm].component[0].maxRetries = val;
			config.instance[instFe].component[1].maxRetries = val;
			token = strtok(NULL, ";");
			if (token == NULL)
				break;
			val = atoi(token);
			config.instance[instSm].component[0].retryTimeout = val * 60;
			config.instance[instFe].component[1].retryTimeout = val * 60;
			break;
		case 8://Fm.Sm.StartupRetries & Fm.Sm.StartupStableWait
			val = atoi(token);
			config.instance[instSm].component[0].maxRetries = val;
			token = strtok(NULL, ";");
			if (token == NULL)
				break;
			val = atoi(token);
			config.instance[instSm++].component[0].retryTimeout = val * 60;
			break;
		case 10://Fm.Fe.StartupRetries & Fm.Fe.StartupStableWait
			val = atoi(token);
			config.instance[instFe].component[1].maxRetries = val;
			token = strtok(NULL, ";");
			if (token == NULL)
				break;
			val = atoi(token);
			config.instance[instFe++].component[1].retryTimeout = val * 60;
			break;
		}
	}
}

/**
 * Load up xml parser and extract values 
 * 
 * @return int 0 on success.
 */
int loadConfig(void){
	char buf[512] = {0};
	FILE *pd = NULL;
	snprintf(buf, 512, "%s %s %s", OPAXMLEXTRACT, OPAXMLEXTRACT_RESTART_PARAMS, FM_XML_CONFIG);
	pd = popen(buf, "r");
	if(!pd) {
		return -1;
	} else {
		parseRestartConfig(pd);
		pclose(pd);
	}
	snprintf(buf, 512, "%s %s %s", OPAXMLEXTRACT, OPAXMLEXTRACT_COMMON_PARAMS, FM_XML_CONFIG);
	pd = popen(buf, "r");
	if(!pd) {
		return -1;
	} else {
		parseInstanceConfig(pd, 1);
		pclose(pd);
	}
        snprintf(buf, 512, "%s %s %s", OPAXMLEXTRACT, OPAXMLEXTRACT_INSTANCE_PARAMS, FM_XML_CONFIG);
        pd = popen(buf, "r");
        if(!pd) {
                return -1;
        } else {
		parseInstanceConfig(pd, 0);
                pclose(pd);
        }
	return 0;
}

/**
 * Daemon main loop. Sets up named pipe and captures commands 
 * given through named pipe. 
 *  
 * Can be stopped with "stop" command or with SIGTERM. 
 * 
 * @return int Returns negative integer on error.
 */
int daemon_main(){
	int fd, stop = 0;
	char *pipe = "/var/run/opafmd";
	char buf[128] = {0};
	struct sigaction act;

	bzero(&act, sizeof(act));
	act.sa_handler = sig_handler;
	sigaction(SIGTERM, &act, NULL);
	sigaction(SIGCHLD, &act, NULL);
	sigaction(SIGHUP, &act, NULL);



	if(loadConfig()){
		fprintf(stderr, "Failed to load conf, continuing with defaults.");
	}
	updateInstances();

	unlink(pipe); // cleanup any bad states
	if (mkfifo(pipe, 0660) == -1){
		fprintf(stderr, "Failed to create pipe: %s\n", strerror(errno));
		return(-1);
	}
	if((fd = open(pipe, O_RDWR)) == -1){
		fprintf(stderr, "Failed to open pipe for reading: %s\n", strerror(errno));
		unlink(pipe); // cleanup any bad states
		return(-2);
	}


	while(!doStop && !stop){
		if (read(fd,buf,127) != 0){
				stop = parseInput(buf);
		}
	}
	close(fd);
	doStop = -((doStop | stop) - 1);	// Use doStop value as return value. Allows systemd to show daemon status on failure.
	unlink(pipe);			// Destroy pipe
	pthread_exit(&doStop);	// Allows any running background threads to terminate properly.
}

int main(int argc, char* argv[]) {
	char opts[] = "Di:";
	char c;
	int isDaemon = 0, res = 0;
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
				    int fd;
				    if((fd = open("/var/run/opafmd", O_WRONLY|O_NONBLOCK)) == -1){
					  fprintf(stderr, "Failed to open pipe to daemon: %s\nMake sure daemon is running and you have permission to speak with it.\n", strerror(errno));
					  exit(-2);
				    }
				res = write(fd, "stop", 4);	// write stop command without parameters to named pipe to kill it softly. With our words.
				if(res <= 0){
					//Something went wrong while writing to pipe.
					fprintf(stderr, "Failed to send stop command to daemon: %s\n", strerror(errno));
					close(fd);
					exit(-2);
				}
				close(fd);
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
			return 0;
		}
	} else {
		char prog[] = "/opt/opafm/bin/opafmctrl";
		if(inst[0] != 0){
			if(comp != NULL)
				execle(prog, prog, cmd, "-i", inst, comp, NULL, nullEnv);	// Call opafmctl with instance number and component,
			else
				execle(prog, prog, cmd, "-i", inst, NULL, nullEnv);			// only with instance number,
		} else {
			if(comp != NULL)
				execle(prog, prog, cmd, comp, NULL, nullEnv);				// only with component,
			else
				execle(prog, prog, cmd, NULL, nullEnv);						// or without additional arguments.
		}
	}
	return 0;
}
