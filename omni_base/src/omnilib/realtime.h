

#define OMNICOM_MAGIC_VERSION 1003

/* Data we read from the EtherCAT slaves */
typedef struct omniread {
	int16_t  magic_version; // magic number to prevent version clashes
	uint32_t pkg_count;     // Set in omnidrive kernel module


	uint16_t status[4];		//Controller states 
	uint32_t position[4];	//Tick
	
// NO CHANGES BEFORE THIS LINE //////////////////////////////////////////////////////

	int32_t actual_velocity[4]; //Tick/s

	// ethercat states
	int slave_state[4];
	int slave_online[4];
	int slave_operational[4];
	int master_link;
	int master_al_states;
	int master_slaves_responding;
	int working_counter;
	int working_counter_state;
} omniread_t;

/* Data we write to the EtherCAT slaves */
typedef struct omniwrite { 
	int16_t  magic_version;         // magic number to prevent version clashes
	int32_t target_velocity[4];     // Tick/s
} omniwrite_t;

// realtime interface

void omni_write_data(struct omniwrite data);
struct omniread omni_read_data();

int start_omni_realtime(int max_vel);
void stop_omni_realtime();
