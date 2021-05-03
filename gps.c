// C library headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Linux headers
#include <fcntl.h> 	// Contains file controls like O_RDWR
#include <errno.h> 	// Error integer and strerror() function
#include <termios.h> 	// Contains POSIX terminal control definitions
#include <unistd.h> 	// write(), read(), close()
#include <pthread.h> 	// POSIX threading

// project headers
#include "gps.h"

// global variables
volatile speed_t speed = B9600; 
int available_to_write=1;
struct termios tty;
struct gps_data location;							// global buffer for GPS data
pthread_mutex_t mutex =	PTHREAD_MUTEX_INITIALIZER;	// mutex lock
pthread_cond_t  new  = PTHREAD_COND_INITIALIZER;	// conditional variable for producer to signal consumer that it has fetched new reading from receiver
pthread_cond_t  old  = PTHREAD_COND_INITIALIZER;	// conditional variable for consumer to signal producer that it's current reading is old (already written)


int init_comms(char *portname){
	/* Initialize serial communications for GPS 	*/
	/* portname:	linux serial character device  	*/
	/* serial_port:	returned port object 		*/

	int serial_port = open(portname, O_RDWR); 

	if (serial_port < 0) {
    printf("Error %i from open: %s\n", errno, strerror(errno));
	}	

	memset(&tty, 0, sizeof tty);

	// Read in existing settings, and handle any error
	if(tcgetattr(serial_port, &tty) != 0) {
	    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)


	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

	tty.c_lflag &= ~ICANON; // enable non-canonical read
	tty.c_cc[VTIME] = 0;    // no interbyte timeout
	tty.c_cc[VMIN] = 200;	// if nbytes is not immediately satisfied, block for up to 50 char before returning on read()

	// Set in/out baud rate 
	cfsetispeed(&tty, speed);
	cfsetospeed(&tty, speed);

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
	    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}

	return serial_port;

	}

int comms_check_ubxAck(int port){
	
	int i;
	int ret=1;
	unsigned char *p = (unsigned char *) read_buf;
	// Allocate memory for read buffer, set size according to your needs
	char read_buf [256];
	memset(&read_buf, '\0', sizeof(read_buf));
	int num_bytes = read(port, &read_buf, sizeof(read_buf));

	if (num_bytes < 0) {
		printf("Error reading: %s", strerror(errno));
		return ret;
	}

	for (i=0; i < num_bytes ; i++)
	{
		// check for UBX CFG MSG Header AND ACK message CLASS
		if ((p[i] == '\xB5')&&(p[i+1] == '\x62')&&(p[i+2] == '\x05')){
    		printf("[comms_put_ubxCfg] received UBX-HDR 0xB562 CLASS 0x05 with ID 0x%02x\n\r", p[i+3]);
    		ret=0;
			break;
		}
	}
	if (ret){
		printf("[comms_put_ubxCfg] [ERROR] failed to receive UBX-ACK-ACK\n\r");
	}
	return ret;
}

int comms_put_ubxCfg(char *port, unsigned char *message, size_t elems){

	int ret=0;
	int serial_port = init_comms(port);
	usleep(1000);
    tcflush(serial_port, TCIOFLUSH);
	write(serial_port, message, elems);

	if(message[3] !='\x00'){
		ret = comms_check_ubxAck(serial_port); // check for ACK unless message is baud rate change
	}
	else{
		printf("warning, ignoring ack\n\r");
	}

	close(serial_port);

	return ret;
}


void split_nmeaMsg(char *msg, char **str, size_t len, int max_entries)
{

	int j=0;
	int elems=0;
	char delimiter[4]=",";

	printf("[split_nmeaMsg] splitting nmea msg - %s\n\r", msg);
	
	char *ptr = strtok(msg, delimiter); 				// get first substring


	for (int i =0 ; i < max_entries; i++){
		str[i] = calloc(len, sizeof(char));
	}

	while (ptr != NULL){
		
		strncpy(str[j], ptr, len-1);				// copy substring to string array		

  		ptr = strtok(NULL, delimiter);

		j++;
		elems++;
	}
}


char* format_time(char *utc_time)
{
	// format time from UTC - hh:mm:ss
	char* utc = malloc(11*sizeof(char)); // can't pass back pointer to local data (would be wiped)				
	char hour[2];
	char minute[2];
	char second[2];

	if (utc_time != NULL)
	{
		strncpy(hour, utc_time, 2);
		strncpy(minute, (utc_time+2), 2);
		strncpy(second, (utc_time+4), 2);

		int hr =atoi(hour);
		int min=atoi(minute);
		int sec=atoi(second);

		sprintf(utc, "%02d:%02d:%02d", hr, min, sec);
		utc[10] = '\0'; //null termination
		return utc;
	}
	else
	{
		printf("[format_time] error time string was NULL"); 
		return 0;
	}
}	



float convert_latitude(char* lat, char *dir)
{
	// convert latitude coordinates - ((d)dd + (mm.mmmm/60)) (* -1 for W and S)
	float latitude=0;
	char lat_deg[2];
	char lat_min[7];
	strncpy(lat_deg, lat, 2); 	// copy dd
	strncpy(lat_min, (lat+2), 7); 	// copy mm.mmmm

	latitude = atoi(lat_deg) + (strtof(lat_min, NULL)/60);

	if (strcmp(dir, "S") == 0)
		latitude = latitude*-1;
	
	return latitude;
}


float convert_longitude(char* lon, char* dir)
{
	// convert longitude coordinates - (ddd + (mm.mmmm/60)) (* -1 for W and S)
	float longitude=0;
	char lon_deg[3];
	char lon_min[9];
	strncpy(lon_deg, lon, 3); 	// copy ddd
	strncpy(lon_min, (lon+3), 8); 	// copy mm.mmmm

	longitude = atoi(lon_deg) + (strtof(lon_min, NULL)/60);

	if (strcmp(dir, "W") == 0)
		longitude = longitude*-1;
	return longitude;
}


int comms_get_nmea_msg(void *portname, char* nmea_msg, char **str, int entries)
{
	// GPS data fetch function for pthread
	// portname - pointer to serial port string
	// nmea_msg - GNSS msg type to parse UART buffer for
	// str 		- pointer to string array for data to copy to
	// entries  - dimension of string array
	
	int ret=0;
	char *port = (char *) portname;

	printf("[comms_get_nmea_msg] init serial port\n\r");
	
	int serial_port = init_comms(port);
	char read_buf[256];
	memset(&read_buf, '\0', sizeof(read_buf));
	int num_bytes = read(serial_port, &read_buf, sizeof(read_buf)-1);	// read up to second last byte to preserve null termination

	if (num_bytes < 0) {
		printf("[comms_get_nmea_msg] Error reading: %s\n\r", strerror(errno));
		close(serial_port);
	}
	else{
		printf("[comms_get_nmea_msg] received message, looking for: %s\n\r", nmea_msg);
	}

	int length = sizeof(read_buf);
	int offset=0;
	char *ptr;

	// check when NMEA message begins, UART read could start anywhere with multiple messages in receive buffer
	for (int i=0; i < (length-sizeof(nmea_msg)) ; i++)
	{
 		if (strncmp((char*)(read_buf+i), nmea_msg, sizeof(nmea_msg)-1)==0)
		{
			offset=i;
			printf("[comms_get_nmea_msg] message received for %s\n\r", nmea_msg);
			ptr = strtok((char*)read_buf+offset, "\n");
			split_nmeaMsg(ptr, str, sizeof(read_buf), entries);
			ret=1;
			break;
		}
	}

	if(! ret){
		printf("[comms_get_nmea_msg] ERROR - did not receive %s\n\r", nmea_msg);
	}
	
	close(serial_port);
	return ret;
}


void* comms_get_location(void *portname)
{
	
	char* port = (char *) portname;
	char* coordinates_msg = "GNGGA\0";
	char* time_msg = "GNZDA\0";
	int coordinates_fields = 15;						// 15 fields in GNGGA msg
	int time_fields = 7;							// 8 fields in GNZDA msg
	char **coordinate_entries = malloc(coordinates_fields * sizeof(char*)); // allocate memory for 12 substrings of max size 255
	char **time_entries = malloc(time_fields * sizeof(char*)); 		// allocate memory for 12 substrings of max size 255
	char utc_date[21];
	memset(&utc_date, '\0', sizeof(utc_date));
	int ret=1;

	struct gps_data *data = NULL;
	data = malloc(sizeof(struct gps_data)); 				// gps coordinate struct data to be returned to main
	(*data).t[0] = '\0';									// fill as empty to check on return for error condition 


	// fetch GPS coordinates from GNGGA message
	ret &= comms_get_nmea_msg(port, coordinates_msg, coordinate_entries, coordinates_fields); // get sub-string array of nmea msg entries
	
	// fetch UTC time from GNZDA message
	ret &= comms_get_nmea_msg(port, time_msg, time_entries, time_fields); 	// get sub-string array of nmea msg entries

	if(ret)
	{
		char* lat  = coordinate_entries[2]; 				// latitude  - ddmm.mmmmm
		char* ns   = coordinate_entries[3]; 				// N/S
		char* lon  = coordinate_entries[4]; 				// longitude - dddmm.mmmmm
		int   gps_fix= (int) coordinate_entries[6][0];		// gps fix
		char* ew   = coordinate_entries[5]; 				// E/W
		char* alt  = coordinate_entries[9];					// elevation (meters)

		int year = atoi(time_entries[4]);
		int month = atoi(time_entries[3]);
		int day = atoi(time_entries[2]);
		char *utc_time = format_time(time_entries[1]);

		sprintf(utc_date, "%04d-%02d-%02dT%sZ", year, month, day, utc_time);

		printf("[comms_get_location] updated GPS global variables at time %s\n\r", utc_date);

		// fill gps data struct with receiver response
		strncpy((*data).t, utc_date, 21);
		(*data).fix = gps_fix;
		(*data).lat = convert_latitude(lat, ns);
		(*data).lon = convert_longitude(lon, ew);
		(*data).alt = strtof(alt, NULL);
		
		free(utc_time);
	}			
	else{
		printf("[comms_get_location] failed to update GPS global variables\n\r");
	}			

	// release memory
	for (int j = 0; j < coordinates_fields; j++)
	{
	   free(coordinate_entries[j]);
	}
	free(coordinate_entries);

	for (int k = 0; k < time_fields; k++)
	{
	   free(time_entries[k]);
	}
	free(time_entries);

	return ((void *) data);
}


int gpx_fix_LED(int val)
{

	int ret=0;

	char* cmd="gpioset gpiochip1 0=1";
	ret = system(cmd); // mount block device on specified location

	if (ret==0)
	{
		printf("[gpx_fix_LED] set PB0=%d with exit-code %d\n\r", val, ret);
	}
	return ret;
}


void* gpx_fetch(void *portname)
{
	char* port = (char *) portname;
	struct gps_data *receiver_data;

	while(1)
	{	
		pthread_mutex_lock(&mutex);

		while (available_to_write==0){						// check if we can write new data to buffer
			printf("[gpx_fetch] sleeping until buffer is free\n\r");
			pthread_cond_wait(&new, &mutex);				// sleep calling thread if fresh data is still in coordinate buffer
		}

		receiver_data = (struct gps_data*) comms_get_location(port);

		if ((*receiver_data).t != NULL){
			memcpy(&location, receiver_data, sizeof(struct gps_data));
			free(receiver_data);

			if (((location).fix == 49) | ((location).fix == 50))		// only wake writing thread if gps fix present
			{
				available_to_write = 0; 
				gpx_fix_LED((int) 1);
				pthread_cond_signal(&old);				// wake sleeping threads which are waiting for new coordinates
			}
		}	

		pthread_mutex_unlock(&mutex);						// release lock
	}
}


int gpx_fs_mount(char* blk, char* loc)
{

	int ret = 0;
	int attempts;

	char* cmd;
	cmd = calloc(50, sizeof(char));
	sprintf(cmd, "mount -t ext3 %s %s", blk, loc); 

	for (attempts=0; attempts<3; attempts++)
	{
		printf("[gpx_fs_mount] attempting to mount %s on %s...\n\r", blk, loc);
		ret = system(cmd); // mount block device on specified location

		if (ret==0)
		{
			printf("[gps_fs_mount] mounted %s on %s with exit-code %d\n\r", blk, loc, ret);
			break;
		}
	}

	return ret;
}


void gpx_file_init(char *filename)
{

	printf("[gpx_file_init] attempting to initialize '%s'\n\r", filename);
	char* fname = filename;
	FILE *fp;
	fp = fopen(fname, "w");

	if (fp==NULL){
		printf("[gpx_file_init] file %s failed to open!", filename);
	}

	fprintf(fp, 
"<?xml version=\"1.0\" encoding=\"UTF-8\"?> \n\
<gpx version=\"1.1\" creator=\"lightning-gps\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\" \n\
 <trk> \n\
  <name>test</name> \n\
  <type>1</type> \n\
  <trkseg>\n\r");
	fclose(fp);
}


void gpx_write_trk_seg(char *filename)
{

	printf("[gpx_write_trk_seg] attempting to open file for writing\n\r");
	FILE *fp;
	fp = fopen(filename, "a");

	if (fp==NULL){
		printf("file %s failed to open!", filename);
	}

	printf("[gpx_write_trk_seg] attempting to write wpt to file\n\r");

	fprintf(fp, 
"   <trkpt lat=\"%f\" lon=\"%f\">\n\
    <ele>%f</ele>\n\
    <time>%s</time>\n\
    <extensions>\n\
     <gpxtpx:TrackPointExtension>\n\
     </gpxtpx:TrackPointExtension>\n\
    </extensions>\n\
   </trkpt>\n\r", (location).lat, (location).lon, (location).alt, (location).t); 
	fclose(fp);

	printf("[gpx_write_trk_seg] finished writing waypoint to file\n\r");
}


void* gpx_write(void *filename)
{

	char* file = "/media/test.gpx";

	while(1){
		pthread_mutex_lock(&mutex); 						// lock before sleeping to ensure no race condition for child waking parent 
		while(available_to_write==1){						// check if buffer is OK to be written to
			printf("[gpx_write] sleeping until new data is available\n\r");
			pthread_cond_wait(&old, &mutex); 				// block calling thread until producer updates buffer
											// wait releases mutex lock automatically
		}

		printf("[gpx_write] GPS fix aquired with %f %f at %s\n\r", (location).lat, (location).lon, (location).t); // consume data now that is is available
		printf("[gpx_write] attempting to open file for appending\n\r");
		//usleep(10000);		
		gpx_write_trk_seg(file);
		available_to_write=1;							// change state variable now that data has been read
		pthread_cond_signal(&new);						// wake sleeping threads which are waiting for new coordinates
		pthread_mutex_unlock(&mutex); 					// unlock mutex
	}
}


int comms_ubx_configure(char *port)
{

	int ret=0;
	unsigned char gsv_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x03\x00\x00\x00\x00\x00\x00\x02\x38"; // disable NMEA GxGSV messages
	ret |= comms_put_ubxCfg(port, gsv_msg, sizeof(gsv_msg));

	unsigned char gll_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x01\x00\x00\x00\x00\x00\x00\x00\x2A"; // disable NMEA GxGLL messages
	ret |= comms_put_ubxCfg(port, gll_msg, sizeof(gll_msg));

	unsigned char vtg_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x05\x00\x00\x00\x00\x00\x00\x04\x46"; // disable NMEA GxVTG messages
	ret |= comms_put_ubxCfg(port, vtg_msg, sizeof(vtg_msg));

	unsigned char rmc_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x04\x00\x00\x00\x00\x00\x00\x03\x3F"; // disable NMEA GxRMC messages
	ret |= comms_put_ubxCfg(port, rmc_msg, sizeof(rmc_msg));

	unsigned char gsa_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x02\x00\x00\x00\x00\x00\x00\x01\x31"; // disable NMEA GxGSA messages
	ret |= comms_put_ubxCfg(port, gsa_msg, sizeof(gsa_msg));

	unsigned char zda_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x08\x00\x01\x00\x00\x00\x00\x08\x60"; // enable NMEA GxZDA messages
	ret |= comms_put_ubxCfg(port, zda_msg, sizeof(zda_msg));
	
	// set baud rate
	unsigned char prt_msg[] = "\xB5\x62\x06\x00\x14\x00\x01\x00\x00\x00\xD0\x08\x00\x00\x00\x4B\x00\x00\x23\x00\x03\x00\x00\x00\x00\x00\x64\x37"; // enable UART1 19200 baud
	comms_put_ubxCfg(port, prt_msg, sizeof(prt_msg)); // ignore ack

	speed = B19200; 
	
	// set msg rate 
	unsigned char cfg_msg[] = "\xB5\x62\x06\x08\x06\x00\xC8\x00\x01\x00\x01\x00\xDE\x6A"; // set measurement period as 200ms
	ret |= comms_put_ubxCfg(port, cfg_msg, sizeof(cfg_msg));
	printf("[comms_ubx_configure] completed UBX config with status %d\n", ret);

	return ret;
}


int main(void){

	pthread_t location_fetch;
	pthread_t location_consume;
	int status=0;
		
	char *portname  = "/dev/ttySTM1"; 		// USART2 GPS communication device
	char *file_name = "/media/test.gpx"; 	// test file to write to
	comms_ubx_configure(portname);
	char* device = "/dev/mmcblk0p1";
	char* location = "/media";
	
	if (gpx_fs_mount(device, location) != 0)
	{
		printf("[main] WARNING ! failed to mount %s, gpx will be written to RAM\n\r", device);
	}

	gpx_file_init(file_name);
	
	while(1)
	{
		status = pthread_create(&location_fetch, NULL, gpx_fetch, portname);
		printf("[main] attempt to create thread with status=%d\n\r", status);	

		status = pthread_create(&location_consume, NULL, gpx_write, NULL);
		printf("[main] attempt to create thread with status=%d\n\r", status);	

		pthread_join( location_fetch, NULL);
		pthread_join( location_consume,  NULL);
	}
}
