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
int available_to_write=1;
struct gps_data location;				// global buffer for GPS data
pthread_mutex_t mutex =	PTHREAD_MUTEX_INITIALIZER;	// mutex lock
pthread_cond_t  new  = PTHREAD_COND_INITIALIZER;	// conditional variable for producer to signal consumer that it has fetched new reading from receiver
pthread_cond_t  old  = PTHREAD_COND_INITIALIZER;	// conditional variable for consumer to signal producer that it's current reading is old (already written)


int init_comms(char *portname){
	/* Initialize serial communications for GPS 	*/
	/* portname:	linux serial character device  	*/
	/* serial_port:	returned port object 		*/

	int serial_port = open(portname, O_RDWR); 

	// Create new termios struc, we call it 'tty' for convention
	struct termios tty;
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

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 3;    // Wait for up to 10 deciseconds with line idle
	tty.c_cc[VMIN] = 150;	// if nbytes is not immediately satisfied, block for up to 50 char before returning on read()

	// Set in/out baud rate to be 9600
	cfsetispeed(&tty, B9600);
	cfsetospeed(&tty, B9600);

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
	    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}

	return serial_port;

	}


int comms_put_ubxCfg(char *port, unsigned char *message, size_t elems){


	int serial_port = init_comms(port);

	write(serial_port, message, elems);

	// Allocate memory for read buffer, set size according to your needs
	char read_buf [510];
	memset(&read_buf, '\0', sizeof(read_buf));

	int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

	if (num_bytes < 0) {
		printf("Error reading: %s", strerror(errno));
		close(serial_port);

		return -1;
	}
	
	int i;
	unsigned char *p = (unsigned char *) read_buf;
	
	for (i=0; i < 72 ; i++)
	{
		// check for UBX CFG MSG Header AND ACK message CLASS
		if ((p[i] == '\xB5')&&(p[i+1] == '\x62')&&(p[i+2] == '\x05')){
	    		printf("[comms_put_ubxCfg] received UBX-HDR 0xB562 CLASS 0x05 with ID 0x%02x\n\r", p[i+3]);
			break;
		}

	}

	close(serial_port);

	return 0;
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

		//printf("%s\n\r", str[j]);	
  		ptr = strtok(NULL, delimiter);

		j++;
		elems++;
	}

	//printf("%d\n\r", elems);
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

		//if ( hr < 7 ){		// convert 24hr time, UTC is 7hrs ahead of PST
		//	hr = hr+5; 
		//}
		//else
		//	hr = hr-7;

		sprintf(utc, "%02d:%02d:%02d\0", hr, min, sec);
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


void comms_get_nmea_msg(void *portname, char* nmea_msg, char **str, int entries)
{
	// GPS data fetch function for pthread
	// portname - pointer to serial port string
	// 

	char *port = (char *) portname;

	printf("[comms_get_nmea_msg] init serial port\n\r");
	
	int serial_port = init_comms(port);

	// Allocate memory for read buffer, set size according to your needs
	char read_buf[255];
	memset(&read_buf, '\0', sizeof(read_buf));
	int num_bytes = read(serial_port, &read_buf, sizeof(read_buf)-1);	// read up to second last byte to preserve null termination

	if (num_bytes < 0) {
		printf("[comms_get_nmea_msg] Error reading: %s\n\r", strerror(errno));
		close(serial_port);
	}
	else{
		printf("[comms_get_nmea_msg] received message, looking for: %s\n\r", nmea_msg);
	}

	char* tempstr = (char *) calloc(sizeof(read_buf), sizeof(char));	// fill with NULL
	//char tempstr[sizeof(read_buf)];
	//memset(&tempstr, '\0', sizeof(read_buf));
	char delim[] = "$\n";
	int length = sizeof(read_buf);
	int offset=0;

	// check when NMEA message begins, UART read could start anywhere with multiple messages in receive buffer
	for (int i=0; i < length ; i++)
	{
		if (read_buf[i]=='$')
		{
			offset=i;
			//printf("offset=%d\n\r", offset);
			break;
		}
	}


	strncpy(tempstr, read_buf + offset, (length)-offset);
	
	//printf("%s\n\r", tempstr);
	//strncpy(tempstr, read_buf, sizeof(read_buf)-1);
	char found_msg[sizeof(read_buf)];
	memset(&found_msg, '\0', sizeof(found_msg));
	char *ptr = strtok(tempstr, delim);	// tokenize to first $
	char nmea_id[6];
	memset(&nmea_id, '\0', sizeof(nmea_id));

	while (ptr !=NULL){

		//printf("'%s'\n\r", ptr);
		strncpy(nmea_id, ptr, sizeof(nmea_id)-1); // preserve null terminator
		//printf("%s\n\r", nmea_id);

		if(strcmp(nmea_id, nmea_msg)==0){
			printf("[comms_get_nmea_msg] message received for %s\n\r", nmea_id);
			strncpy(found_msg, ptr, sizeof(found_msg)-1); // preserve null terminator
			split_nmeaMsg(found_msg, str, sizeof(read_buf), entries);
			break;
		}

		ptr = strtok(NULL, delim);
	}

	if(found_msg[0]=='\0'){
		printf("[comms_get_nmea_msg] ERROR - no message received for %s\n\r", nmea_id);
	}
	
	free(tempstr);
	close(serial_port);
}


void* comms_get_location(void *portname)
{
	
	char* port = (char *) portname;
	char* coordinates_msg = "GNGGA\0";
	char* time_msg = "GNZDA\0";
	int coordinates_fields = 15;	// 15 fields in GNGGA msg
	int time_fields = 7;			// 8 fields in GNZDA msg
	struct gps_data *data = NULL;
	data = malloc(sizeof(struct gps_data)); // gps coordinate struct data to be returned to main
	char **coordinate_entries = malloc(coordinates_fields * sizeof(char*)); 		// allocate memory for 12 substrings of max size 255
	char **time_entries = malloc(time_fields * sizeof(char*)); 		// allocate memory for 12 substrings of max size 255
	char utc_date[21];
	memset(&utc_date, '\0', sizeof(utc_date));

	// fetch GPS coordinates from GNGGA message
	comms_get_nmea_msg(port, coordinates_msg, coordinate_entries, coordinates_fields); // get sub-string array of nmea msg entries

	char* lat  = coordinate_entries[2]; 				// latitude  - ddmm.mmmmm
	char* ns   = coordinate_entries[3]; 				// N/S
	char* lon  = coordinate_entries[4]; 				// longitude - dddmm.mmmmm
	int   gps_fix= (int) coordinate_entries[6][0];		// gps fix
	char* ew   = coordinate_entries[5]; 				// E/W
	char* alt  = coordinate_entries[9];				// elevation (meters)

	
	// fetch UTC time from GNZDA message
	comms_get_nmea_msg(port, time_msg, time_entries, time_fields); // get sub-string array of nmea msg entries

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
	free(utc_time);

	return ((void *) data);
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

		if (receiver_data != NULL){
			memcpy(&location, receiver_data, sizeof(struct gps_data));
			free(receiver_data);

			if (((location).fix == 49) | ((location).fix == 50))		// only wake writing thread if gps fix present
			{
				available_to_write = 0; 
				pthread_cond_signal(&old);				// wake sleeping threads which are waiting for new coordinates
			}
		}	

		pthread_mutex_unlock(&mutex);						// release lock
	}
}


void gpx_file_init(char *filename)
{

	printf("[gpx_file_init] attempting to initialize '%s'", filename);
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

	// disable default NMEA messages from receiver
	unsigned char gsv_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x03\x00\x00\x00\x00\x00\x00\x02\x38"; // disable NMEA GxGSV messages
	comms_put_ubxCfg(port, gsv_msg, sizeof(gsv_msg));

	unsigned char gll_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x01\x00\x00\x00\x00\x00\x00\x00\x2A"; // disable NMEA GxGLL messages
	comms_put_ubxCfg(port, gll_msg, sizeof(gll_msg));

	unsigned char vtg_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x05\x00\x00\x00\x00\x00\x00\x04\x46"; // disable NMEA GxVTG messages
	comms_put_ubxCfg(port, vtg_msg, sizeof(vtg_msg));

	unsigned char rmc_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x04\x00\x00\x00\x00\x00\x00\x03\x3F"; // disable NMEA GxRMC messages
	comms_put_ubxCfg(port, rmc_msg, sizeof(rmc_msg));

	unsigned char gsa_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x02\x00\x00\x00\x00\x00\x00\x01\x31"; // disable NMEA GxGSA messages
	comms_put_ubxCfg(port, gsa_msg, sizeof(gsa_msg));

	unsigned char zda_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x08\x00\x01\x00\x00\x00\x00\x08\x60"; // enable NMEA GxZDA messages
	comms_put_ubxCfg(port, zda_msg, sizeof(zda_msg));

	return 0;
}


int main(void){

	pthread_t location_fetch;
	pthread_t location_consume;
	int status=0;
		
	char *portname  = "/dev/ttySTM1"; 	// USART2 GPS communication device
	char *file_name = "/media/test.gpx"; 	// test file to write to
	comms_ubx_configure(portname);
	comms_ubx_configure(portname);

	gpx_file_init(file_name);
	//gpx_file_write_location(file_name);
	
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
