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

	tty.c_cc[VTIME] = 1;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 56;

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
	char read_buf [72];
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
	    		printf("received UBX-HDR 0xB562 CLASS 0x05 with ID 0x%02x\n\r", p[i+3]);
			break;
		}

	}

	close(serial_port);

	return 0;
}


char** split_nmeaMsg(char *msg, char *delimiter, size_t len, int max_entries){

	int j=0;

	char *ptr = strtok(msg, delimiter); 				// get first substring

    	char ** sub_str = malloc(max_entries * sizeof(char*)); 		// allocate memory for 12 substrings of max size 255    	
	for (int i =0 ; i < max_entries; i++)
        	sub_str[i] = malloc((len+1) * sizeof(char));

	while (ptr != NULL){
		
		strcpy(sub_str[j], ptr);				// copy substring to string array		

		//printf("%s\n\r", sub_str[j]);	
      		ptr = strtok(NULL, delimiter);

		j++;
	}

	return sub_str;
}


char* format_time(char *utc_time)
{

	// format time from UTC - hh:mm:ss
	char* utc = malloc(10*sizeof(char*)); // can't pass back pointer to local data (would be wiped)				
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

		if ( hr < 7 ){		// convert 24hr time, UTC is 7hrs ahead of PST
			hr = hr+5; 
		}
		else
			hr = hr-7;

		sprintf(utc, "%02d:%02d:%02d", hr, min, sec);
		return utc;
	}
	else
	{
		printf("error time string was NULL"); 
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
	char lon_min[7];
	strncpy(lon_deg, lon, 3); 	// copy ddd
	strncpy(lon_min, (lon+3), 7); 	// copy mm.mmmm

	longitude = atoi(lon_deg) + (strtof(lon_min, NULL)/60);

	if (strcmp(dir, "W") == 0)
		longitude = longitude*-1;
	return longitude;
}


int comms_get_location(char *port, char *nmea_msg)
{
	
	int i;
	int serial_port = init_comms(port);

	struct gps_data data; //data to be returned to main

	// Allocate memory for read buffer, set size according to your needs
	char read_buf[72];
	memset(&read_buf, '\0', sizeof(read_buf));

	int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

	if (num_bytes < 0) {
		printf("Error reading: %s", strerror(errno));
		close(serial_port);

		return -1;
	}


	char* tempstr = calloc(strlen(read_buf)+1, sizeof(char));
	strcpy(tempstr, read_buf);	
	char *p = (char *) read_buf;
	int entries=12;

	for (i=0; i < strlen(tempstr) ; i++){

		if (p[i] == '$'){ 	// check for NMEA start character $

			char delim[4] = ",";
			char *ptr = strtok(read_buf, delim);
			
			if (strcmp(ptr, nmea_msg) == 0){
				size_t length = strlen(tempstr);
				char **str=split_nmeaMsg(tempstr, delim, length, entries);

				int gps_fix = (int) str[6][0];				// fix is single char 

				if ((gps_fix == 49) | (gps_fix == 50)) 			// check for GNNS or GPS fix, ascii 49="1" or ascii 50="2"
				{

					char* time = str[1]; 				// UTC time
					char* lat  = str[2]; 				// latitude  - ddmm.mmmmm
					char* ns   = str[3]; 				// N/S
					char *lon  = str[4]; 				// longitude - dddmm.mmmmm
					char *ew   = str[5]; 				// E/W
					char *alt  = str[9];				// elevation (meters)

					//char* utc = format_time(time);			// format time as UTC
					//float latitude = convert_latitude(lat, ns);	// convert latitude from NMEA to degrees
					//float longitude = convert_longitude(lon, ew);	// convert longitude from NMEA to degrees 
					//float elevation = strtof(alt, NULL);
					
					//printf("GPS fix aquired with %f %f at %s\n\r", latitude, longitude, utc);	


					// fill gps data struct with receiver response
					data.t   = format_time(time);
					data.lat = convert_latitude(lat, ns);
					data.lon = convert_longitude(lon, ew);
					data.alt = strtof(alt, NULL);					
					
					//free(utc);		
					
				}
				
				else {
				
					printf("waiting for fix..\n\r");	

				}		

				// release memory
				for (int j = 0; j < entries; j++)
				{
				   free(str[j]);
				}
				free(str);
			}	
		}
	}

	free(tempstr);

	close(serial_port);
	return 0;
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

	//unsigned char gga_msg[] = "\xB5\x62\x06\x01\x08\x00\xF0\x00\x00\x00\x00\x00\x00\x00\xFF\x23"; // disable NMEA GxGGA messages
	//comms_put_ubxCfg(port, gga_msg, sizeof(gga_msg));

	// configure constellations for receiver 
	//unsigned char config_msg[] = "\xB5\x62\x06\x3E\x0C\x00\x00\x00\x00\x01\x00\x04\x20\x00\x01\x00\x00\x01\x77\x94"; 
	//comms_put_ubxCfg(port, config_msg, sizeof(config_msg));
	return 0;
}


int main(void){
	
	char *portname = "/dev/ttySTM1"; // USART2 GPS communication device
	comms_ubx_configure(portname);
	char *nmea="$GNGGA";
	
	while (!(comms_get_location(portname, nmea))){
		printf("looping\n\r");
	}
	

}
