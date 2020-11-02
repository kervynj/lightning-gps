
struct gps_data 
{
	char  t[21];
	int   fix;
	float lat;
	float lon;
	float alt;
};

struct gps_time
{
	char time[21]; 
	int day;
	int month;
	int year;
};