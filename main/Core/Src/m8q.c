#include "m8q.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

GNSSVector calculateGNSSVector(GPS_Data gps_data1, GPS_Data gps_data2) {
    // Earth radius in meters
    const float R = 6371000.0;
    
    // Convert latitude and longitude to radians
    float lat1_rad = gps_data1.latitude * (M_PI / 180.0);
    float lon1_rad = gps_data1.longitude * (M_PI / 180.0);
    float lat2_rad = gps_data2.latitude * (M_PI / 180.0);
    float lon2_rad = gps_data2.longitude * (M_PI / 180.0);
    
    // Calculate differences
    float delta_lat = lat2_rad - lat1_rad;
    float delta_lon = lon2_rad - lon1_rad;
    
    // Calculate distance using haversine formula
    float a = sin(delta_lat/2) * sin(delta_lat/2) +
              cos(lat1_rad) * cos(lat2_rad) * 
              sin(delta_lon/2) * sin(delta_lon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    float distance = R * c; // Distance in meters
    
    // Calculate bearing (direction)
    float y = sin(delta_lon) * cos(lat2_rad);
    float x = cos(lat1_rad) * sin(lat2_rad) - 
              sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon);
    float bearing = atan2(y, x);
    
    // Convert bearing to degrees (0-360)
    bearing = bearing * (180.0 / M_PI);
    if (bearing < 0) {
        bearing += 360.0;
    }
    
    // Calculate vector components (North and East)
    float vector_north = distance * cos(bearing * (M_PI / 180.0));
    float vector_east = distance * sin(bearing * (M_PI / 180.0));
    
    return (GNSSVector){distance, bearing, vector_north, vector_east};
}

bool M8Q_ParseGNRMC(const char* sentence, GPS_Data* data) {
    char *saveptr;
    char *token = strtok_r((char*)sentence, ",", &saveptr);
    int field = 0;
    
    while (token != NULL) {
        switch(field) {
            case 1: // Time
                if (strlen(token) >= 6) {
                    char time[3] = {0};
                    // Hours
                    strncpy(time, token, 2);
                    data->hours = atoi(time);
                    // Minutes
                    strncpy(time, token + 2, 2);
                    data->minutes = atoi(time);
                    // Seconds
                    strncpy(time, token + 4, 2);
                    data->seconds = atoi(time);
                }
                break;
                
            case 2: // Status
                data->fix_valid = (token[0] == 'A');
                break;
                
            case 3: // Latitude
                if (strlen(token) > 0) {
                    data->latitude = atof(token);
                }
                break;
                
            case 4: // N/S
                data->lat_direction = token[0];
                break;
                
            case 5: // Longitude
                if (strlen(token) > 0) {
                    data->longitude = atof(token);
                }
                break;
                
            case 6: // E/W
                data->lon_direction = token[0];
                break;
                
            case 7: // Speed
                if (strlen(token) > 0) {
                    data->speed_knots = atof(token);
                }
                break;
                
            case 8: // Course
                if (strlen(token) > 0) {
                    data->course = atof(token);
                }
                break;
                
            case 9: // Date
                if (strlen(token) >= 6) {
                    char date[3] = {0};
                    // Day
                    strncpy(date, token, 2);
                    data->day = atoi(date);
                    // Month
                    strncpy(date, token + 2, 2);
                    data->month = atoi(date);
                    // Year
                    strncpy(date, token + 4, 2);
                    data->year = 2000 + atoi(date);
                }
                break;
        }
        token = strtok_r(NULL, ",", &saveptr);
        field++;
    }
    
    // Convert DDMM.MMMMM to decimal degrees
    if (data->fix_valid) {
        double lat_degrees = floor(data->latitude / 100);
        double lat_minutes = data->latitude - (lat_degrees * 100);
        data->latitude = lat_degrees + (lat_minutes / 60);
        
        double lon_degrees = floor(data->longitude / 100);
        double lon_minutes = data->longitude - (lon_degrees * 100);
        data->longitude = lon_degrees + (lon_minutes / 60);
        
        // Apply direction
        if (data->lat_direction == 'S') data->latitude = -data->latitude;
        if (data->lon_direction == 'W') data->longitude = -data->longitude;
        
        return true;
    }
    
    return false;
}