#include "m8q.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static float ParseLatLong(const char* str, char direction) {
    float degrees = atof(str);
    float decimal_degrees = (int)(degrees / 100);
    float minutes = degrees - (decimal_degrees * 100);
    float result = decimal_degrees + (minutes / 60.0f);
    
    if (direction == 'S' || direction == 'W') {
        result = -result;
    }
    
    return result;
}

static bool ParseTime(const char* time_str, GPS_Data* data) {
    if (strlen(time_str) < 6) return false;
    
    data->hours = (time_str[0] - '0') * 10 + (time_str[1] - '0');
    data->minutes = (time_str[2] - '0') * 10 + (time_str[3] - '0');
    data->seconds = (time_str[4] - '0') * 10 + (time_str[5] - '0');
    
    return true;
}

static bool ParseDate(const char* date_str, GPS_Data* data) {
    if (strlen(date_str) < 6) return false;
    
    data->day = (date_str[0] - '0') * 10 + (date_str[1] - '0');
    data->month = (date_str[2] - '0') * 10 + (date_str[3] - '0');
    data->year = 2000 + (date_str[4] - '0') * 10 + (date_str[5] - '0');
    
    return true;
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