#include "m8q.h"
#include <string.h>
#include <stdlib.h>

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

bool M8Q_ParseGPRMC(const char* sentence, GPS_Data* data) {
    char buffer[128];
    char* tokens[15] = {0};
    int token_count = 0;
    
    strncpy(buffer, sentence, sizeof(buffer)-1);
    buffer[sizeof(buffer)-1] = '\0';
    
    // Split sentence into tokens
    char* token = strtok(buffer, ",");
    while (token && token_count < 15) {
        tokens[token_count++] = token;
    }
    
    // Verify it's an RMC sentence
    if (token_count < 12 || strncmp(tokens[0], "$GPRMC", 6) != 0) {
        return false;
    }
    
    // Parse time
    ParseTime(tokens[1], data);
    
    // Parse status
    data->fix_valid = (tokens[2][0] == 'A');
    
    // Parse latitude
    if (strlen(tokens[3]) > 0 && strlen(tokens[4]) > 0) {
        data->latitude = ParseLatLong(tokens[3], tokens[4][0]);
        data->lat_direction = tokens[4][0];
    }
    
    // Parse longitude
    if (strlen(tokens[5]) > 0 && strlen(tokens[6]) > 0) {
        data->longitude = ParseLatLong(tokens[5], tokens[6][0]);
        data->lon_direction = tokens[6][0];
    }
    
    // Parse speed
    if (strlen(tokens[7]) > 0) {
        data->speed_knots = atof(tokens[7]);
    }
    
    // Parse course
    if (strlen(tokens[8]) > 0) {
        data->course = atof(tokens[8]);
    }
    
    // Parse date
    ParseDate(tokens[9], data);
    
    return true;
}