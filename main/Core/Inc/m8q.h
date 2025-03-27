#ifndef M8Q_H
#define M8Q_H

#include <stdint.h>
#include <stdbool.h>

// GPS data structure
typedef struct {
    // Time
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    
    // Date
    uint8_t day;
    uint8_t month;
    uint16_t year;
    
    // Position
    float latitude;
    char lat_direction;  // N or S
    float longitude;
    char lon_direction;  // E or W
    
    // Additional data
    float speed_knots;
    float course;
    bool fix_valid;
} GPS_Data;

// GNSS vector structure
typedef struct {
    float distance;
    float bearing;
    float vector_north;
    float vector_east;
} GNSSVector;

// Function prototypes
bool M8Q_ParseGNRMC(const char* sentence, GPS_Data* data);
GNSSVector calculateGNSSVector(GPS_Data gps_data1, GPS_Data gps_data2);

#endif // M8Q_H