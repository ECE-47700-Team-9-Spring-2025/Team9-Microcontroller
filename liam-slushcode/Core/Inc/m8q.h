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

// Function prototypes
void M8Q_Init(void);
bool M8Q_ParseNMEA(const char* sentence, GPS_Data* data);
bool M8Q_ParseGPRMC(const char* sentence, GPS_Data* data);
bool M8Q_ParseGPGGA(const char* sentence, GPS_Data* data);

#endif // M8Q_H