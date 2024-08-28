/*
https://github.com/sebnil/Moving-Avarage-Filter--Arduino-Library-
*/
#include "movingaverage_filter.h"

void MovingAverageFilter_begin(MovingAverageFilter* obj, unsigned int newDataPointsCount) {
    obj->k = 0; //initialize so that we start to write at index 0
    if (newDataPointsCount < MAX_DATA_POINTS)   obj->dataPointsCount = newDataPointsCount;
    else obj->dataPointsCount = MAX_DATA_POINTS;
    for (obj->i = 0; obj->i < obj->dataPointsCount; obj->i++) obj->values[obj->i] = 0; // fill the array with 0's
}

float MovingAverageFilter_process(MovingAverageFilter* obj, float in) {
    obj->out = 0;
    obj->values[obj->k] = in;
    obj->k = (obj->k + 1) % obj->dataPointsCount;
    for (obj->i = 0; obj->i < obj->dataPointsCount; obj->i++) obj->out += obj->values[obj->i]; 
    return obj->out / obj->dataPointsCount;
}