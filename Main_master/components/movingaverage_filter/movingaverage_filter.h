/*
https://github.com/sebnil/Moving-Avarage-Filter--Arduino-Library-
*/
#ifndef MovingAverageFilter_h
#define MovingAverageFilter_h

#define MAX_DATA_POINTS 20

typedef struct {
    float values[MAX_DATA_POINTS];
    int k; // k stores the index of the current array read to create a circular memory through the array
    int dataPointsCount;
    float out;
    int i; // just a loop counter
} MovingAverageFilter;

void MovingAverageFilter_begin(MovingAverageFilter* obj, unsigned int newDataPointsCount);
float MovingAverageFilter_process(MovingAverageFilter* obj, float in);

#endif