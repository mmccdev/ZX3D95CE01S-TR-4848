#pragma once
// minutes
#define LOG_INTERVAL_SEC (60 * 5)
#define LOG_COUNT_DAY (60 * 60 * 24) / LOG_INTERVAL_SEC

#define CANVAS_WIDTH  288
#define CANVAS_HEIGHT 112
#define HEAT_LINE_HEIGHT 12

struct watts 
{
    unsigned long long elapsed_usect;
    unsigned long long since70_usect;
    unsigned long long millijouleIn;
    unsigned long long millijouleOu;
};
void savestat(struct watts Watts);
void initstats(void);
short int *getstatsa(int wday);