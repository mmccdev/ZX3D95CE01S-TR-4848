#pragma once
// minutes
#define LOG_INTERVAL_SEC (60 * 5)
#define LOG_COUNT_DAY (60 * 60 * 24) / LOG_INTERVAL_SEC
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