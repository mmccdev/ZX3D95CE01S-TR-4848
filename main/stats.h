#pragma once
struct watts 
{
    unsigned long long elapsed_usect;
    unsigned long long since70_usect;
    unsigned long long millijouleIn;
    unsigned long long millijouleOu;
};
void savestat(struct watts Watts);
void liststats();