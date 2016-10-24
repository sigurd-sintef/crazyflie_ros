#ifndef COMMONS_H
#define COMMONS_H

int constrain(int a, int b, int c);
int dead_zone(int a, int b);
float constrain_f(float a, float b, float c);
float dead_zone_f(float a, float b);


#define GRAVITY 9810
#define VEHICLE_MASS 30
#define DEG2RAD 0.01745
#define RAD2DEG 57.3

enum enum_mode{AttCtrl, PosCtrl, TrjCtrl};
enum enum_state{TakingOff, Landing, Automatic, Idle};

#endif
