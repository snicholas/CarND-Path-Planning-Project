#include "vehicle.h"
#include "math.h"

Vehicle::Vehicle()
{

}


Vehicle::Vehicle(json & j, bool isSelf)
{
    if(isSelf) {
        x = j[1]["x"];
        y = j[1]["y"];
        s = j[1]["s"];
        d = j[1]["d"];
        yaw = j[1]["yaw"];
        yaw_rad = deg2rad(yaw);

        speed = MPH2mps(j[1]["speed"]);

    } else {
        id = j[0];
        x = j[1];
        y = j[2];
        vx = j[3];
        vy = j[4];
        s = j[5];
        d = j[6];
        speed = sqrt(vx*vx+vy*vy);
    }

    lane = (int)(d/4.0);
}


