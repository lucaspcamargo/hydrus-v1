#pragma once

struct Waypoint
{
    double longitude, latitude;
    double custom_radius;
    bool acquire;
};

typedef std::vector<Waypoint> Waypoints;