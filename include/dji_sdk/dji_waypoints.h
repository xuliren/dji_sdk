//
// Created by Hao Xu on 15/6/8.
//

#ifndef DJI_SDK_DJI_WAYPOINTS_H
#define DJI_SDK_DJI_WAYPOINTS_H

#include <string>
#include <dji_sdk/global_position.h>
#include <dji_sdk/local_position.h>
#include <dji_sdk/mavlink_connector.h>
#include <vector>
#include <string>
#include <stdio.h>

#define STATUS_STANDBY 1
#define STATUS_FLYWAYPOINT 2
#define STATUS_PAUSE 3

#define ACTION_START_MISSION 1
#define ACTION_PAUSE_MISSION 2
#define ACTION_STOP_MISSION 3
#define ACTION_CONT_MISSION 4
#include <iostream>
#include <dji_mavlink/dji_sdk_onboard/mavlink.h>
#include <vector>
using namespace std;
struct mission
{
    std::vector<dji_sdk::global_position> waypoints;
    std::string name = "";
    int mid = 0;
    dji_sdk::global_position operator[](int id)
    {
        return waypoints[id];
    }
    friend  istream & operator >> (istream & is, mission & ms)
    {
        int num ;
        is >> num;
        double lat,lon,alt,uns;
        for (int i = 0;i< num ; i++) {
            is >> lon >> lat >> alt >> uns;
            dji_sdk::global_position gp;
            gp.lon = lon * M_PI /180.0f;
            gp.lat = lat * M_PI /180.0f;
            gp.alti = alt;
            ms.waypoints.push_back(gp);
        }
        return is;
    }

    friend  ostream & operator << (ostream & os, mission & ms)
    {
        for (int i = 0;i<ms.waypoints.size();i++)
        {
            auto wp = ms.waypoints[i];
            os<<  "id "<< i <<"lon: "<< wp.lon <<" lat:"<< wp.lat <<" alt :"<<wp.alti
                    << "uncertain:" << wp.uncertain <<"\n";
        }
        return os;
    }

    float size()
    {
        return waypoints.size();
    }
    std::vector <mavlink_mission_item_t> to_mission_items()
    {
        std::vector <mavlink_mission_item_t> res;

        for (int i = 0;i<waypoints.size();i++) {
            mavlink_mission_item_t mission_item_t;
            memset(&mission_item_t,0,sizeof(mission_item_t));
            mission_item_t.target_system = 0;
            mission_item_t.target_component = 200;
            mission_item_t.frame = MAV_FRAME_GLOBAL;
            mission_item_t.command = MAV_CMD_NAV_WAYPOINT;
            mission_item_t.current = false;
            mission_item_t.seq = i;
            mission_item_t.x = (waypoints[i].lat*180.0f/M_PI);// * 10000000;
            mission_item_t.y = (waypoints[i].lon*180.0f/M_PI);// * 10000000;
            mission_item_t.z = waypoints[i].alti;

            res.push_back(mission_item_t);
        }
        return res;
    }
    void clear()
    {
        this->waypoints.clear();
    }
};


class dji_waypoints
{
    int switch_status(int status,int action);
    int state_machine[10][10] = {{0}};
public:
    int status = STATUS_STANDBY,mission_id = -1,waypoint_ptr = -1;
    std::vector<mission> missions;
    //Mission always leave for mavlink waypoint protocol

    dji_waypoints(std::string path);
    dji_waypoints();
    int add_waypoints(mission mission1);
    int load(std::string file);
    int begin_fly_waypoints(int id);
    int begin_fly_waypoints(int id,int ptr);
    int pause_flying();
    int cont_flying();
    int loop();
    int continue_mission();
    static bool approach(dji_sdk::global_position wp1, dji_sdk::global_position wp2,float uncertain);
    void init_state_machine();
    int count ()
    {
        return missions.size();
    }
};


#endif //DJI_SDK_DJI_WAYPOINTS_H
