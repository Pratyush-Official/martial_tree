#pragma once
#include <behaviortree_cpp/blackboard.h>

enum class FLAG_ALTER_CASE{
    GAME_START,
    SPEAR_ASSEMBLED, //alters "spear_picked" flagand "stage1_roi" flag
    SCROLL_PICKED_FROM_ENTRANCE, //alters "scroll_picked" flag and "store" flag
    CLIMBED_THREE_PHOTOS, //alters the "climbed_three_photos" flag and the three "stage_roi" flags and resets in_martial and sets in_forest
    STORE, //alters the "store" flag
    CARRY, //alters the "carry" flag
    RESET_IN_FOREST, //resets "in_forest", sets "in_martial", sets "photo_sequence", resets "climbed_three_photots" flags
    RESET_TO_MARTIAL_FROM_ARENA
};

inline void reset_flags(BT::Blackboard::Ptr bb){
    bb->set<int>("in_martial", 0);
    bb->set<int>("climbed_three_photos", 0);
    bb->set<int>("in_forest", 0);
    bb->set<int>("photo_sequence", 0);
    bb->set<int>("in_arena", 0);
    bb->set<int>("reset_to_martial", 0);
}

inline void flag_alter(BT::Blackboard::Ptr bb, const FLAG_ALTER_CASE case_){
    switch(case_){
        case FLAG_ALTER_CASE::GAME_START:
            reset_flags(bb);
            bb->set<int>("in_martial", 1); 
            break;
        case FLAG_ALTER_CASE::SPEAR_ASSEMBLED:
            reset_flags(bb);
            bb->set<int>("in_martial", 1);
            bb->set<int>("spear_assembled", 1);
            break;
        case FLAG_ALTER_CASE::SCROLL_PICKED_FROM_ENTRANCE:
            reset_flags(bb);
            bb->set<int>("in_martial", 1);
            bb->set<int>("spear_assembled", 1);
            bb->set<int>("scroll_picked_from_entrance", 1);
            bb->set<int>("store", 1);
            break;
        case FLAG_ALTER_CASE::CLIMBED_THREE_PHOTOS:
            reset_flags(bb);
            bb->set<int>("spear_assembled", 1);
            bb->set<int>("climbed_three_photos", 1);
            bb->set<int>("in_forest", 1);
            break;
        case FLAG_ALTER_CASE::STORE:
            bb->set<int>("store", 1);
            break;
        case FLAG_ALTER_CASE::CARRY:
            bb->set<int>("carry", 1);
            break;
        case FLAG_ALTER_CASE::RESET_IN_FOREST:
            reset_flags(bb);
            bb->set<int>("in_martial", 1);
            bb->set<int>("spear_assembled", 1);
            bb->set<int>("photo_sequence", 1);
            break;
        case FLAG_ALTER_CASE::RESET_TO_MARTIAL_FROM_ARENA:
            reset_flags(bb);
            bb->set<int>("in_martial", 1);
            bb->set<int>("spear_assembled", 1);
            bb->set<int>("photo_sequence", 1);
            break;
        default:
            break;
    }
}