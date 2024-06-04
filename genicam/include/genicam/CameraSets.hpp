#pragma once

#include <string>
#include <unordered_map>

namespace camset {
    const std::string c_left   = "192.168.2.253";  // 9 LEFT      70:B3:D5:DD:90:ED
    const std::string c_right  = "192.168.2.251";  // 7 RIGHT     70:B3:D5:DD:90:FD
    const std::string c_center = "192.168.2.252";  // - CENTER    70:B3:D5:DD:90:EF

    const int GAIN = 1;
    const float EXP = 1.2;

    struct Data {
        std::string ip;         // Light IP
        std::string channel;    // Light Channel
        std::string name;       // Camera name
        double exp;             // Exposure
        int gain;               // Gain
        int sw;                 // Switch
        int scpd;               // StreamChannelPacketDelay
        double scftd;           // StreamChannelFrameTransmissionDelay
    };

    std::unordered_map<uint64_t, Data> by_mac = {
        {30853686642969, {c_left,     "7",   "9b",   521 * EXP,    30 * GAIN,   1,   240000,    0 * 80000}},
        {30853686643065, {c_left,     "1",   "9a",   521 * EXP,    30 * GAIN,   1,   240000,    1 * 80000}},
        {30853686646563, {c_left,     "2",   "9c",   521 * EXP,    30 * GAIN,   1,   240000,    2 * 80000}},
        {30853686653149, {c_left,     "5",   "9d",   521 * EXP,    30 * GAIN,   1,   240000,    3 * 80000}},
        {30853686643056, {c_left,     "6",   "9e",   521 * EXP,    30 * GAIN,   2,   240000,    0 * 80000}},
        {30853686646554, {c_right,    "6",   "7a",   521 * EXP,    30 * GAIN,   2,   240000,    1 * 80000}},
        {30853686653140, {c_right,    "3",   "7e",   521 * EXP,    30 * GAIN,   2,   240000,    2 * 80000}},
        {30853686445113, {c_right,    "5",   "7c",   521 * EXP,    30 * GAIN,   2,   240000,    3 * 80000}},
        {30853686646528, {c_right,    "2",   "7d",   521 * EXP,    30 * GAIN,   3,   240000,    0 * 80000}},
        {30853686652294, {c_right,    "1",   "7b",   521 * EXP,    30 * GAIN,   3,   240000,    2 * 80000}},
        {30853686643187, {c_right,    "0",   "1",    2847 * 1.1,   20 * GAIN,   3,   240000,    1 * 80000}},
        {30853686650340, {c_center,   "6",   "4",    528 * 2.8,    30 * GAIN,   3,   240000,    3 * 80000}},
        {30853686646397, {c_center,   "5",   "3",    810 * 2.8,    20 * GAIN,   4,   240000,    0 * 80000}},
        {30853686643152, {c_center,   "1",   "12",   460 * 3.8,    40 * GAIN,   4,   240000,    1 * 80000}},
        {30853686646406, {c_center,   "3",   "10",   867.0,        30 * GAIN,   4,   240000,    2 * 80000}},
        {30853686643161, {c_center,   "2",   "13",   805 * 3.8,    20 * GAIN,   0,   240000,    0 * 80000}}, // directly
        {30853686650366, {c_center,   "7",   "11b",  498.0,        15 * GAIN,   0,   240000,    0 * 80000}}, // directly
        {30853686650497, {c_center,   "7",   "11a",  498.0,        15 * GAIN,   0,   240000,    0 * 80000}}  // directly
    };
}