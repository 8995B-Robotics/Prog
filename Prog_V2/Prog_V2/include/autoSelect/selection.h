#pragma once

#include <string>

//selector configuration
#define HUE 275
#define DEFAULT 1
#define AUTONS "Offense", "Defense", "AWP Offense"

namespace selector{

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
