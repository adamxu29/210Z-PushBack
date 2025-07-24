#include "main.h"
#include "pros/motors.h"
#include "map"

/*
Odometry coord finder: //https://path.jerryio.com/
*/

using namespace Eclipse;

// Translation PID constants: 5, 0.0025, 35, 600
// Rotation PID constants: 4.5, 0, (27 (θ = 45), 30 (θ = 90), 31 (θ = 180))
// Curve PID constants: 5, 0, 30

// Red paths
void Eclipse::Autonomous_Paths::Red::solo_awp()
{
}

void Eclipse::Autonomous_Paths::Red::left_half_awp()
{
   
}

void Eclipse::Autonomous_Paths::Red::right_half_awp()
{
    
}

void Eclipse::Autonomous_Paths::Red::goal_side_rush()
{
    
}

void Eclipse::Autonomous_Paths::Red::ring_side_rush()
{
}

// Blue paths
void Eclipse::Autonomous_Paths::Blue::solo_awp()
{
}

void Eclipse::Autonomous_Paths::Blue::left_half_awp()
{   
}

void Eclipse::Autonomous_Paths::Blue::right_half_awp()
{
}

void Eclipse::Autonomous_Paths::Blue::goal_side_rush()
{
}

void Eclipse::Autonomous_Paths::Blue::ring_side_rush()
{
}

// Misc paths
void Eclipse::Autonomous_Paths::skills()
{

}

void Eclipse::Autonomous_Paths::test()
{   
    drive.set_constants(5, 0, 35, 4.5, 0, 30, 90, 90);
    drive.move_to_point(12, 24, false, false, 2);
}