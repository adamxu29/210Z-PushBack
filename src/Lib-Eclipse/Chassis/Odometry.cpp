#include "main.h"
#include "cmath"

using namespace Eclipse;

// global position variables
double robot_x = 0;
double robot_y = 0;
double robot_theta;

// helper functions
void Odom::set_horizontal_tracker_specs(double diameter, double offset){
    odom.horizontal_wheel_diameter = diameter;
    odom.horizontal_wheel_offset = offset;
}

void Odom::set_vertical_tracker_specs(double diameter, double offset){
    odom.vertical_wheel_diameter = diameter;
    odom.vertical_wheel_offset = offset;
}

float Odom::get_horizontal_displacement(){
    return  (((float)horizontal_tracking_wheel.get_position() / 100) * odom.horizontal_wheel_diameter * M_PI / 360);
}

float Odom::get_left_displacement(){
    return (float) (left_drive.get_positions()[2]) * odom.vertical_wheel_diameter * M_PI / 360;
}

float Odom::get_right_displacement(){
    return (float) (right_drive.get_positions()[2] * odom.vertical_wheel_diameter * M_PI / 360);
}

float Odom::get_vertical_displacement(){
    return  (((float)vertical_tracking_wheel.get_position() / 100) * odom.vertical_wheel_diameter * M_PI / 360);
}

double prev_horizontal_displacement = 0;
double prev_vertical_displacement = 0;
double prev_heading = 0;

// Pilons odom implementation: https://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
void Odom::update_position(){
    double horizontal_pos = get_horizontal_displacement();
    double vertical_pos = get_right_displacement();

    double delta_horizontal = horizontal_pos - prev_horizontal_displacement;
    double delta_vertical = vertical_pos - prev_vertical_displacement;

    double heading = util.get_min_angle((90 - util.get_heading())) * M_PI / 180.0; // convert to radians
    double delta_heading = heading - prev_heading;
    
    prev_horizontal_displacement = horizontal_pos;
    prev_vertical_displacement = vertical_pos;
    prev_heading = heading;

    double local_x;
    double local_y;

    if(delta_heading == 0){
        local_x = delta_horizontal;
        local_y = delta_vertical;
    }
    else{
        local_x = (2 * sin(delta_heading / 2) * ((delta_horizontal / delta_heading) + horizontal_wheel_offset));
        local_y = (2 * sin(delta_heading / 2) * ((delta_vertical / delta_heading) + vertical_wheel_offset));
    }

    double avg_heading = prev_heading + (delta_heading / 2);

    odom.x += local_y * cos(avg_heading) + local_x * sin(avg_heading);
    odom.y += local_y * sin(avg_heading) - local_x * cos(avg_heading);

    util.set_robot_position(odom.x, odom.y, util.get_heading());

    robot_theta = heading;

    gui.update_sensors();
    gui.update_temps();
    gui.update_match_checklist();
}

void Odom::update_position_single_vertical(){
    double vertical_pos = get_vertical_displacement();
    double delta_vertical = vertical_pos - prev_vertical_displacement;

    prev_vertical_displacement = vertical_pos;

    double heading = util.get_min_angle(util.get_heading()) * M_PI / 180; // convert to radians
    prev_heading = heading;

    odom.x += delta_vertical * sin(heading);
    odom.y += delta_vertical * cos(heading);

    util.set_robot_position(odom.x, odom.y, util.get_heading());

    gui.update_sensors();
    gui.update_temps();
    gui.update_match_checklist();
}

std::pair<int, int> Eclipse::Odom::get_wall(double heading) {
    // Normalize heading to [0, 360)
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;

    if(heading >= 315 || heading < 45){
        // facing 0°
        return {1, 2}; // right wall, back wall
    }
    else if((heading >= 45 || heading < 135)){
        // facing 90°
        return {2, 3}; // back wall, left wall
    }  
    else if((heading >= 135 || heading < 225)){
        // facing 180°
        return {3, 4}; // left wall, front wall
    }
    else if((heading >= 225 || heading < 315)){
        // facing 270°
        return {4, 1}; // front wall, right wall
    }
    return {0, 0}; // something broken
}


void Odom::distance_sensor_reset(int readings, bool create_task){
    pros::Task task([&]() {
        distance_sensor_reset(readings, false);
        pros::delay(10);
        return;
    });

    while(readings > 0){
        double distance_1 = (right_sensor.get() - right_offset) * mm_to_inches;
        double distance_2 = (back_sensor.get() - back_offset) * mm_to_inches;

        double confidence_1 = right_sensor.get_confidence();
        double confidence_2 = back_sensor.get_confidence();

        right_weightings += confidence_1;
        right_weighted_distance += distance_1 * confidence_1;

        back_weightings += confidence_2;
        back_weighted_distance += distance_2 * confidence_2;

        pros::delay(10);
        readings--;
    }

    double avg_right_distance = right_weighted_distance / right_weightings;
    double avg_back_distance = back_weighted_distance / back_weightings;

    char buffer[300];
    sprintf(buffer, "Prev X: %.2f Y: %.2f", util.get_robot_x(), util.get_robot_y());
    lv_label_set_text(gui.debug_line_5, buffer);

    auto [wall_1, wall_2] = get_wall(util.get_heading());

    switch(wall_1){
        case 1:
            // 0° wall
            util.set_robot_position(util.get_robot_x(), max_y - avg_right_distance, util.get_heading());
            break;
        case 2:
            // 90° wall
            util.set_robot_position(max_x - avg_right_distance, util.get_robot_y(), util.get_heading());
            break;
        case 3:
            // 180° wall
            util.set_robot_position(util.get_robot_x(), min_y + avg_right_distance, util.get_heading());
            break;
        case 4:
            // 270° wall
            util.set_robot_position(min_x + avg_right_distance, util.get_robot_y(), util.get_heading());
            break;
        default:
            break;
    }

    switch(wall_2){
        case 1:
            // 0° wall
            util.set_robot_position(util.get_robot_x(), max_y - avg_right_distance, util.get_heading());
            break;
        case 2:
            // 90° wall
            util.set_robot_position(max_x - avg_right_distance, util.get_robot_y(), util.get_heading());
            break;
        case 3:
            // 180° wall
            util.set_robot_position(util.get_robot_x(), min_y + avg_right_distance, util.get_heading());
            break;
        case 4:
            // 270° wall
            util.set_robot_position(min_x + avg_right_distance, util.get_robot_y(), util.get_heading());
            break;
        default:
            break;
    }

    sprintf(buffer, "New X: %.2f Y: %.2f", util.get_robot_x(), util.get_robot_y());
    lv_label_set_text(gui.debug_line_6, buffer);
 }