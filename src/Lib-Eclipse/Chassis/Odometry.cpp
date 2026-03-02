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

void Odom::update_telemetry_fn(void* param){
    while(true){
        odom.update_position();
        pros::delay(10);
    }
}

double Odom::get_robot_x(){ 
    return robot_x;
}
double Odom::get_robot_y(){
    return robot_y;
}
void Odom::set_robot_position(double x, double y){
    robot_x = x;
    robot_y = y;

    odom.x = odom.get_robot_x();
    odom.y = odom.get_robot_y();
}

void Odom::set_robot_heading(double heading){
    imu1.set_rotation(heading);
}


// Pilons odom implementation: https://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
void Odom::update_position(){
    double horizontal_pos = get_horizontal_displacement();
    double vertical_pos = get_vertical_displacement();

    double delta_horizontal = horizontal_pos - prev_horizontal_displacement;
    double delta_vertical = vertical_pos - prev_vertical_displacement;

    double heading = util.get_min_angle((90 - util.get_heading())) * M_PI / 180.0; // convert to radians
    double delta_heading = heading - prev_heading;
    double prev_heading_snapshot = prev_heading;

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

    double avg_heading = prev_heading_snapshot + (delta_heading / 2);

    odom.x += local_y * cos(avg_heading) + local_x * sin(avg_heading);
    odom.y += local_y * sin(avg_heading) - local_x * cos(avg_heading);

    odom.set_robot_position(odom.x, odom.y);
    robot_theta = heading;

    // gui.update_sensors();
    // gui.update_temps();
    // gui.update_match_checklist();
}

void Odom::update_position_single_vertical(){
    double vertical_pos = get_vertical_displacement();
    double delta_vertical = vertical_pos - prev_vertical_displacement;

    prev_vertical_displacement = vertical_pos;

    double heading = util.get_min_angle(util.get_heading()) * M_PI / 180; // convert to radians
    prev_heading = heading;

    odom.x += delta_vertical * sin(heading);
    odom.y += delta_vertical * cos(heading);

    odom.set_robot_position(odom.x, odom.y);
    
    // gui.update_sensors();
    // gui.update_temps();
    // gui.update_match_checklist();
}

int Eclipse::Odom::get_wall(double heading) {
    // Normalize heading to [0, 360)
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;

    if(heading >= 315 || heading < 45){
        // facing 0°
        return 1;
    }
    else if((heading >= 45 && heading < 135)){
        // facing 90°
        return 2;
    }  
    else if((heading >= 135 && heading < 225)){
        // facing 180°
        return 3;
    }
    else if((heading >= 225 && heading < 315)){
        // facing 270°
        return 4;
    }
    return 0;
}

void Odom::distance_sensor_reset(int readings, bool create_task, SensorIndex sensor){
    update_telemetry->suspend();
    if(create_task){
        pros::Task task([=]() {distance_sensor_reset(readings, false, sensor);});
        pros::delay(10);
        update_telemetry->resume();
        return;
    }

    pros::Distance* sensors[3] = {&front_sensor, &left_sensor, &right_sensor};
    double offsets[3] = {front_offset, left_offset, right_offset};
    double heading_offsets[3] = {0.0, -90.0, 90.0};

    int sensor_index = static_cast<int>(sensor);
    double sensor_weightings = 0;
    double sensor_weighted_distance = 0;

    while(readings--){
        int raw_distance = sensors[sensor_index]->get();
        if(raw_distance > 9999){
            pros::delay(10);
            continue;
        }

        double distance = (raw_distance - offsets[sensor_index]) * mm_to_inches;
        double confidence = sensors[sensor_index]->get_confidence();

        sensor_weightings += confidence;
        sensor_weighted_distance += distance * confidence;

        pros::delay(10);
    }

    // No valid samples: keep current pose and skip reset.
    if(sensor_weightings <= 0){
        update_telemetry->resume();
        return;
    }

    double avg_distance = sensor_weighted_distance / sensor_weightings;

    // std::cout << "avg distance: " << avg_distance << std::endl;
    // char buffer[300];
    // sprintf(buffer, "Prev X: %.2f Y: %.2f", odom.get_robot_x(), odom.get_robot_y());
    // lv_label_set_text(gui.debug_line_5, buffer);

    double heading = util.get_heading();

    double corrected_heading = heading + heading_offsets[sensor_index];
    if (corrected_heading < 0) { corrected_heading += 360; }
    else if (corrected_heading > 360) { corrected_heading -= 360; }
    int wall = get_wall(corrected_heading);
    switch(wall){
        case 1:
            avg_distance = avg_distance * cos(corrected_heading * M_PI / 180);
            odom.set_robot_position(odom.get_robot_x(), max_y - avg_distance);
            break;
        case 2:
            avg_distance = avg_distance * cos((corrected_heading - 90) * M_PI / 180);
            odom.set_robot_position(max_x - avg_distance, odom.get_robot_y());
            break;
        case 3:
            avg_distance = avg_distance * cos((corrected_heading - 180) * M_PI / 180);
            odom.set_robot_position(odom.get_robot_x(), min_y + avg_distance);
            break;
        case 4:
            avg_distance = avg_distance * cos((corrected_heading - 270) * M_PI / 180);
            odom.set_robot_position(min_x + avg_distance, odom.get_robot_y());
            break;
        default:
            break;
    }

    // sprintf(buffer, "New X: %.2f Y: %.2f", odom.get_robot_x(), odom.get_robot_y());
    // lv_label_set_text(gui.debug_line_6, buffer);
    // gui.display_debug_terminal();

    odom.x = odom.get_robot_x();
    odom.y = odom.get_robot_y();

    update_telemetry->resume();
    // std::cout << "inside reset" << std::endl << "x: " << odom.get_robot_x() << " y: " << odom.get_robot_y() << "h: " << util.get_heading() << std::endl;
}
