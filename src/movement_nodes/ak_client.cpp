#include "movement_nodes/ak_client.hpp"

AkClient::AkClient(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    client_c_down = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_up_execute"); //c section applies normal force
    client_c_up = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_down_exec"); //body "applies" normal force

    client_roll_forward = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_roll_down"); //rolls robot forwards
    client_roll_backward = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_roll_up"); //rolls robot forwards

    client_roll = node_->create_client<motor_interfaces::srv::MotorState>("ak_motor_roll_up");

    rear_pub = node_->create_publisher<std_msgs::msg::Byte>("/rear_drive", cmd_vel_qos());
    prox_sub = node_->create_subscription<std_msgs::msg::Byte>(
        "/prox_data", sensor_qos(), std::bind(&AkClient::prox_callback, this, std::placeholders::_1));

    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
    
    request_sent_climb = false;
    request_sent_roll = false;
    rear_status_forward = false;
    rear_status_reverse = false;
    ran = false;

    prox_data = 0x00;
    movement_state = Movement::IDLE;
    actuation_state = Actuation::IDLE;

    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;
    
    // front_both_triggered = 0x03;
    // all_proximity_triggeded = 0x07;

    RCLCPP_INFO(node_->get_logger(),"AkClient initialized");
}

BT::PortsList AkClient::providedPorts()
{
    return {
        BT::InputPort<bool>("up", "Send true to raise the mechanism"),
        BT::InputPort<bool>("down", "Send true to lower the mechanism")
    };
}

BT::NodeStatus AkClient::onStart()
{
    active_client_climb.reset();
    active_client_roll.reset();      

    request_sent_climb = false;
    request_sent_roll = false;

    service_done_climb = false;
    service_done_roll = false;  

    service_success_climb = false;
    service_success_roll = false; 

    rear_status_forward = false;     
    rear_status_reverse = false;     

    ran = false;

    movement_state = Movement::IDLE;
    actuation_state = Actuation::IDLE;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AkClient::onRunning()
{
    auto up_res = getInput<bool>("up");
    auto down_res = getInput<bool>("down");
    bool up = up_res.value_or(false);
    bool down = down_res.value_or(false);

    if(!ran && movement_state == Movement::IDLE && (up && !down)) {
        movement_state = Movement::UP;
        actuation_state = Actuation::C_DOWN;
    }
    
    else if(!ran && movement_state == Movement::IDLE && (!up && down)) {
        movement_state = Movement::DOWN;
        actuation_state = Actuation::C_DOWN;
    }

    else if(!ran && movement_state == Movement::IDLE && (!up && !down)){
        movement_state = Movement::IDLE;
        actuation_state = Actuation::IDLE;
    }

    if (!active_client_climb)
    {
        // Only trigger if we are in the starting state (0x09)
        if(!ran && actuation_state == Actuation::C_DOWN && movement_state == Movement::UP){ //If ak-60 hasnt been actuated, give +ve x vel
            cmd.linear.x = 0.3f;
            cmd.linear.y = 0.0f;
            cmd_pub_->publish(cmd);
        }

        else if(!ran && actuation_state == Actuation::C_DOWN && movement_state == Movement::DOWN){ //If ak-60 hasnt been actuated, give -ve x vel
            cmd.linear.x = -0.3f;
            cmd.linear.y = 0.0f;
            cmd_pub_->publish(cmd);
        }

        //Assigning ak motion here based on prox data and current state
        if(actuation_state == Actuation::C_DOWN) {
            if(prox_data == 0x07 && movement_state == Movement::UP) { //When the prox data is 07, (all are triggered), and needs to climb 
                active_client_climb = client_c_down;

                cmd.linear.x = 0.0f;
                cmd.linear.y = 0.0f;
                cmd_pub_->publish(cmd);
            }

            else if(prox_data == 0x00 && movement_state == Movement::DOWN) { //When the prox data is 0 (no prox is triggered), and robot needs to go down
                active_client_climb = client_c_down;

                cmd.linear.x = 0.0f;
                cmd.linear.y = 0.0f;
                cmd_pub_->publish(cmd);
            }
        }

        else if(actuation_state == Actuation::C_UP) {
            if((prox_data == 0x03 || prox_data == 0x00) && movement_state == Movement::DOWN) { //When robot has climbed and needs to retract, or the robot needs to go down and is currently hanging
                active_client_climb = client_c_up;

                cmd.linear.x = 0.0f;
                cmd.linear.y = 0.0f;
                cmd_pub_->publish(cmd);
            }

            else if(prox_data == 0x07 && movement_state == Movement::UP) { //All 3 prox triggered = 0x07
                active_client_climb = client_c_up;

                cmd.linear.x = 0.0f;
                cmd.linear.y = 0.0f;
                cmd_pub_->publish(cmd);
            }
        }
    
        if (!active_client_climb)
            return BT::NodeStatus::RUNNING;
    }

    if (!active_client_climb->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_WARN(node_->get_logger(), "Service not available...");
        return BT::NodeStatus::RUNNING;
    }

    if (!request_sent_climb)
    {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        auto result = active_client_climb->async_send_request(req);
        future_climb = result.future.share();

        request_sent_climb = true;
        return BT::NodeStatus::RUNNING;
    }

    if (request_sent_climb && future_climb.valid() && !service_done_climb)
    {
        auto status = future_climb.wait_for(std::chrono::seconds(0));
        if (status == std::future_status::ready)
        {
            auto res = future_climb.get();
            service_done_climb = true;
            service_success_climb = res->success;
            RCLCPP_INFO(node_->get_logger(), "Service call for movement %s completed with response: %s", movement_state == Movement::UP ? "UP" : "DOWN", res->message.c_str());
            ran = true;

            if(!service_success_climb) {
                RCLCPP_ERROR(node_->get_logger(), "Service call failed!");
                return BT::NodeStatus::FAILURE;
            }
        }
    }

    if(!service_done_climb) return BT::NodeStatus::RUNNING;

    if(service_done_climb) {

        printf("Service done, prox_data: %d\n", prox_data);

        if(!active_client_roll || service_done_roll){

            if (movement_state == Movement::UP) {
            
            switch(prox_data){
                case 0x00: break;

                case 0x03:  //When front 2 prox triggered
                    if(actuation_state == Actuation::C_DOWN) {
                        cmd.linear.x = 0.0f; //Resetting velocity when prox detects 0x01
                        cmd.linear.y = 0.0f; 
                        cmd_pub_->publish(cmd);

                        // rear.data = 0x69;
                        active_client_roll = client_roll_forward;

                        // rear_pub->publish(rear);
                        // rear_status_forward = true; //This is set where the service is retrieved and checked
                    }
                    break;

                    
                case 0x07: //All prox triggered
                    cmd.linear.x = 0.0f; //Resetting velocity when prox detects 0x07
                    cmd.linear.y = 0.0f;              
                    cmd_pub_->publish(cmd);     

                    if(rear_status_forward) { //If the rear wheel has ALREADY been driven (i.e, robot has climbed and driven), execute this if-command
                        // rear.data = 0x00;
                        // rear_pub->publish(rear);

                        actuation_state = Actuation::C_UP;
                        reset();

                    }

                    if(!rear_status_forward){

                        active_client_roll = client_roll_forward;
                        // rear.data = 0x69;
                        // rear_pub->publish(rear);                     
                    }
                    
                    break;

                case 0x04: //Rear prox triggered
                    if(rear_status_forward) {
                        reset();
                        ran = false;
                        return BT::NodeStatus::SUCCESS; //The robot has successfully climbed up, driven the rear wheel forward, and reached the 0x08 state, which means it can stop here and return success.
                    }

                    break;
            }
        }

        if (movement_state == Movement::DOWN) {
            switch(prox_data){

                case 0x00: 
                    printf("0x00 state reached for movement down\n"); 
                    if(rear_status_reverse == false) { //By default actuation_status_ = C_DOWN, so if prox detects 0x00 and rear_status_reverse is false, it means the robot is climbing down and has not driven the rear wheel yet, but has actuated the ak-60
                    //Hence, publish the rear command to drive the rear wheel in reverse.
                        // rear.data = 0x76;
                        active_client_roll = client_roll_backward;
                        // rear_pub->publish(rear);
                    }

                    if(rear_status_reverse == true) {

                        // rear.data = 0x00;
                        // rear_pub->publish(rear);
                        reset();
                        actuation_state = Actuation::C_UP;
                        
                    }

                    cmd.linear.x = 0.0f; //Resetting velocity when prox detects 0x00
                    cmd.linear.y = 0.0f;
                    cmd_pub_->publish(cmd);
                    break;

                case 0x03:  //Front prox triggered
                    printf("0x03 state reached for movement down\n"); 
                    cmd.linear.x = 0.0f; //Resetting velocity when prox detects 0x03
                    cmd.linear.y = 0.0f;
                    cmd_pub_->publish(cmd);

                    // rear.data = 0x76;
                    // rear_pub->publish(rear);

                    active_client_roll = client_roll_backward;
                    // rear_status_reverse = true; //This is set where server is requested and validity is checked for
                    break;
                    
                case 0x07: //All prox triggered
                    printf("0x07 state reached for movement down\n"); 
                    // if(actuation_state == Actuation::C_DOWN) {
                    //     rear.data = 0x00;
                    //     rear_pub->publish(rear);
                    // }
                    break;

                case 0x04: //Back prox triggered
                    printf("0x04 state reached for movement down\n"); 
                    if(rear_status_reverse){
                        reset();
                        ran = false;
                        return BT::NodeStatus::SUCCESS; //The robot has successfully climbed down, driven the rear wheel in reverse, and reached the 0x08 
                        // state, which means it can stop here and return success.
                    }
                    
                    break;  
            }
        }
        // Still waiting for prox_data to change for success
        return BT::NodeStatus::RUNNING;
        }
    }

    if(active_client_roll){

        if (!active_client_roll->wait_for_service(std::chrono::milliseconds(100)))
        {
            RCLCPP_WARN(node_->get_logger(), "Roll Service not available...");
            return BT::NodeStatus::RUNNING;
        }

        if (!request_sent_roll)
        {
            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
            req->data = true;
            auto result = active_client_roll->async_send_request(req);
            future_roll = result.future.share();

            request_sent_roll = true;
            return BT::NodeStatus::RUNNING;
        }

        if (request_sent_roll && future_roll.valid() && !service_done_roll)
        {
            auto status = future_roll.wait_for(std::chrono::seconds(0));
            if (status == std::future_status::ready)
            {
                auto res = future_roll.get();
                service_done_roll = true;
                service_success_roll = res->success;

                if (service_success_roll) {
                    if (movement_state == Movement::UP)   rear_status_forward = true;
                    if (movement_state == Movement::DOWN) rear_status_reverse = true;
                    active_client_roll.reset();
                }

                // RCLCPP_INFO(node_->get_logger(), "Service call for movement %s completed with response: %s", movement_state == Movement::UP ? "UP" : "DOWN", res->message.c_str());

                if(!service_success_roll) {
                    RCLCPP_ERROR(node_->get_logger(), "Service call failed!");
                    return BT::NodeStatus::FAILURE;
                }
            }
        }

        if(!service_done_roll) return BT::NodeStatus::RUNNING;

    }

    return BT::NodeStatus::RUNNING;
    }
    

void AkClient::prox_callback(const std_msgs::msg::Byte::SharedPtr msg)
{
    prox_data = msg->data;
    printf("Prox data received: %d\n", prox_data);

}

void AkClient::onHalted()
{
    RCLCPP_WARN(node_->get_logger(), "AkClient halted!");
}

void AkClient::reset(){
        request_sent_climb = false;
        service_done_climb = false;
        service_success_climb = false;
        active_client_climb.reset();

        cmd.linear.x = 0.0f; //Resetting velocity when prox detects 0x00
        cmd.linear.y = 0.0f;
        cmd_pub_->publish(cmd);
}