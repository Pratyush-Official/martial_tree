#include "ak_client.hpp"

AkClient::AkClient(const std::string &name, const BT::NodeConfig &config, rclcpp::Node::SharedPtr node)
    : BT::StatefulActionNode(name, config), node_(node)
{
    client_c_down = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_up_execute"); //c section applies normal force
    client_c_up = node_->create_client<std_srvs::srv::SetBool>("/ak_motor_down_exec"); //body "applies" normal force

    rear_pub = node_->create_publisher<std_msgs::msg::Byte>("/rear_drive", cmd_vel_qos());
    prox_sub = node_->create_subscription<std_msgs::msg::Byte>(
        "/prox_data", sensor_qos(), std::bind(&AkClient::prox_callback, this, std::placeholders::_1));

    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_vel_qos());
    
    request_sent = false;
    rear_status_forward = false;
    rear_status_reverse = false;
    ran = false;

    rear.data = 0x00;
    prox_data = 0x00;

    movement_state = Movement::IDLE;
    actuation_state = Actuation::IDLE;

    cmd.linear.x = 0.0f;
    cmd.linear.y = 0.0f;

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
    active_client.reset();
    request_sent = false;

    service_done_ = false;
    service_success_ = false;
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

    if (!active_client)
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
            if(prox_data == 0x09 && movement_state == Movement::UP) {
                active_client = client_c_down;

                cmd.linear.x = 0.0f;
                cmd.linear.y = 0.0f;
                cmd_pub_->publish(cmd);
            }

            else if(prox_data == 0x00 && movement_state == Movement::DOWN) {
                active_client = client_c_down;

                cmd.linear.x = 0.0f;
                cmd.linear.y = 0.0f;
                cmd_pub_->publish(cmd);
            }
        }

        else if(actuation_state == Actuation::C_UP) {
            if((prox_data == 0x01 || prox_data == 0x00) && movement_state == Movement::DOWN) {
                active_client = client_c_up;

                cmd.linear.x = 0.0f;
                cmd.linear.y = 0.0f;
                cmd_pub_->publish(cmd);
            }

            else if(prox_data == 0x09 && movement_state == Movement::UP) {
                active_client = client_c_up;

                cmd.linear.x = 0.0f;
                cmd.linear.y = 0.0f;
                cmd_pub_->publish(cmd);
            }
        }
    
        if (!active_client)
            return BT::NodeStatus::RUNNING;
    }

    if (!active_client->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_WARN(node_->get_logger(), "Service not available...");
        return BT::NodeStatus::RUNNING;
    }

    if (!request_sent)
    {
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        auto result = active_client->async_send_request(req);
        future_ = result.future.share();

        request_sent = true;
        return BT::NodeStatus::RUNNING;
    }

    if (request_sent && future_.valid() && !service_done_)
    {
        auto status = future_.wait_for(std::chrono::seconds(0));
        if (status == std::future_status::ready)
        {
            auto res = future_.get();
            service_done_ = true;
            service_success_ = res->success;
            RCLCPP_INFO(node_->get_logger(), "Service call for movement %s completed with response: %s", movement_state == Movement::UP ? "UP" : "DOWN", res->message.c_str());
            ran = true;

            if(!service_success_) {
                RCLCPP_ERROR(node_->get_logger(), "Service call failed!");
                return BT::NodeStatus::FAILURE;
            }
        }
    }

    if(!service_done_) return BT::NodeStatus::RUNNING;

    if(service_done_) {

        printf("Service done, prox_data: %d\n", prox_data);
    // Publish rear command if prox_data is 0x01

        if (movement_state == Movement::UP) {
            switch(prox_data){
                case 0x00: break;

                case 0x01:  
                    if(actuation_state == Actuation::C_DOWN) {
                        cmd.linear.x = 0.0f; //Resetting velocity when prox detects 0x01
                        cmd.linear.y = 0.0f; 
                        cmd_pub_->publish(cmd);

                        rear.data = 0x69;

                        rear_pub->publish(rear);
                        rear_status_forward = true;
                    }
                    break;

                    
                case 0x09: 
                    cmd.linear.x = 0.0f; //Resetting velocity when prox detects 0x09
                    cmd.linear.y = 0.0f;              
                    cmd_pub_->publish(cmd);     

                    if(rear_status_forward) { //If the rear wheel has ALREADY been driven (i.e, robot has climbed and driven), execute this if-command
                        rear.data = 0x00;
                        rear_pub->publish(rear);

                        actuation_state = Actuation::C_UP;
                        reset();

                    }

                    if(!rear_status_forward){
                        rear.data = 0x69;
                        rear_pub->publish(rear);                     
                    }
                    
                    break;

                case 0x08: 
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
                        rear.data = 0x76;
                        rear_pub->publish(rear);
                    }

                    if(rear_status_reverse == true) {

                        rear.data = 0x00;
                        rear_pub->publish(rear);

                        reset();
                        actuation_state = Actuation::C_UP;
                        
                    }

                    cmd.linear.x = 0.0f; //Resetting velocity when prox detects 0x00
                    cmd.linear.y = 0.0f;
                    cmd_pub_->publish(cmd);
                    break;

                case 0x01:  
                    printf("0x01 state reached for movement down\n"); 
                    cmd.linear.x = 0.0f; //Resetting velocity when prox detects 0x00
                    cmd.linear.y = 0.0f;
                    cmd_pub_->publish(cmd);

                    rear.data = 0x76;
                    rear_pub->publish(rear);
                    rear_status_reverse = true;
                
                    break;
                    
                case 0x09: 
                    printf("0x09 state reached for movement down\n"); 
                    if(actuation_state == Actuation::C_DOWN) {
                        rear.data = 0x00;
                        rear_pub->publish(rear);
                    }
                    break;

                case 0x08: 
                    printf("0x08 state reached for movement down\n"); 
                    if(rear_status_reverse){
                        reset();
                        ran = false;
                        return BT::NodeStatus::SUCCESS; //The robot has successfully climbed down, driven the rear wheel in reverse, and reached the 0x08 
                        // state, which means it can stop here and return success.
                    }
                    
                    break;  
            }
        }


    // Still waiting for prox_data to change
        return BT::NodeStatus::RUNNING;
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
        request_sent = false;
        service_done_ = false;
        service_success_ = false;
        active_client.reset();

        cmd.linear.x = 0.0f; //Resetting velocity when prox detects 0x00
        cmd.linear.y = 0.0f;
        cmd_pub_->publish(cmd);
}
