#include "kb_coms_micro/kb_coms_micro.hpp"

KB_coms_micro::KB_coms_micro() : Node("kb_coms_micro_node") {

    // Declare parameters with defaults
    this->declare_parameter<std::string>("serial_port", "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0");
    this->declare_parameter<int>("baudrate", 115200);

    // Get parameters
    std::string port;
    int baud;
    this->get_parameter("serial_port", port);
    this->get_parameter("baudrate", baud);

    // Crea publisher y suscribers
    esp_heart_pub_ = create_publisher<kb_interfaces::msg::Frame>("/esp32/heartbeat", 10);

    tx_sub_ = create_subscription<kb_interfaces::msg::Frame>(
        "/ejemplo_sub", 10, std::bind(&KB_coms_micro::tx_callback, this, std::placeholders::_1));

    // Inicializa la librería serial
    serial_ = std::make_unique<SerialDriver>(
        port, baud, [this](const SerialDriver::Frame &frame) { this->rx_callback(frame); });

    serial_->start();
}

KB_coms_micro::~KB_coms_micro() { serial_->stop(); }

// Callback de mensajes de la ESP
void KB_coms_micro::rx_callback(const SerialDriver::Frame &frame_esp) {
    RCLCPP_INFO(this->get_logger(), "Se ha recibido un msg");

    switch(frame_esp.type) {
    case kb_interfaces::msg::Frame::ESP_ACT_SPEED: {
        kb_interfaces::msg::Frame msg_orin1;

        msg_orin1.type = frame_esp.type;
        msg_orin1.payload = frame_esp.payload;

        // Publish frame

        break;
    }

    case kb_interfaces::msg::Frame::ESP_ACT_ACCELERATION: {
        kb_interfaces::msg::Frame msg_orin2;

        msg_orin2.type = frame_esp.type;
        msg_orin2.payload = frame_esp.payload;

        break;
    }

    case kb_interfaces::msg::Frame::ESP_ACT_BRAKING: {
        kb_interfaces::msg::Frame msg_orin3;

        msg_orin3.type = frame_esp.type;
        msg_orin3.payload = frame_esp.payload;

        break;
    }

    case kb_interfaces::msg::Frame::ESP_ACT_STEERING: {
        kb_interfaces::msg::Frame msg_orin4;

        msg_orin4.type = frame_esp.type;
        msg_orin4.payload = frame_esp.payload;

        break;
    }

    case kb_interfaces::msg::Frame::ESP_MISION: {
        kb_interfaces::msg::Frame msg_orin5;

        msg_orin5.type = frame_esp.type;
        msg_orin5.payload = frame_esp.payload;

        break;
    }

    case kb_interfaces::msg::Frame::ESP_MACHINE_STATE: {
        kb_interfaces::msg::Frame msg_orin6;

        msg_orin6.type = frame_esp.type;
        msg_orin6.payload = frame_esp.payload;

        break;
    }

    case kb_interfaces::msg::Frame::ESP_ACT_SHUTDOWN: {
        kb_interfaces::msg::Frame msg_orin7;

        msg_orin7.type = frame_esp.type;
        msg_orin7.payload = frame_esp.payload;

        break;
    }

    case kb_interfaces::msg::Frame::ESP_HEARTBEAT: {
        kb_interfaces::msg::Frame msg_orin8;

        msg_orin8.type = frame_esp.type;
        msg_orin8.payload = frame_esp.payload;

        esp_heart_pub_->publish(msg_orin8);

        break;
    }

    case kb_interfaces::msg::Frame::ESP_COMPLETE: {
        kb_interfaces::msg::Frame msg_orin9;

        msg_orin9.type = frame_esp.type;
        msg_orin9.payload = frame_esp.payload;

        break;
    }

        // // En principio esto no se necesita ya que esta funcion recibe msg de la ESP solo
        // case kb_interfaces::msg::Frame::ORIN_TARG_THROTTLE:
        //     kb_interfaces::msg::Frame msg_orin;

        //     msg_orin.type = frame_esp.type;
        //     msg_orin.payload = frame_esp.payload;

        //     break;

        // case kb_interfaces::msg::Frame::ORIN_TARG_BRAKING:
        //     kb_interfaces::msg::Frame msg_orin;

        //     msg_orin.type = frame_esp.type;
        //     msg_orin.payload = frame_esp.payload;

        //     break;

        // case kb_interfaces::msg::Frame::ORIN_TARG_STEERING:
        //     kb_interfaces::msg::Frame msg_orin;

        //     msg_orin.type = frame_esp.type;
        //     msg_orin.payload = frame_esp.payload;

        //     break;

        // case kb_interfaces::msg::Frame::ORIN_MISION:
        //     kb_interfaces::msg::Frame msg_orin;

        //     msg_orin.type = frame_esp.type;
        //     msg_orin.payload = frame_esp.payload;

        //     break;

        // case kb_interfaces::msg::Frame::ORIN_MACHINE_STATE:
        //     kb_interfaces::msg::Frame msg_orin;

        //     msg_orin.type = frame_esp.type;
        //     msg_orin.payload = frame_esp.payload;

        //     break;

        // case kb_interfaces::msg::Frame::ORIN_HEARTBEAT:
        //     kb_interfaces::msg::Frame msg_orin;

        //     msg_orin.type = frame_esp.type;
        //     msg_orin.payload = frame_esp.payload;

        //     orin_heart_pub_->publish(msg_orin);

        //     break;

        // case kb_interfaces::msg::Frame::ORIN_SHUTDOWN:
        //     kb_interfaces::msg::Frame msg_orin;

        //     msg_orin.type = frame_esp.type;
        //     msg_orin.payload = frame_esp.payload;

        //     break;

        // case kb_interfaces::msg::Frame::ORIN_COMPLETE:
        //     kb_interfaces::msg::Frame msg_orin;

        //     msg_orin.type = frame_esp.type;
        //     msg_orin.payload = frame_esp.payload;

        //     break;

    default:
        break;
    }
}

// Callback de mensajes de la ORIN
void KB_coms_micro::tx_callback(const kb_interfaces::msg::Frame::SharedPtr msg) {

    serial_->send(msg->type, msg->payload);
}
