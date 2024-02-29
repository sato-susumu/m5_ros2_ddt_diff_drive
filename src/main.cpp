#include <M5Unified.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <DDT_Motor_M15M06.h>

#define RCCHECK(fn)                               \
  {                                               \
    rcl_ret_t temp_rc = fn;                       \
    if ((temp_rc != RCL_RET_OK))                  \
    {                                             \
      M5Controller::showError(__LINE__, temp_rc); \
    }                                             \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

#if USE_ATOMS3
#define MainSerial USBSerial
#else
#define MainSerial Serial
#endif

#define ENABLE_M5_UPDATE_DISPLAY

const float WHEEL_RADIUS = 0.1007 / 2; // Radius of the wheel (meters)
const float WHEEL_SEPARATION = 0.16;   // Distance between wheels (meters)
const int MAX_RPM = 200;               // Maximum RPM of the motor
const int MOTOR_ID_LEFT = 1;
const int MOTOR_ID_RIGHT = 2;

static int msg_count = 0;
static auto loop_count = 0;
static int displayMode = 0;

int control_motor_proc_ms;
int display_proc_ms;

static float linear_velocity = 0;
static float angular_velocity = 0;

class MotorController
{
public:
  MotorController() : motor_handler(RX_GPIO, TX_GPIO) {}
  Receiver received_data[2]; // Received data for the left and right motors
  int target_velocity[2];    // Target velocities for the left and right motors

  void controlMotors()
  {
    float v_r = linear_velocity - (angular_velocity * WHEEL_SEPARATION / 2);
    float v_l = linear_velocity + (angular_velocity * WHEEL_SEPARATION / 2);

    // Convert velocity to RPM
    int rpm_r = (int)((v_r / (2 * 3.1416 * WHEEL_RADIUS)) * 60);
    int rpm_l = (int)((v_l / (2 * 3.1416 * WHEEL_RADIUS)) * 60);

    // Limit RPM to maximum value
    rpm_r = std::min(std::max(rpm_r, -MAX_RPM), MAX_RPM);
    rpm_l = std::min(std::max(rpm_l, -MAX_RPM), MAX_RPM);

    target_velocity[0] = rpm_l;
    target_velocity[1] = rpm_r;

    motor_handler.Control_Motor(rpm_l, MOTOR_ID_LEFT, 0, 0, &received_data[0]);
    motor_handler.Control_Motor(-rpm_r, MOTOR_ID_RIGHT, 0, 0, &received_data[1]);
  }

  void getMotorsInfo()
  {
    motor_handler.Get_Motor(MOTOR_ID_LEFT, &received_data[0]);
    motor_handler.Get_Motor(MOTOR_ID_RIGHT, &received_data[1]);
  }

private:
  MotorHandler motor_handler;
};

MotorController motorController;

class M5Controller
{
public:
  static void begin(const char *node_name)
  {
    M5.begin();
    M5.Lcd.setTextSize(2);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf("%s\n", node_name);
  }

  static void update()
  {
    M5.update();
    if (M5.BtnA.wasPressed())
    {
      displayMode = (displayMode + 1) % 2; // Switch display mode
    }
  }


  static void showError(int line_no, int ret_code)
  {
    bool isBlack = true; // Flag to track the color of the screen

    while (1)
    {
      M5.Lcd.fillScreen(isBlack ? BLACK : RED); // Switch color depending on the condition
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.print("ERROR\n");
      M5.Lcd.printf("line %d: %d\n", line_no, ret_code);
      delay(1000);
      isBlack = !isBlack; // Invert the flag for the next loop to switch color
    }
  }

  static void updateDisplay()
  {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);

    switch (displayMode)
    {
    case 0:
      displayMode0();
      break;
    case 1:
      displayMode1();
      break;
    }
  }

private:
  static void displayMode0()
  {
    M5.Lcd.printf("cmd_vel\n");
    M5.Lcd.printf("%0.2f\n%0.2f\n", linear_velocity, angular_velocity);
    M5.Lcd.printf("\n");
    M5.Lcd.printf("TargetRPM:\n");
    M5.Lcd.printf("%d\n%d\n", motorController.target_velocity[0], motorController.target_velocity[1]);
  }

  static void displayMode1()
  {
    M5.Lcd.printf("Message:\n");
    M5.Lcd.printf("%d\n", msg_count);
    M5.Lcd.printf("\n");
    M5.Lcd.printf("Loop:\n");
    M5.Lcd.printf("%d\n", loop_count);
  }
};

class MicroROSNode
{
public:
  MicroROSNode() : executor(), node(), cmd_vel_subscriber()
  {
    allocator = rcl_get_default_allocator();
  }

  void begin(const char *node_name)
  {
    MainSerial.begin(115200);
    set_microros_serial_transports(MainSerial);

    while(1) {
      auto result = rclc_support_init(&support, 0, NULL, &allocator);
      if(result == RCL_RET_OK) {
        break;
      }
      delay(2000);
    }
    RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_vel_subscriber,
        &cmd_vel_msg,
        &MicroROSNode::cmd_vel_callback,
        ON_NEW_DATA));
  }

  void spin_some()
  {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }

private:
  rclc_executor_t executor;
  rcl_node_t node;
  rcl_subscription_t cmd_vel_subscriber;
  geometry_msgs__msg__Twist cmd_vel_msg;
  rclc_support_t support;
  rcl_allocator_t allocator;

  static void cmd_vel_callback(const void *msgin)
  {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    linear_velocity = msg->linear.x;
    angular_velocity = msg->angular.z;
    msg_count += 1;
  }
};

MicroROSNode node;

void setup()
{
  const char *node_name = "m5_ros2_ddt_diff_drive";
  M5Controller::begin(node_name);
  node.begin(node_name);
}

void loop()
{
  auto now = millis();
  static auto last_motor_control_time = now;

  M5Controller::update();
  node.spin_some();

  if (now - last_motor_control_time > 200)
  {
    last_motor_control_time = now;

    loop_count += 1;
    motorController.controlMotors();
    M5Controller::updateDisplay();
  }
  delay(10);
}

