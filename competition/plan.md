一、先冻结当前硬件接口，不要再边调边改架构

现在 src/ 里已经有这些关键基础：

device/
  bus_motor/        电机驱动
  bus_servo/        舵机驱动
  imu/              IMU
  gw_gray.*         灰度传感器

domain/
  steer_wheel_kine.*
  serial_arm/
    serial_arm_kine.*
    five_dof_arm_kine.*

service/
  chassis.*
  arm.*
  assemble/
    assemble_chassis.c
    assemble_arm.c
    assemble_sensor.c

app/
  remote.*

已经有了：

底盘驱动 + 机械臂驱动 + IMU + 遥控 + 灰度传感器 + 运动学模型

接下来应该新增的是：

service/line_sensor.*
service/navigation.*
service/vision_comm.*
service/mission.*
app/competition.*

不要继续把比赛逻辑写进 remote.c，否则后面会很乱。remote.c 只负责手动调试和紧急接管。

二、灰度传感器具体功能

比赛场地地面有地图和标记，灰度传感器可以用于辅助定位、巡线、过点检测。

你现在已经有：

gw_gray_get_front_black();
gw_gray_get_back_black();

接下来要做一个灰度解释层，不要让 app 直接读 bit。

建议新增：

src/service/line_sensor.h
src/service/line_sensor.c

提供：

typedef struct {
    bool line_detected;
    float lateral_error;
    float heading_error;
    uint8_t front_mask;
    uint8_t back_mask;
} LineSensorState;

void line_sensor_update(void);
const LineSensorState* line_sensor_get_state(void);

最开始可以只做：

8 路灰度 bit mask → 横向偏差 error

例如：

error = 加权平均黑线位置 - 中心位置;

然后用于：

路线修正
停车点检测
十字标记识别

三、语音播报由视觉系统管理，电控系统只负责底盘和机械臂的状态机

四、建立比赛场地坐标系和导航规划

比赛场地是约：

3790 mm × 3000 mm

你应该统一成米：

3.79 m × 3.00 m

建议定义比赛坐标系：

原点：起止区左下角或场地左下角
X：向右
Y：向上
Yaw = 0：机器人朝 +X
单位：m

新增：

src/service/navigation.*

里面写死场地尺寸和区域点位。

例如：

#define FIELD_WIDTH_M   3.79f
#define FIELD_HEIGHT_M  3.00f

typedef struct {
    float x;
    float y;
    float yaw;
} FieldPose;

然后定义：

extern const FieldPose START_POSE;
extern const FieldPose A_APPROACH_POSE[6];
extern const FieldPose B_APPROACH_POSE[3];
extern const FieldPose C_APPROACH_POSE[3];
extern const FieldPose D_UAV_START_POSE;

这一步非常重要。后面所有导航、视觉、机械臂动作都要基于统一坐标系。

并且应该先做“固定路径版自动任务”：比赛任务里 A/B/C 区花的位置相对固定，视觉只是负责识别雌雄和精确对准。你不要一上来做完全智能规划，应该先做固定路径版。

五、完成比赛状态机

项目里已经有：

src/infra/hfsm/

所以自动比赛应该用状态机实现。

建议新增：

src/app/competition.h
src/app/competition.c

状态设计：

typedef enum {
    COMP_STATE_IDLE = 0,
    COMP_STATE_START_BROADCAST,
    COMP_STATE_GO_A,
    COMP_STATE_A_SCAN,
    COMP_STATE_A_POLLEN,
    COMP_STATE_GO_B,
    COMP_STATE_B_SCAN,
    COMP_STATE_B_POLLEN,
    COMP_STATE_GO_C,
    COMP_STATE_C_SCAN,
    COMP_STATE_C_POLLEN,
    COMP_STATE_GO_HOME,
    COMP_STATE_FINISH,
    COMP_STATE_ERROR,
    COMP_STATE_ESTOP
} CompetitionState;

主循环里这样跑：

void competition_process(void) {
    switch(state) {
    case COMP_STATE_IDLE:
        break;

    case COMP_STATE_START_BROADCAST:
        voice_play_start_info();
        state = COMP_STATE_GO_A;
        break;

    case COMP_STATE_GO_A:
        if(nav_go_to(&A_pose)) {
            state = COMP_STATE_A_SCAN;
        }
        break;

    case COMP_STATE_A_SCAN:
        if(vision_get_result(&result)) {
            state = COMP_STATE_A_POLLEN;
        }
        break;

    case COMP_STATE_A_POLLEN:
        if(pollen_do_once(result.x, result.y, result.z)) {
            state = COMP_STATE_GO_B;
        }
        break;

    default:
        break;
    }
}

六、机械臂和视觉标定

末端主要由当前机械臂模型末端、视觉系统、记号笔，需要标定视觉系统坐标系、记号笔坐标系与机械臂末端坐标系的变换关系

七、无人机系统不需要该工作区专门负责