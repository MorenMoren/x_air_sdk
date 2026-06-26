// 在你的代码中创建新文件，如 my_control.hpp
#include <controller/control.hpp>

class MyControl : public Control {
public:
    // 继承构造函数
    using Control::Control;
    
    // 覆盖 AdjustPosition 函数
    bool AdjustPosition(void) override {
        // 你的自定义实现
        std::cout << "Custom AdjustPosition called" << std::endl;
        return true;
	int nstep = 220;
	    double alpha;

	    std::vector<MotorState> arm_motor_states;
	    for (const auto& motor : openarm_->get_arm().get_motors()) {
		arm_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
	    }

	    std::vector<MotorState> gripper_motor_states;
	    for (const auto& motor : openarm_->get_gripper().get_motors()) {
		gripper_motor_states.push_back({motor.get_position(), motor.get_velocity(), 0.0});
	    }

	    std::vector<JointState> joint_arm_now =
		openarmjointconverter_->motor_to_joint(arm_motor_states);
	    std::vector<JointState> joint_hand_now =
		openarmgripperjointconverter_->motor_to_joint(gripper_motor_states);

	    std::vector<JointState> joint_arm_goal(NMOTORS - 1);
	    for (size_t i = 0; i < NMOTORS - 1; ++i) {
		joint_arm_goal[i].position = INITIAL_POSITION[i];
		joint_arm_goal[i].velocity = 0.0;
		joint_arm_goal[i].effort = 0.0;
	    }

	    std::vector<JointState> joint_hand_goal(joint_hand_now.size());
	    for (size_t i = 0; i < joint_hand_goal.size(); ++i) {
		joint_hand_goal[i].position = 0.0;
		joint_hand_goal[i].velocity = 0.0;
		joint_hand_goal[i].effort = 0.0;
	    }

	    std::vector<double> kp_arm_temp = {50, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0};
	    std::vector<double> kd_arm_temp = {1.2, 1.2, 1.2, 1.2, 0.3, 0.2, 0.3};

	    std::vector<double> kp_hand_temp = {10.0};
	    std::vector<double> kd_hand_temp = {0.5};

	    for (int step = 0; step < nstep; ++step) {
		alpha = static_cast<double>(step + 1) / nstep;

		std::vector<JointState> joint_arm_interp(NMOTORS - 1);
		for (size_t i = 0; i < NMOTORS - 1; ++i) {
		    joint_arm_interp[i].position =
		        joint_arm_goal[i].position * alpha + joint_arm_now[i].position * (1.0 - alpha);
		    joint_arm_interp[i].velocity = 0.0;
		}

		std::vector<JointState> joint_hand_interp(joint_hand_goal.size());
		for (size_t i = 0; i < joint_hand_interp.size(); ++i) {
		    joint_hand_interp[i].position =
		        joint_hand_goal[i].position * alpha + joint_hand_now[i].position * (1.0 - alpha);
		    joint_hand_interp[i].velocity = 0.0;
		}

		std::vector<MotorState> arm_motor_refs =
		    openarmjointconverter_->joint_to_motor(joint_arm_interp);
		std::vector<MotorState> hand_motor_refs =
		    openarmgripperjointconverter_->joint_to_motor(joint_hand_interp);

		std::vector<openarm::damiao_motor::MITParam> arm_cmds;
		arm_cmds.reserve(arm_motor_refs.size());
		for (size_t i = 0; i < arm_motor_refs.size(); ++i) {
		    arm_cmds.emplace_back(openarm::damiao_motor::MITParam{kp_arm_temp[i], kd_arm_temp[i],
		                                                          arm_motor_refs[i].position,
		                                                          arm_motor_refs[i].velocity, 0.0});
		}

		std::vector<openarm::damiao_motor::MITParam> hand_cmds;
		hand_cmds.reserve(hand_motor_refs.size());
		for (size_t i = 0; i < hand_motor_refs.size(); ++i) {
		    hand_cmds.emplace_back(openarm::damiao_motor::MITParam{
		        kp_hand_temp[i], kd_hand_temp[i], hand_motor_refs[i].position,
		        hand_motor_refs[i].velocity, 0.0});
		}

		openarm_->get_arm().mit_control_all(arm_cmds);
		openarm_->get_gripper().mit_control_all(hand_cmds);

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		openarm_->recv_all();
	    }

	    std::vector<MotorState> arm_motor_states_final;
	    for (const auto& motor : openarm_->get_arm().get_motors()) {
		arm_motor_states_final.push_back({motor.get_position(), motor.get_velocity(), 0.0});
	    }

	    std::vector<MotorState> gripper_motor_states_final;
	    for (const auto& motor : openarm_->get_gripper().get_motors()) {
		gripper_motor_states_final.push_back({motor.get_position(), motor.get_velocity(), 0.0});
	    }

	    std::vector<JointState> joint_arm_final =
		openarmjointconverter_->motor_to_joint(arm_motor_states_final);
	    std::vector<JointState> joint_hand_final =
		openarmgripperjointconverter_->motor_to_joint(gripper_motor_states_final);

	    robot_state_->arm_state().set_all_references(joint_arm_final);
	    robot_state_->hand_state().set_all_references(joint_hand_final);

	    return true;
    }
    
    // 添加新函数
    void MyNewFunction(void) {
        // 你的新逻辑
    }
};
