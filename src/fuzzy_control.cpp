#include <fuzzy_control.h>


FuzzyControl::FuzzyControl(){}

FuzzyControl::FuzzyControl(ros::NodeHandle &paramGet)
{
    /* Get Param */
    paramGet.param<double>("min_dis", min_dis_, 0.11);
    paramGet.param<double>("max_dis", max_dis_, 0.99);
    paramGet.param<double>("min_steer", min_steer_, -1.495);
    paramGet.param<double>("max_steer", max_steer_, 1.495);
    paramGet.param<double>("min_vel", min_vel_, 0.02);
    paramGet.param<double>("max_vel", max_vel_, 0.5);

    goal_S_b_ = min_dis_ - 0.01;
    goal_B_b_ = max_dis_ + 0.01;
    // paramGet.param<double>("goal_S_b", goal_S_b_, 0.1);
    paramGet.param<double>("goal_M_b", goal_M_b_, 0.5);
    // paramGet.param<double>("goal_B_b", goal_B_b_, 1.0);

    steer_NB_b_ = min_steer_ - 0.005;
    steer_PB_b_ = max_steer_ + 0.005;
    // paramGet.param<double>("steer_NB_b", steer_NB_b_, -1.5);
    paramGet.param<double>("steer_NS_b", steer_NS_b_, -0.8);
    paramGet.param<double>("steer_Z_b", steer_Z_b_, 0.0);
    paramGet.param<double>("steer_PS_b", steer_PS_b_, 0.8);
    // paramGet.param<double>("steer_PB_b", steer_PB_b_, 1.5);

    output_Z_ = min_vel_;
    output_B_ = max_vel_;
    // paramGet.param<double>("output_VS", output_VS_, 0.05);
    paramGet.param<double>("output_VS", output_VS_, 0.05);
    paramGet.param<double>("output_S", output_S_, 0.15);
    paramGet.param<double>("output_M", output_M_, 0.3);
    // paramGet.param<double>("output_B", output_B_, 0.4);

    paramGet.param<double>("max_abs_vel_change", max_abs_vel_change_, 0.1);

    // Update parameters for membership function 
    goal_S_a_ = goal_S_b_;           
    goal_M_a_ = goal_S_b_;
    goal_S_c_ = goal_M_b_;
    goal_B_a_ = goal_M_b_;
    goal_M_c_ = goal_B_b_;
    goal_B_c_ = goal_B_b_;

    steer_NB_a_ = steer_NB_b_;
    steer_NS_a_ = steer_NB_b_;
    steer_NB_c_ = steer_NS_b_;
    steer_Z_a_ = steer_NS_b_;
    steer_NS_c_ = steer_Z_b_;
    steer_PS_a_ = steer_Z_b_;
    steer_Z_c_ = steer_PS_b_;
    steer_PB_a_ = steer_PS_b_;
    steer_PS_c_ = steer_PB_b_;
    steer_PB_c_ = steer_PB_b_;

    // Membership function initialize
    goal_S = trimfFunction(goal_S_a_, goal_S_b_, goal_S_c_);
    goal_M = trimfFunction(goal_M_a_, goal_M_b_, goal_M_c_);
    goal_B = trimfFunction(goal_B_a_, goal_B_b_, goal_B_c_);

    steer_NB = trimfFunction(steer_NB_a_, steer_NB_b_, steer_NB_c_);
    steer_NS = trimfFunction(steer_NS_a_, steer_NS_b_, steer_NS_c_);
    steer_Z = trimfFunction(steer_Z_a_, steer_Z_b_, steer_Z_c_);
    steer_PS = trimfFunction(steer_PS_a_, steer_PS_b_, steer_PS_c_);
    steer_PB = trimfFunction(steer_PB_a_, steer_PB_b_, steer_PB_b_);

    // Output vector date based on the following rules
    output_vect_.push_back(output_Z_);
    output_vect_.push_back(output_Z_);
    output_vect_.push_back(output_Z_);
    output_vect_.push_back(output_Z_);
    output_vect_.push_back(output_Z_);

    output_vect_.push_back(output_Z_);
    output_vect_.push_back(output_S_);
    output_vect_.push_back(output_M_);
    output_vect_.push_back(output_S_);
    output_vect_.push_back(output_Z_);

    output_vect_.push_back(output_Z_);
    output_vect_.push_back(output_M_);
    output_vect_.push_back(output_B_);
    output_vect_.push_back(output_M_);
    output_vect_.push_back(output_Z_);

}

void FuzzyControl::inputSolveGoal(double distance)
{
    if(distance < min_dis_) distance = min_dis_;
    if(distance > max_dis_) distance = max_dis_;

    muy_goal_.clear();
    muy_goal_.push_back(goal_S.cal(distance));
    muy_goal_.push_back(goal_M.cal(distance));
    muy_goal_.push_back(goal_B.cal(distance));

    // ROS_INFO("Input goal results:");
    // for (int i = 0; i < muy_goal_.size(); i++)
    // {
    //     ROS_INFO("muy goal [%d]: %f", i, muy_goal_.at(i));
    // }
}

void FuzzyControl::inputsolveSteering(double steering)
{
    if(steering < min_steer_) steering = min_steer_;
    if(steering > max_steer_) steering = max_steer_;

    muy_steer_.clear();
    muy_steer_.push_back(steer_NB.cal(steering));
    muy_steer_.push_back(steer_NS.cal(steering));
    muy_steer_.push_back(steer_Z.cal(steering));
    muy_steer_.push_back(steer_PS.cal(steering));
    muy_steer_.push_back(steer_PB.cal(steering));
    // ROS_INFO("Input steering results:");
    // for (int i = 0; i < muy_steer_.size(); i++)
    // {
    //     ROS_INFO("muy steering [%d]: %f", i, muy_steer_.at(i));
    // }
}

void FuzzyControl::inputResults()
{
    input_result_.clear();
    for (int i = 0; i < muy_goal_.size(); i ++)
    {
        for (int j = 0; j < muy_steer_.size(); j++)
        {
            // if (muy_goal_.at(i) == 0) input_result_.push_back(muy_steer_.at(i));
            // else if (muy_steer_.at(i) == 0) input_result_.push_back(muy_goal_.at(i));
            // else
            // {
                input_result_.push_back(std::min(muy_steer_.at(j), muy_goal_.at(i)));
            // }
        }
    }
    // ROS_INFO("Input results:");
    // for (int i = 0; i < input_result_.size(); i++)
    // {
    //     ROS_INFO("input [%d]: %f", i, input_result_.at(i));
    // }
}

double FuzzyControl::cal_fuzzy_output()
{
    double fuzzy_output = 0;
    double input_sum = 0;
    for (int i = 0; i < input_result_.size(); i++)
    {
        fuzzy_output += input_result_.at(i)*output_vect_.at(i);
        input_sum += input_result_.at(i);
    }
    fuzzy_out_ = fuzzy_output/input_sum;

    // limit acceleration
    if (abs(fuzzy_out_ - last_fuzzy_out_ > max_abs_vel_change_))
    {
        // ROS_INFO("Correct the fuzzy velocity!!!");
        if (fuzzy_out_ > last_fuzzy_out_) fuzzy_out_ = last_fuzzy_out_ + max_abs_vel_change_;
        else fuzzy_out_ = last_fuzzy_out_ - max_abs_vel_change_;
    }
    last_fuzzy_out_ = fuzzy_out_;
    return fuzzy_out_;
}
