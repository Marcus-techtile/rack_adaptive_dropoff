#include <fuzzy_control.h>


FuzzyControl::FuzzyControl(){}

FuzzyControl::FuzzyControl(ros::NodeHandle &paramGet)
{
    /* Get Param */
    paramGet.param<double>("min_dis", min_dis_, 0.11);
    paramGet.param<double>("max_dis", max_dis_, 0.99);
    paramGet.param<double>("min_curve", min_curve_, -1.495);
    paramGet.param<double>("max_curve", max_curve_, 1.495);
    paramGet.param<double>("min_vel", min_vel_, 0.02);
    paramGet.param<double>("max_vel", max_vel_, 0.5);

    goal_S_b_ = min_dis_ - 0.01;
    goal_B_b_ = max_dis_ + 0.01;
    // paramGet.param<double>("goal_S_b", goal_S_b_, 0.1);
    paramGet.param<double>("goal_M_b", goal_M_b_, 0.5);
    // paramGet.param<double>("goal_B_b", goal_B_b_, 1.0);

    curve_NB_b_ = min_curve_ - 0.005;
    curve_PB_b_ = max_curve_ + 0.005;
    // paramGet.param<double>("curve_NB_b", curve_NB_b_, -1.5);
    paramGet.param<double>("curve_NS_b", curve_NS_b_, -0.8);
    paramGet.param<double>("curve_Z_b", curve_Z_b_, 0.0);
    paramGet.param<double>("curve_PS_b", curve_PS_b_, 0.8);
    // paramGet.param<double>("curve_PB_b", curve_PB_b_, 1.5);

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

    curve_NB_a_ = curve_NB_b_;
    curve_NS_a_ = curve_NB_b_;
    curve_NB_c_ = curve_NS_b_;
    curve_Z_a_ = curve_NS_b_;
    curve_NS_c_ = curve_Z_b_;
    curve_PS_a_ = curve_Z_b_;
    curve_Z_c_ = curve_PS_b_;
    curve_PB_a_ = curve_PS_b_;
    curve_PS_c_ = curve_PB_b_;
    curve_PB_c_ = curve_PB_b_;

    // Membership function initialize
    goal_S = trimfFunction(goal_S_a_, goal_S_b_, goal_S_c_);
    goal_M = trimfFunction(goal_M_a_, goal_M_b_, goal_M_c_);
    goal_B = trimfFunction(goal_B_a_, goal_B_b_, goal_B_c_);

    curve_NB = trimfFunction(curve_NB_a_, curve_NB_b_, curve_NB_c_);
    curve_NS = trimfFunction(curve_NS_a_, curve_NS_b_, curve_NS_c_);
    curve_Z = trimfFunction(curve_Z_a_, curve_Z_b_, curve_Z_c_);
    curve_PS = trimfFunction(curve_PS_a_, curve_PS_b_, curve_PS_c_);
    curve_PB = trimfFunction(curve_PB_a_, curve_PB_b_, curve_PB_b_);

    // Output vector date based on the following rules
    output_vect_.push_back(output_Z_);
    output_vect_.push_back(output_Z_);
    output_vect_.push_back(output_Z_);
    output_vect_.push_back(output_Z_);
    output_vect_.push_back(output_Z_);

    output_vect_.push_back(output_VS_);
    output_vect_.push_back(output_S_);
    output_vect_.push_back(output_M_);
    output_vect_.push_back(output_S_);
    output_vect_.push_back(output_VS_);

    output_vect_.push_back(output_S_);
    output_vect_.push_back(output_M_);
    output_vect_.push_back(output_B_);
    output_vect_.push_back(output_M_);
    output_vect_.push_back(output_S_);

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
    if(steering < min_curve_) steering = min_curve_;
    if(steering > max_curve_) steering = max_curve_;

    muy_curve_.clear();
    muy_curve_.push_back(curve_NB.cal(steering));
    muy_curve_.push_back(curve_NS.cal(steering));
    muy_curve_.push_back(curve_Z.cal(steering));
    muy_curve_.push_back(curve_PS.cal(steering));
    muy_curve_.push_back(curve_PB.cal(steering));
    // ROS_INFO("Input steering results:");
    // for (int i = 0; i < muy_curve_.size(); i++)
    // {
    //     ROS_INFO("muy steering [%d]: %f", i, muy_curve_.at(i));
    // }
}

void FuzzyControl::inputResults()
{
    input_result_.clear();
    for (int i = 0; i < muy_goal_.size(); i ++)
    {
        for (int j = 0; j < muy_curve_.size(); j++)
        {
            // if (muy_goal_.at(i) == 0) input_result_.push_back(muy_curve_.at(i));
            // else if (muy_curve_.at(i) == 0) input_result_.push_back(muy_goal_.at(i));
            // else
            // {
                input_result_.push_back(std::min(muy_curve_.at(j), muy_goal_.at(i)));
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
