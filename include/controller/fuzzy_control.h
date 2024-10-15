#include <ros/ros.h>

/* Triangle function */
class trimfFunction
{
private:
    double vertex_a_, vertex_b_, vertex_c_;
public:
    trimfFunction(){};
    trimfFunction(double a, double b, double c) // three vertex of the triangle
    {
        vertex_a_ = a;      vertex_b_ = b;      vertex_c_ = c;
    }

    double cal(double x)
    {
        double a = vertex_a_, b = vertex_b_, c = vertex_c_;
        if (x < a) return 0;
        else if (x >= a && x < b) return (x-a)/(b-a);
        else if (x >= b && x < c) return (c-x)/(c-b);
        else return 0;  // x >= c
    }
};

/***************************************************************************/
/* A class to implement multi input fuzzy controller for the forklift linear velocity */
// Input x1 is the forward distance (x in local frame): 
//      3 language variables: S, M, B (triangular function)
// Input x2 is the steering angle (steering angle): 
//      5 language variables: NB, NS, Z, PS, PB (triangular function)

/* Fuzzy laws (15 laws)*/
// if r0(x1 is S and x2 is NB), then y is Z
// if r1(x1 is S and x2 is NS), then y is Z
// if r2(x1 is S and x2 is Z), then y is VS
// if r3(x1 is S and x2 is PS), then y is Z
// if r4(x1 is S and x2 is PB), then y is Z

// if r5(x1 is M and x2 is NB), then y is VS
// if r6(x1 is M and x2 is NS), then y is S
// if r7(x1 is M and x2 is Z), then y is M
// if r8(x1 is M and x2 is PS), then y is S
// if r9(x1 is M and x2 is PB), then y is VS

// if r10(x1 is B and x2 is NB), then y is VS
// if r11(x1 is B and x2 is NS), then y is M
// if r12(x1 is B and x2 is Z), then y is B
// if r13(x1 is B and x2 is PS), then y is M
// if r14(x1 is B and x2 is PB), then y is VS
/* Output is the linear velocity: Z, VS, S, M, B*/
/***************************************************************************/


class FuzzyControl
{
private:
    ros::NodeHandle nh_;

    double min_dis_, max_dis_;
    double min_curve_, max_curve_;
    double min_vel_, max_vel_;

    double  goal_S_a_, goal_S_b_, goal_S_c_;    // vertex of Small
    double  goal_M_a_, goal_M_b_, goal_M_c_;    // vertex of Medium
    double  goal_B_a_, goal_B_b_, goal_B_c_;    // vertex of Big
    
    double  curve_NB_a_, curve_NB_b_, curve_NB_c_;    // vertex of Negative Big
    double  curve_NS_a_, curve_NS_b_, curve_NS_c_;    // vertex of Negative Small
    double  curve_Z_a_, curve_Z_b_, curve_Z_c_;       // vertex of Zero
    double  curve_PS_a_, curve_PS_b_, curve_PS_c_;       // vertex of Positive Small
    double  curve_PB_a_, curve_PB_b_, curve_PB_c_;       // vertex of Positive Big

    double output_Z_, output_VS_, output_S_, output_M_, output_B_;
    std::vector<double> output_vect_;

    std::vector<double> muy_goal_;        // satisfaction ratio of goal input 
    std::vector<double> muy_curve_;       // satisfaction ratio of distance input 

    std::vector<double> input_result_;     // input solve results of each fuzzy statement


    trimfFunction goal_S, goal_M, goal_B;
    trimfFunction curve_NB, curve_NS, curve_Z, curve_PS, curve_PB;

    double fuzzy_out_{0.0}, last_fuzzy_out_{0.0};
    double max_abs_vel_change_;
 
public:
    FuzzyControl();
    FuzzyControl(ros::NodeHandle &paramGet);
   
    /* Solve membership of distance input */
    void inputSolveGoal(double distance);

    /* Solve membership of steering input */
    void inputsolveSteering(double steering);
  
    /* Solve multi input statement */
    void inputResults();
    
    /* Cal fuzzy output */
    double cal_fuzzy_output();
};
