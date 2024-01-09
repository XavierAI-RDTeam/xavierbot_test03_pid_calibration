class PIDEstimator
{
private:
    float error             = 0;
    float propotional_out   = 0;
    float integral          = 0;
    float integral_out      = 0;
    float derivative	    = 0;
    float derivative_out    = 0;
    float previous_error    = 0;
    float corrected         = 0;

    float k_p;
    float k_i;
    float k_d;
    float max;

public:
    PIDEstimator(float kp, float ki, float kd, float maxValue)
    {
        k_p = kp;
        k_i = ki;
        k_d = kd;
        max = maxValue;
    }

    float evaluate(float target, float feedback, float timeDuration)
    {
        error           = target - feedback;

        propotional_out = k_p * error;

        integral        = integral + error*timeDuration;
        integral_out    = k_i*integral;

        derivative      = (error - previous_error) / timeDuration;
        derivative_out  = k_d * derivative;

        corrected       = feedback + propotional_out + integral_out + derivative_out;
        previous_error  = error;

        if (corrected > max) corrected = max;
        else if (corrected < -max) corrected = -max;
        else corrected = corrected;

        return corrected;
    }
};