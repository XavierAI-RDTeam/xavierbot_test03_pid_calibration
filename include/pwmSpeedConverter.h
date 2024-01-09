class PwmSpeedConverter
{
private:
    float speedMax          = 0;
    float speedMin          = 0;
    float speedTransition    = 0;

    float pwmMax          = 0;
    float pwmMin          = 0;
    float pwmEffectiveMax = 0;
    float pwmEffectiveMin = 0;
    float pwmTransition    = 0;

    float m1 = 0;
    float m2 = 0;
    float c1 = 0;
    float c2 = 0;

public:
    PwmSpeedConverter(float inSpeedMax, float inSpeedMin, float inSpeedTransition,
                      float inPwmMax, float inPwmMin, 
                      float inPwmEffectiveMax, float inPwmEffectiveMin, float inPwmTransition )
    {
        speedMax          = inSpeedMax; 
        speedMin          = inSpeedMin;
        speedTransition    = inSpeedTransition;

        pwmMax            = inPwmMax;      
        pwmMin            = inPwmMin;   
        pwmEffectiveMax   = inPwmEffectiveMax;
        pwmEffectiveMin   = inPwmEffectiveMin;
        pwmTransition      = inPwmTransition; 

        m1 = (speedTransition-speedMin) / (pwmTransition-pwmEffectiveMin);
        m2 = (speedMax-speedTransition) / (pwmEffectiveMax-pwmTransition);

        c1 = speedTransition - (m1 * pwmTransition);
        c2 = speedMax - (m2 * pwmEffectiveMax);
    }

    int convert(float speed)
    {
        if (speed == speedMin)
        {
            return int(pwmMin);
        }
        else if ((speed > speedMin) && (speed < speedTransition))
        {
            return int((speed - c1) / m1);
        }
        else if ((speed >= speedTransition) && (speed < speedMax))
        {
            return int((speed - c2) / m2);
        }
        else
        {
            return int(pwmMax);
        }
    }
};