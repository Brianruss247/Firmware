#include "estimator_example.h"

estimator_example::estimator_example() : estimator_base()
{
    alpha = 0;
    alpha1 = 0;
    lpf_gyro_x = 0;
    lpf_gyro_y = 0;
    lpf_gyro_z = 0;
    lpf_static = 0;
    lpf_diff = 0;
    lpf_accel_x = 0;
    lpf_accel_y = 0;
    lpf_accel_z = 0;
    xhat_a.zero();
    P_a.identity();
    P_a *= powf(math::radians(20.0f),2);
    xhat_p.zero();
    xhat_p(2) = 9;
    P_p.identity();
    P_p(0,0) = .03;
    P_p(1,1) = .03;
    P_p(2,2) = .01;
    P_p(3,3) = math::radians(5.0f);
    P_p(4,4) = .04;
    P_p(5,5) = .04;
    P_p(6,6) = math::radians(5.0f);
    gps_n_old = 0;
    gps_e_old = 0;
    gps_Vg_old = 5;
    gps_course_old = 0;
}

void estimator_example::estimate(const params_s &params, const input_s &input, output_s &output)
{
    if(alpha <= 0.00001f)
    {
        lpf_static = 0;//params.rho*params.gravity*100;
        lpf_diff = 0;//1/2 * params.rho*11*11;

        Q_a.zero();
        Q_a(0,0) = 0.0000001;
        Q_a(1,1) = 0.0000001;

        R_accel.zero();
        R_accel(0,0) = powf(params.sigma_accel,2);
//        R_accel(1,1) = powf(params.sigma_accel,2);


        Q_p.identity();
        Q_p *= 0.0001f;
        Q_p(3,3) = 0.000001f;

        R_p.zero();
        R_p(0,0) = powf(params.sigma_n_gps,2);
        R_p(1,1) = powf(params.sigma_e_gps,2);
        R_p(2,2) = powf(params.sigma_Vg_gps,2);
        R_p(3,3) = powf(params.sigma_course_gps,2);
        R_p(4,4) = 0.001;
        R_p(5,5) = 0.001;
    }

    float lpf_a = 50;
    float lpf_a1 = 2;
    alpha = exp(-lpf_a*input.Ts);
    alpha1 = exp(-lpf_a1*input.Ts);

    // low pass filter gyros to estimate angular rates
    lpf_gyro_x = alpha*lpf_gyro_x + (1-alpha)*input.gyro_x;
    lpf_gyro_y = alpha*lpf_gyro_y + (1-alpha)*input.gyro_y;
    lpf_gyro_z = alpha*lpf_gyro_z + (1-alpha)*input.gyro_z;

    float phat = lpf_gyro_x;
    float qhat = lpf_gyro_y;
    float rhat = lpf_gyro_z;

    // low pass filter static pressure sensor and invert to esimate altitude
    lpf_static = alpha1*lpf_static + (1-alpha1)*input.static_pres;
    float hhat = lpf_static/params.rho/params.gravity;

    // low pass filter diff pressure sensor and invert to extimate Va
    lpf_diff = alpha1*lpf_diff + (1-alpha1)*input.diff_pres;
    float Vahat = sqrt(2/params.rho*lpf_diff);

//    if(!isfinite(hhat) || hhat < -500 || hhat > 500)
//    {
//        warnx("problem 20");
//        hhat = 10;
//    }
//    if(!isfinite(Vahat) || Vahat < 0 || Vahat > 25)
//    {
//        warnx("problem 21");
//        Vahat = 9;
//    }
//    if(!isfinite(phat) || phat < math::radians(-420.0f) || phat > math::radians(420.0f))
//    {
//        warnx("problem 22");
//        phat = 0;
//    }
//    if(!isfinite(qhat) || qhat < math::radians(-420.0f) || qhat > math::radians(420.0f))
//    {
//        warnx("problem 23");
//        qhat = 0;
//    }
//    if(!isfinite(rhat) || rhat < math::radians(-420.0f) || rhat > math::radians(420.0f))
//    {
//        warnx("problem 24");
//        rhat = 0;
//    }

    // low pass filter accelerometers
    lpf_accel_x = alpha*lpf_accel_x + (1-alpha)*input.accel_x;
    lpf_accel_y = alpha*lpf_accel_y + (1-alpha)*input.accel_y;
    lpf_accel_z = alpha*lpf_accel_z + (1-alpha)*input.accel_z;

    // inplement continuous-discrete EKF to estimate roll and pitch angles

    // prediction step
    float cp; // cos(phi)
    float sp; // sin(phi)
    float tt; // tan(thata)
    float ct; // cos(thata)
    for(int i=0;i<N;i++)
    {
        cp = cos(xhat_a(0)); // cos(phi)
        sp = sin(xhat_a(0)); // sin(phi)
        tt = tan(xhat_a(1)); // tan(thata)
        ct = cos(xhat_a(1)); // cos(thata)

        f_a(0) = phat + (qhat*sp + rhat*cp)*tt;
        f_a(1) = qhat*cp - rhat*sp;

        A_a.zero();
        A_a(0,0) = (qhat*cp - rhat*sp)*tt;
        A_a(0,1) = (qhat*sp + rhat*cp)/ct/ct;
        A_a(1,0) = -qhat*sp - rhat*cp;
//        math::Matrix<2,3> G_a;
//        G_a.zero();
//        G_a(0,0) = 1;
//        G_a(0,1) = sp*tt;
//        G_a(0,2) = cp*tt;
//        G_a(1,1) = cp;
//        G_a(1,2) = -sp;

        xhat_a += f_a *(input.Ts/N);
        P_a += (A_a*P_a + P_a*A_a.transposed() + Q_a)*(input.Ts/N);
    }
    // measurement updates
    cp = cos(xhat_a(0));
    sp = sin(xhat_a(0));
    ct = cos(xhat_a(1));
    float st = sin(xhat_a(1)); // sin(theta)
//    math::Matrix<2,2> I;
    I.identity();

    // x-axis accelerometer
    h_a = qhat*Vahat*st + params.gravity*st;
    C_a.zero();
    C_a(0,1) = qhat*Vahat*ct + params.gravity*ct;
    C_a_t.zero();
    C_a_t(1,0) = qhat*Vahat*ct + params.gravity*ct;
    L_a = (P_a*C_a_t) * (R_accel + C_a*P_a*C_a_t).inversed();
    P_a = (I - L_a*C_a)*P_a;
    math::Vector<2> K_a;
    K_a(0) = L_a(0,0);
    K_a(1) = L_a(1,0);
    xhat_a += K_a *(lpf_accel_x - h_a);//input.accel_x - h_a);

    // y-axis accelerometer
    h_a = rhat*Vahat*ct - phat*Vahat*st - params.gravity*ct*sp;
    C_a.zero();
    C_a(0,0) = -params.gravity*cp*ct;
    C_a(0,1) = -rhat*Vahat*st - phat*Vahat*ct + params.gravity*st*sp;
    C_a_t.zero();
    C_a_t(0,0) = -params.gravity*cp*ct;
    C_a_t(1,0) = -rhat*Vahat*st - phat*Vahat*ct + params.gravity*st*sp;
    L_a = (P_a*C_a_t) * (R_accel + C_a*P_a*C_a_t).inversed();
    P_a = (I - L_a*C_a)*P_a;
    K_a(0) = L_a(0,0);
    K_a(1) = L_a(1,0);
    xhat_a += K_a *(lpf_accel_y - h_a);//input.accel_y - h_a);

    // z-axis accelerometer
    h_a = -qhat*Vahat*ct - params.gravity*ct*cp;
    C_a.zero();
    C_a(0,0) = params.gravity*sp*ct;
    C_a(0,1) = (qhat*Vahat + params.gravity*cp)*st;
    C_a_t.zero();
    C_a_t(0,0) = -params.gravity*sp*ct;
    C_a_t(1,0) = (qhat*Vahat + params.gravity*cp)*st;
    L_a = (P_a*C_a_t) * (R_accel + C_a*P_a*C_a_t).inversed();
    P_a = (I - L_a*C_a)*P_a;
    K_a(0) = L_a(0,0);
    K_a(1) = L_a(1,0);
    xhat_a += K_a *(lpf_accel_z - h_a);//input.accel_z - h_a);

    if(xhat_a(0) > (float)math::radians(85.0) || xhat_a(0) < (float)math::radians(-85.0) || !isfinite(xhat_a(0)))
    {
        if(!isfinite(xhat_a(0)))
        {
            xhat_a(0) = 0;
            P_a.identity();
            P_a *= powf(math::radians(20.0f),2);
            warnx("problem 00.0");
        }
        else if(xhat_a(0) > (float)math::radians(85.0))
        {
            xhat_a(0) = (float)math::radians(82.0);
            warnx("problem 00.1");
        }
        else if(xhat_a(0) < (float)math::radians(-85.0))
        {
            xhat_a(0) = (float)math::radians(-82.0);
            warnx("problem 00.2");
        }
    }
    if(xhat_a(1) > (float)math::radians(80.0) || xhat_a(1) < (float)math::radians(-80.0) || !isfinite(xhat_a(1)))
    {
        warnx("problem 01");
        if(!isfinite(xhat_a(1)))
        {
            xhat_a(1) = 0;
            P_a.identity();
            P_a *= powf(math::radians(20.0f),2);
        }
        else if(xhat_a(1) > (float)math::radians(80.0))
        {
            xhat_a(1) = (float)math::radians(77.0);
        }
        else if(xhat_a(1) < (float)math::radians(-80.0))
        {
            xhat_a(1) = (float)math::radians(-77.0);
        }
    }
    float phihat = xhat_a(0);
    float thetahat = xhat_a(1);


    // implement continous-discrete EKF to estimate pn, pe, chi, Vg
    // prediction step
    float psidot, tmp, Vgdot;
    if(fabsf(xhat_p(2)) < 0.01f)
    {
        xhat_p(2) = 0.01;
    }

    for(int i=0;i<N;i++)
    {
        psidot = (qhat*sinf(phihat) + rhat*cosf(phihat))/cosf(thetahat);
        tmp = -psidot*Vahat*(xhat_p(4)*cosf(xhat_p(6)) + xhat_p(5)*sinf(xhat_p(6)))/xhat_p(2);
        Vgdot = ((Vahat*cosf(xhat_p(6)) + xhat_p(4))*(-psidot*Vahat*sinf(xhat_p(6))) + (Vahat*sinf(xhat_p(6)) + xhat_p(5))*(psidot*Vahat*cosf(xhat_p(6))))/xhat_p(2);

        f_p.zero();
        f_p(0) = xhat_p(2)*cosf(xhat_p(3));
        f_p(1) = xhat_p(2)*sinf(xhat_p(3));
        f_p(2) = Vgdot;
        f_p(3) = params.gravity/xhat_p(2)*tanf(phihat)*cosf(xhat_p(3) - xhat_p(6));
        f_p(6) = psidot;

        A_p.zero();
        A_p(0,2) = cos(xhat_p(3));
        A_p(0,3) = -xhat_p(2)*sinf(xhat_p(3));
        A_p(1,2) = sin(xhat_p(3));
        A_p(1,3) = xhat_p(2)*cosf(xhat_p(3));
        A_p(2,2) = -Vgdot/xhat_p(2);
        A_p(2,4) = -psidot*Vahat*sinf(xhat_p(6))/xhat_p(2);
        A_p(2,5) = psidot*Vahat*cosf(xhat_p(6))/xhat_p(2);
        A_p(2,6) = tmp;
        A_p(3,2) = -params.gravity/powf(xhat_p(2),2)*tanf(phihat)*cosf(xhat_p(3) - xhat_p(6));
        A_p(3,3) = -params.gravity/xhat_p(2)*tanf(phihat)*sinf(xhat_p(3) - xhat_p(6));
        A_p(3,6) = params.gravity/xhat_p(2)*tanf(phihat)*sinf(xhat_p(3) - xhat_p(6));

        xhat_p += f_p *(input.Ts/N);
        P_p += (A_p*P_p + P_p*A_p.transposed() + Q_p)*(input.Ts/N);
    }

//    while(xhat_p(3) > math::radians(180.0f)) xhat_p(3) = xhat_p(3) - math::radians(360.0f);
//    while(xhat_p(3) < math::radians(-180.0f)) xhat_p(3) = xhat_p(3) + math::radians(360.0f);
//    if(xhat_p(3) > math::radians(180.0f) || xhat_p(3) < math::radians(-180.0f))
//    {
//        warnx("problem 17");
//        xhat_p(3) = 0;
//    }

    // measurement updates
//    if(input.gps_n != gps_n_old ||
//            input.gps_e != gps_e_old ||
//            input.gps_Vg != gps_Vg_old ||
//            input.gps_course != gps_course_old)
    if(input.gps_new)
    {
        math::Matrix<7,7> I_p;
        I_p.identity();

        // gps North position
        h_p = xhat_p(0);
        C_p.zero();
        C_p(0,0) = 1;
        C_p_t.zero();
        C_p_t(0,0) = 1;
        math::Matrix<1,1> denom;
        denom(0,0) = (R_p(0,0) + (C_p*P_p*C_p_t)(0,0));
        L_p = (P_p*C_p_t) * denom.inversed();
        P_p = (I_p - L_p*C_p)*P_p;
        math::Vector<7> K_p;
        K_p(0) = L_p(0,0);
        K_p(1) = L_p(1,0);
        K_p(2) = L_p(2,0);
        K_p(3) = L_p(3,0);
        K_p(4) = L_p(4,0);
        K_p(5) = L_p(5,0);
        K_p(6) = L_p(6,0);
        xhat_p = xhat_p + K_p*(input.gps_n - h_p);

        // gps East position
        h_p = xhat_p(1);
        C_p.zero();
        C_p(0,1) = 1;
        C_p_t.zero();
        C_p_t(1,0) = 1;
        denom(0,0) = (R_p(1,1) + (C_p*P_p*C_p_t)(0,0));
        L_p = (P_p*C_p_t) * denom.inversed();
        P_p = (I_p - L_p*C_p)*P_p;
        K_p(0) = L_p(0,0);
        K_p(1) = L_p(1,0);
        K_p(2) = L_p(2,0);
        K_p(3) = L_p(3,0);
        K_p(4) = L_p(4,0);
        K_p(5) = L_p(5,0);
        K_p(6) = L_p(6,0);
        xhat_p = xhat_p + K_p*(input.gps_e - h_p);

        // gps ground speed
        h_p = xhat_p(2);
        C_p.zero();
        C_p(0,2) = 1;
        C_p_t.zero();
        C_p_t(2,0) = 1;
        denom(0,0) = (R_p(2,2) + (C_p*P_p*C_p_t)(0,0));
        L_p = (P_p*C_p_t) * denom.inversed();
        P_p = (I_p - L_p*C_p)*P_p;
        K_p(0) = L_p(0,0);
        K_p(1) = L_p(1,0);
        K_p(2) = L_p(2,0);
        K_p(3) = L_p(3,0);
        K_p(4) = L_p(4,0);
        K_p(5) = L_p(5,0);
        K_p(6) = L_p(6,0);
        xhat_p = xhat_p + K_p*(input.gps_Vg - h_p);

        // gps course
        //wrap course measurement
        float gps_course = fmodf(input.gps_course, math::radians(360.0f));

        while(gps_course - xhat_p(3) > math::radians(180.0f)) gps_course = gps_course - math::radians(360.0f);
        while(gps_course - xhat_p(3) < math::radians(-180.0f)) gps_course = gps_course + math::radians(360.0f);
        h_p = xhat_p(3);
        C_p.zero();
        C_p(0,3) = 1;
        C_p_t.zero();
        C_p_t(3,0) = 1;
        denom(0,0) = (R_p(3,3) + (C_p*P_p*C_p_t)(0,0));
        L_p = (P_p*C_p_t) * denom.inversed();
        P_p = (I_p - L_p*C_p)*P_p;
        K_p(0) = L_p(0,0);
        K_p(1) = L_p(1,0);
        K_p(2) = L_p(2,0);
        K_p(3) = L_p(3,0);
        K_p(4) = L_p(4,0);
        K_p(5) = L_p(5,0);
        K_p(6) = L_p(6,0);
        xhat_p = xhat_p + K_p*(gps_course - h_p);


//        // pseudo measurement #1 y_1 = Va*cos(psi)+wn-Vg*cos(chi)
//        h_p = Vahat*cosf(xhat_p(6)) + xhat_p(4) - xhat_p(2)*cosf(xhat_p(3));  // pseudo measurement
//        C_p.zero();
//        C_p(0,2) = -cos(xhat_p(3));
//        C_p(0,3) = xhat_p(2)*sinf(xhat_p(3));
//        C_p(0,4) = 1;
//        C_p(0,6) = -Vahat*sinf(xhat_p(6));
//        C_p_t.zero();
//        C_p_t(2,0) = -cos(xhat_p(3));
//        C_p_t(3,0) = xhat_p(2)*sinf(xhat_p(3));
//        C_p_t(4,0) = 1;
//        C_p_t(6,0) = -Vahat*sinf(xhat_p(6));
//        denom(0,0) = (R_p(4,4) + (C_p*P_p*C_p_t)(0,0));
//        L_p = (P_p*C_p_t) * denom.inversed();
//        P_p = (I_p - L_p*C_p)*P_p;
//        K_p(0) = L_p(0,0);
//        K_p(1) = L_p(1,0);
//        K_p(2) = L_p(2,0);
//        K_p(3) = L_p(3,0);
//        K_p(4) = L_p(4,0);
//        K_p(5) = L_p(5,0);
//        K_p(6) = L_p(6,0);
//        xhat_p = xhat_p + K_p*(0 - h_p);

//        // pseudo measurement #2 y_2 = Va*sin(psi) + we - Vg*sin(chi)
//        h_p = Vahat*sinf(xhat_p(6))+xhat_p(5)-xhat_p(2)*sinf(xhat_p(3));  // pseudo measurement
//        C_p.zero();
//        C_p(0,2) = -sin(xhat_p(3));
//        C_p(0,3) = -xhat_p(2)*cosf(xhat_p(3));
//        C_p(0,5) = 1;
//        C_p(0,6) = Vahat*cosf(xhat_p(6));
//        C_p_t.zero();
//        C_p_t(2,0) = -sin(xhat_p(3));
//        C_p_t(3,0) = -xhat_p(2)*cosf(xhat_p(3));
//        C_p_t(5,0) = 1;
//        C_p_t(6,0) = Vahat*cosf(xhat_p(6));
//        denom(0,0) = (R_p(5,5) + (C_p*P_p*C_p_t)(0,0));
//        L_p = (P_p*C_p_t) * denom.inversed();
//        P_p = (I_p - L_p*C_p)*P_p;
//        K_p(0) = L_p(0,0);
//        K_p(1) = L_p(1,0);
//        K_p(2) = L_p(2,0);
//        K_p(3) = L_p(3,0);
//        K_p(4) = L_p(4,0);
//        K_p(5) = L_p(5,0);
//        K_p(6) = L_p(6,0);
//        xhat_p = xhat_p + K_p*(0 - h_p);

        gps_n_old      = input.gps_n;
        gps_e_old      = input.gps_e;
        gps_Vg_old     = input.gps_Vg;
        gps_course_old = input.gps_course;

        if(xhat_p(0) > 1000 || xhat_p(0) < -1000)
        {
            warnx("problem 11");
            xhat_p(0) = input.gps_n;
        }
        if(xhat_p(1) > 1000 || xhat_p(1) < -1000)
        {
            warnx("problem 12");
            xhat_p(1) = input.gps_e;
        }
//        if(xhat_p(2) > 35 || xhat_p(2) < 0)
//        {
//            warnx("problem 13");
//            xhat_p(2) = input.gps_Vg;
//        }
//        if(xhat_p(3) > math::radians(720.0f) || xhat_p(3) < math::radians(-720.0f))
//        {
//            warnx("problem 14");
//            xhat_p(3) = input.gps_course;
//        }
//        if(xhat_p(6) > math::radians(720.0f) || xhat_p(6) < math::radians(-720.0f))
//        {
//            warnx("problem 15");
//            xhat_p(6) = input.gps_course;
//        }
    }

    bool problem = false;
    int prob_index;
    for(int i=0;i<7;i++)
    {
        if(!isfinite(xhat_p(i)))
        {
            if(!problem)
            {
                problem = true;
                prob_index = i;
            }
            switch(i)
            {
            case 0:
                xhat_p(i) = gps_n_old;
                break;
            case 1:
                xhat_p(i) = gps_e_old;
                break;
            case 2:
                xhat_p(i) = gps_Vg_old;
                break;
            case 3:
                xhat_p(i) = gps_course_old;
                break;
            case 6:
                xhat_p(i) = gps_course_old;
                break;
            default:
                xhat_p(i) = 0;
            }
            P_p.identity();
            P_p(0,0) = .03;
            P_p(1,1) = .03;
            P_p(2,2) = .01;
            P_p(3,3) = math::radians(5.0f);
            P_p(4,4) = .04;
            P_p(5,5) = .04;
            P_p(6,6) = math::radians(5.0f);
        }
    }
    if(problem) { warnx("problem 10 %d %d", prob_index, (input.gps_new ? 1 : 0)); }
    if(xhat_p(6) - xhat_p(3) > math::radians(360.0f) || xhat_p(6) - xhat_p(3) < math::radians(-360.0f))
    {
        //xhat_p(3) = fmodf(xhat_p(3),math::radians(360.0f));
        xhat_p(6) = fmodf(xhat_p(6),math::radians(360.0f));
    }

    float pnhat = xhat_p(0);
    float pehat = xhat_p(1);
    float Vghat = xhat_p(2);
    float chihat = xhat_p(3);
    float wnhat = xhat_p(4);
    float wehat = xhat_p(5);
    float psihat = xhat_p(6);

    output.pn = pnhat;
    output.pe = pehat;
    output.h = hhat;
    output.Va = Vahat;
    output.alpha = 0;
    output.beta = 0;
    output.phi = phihat;
    output.theta = thetahat;
    output.chi = chihat;
    output.p = phat;
    output.q = qhat;
    output.r = rhat;
    output.Vg = Vghat;
    output.wn = wnhat;
    output.we = wehat;
    output.psi = psihat;
}
